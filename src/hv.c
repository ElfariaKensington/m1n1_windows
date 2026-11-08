/* SPDX-License-Identifier: MIT */

#include "../build/build_cfg.h"
#include "hv.h"
#include "hv_tick_policy.h"
#include "hv_runtime_diag.h"
#include "hv_wfx_policy.h"
#include "assert.h"
#include "cpu_regs.h"
#include "display.h"
#include "gxf.h"
#include "hv_diag.h"
#include "hv_fb_stream.h"
#include "hv_guest_cpu_state.h"
#include "hv_apple_input.h"
#include "hv_irq_routes.h"
#ifdef M1N1_J316S_SETUP
#include "hv_j316s_setup.h"
#include "hv_j316s_wifi.h"
#endif
#include "hv_nvme_queue.h"
#include "hv_vgic.h"
#include "memory.h"
#include "pcie.h"
#include "smp.h"
#include "string.h"
#include "usb.h"
#include "uartproxy.h"
#include "utils.h"
#include "adt.h"
#include "xnuboot.h"

#define HV_TICK_RATE      1000
#define HV_SLOW_TICK_RATE 1

DECLARE_SPINLOCK(bhl);

void hv_enter_guest(u64 x0, u64 x1, u64 x2, u64 x3, void *entry);
void hv_exit_guest(void) __attribute__((noreturn));

extern char _hv_vectors_start[0];

u64 hv_tick_interval;
static u64 hv_runtime_tick_interval;
static u64 hv_guest_irq_recovery_tick_interval;
u64 hv_tick_arm_count[MAX_CPUS];
u64 hv_recovery_tick_arm_count[MAX_CPUS];
static bool hv_guest_runtime_ready;
static bool hv_isolated_setup;
u64 hv_secondary_tick_interval;

int hv_pinned_cpu;
int hv_want_cpu;

static bool hv_has_ecv;
static bool hv_should_exit[MAX_CPUS];
bool hv_started_cpus[MAX_CPUS];
u64 hv_cpus_in_guest;
bool hv_rendezvous_active;
u64 hv_saved_sp[MAX_CPUS];
static struct hv_fb_stream hv_framebuffer_stream;
static struct hv_diag_delivery hv_diag_event_delivery;

static void hv_collect_diag(void *opaque, const struct exc_info *ctx,
                            struct hv_diag_sample_v1 *sample)
{
    UNUSED(opaque);
    struct vnvme_snapshot nvme = {0};
    bool nvme_ready = false;
    struct hv_fb_stream_stats fb = hv_fb_stream_get_stats(&hv_framebuffer_stream);

    hv_nvme_get_diag_snapshot(&nvme, &nvme_ready);
    sample->host_fiq_count = hv_fiq_count;
    sample->host_tick_count = hv_fiq_ticks;
    if (ctx) {
        sample->guest_pc = ctx->elr;
        sample->guest_spsr = ctx->spsr;
    }
    sample->nvme_sq_doorbells = nvme.stats.sq_doorbells;
    sample->nvme_cq_doorbells = nvme.stats.cq_doorbells;
    sample->nvme_commands = nvme.stats.commands;
    sample->nvme_completions = nvme.stats.completions;
    sample->fb_completed_frames = fb.completed_frames;
    sample->fb_backpressure_skips = fb.backpressure_skips;
    if (nvme_ready)
        sample->flags |= HV_DIAG_FLAG_NVME_READY;
    if (nvme.irq_asserted)
        sample->flags |= HV_DIAG_FLAG_NVME_IRQ_ASSERTED;
    if (hv_framebuffer_stream.enabled)
        sample->flags |= HV_DIAG_FLAG_FB_ENABLED;

    _Static_assert(HV_DIAG_QUEUE_COUNT == VNVME_MAX_QUEUES, "diagnostic NVMe queue count");
    for (u32 i = 0; i < HV_DIAG_QUEUE_COUNT; i++) {
        sample->queues[i] = (struct hv_diag_queue_v1){
            .sq_head = nvme.queues[i].sq_head,
            .sq_tail = nvme.queues[i].sq_tail,
            .cq_head = nvme.queues[i].cq_head,
            .cq_tail = nvme.queues[i].cq_tail,
        };
    }

#ifdef ENABLE_VGIC_MODULE
    struct hv_vgic_diag_snapshot vgic;
    hv_vgic3_get_diag_snapshot(&vgic);
    sample->vgic_pending_lrs = vgic.pending_lrs;
    sample->vgic_active_lrs = vgic.active_lrs;
    sample->vgic_occupied_lrs = vgic.occupied_lrs;
#endif
}

static bool hv_send_fb_chunk(void *opaque, const struct hv_fb_chunk_header *header,
                             const void *payload)
{
    (void)opaque;
    return uartproxy_try_send_eventv(EVT_FRAMEBUFFER, header, sizeof(*header), payload,
                                     header->payload_size);
}

static bool hv_send_diag_sample(void *opaque, const struct hv_diag_sample_v1 *sample)
{
    UNUSED(opaque);
    return uartproxy_try_send_eventv(EVT_TELEMETRY, NULL, 0, sample, sizeof(*sample));
}

bool hv_configure_fb_stream(u64 ipa, u64 size, u64 width, u64 height, u64 stride)
{
    if (!ipa && !size && !width && !height && !stride) {
        hv_fb_stream_disable(&hv_framebuffer_stream);
        return true;
    }
    if (width > 0xffffffffULL || height > 0xffffffffULL || stride > 0xffffffffULL)
        return false;

    return hv_fb_stream_configure(&hv_framebuffer_stream, ipa, size, width, height, stride,
                                  hv_ipa_to_pa, hv_send_fb_chunk, NULL);
}

struct hv_secondary_info_t {
    uint64_t hcr;
    uint64_t hacr;
    uint64_t vtcr, vttbr;
    uint64_t mdcr;
    uint64_t mdscr;
    uint64_t amx_ctl;
    uint64_t apvmkeylo, apvmkeyhi, apsts;
    uint64_t actlr_el2;
    uint64_t actlr_el1;
    uint64_t cnthctl;
    uint64_t cntvoff;
    uint64_t sprr_config;
    uint64_t gxf_config;
};

static struct hv_secondary_info_t hv_secondary_info;

struct hv_secondary_launch_context {
    void *entry;
    u64 regs[4];
};

static struct hv_secondary_launch_context secondary_launch[MAX_CPUS];

static void hv_configure_guest_wfi(void)
{
    if (!cpu_features->cyc_ovrd)
        return;

    /*
     * Keep guest WFI in the architectural, context-retaining "core up" mode.
     * The generic CPU bring-up code selects mode 2, but the hypervisor used to
     * reset it to mode 0 immediately before entering the guest.  On an idle M1
     * Icestorm secondary that transition loses x18 (Windows' KPCR pointer)
     * across HalProcessorIdle's WFI, and the first KfRaiseIrql dereference then
     * bugchecks at address 0x38.  Mode 2 still waits for an interrupt; it only
     * prevents the undocumented deep power transition from discarding guest
     * architectural state.
     */
    reg_mask(SYS_IMP_APL_CYC_OVRD, CYC_OVRD_WFI_MODE_MASK, CYC_OVRD_WFI_MODE(2));
    sysop("isb");
}

static bool hv_prepare_j313_irq_routes(void)
{
    /* Assisted and autonomous launches share this exact passthrough gate.
     * It only maps reviewed resources and routes the level interrupt; the
     * Windows driver remains the sole owner of SPI/GPIO programming. */
    if (!hv_irq_routes_register_agx_g2()) {
        printf("HV: AGX G2 interrupt route registration failed\n");
        return false;
    }
#ifdef HV_DISABLE_APPLE_INPUT
    /* Narrow hardware A/B profile: preserve every other launch contract while
     * proving whether the optional SPI-HID mapping/route affects guest timing. */
    printf("HV: Apple input passthrough disabled by A/B build\n");
#else
    struct hv_apple_input_prepare_result apple_input;
    if (!hv_apple_input_prepare_runtime(&apple_input))
        printf("HV: Apple input passthrough disabled (prepare=%u preflight=%u)\n",
               apple_input.failure, apple_input.preflight_failure);
    else
        printf("HV: Apple input passthrough mapped, IRQ %u -> vINTID %u\n",
               (u32)HV_APPLE_INPUT_PHYSICAL_PARENT_IRQ,
               (u32)HV_APPLE_INPUT_GUEST_VINTID);
#endif
    return true;
}

static bool hv_init_mode(bool isolated_setup)
{
    hv_isolated_setup = isolated_setup;
    hv_diag_reset();
    hv_diag_set_collector(hv_collect_diag, NULL);
    hv_fb_stream_disable(&hv_framebuffer_stream);
    if (!isolated_setup) {
        pcie_shutdown();
        // Make sure we wake up DCP if we put it to sleep, just quiesce it to match ADT
        if (display_is_external && display_start_dcp() >= 0)
            display_shutdown(DCP_QUIESCED);
        // reenable hpm interrupts for the guest for unused iodevs
        usb_hpm_restore_irqs(0);
    }
    bool secondaries_ready = smp_start_secondaries();
    smp_set_wfe_mode(true);
    hv_wdt_init();

    hv_pt_init();

    bool j313 = adt_is_compatible(adt, 0, "J313AP");
    hv_irq_routes_reset(j313);
    if (j313 && !hv_prepare_j313_irq_routes())
        return false;

    // Configure hypervisor defaults

    //
    // UNKNOWN: do we need to bring TGE back? might have misunderstood why it was there at the start.
    // leaving it off for now.
    //
    hv_write_hcr(HCR_API | // Allow PAuth instructions
                 HCR_APK | // Allow PAuth key registers
                 HCR_TEA | // Trap external aborts
                 HCR_RW |  // AArch64 guest
                 HCR_TSC | // Trap SMC exceptions (only writable on Blizzard/Avalanche cores as the previous generations used a chicken bit for this.)
                 HCR_AMO | // Trap SError exceptions
                 HCR_IMO | // Trap IRQ exceptions (for now)
                 HCR_FMO | // Trap FIQ exceptions (effectively required for now)
                 HCR_TWI | // Halt guest WFI in lock-free EL2; preserve native WFE/SEV
#ifdef ENABLE_VGIC_MODULE
                 //
                 // Trap EL1 reads of the ID registers. Without this the "advertise GIC"
                 // case for ID_AA64PFR0_EL1 in hv_exc.c never runs: the guest reads the
                 // real register, sees GIC = 0 (the M1 implements the GIC system
                 // registers but does not advertise them), and both EDK2 and the Windows
                 // HAL fall back to driving a GICv2 over MMIO -- for which there is no
                 // delivery path. hv_handle_msr_unlocked() serves the whole ID space.
                 //
                 // Note: this must live here and not in exception_initialize(), because
                 // this call rewrites HCR_EL2 wholesale and would drop the bit.
                 //
                 HCR_TID3 | // Trap ID register reads from EL1
#endif
                 HCR_VM);  // Enable stage 2 translation
#ifdef M1N1_J316S_SETUP
    /* Setup enables GXF at guest entry, after configuring the unguarded bank. */
    if (isolated_setup && supports_gxf() && !gxf_enabled())
        gl2_call(hv_write_hcr, mrs(HCR_EL2), 0, 0, 0);
#endif

    // No guest vectors initially
    msr(VBAR_EL12, 0);

    // Preserve the PMUv3 trap used by the established assisted path.
    printf("DEBUG: setting up HACR\n");
    uint64_t hacr_val = mrs(HACR_EL2);
    hacr_val |= BIT(56);
    msr(HACR_EL2, hacr_val);

    //
    // m1n1_windows change: initialize PSCI.
    //
    printf("DEBUG: setting up PSCI\n");
    hv_psci_init();
#ifdef ENABLE_VGIC_MODULE
    //
    // m1n1_windows change: set up the vGIC
    //

    //
    if (!hv_vgicv3_init())
        return false;
    init_vgic_irq_queues();
    //
#endif

    // Compute tick interval
    hv_tick_interval = hv_tick_interval_ticks(mrs(CNTFRQ_EL0), hv_boot_tick_rate());
    hv_runtime_tick_interval =
        hv_tick_interval_ticks(mrs(CNTFRQ_EL0), hv_runtime_tick_rate());
    hv_guest_irq_recovery_tick_interval =
        hv_tick_interval_ticks(mrs(CNTFRQ_EL0), hv_guest_irq_recovery_tick_rate());
    hv_guest_runtime_ready = false;

    hv_has_ecv = mrs(ID_AA64MMFR0_EL1) & (0xfULL << 60);
    u32 secondary_tick_rate = hv_secondary_tick_rate(hv_has_ecv);
    hv_secondary_tick_interval =
        hv_tick_interval_ticks(mrs(CNTFRQ_EL0), secondary_tick_rate);
    printf("HV: tick rates boot=%uHz secondary=%uHz\n", hv_boot_tick_rate(),
           secondary_tick_rate);

    if (hv_has_ecv) {
        printf("HV: ECV enabled\n");
        reg_set(CNTHCTL_EL2,
                CNTHCTL_EL1NVVCT | CNTHCTL_EL1NVPCT | CNTHCTL_EL1TVT | CNTHCTL_EL1PCTEN);
    } else {
        printf("HV: No ECV supported\n");
        // Enable physical timer for EL1
        msr(CNTHCTL_EL2, CNTHCTL_EL1PTEN | CNTHCTL_EL1PCTEN);

    }

    hv_configure_guest_wfi();

    sysop("dsb ishst");
    sysop("tlbi alle1is");
    sysop("dsb ish");
    sysop("isb");
    return secondaries_ready;
}

bool hv_init(void)
{
    return hv_init_mode(false);
}

#ifdef M1N1_J316S_SETUP
bool hv_init_j316s_setup(void)
{
    return adt_is_compatible(adt, 0, "J316sAP") && hv_init_mode(true);
}
#endif

void hv_prepare_guest_cpu_state(void)
{
    /*
     * The assisted launcher performs these writes from Python after hv_init()
     * and immediately before guest launch. Autonomous boot has no host
     * process, so reproduce that boundary explicitly. hv_start() snapshots
     * this state and copies it to every secondary CPU.
     */
    u64 current_guest_actlr;
    if (cpu_features->actlr_el2)
        current_guest_actlr = mrs(SYS_ACTLR_EL12);
    else
        current_guest_actlr = mrs(SYS_IMP_APL_ACTLR_EL12);
    struct hv_guest_cpu_state guest_cpu =
        hv_guest_cpu_state_prepare(mrs(SYS_IMP_APL_AMX_CTL_EL1),
                                   current_guest_actlr);

    printf("DEBUG: setting up guest CPU state\n");
    msr(HACR_EL2, guest_cpu.hacr);
    msr(MDCR_EL2, guest_cpu.mdcr);
    msr(MDSCR_EL1, guest_cpu.mdscr);
    msr(SYS_IMP_APL_AMX_CTL_EL1, guest_cpu.amx_config);
    msr(SYS_IMP_APL_APVMKEYLO_EL2, guest_cpu.apvmkeylo);
    msr(SYS_IMP_APL_APVMKEYHI_EL2, guest_cpu.apvmkeyhi);
    msr(SYS_IMP_APL_APSTS_EL12, guest_cpu.apsts);
    if (cpu_features->actlr_el2)
        msr(SYS_ACTLR_EL12, guest_cpu.actlr);
    else
        msr(SYS_IMP_APL_ACTLR_EL12, guest_cpu.actlr);
    sysop("isb");
}

static void hv_set_gxf_vbar(void)
{
    msr(SYS_IMP_APL_VBAR_GL1, _hv_vectors_start);
}

static u64 hv_read_guest_hcr(void)
{
    return mrs(HCR_EL2);
}

void hv_start(void *entry, u64 regs[4])
{
    if (boot_cpu_idx == -1) {
        printf("Boot CPU has not been found, can't start hypervisor\n");
        return;
    }

    memset(hv_should_exit, 0, sizeof(hv_should_exit));
    memset(hv_started_cpus, 0, sizeof(hv_started_cpus));

    hv_started_cpus[boot_cpu_idx] = true;

    msr(VBAR_EL1, _hv_vectors_start);

    if (gxf_enabled())
        gl2_call(hv_set_gxf_vbar, 0, 0, 0, 0);

    /* GXF has a guarded HCR bank; verify the EL2 idle policy there. */
    u64 guest_hcr;
    if (gxf_enabled())
        guest_hcr = gl2_call(hv_read_guest_hcr, 0, 0, 0, 0);
    else
        guest_hcr = mrs(HCR_EL2);
#ifdef M1N1_J316S_SETUP
    if (hv_isolated_setup &&
        (!(guest_hcr & HCR_VM) || !hv_wfx_policy_satisfied(guest_hcr))) {
        printf("J316s Setup: unsafe guest HCR=0x%lx\n", guest_hcr);
        return;
    }
#endif
    if (!hv_wfx_policy_satisfied(guest_hcr))
        hv_panic("HV: EL2 WFI/WFE policy failed guest preflight\n");
    printf("HV: EL2 WFI/WFE idle policy active HCR=0x%lx\n", guest_hcr);

    hv_secondary_info.hcr = guest_hcr;
    hv_secondary_info.hacr = mrs(HACR_EL2);
    hv_secondary_info.vtcr = mrs(VTCR_EL2);
    hv_secondary_info.vttbr = mrs(VTTBR_EL2);
    hv_secondary_info.mdcr = mrs(MDCR_EL2);
    hv_secondary_info.mdscr = mrs(MDSCR_EL1);
    hv_secondary_info.amx_ctl = mrs(SYS_IMP_APL_AMX_CTL_EL2);
    hv_secondary_info.apvmkeylo = mrs(SYS_IMP_APL_APVMKEYLO_EL2);
    hv_secondary_info.apvmkeyhi = mrs(SYS_IMP_APL_APVMKEYHI_EL2);
    hv_secondary_info.apsts = mrs(SYS_IMP_APL_APSTS_EL12);
    hv_secondary_info.actlr_el2 = mrs(ACTLR_EL2);
    if (cpu_features->actlr_el2)
        hv_secondary_info.actlr_el1 = mrs(SYS_ACTLR_EL12);
    else
        hv_secondary_info.actlr_el1 = mrs(SYS_IMP_APL_ACTLR_EL12);
    hv_secondary_info.cnthctl = mrs(CNTHCTL_EL2);
    hv_secondary_info.cntvoff = mrs(CNTVOFF_EL2);
    hv_secondary_info.sprr_config = mrs(SYS_IMP_APL_SPRR_CONFIG_EL1);
    hv_secondary_info.gxf_config = mrs(SYS_IMP_APL_GXF_CONFIG_EL1);

    //
    // Enable the guest timer FIQs on this core. This register is per-CPU and nothing else
    // initialises it - the boot core gets it from m1n1's startup path, a secondary came up
    // with the physical-timer FIQ masked (observed: vm_tmr=0x2 on CPU1 against 0x3 on CPU0).
    // The guest's per-core timer then never fires there, which wrecks Windows' IRQL and
    // scheduling bookkeeping on that core and shows up as IRQL_NOT_LESS_OR_EQUAL.
    //
    msr(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2, VM_TMR_FIQ_ENA_ENA_P | VM_TMR_FIQ_ENA_ENA_V);

#ifdef ENABLE_VGIC_MODULE
    hv_vgicv3_enable_virtual_interrupts();
    hv_vgicv3_init_list_registers();
#endif

    hv_arm_tick(false);
    hv_pinned_cpu = -1;
    hv_want_cpu = -1;
    hv_cpus_in_guest = BIT(smp_id());

    if (!hv_isolated_setup) {
        u64 adt_base = chip_id == T8103 || chip_id == T8112 ? ADT_EL2_36_BIT : ADT_EL2_42_BIT;
        // Map the EL2 ADT to a fixed location for the established guest path.
        hv_map_hw(adt_base, (u64)adt, ALIGN_UP(cur_boot_args.devtree_size, SZ_16K));
    }

    hv_enter_guest(regs[0], regs[1], regs[2], regs[3], entry);

    __atomic_and_fetch(&hv_cpus_in_guest, ~BIT(smp_id()), __ATOMIC_ACQUIRE);
    spin_lock(&bhl);

    hv_wdt_stop();

    printf("HV: Exiting hypervisor (main CPU)\n");

    spin_unlock(&bhl);
    // Wait a bit for the guest CPUs to exit on their own if they are in the process.
    udelay(200000);
    spin_lock(&bhl);

    hv_started_cpus[boot_cpu_idx] = false;

    for (int i = 0; i < MAX_CPUS; i++) {
        if (i == boot_cpu_idx) {
            continue;
        }
        hv_should_exit[i] = true;
        if (hv_started_cpus[i]) {
            printf("HV: Waiting for CPU %d to exit\n", i);
            spin_unlock(&bhl);
            smp_wait(i);
            spin_lock(&bhl);
            hv_started_cpus[i] = false;
        }
    }

    printf("HV: All CPUs exited\n");
    spin_unlock(&bhl);
}

static void hv_init_secondary(struct hv_secondary_info_t *info)
{
    gxf_init();

    msr(VBAR_EL1, _hv_vectors_start);

    msr(HCR_EL2, hv_wfx_apply_hcr(info->hcr));
    msr(HACR_EL2, info->hacr);
    msr(VTCR_EL2, info->vtcr);
    msr(VTTBR_EL2, info->vttbr);
    msr(MDCR_EL2, info->mdcr);
    msr(MDSCR_EL1, info->mdscr);
    msr(SYS_IMP_APL_AMX_CTL_EL2, info->amx_ctl);
    msr(SYS_IMP_APL_APVMKEYLO_EL2, info->apvmkeylo);
    msr(SYS_IMP_APL_APVMKEYHI_EL2, info->apvmkeyhi);
    msr(SYS_IMP_APL_APSTS_EL12, info->apsts);
    msr(ACTLR_EL2, info->actlr_el2);
    if (cpu_features->actlr_el2)
        msr(SYS_ACTLR_EL12, info->actlr_el1);
    else
        msr(SYS_IMP_APL_ACTLR_EL12, info->actlr_el1);
    msr(CNTHCTL_EL2, info->cnthctl);
    msr(CNTVOFF_EL2, info->cntvoff);
    msr(SYS_IMP_APL_SPRR_CONFIG_EL1, info->sprr_config);
    msr(SYS_IMP_APL_GXF_CONFIG_EL1, info->gxf_config);

    /*
     * VM_TMR_FIQ_ENA_EL2 is per-CPU.  The boot CPU is configured in hv_init(),
     * but an SMP secondary does not inherit that value.  Leaving either route
     * disabled makes Windows lose that processor's architectural clock PPI;
     * the observed failure is CLOCK_WATCHDOG_TIMEOUT with CPU1 named as the
     * hung processor.  Program both physical and virtual guest timer routes
     * before the core enters Windows or arms the EL2 heartbeat.
     */
    msr(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2,
        VM_TMR_FIQ_ENA_ENA_P | VM_TMR_FIQ_ENA_ENA_V);
    sysop("isb");

#ifdef ENABLE_VGIC_MODULE
    hv_vgicv3_enable_virtual_interrupts();
    hv_vgicv3_init_list_registers();
#endif

    hv_configure_guest_wfi();

    if (gxf_enabled())
        gl2_call(hv_set_gxf_vbar, 0, 0, 0, 0);

    hv_arm_tick(true);
}

static void hv_enter_secondary(struct hv_secondary_launch_context *launch)
{
    sysop("dmb ish");
    void *entry = launch->entry;
    u64 regs[4];
    memcpy(regs, launch->regs, sizeof(regs));

    printf("HV: Secondary %d consumed entry=%p x0=0x%lx\n", smp_id(), entry, regs[0]);
    hv_enter_guest(regs[0], regs[1], regs[2], regs[3], entry);

    spin_lock(&bhl);

    printf("HV: Exiting from CPU %d\n", smp_id());

    __atomic_and_fetch(&hv_cpus_in_guest, ~BIT(smp_id()), __ATOMIC_ACQUIRE);

    hv_started_cpus[smp_id()] = false;
    spin_unlock(&bhl);
}

void hv_start_secondary(int cpu, void *entry, u64 regs[4])
{
    if (cpu < 0 || cpu >= MAX_CPUS)
        return;

    struct hv_secondary_launch_context *launch = &secondary_launch[cpu];
    launch->entry = entry;
    memcpy(launch->regs, regs, sizeof(launch->regs));
    sysop("dsb ishst");

    printf("HV: Initializing secondary %d\n", cpu);
    printf("HV: Secondary %d published entry=%p x0=0x%lx\n", cpu, entry, launch->regs[0]);
    iodev_console_flush();

    mmu_init_secondary(cpu);
    iodev_console_flush();
    smp_call4(cpu, hv_init_secondary, (u64)&hv_secondary_info, 0, 0, 0);
    smp_wait(cpu);
    iodev_console_flush();

    printf("HV: Entering guest secondary %d at %p\n", cpu, entry);
    hv_started_cpus[cpu] = true;
    __atomic_or_fetch(&hv_cpus_in_guest, BIT(cpu), __ATOMIC_ACQ_REL);

    iodev_console_flush();
    smp_call1(cpu, hv_enter_secondary, (u64)launch);
}

void hv_exit_cpu(int cpu)
{
    if (cpu == -1)
        cpu = smp_id();

    printf("HV: Requesting exit of CPU#%d from the guest\n", cpu);
    hv_should_exit[cpu] = true;
}

void hv_rendezvous(void)
{
    int timeout = 1000000;

    if (!__atomic_load_n(&hv_cpus_in_guest, __ATOMIC_ACQUIRE))
        return;

    /*
     * Publish this before the physical IPIs.  A secondary FIQ must not consume
     * a rendezvous IPI in its local fast path: it has to enter hv_exc_entry(),
     * which clears that CPU's hv_cpus_in_guest bit for the waiter below.
     */
    __atomic_store_n(&hv_rendezvous_active, true, __ATOMIC_RELEASE);

    /* IPI all CPUs. This might result in spurious IPIs to the guest... */
    for (int i = 0; i < MAX_CPUS; i++) {
        if (i != smp_id() && hv_started_cpus[i]) {
            smp_send_ipi(i);
        }
    }

    while (timeout--) {
        if (!__atomic_load_n(&hv_cpus_in_guest, __ATOMIC_ACQUIRE)) {
            __atomic_store_n(&hv_rendezvous_active, false, __ATOMIC_RELEASE);
            return;
        }
    }

    hv_panic("HV: Failed to rendezvous, missing CPUs: 0x%lx (current: %d)\n",
             __atomic_load_n(&hv_cpus_in_guest, __ATOMIC_ACQUIRE), smp_id());
}

bool hv_switch_cpu(int cpu)
{
    if (cpu > MAX_CPUS || cpu < 0 || !hv_started_cpus[cpu]) {
        printf("HV: CPU #%d is inactive or invalid\n", cpu);
        return false;
    }
    printf("HV: switching to CPU #%d\n", cpu);
    hv_want_cpu = cpu;
    hv_rendezvous();
    return true;
}

void hv_pin_cpu(int cpu)
{
    hv_pinned_cpu = cpu;
}

void hv_write_hcr(u64 val)
{
    /*
     * Callers can read the unguarded HCR bank and then ask this helper to
     * update VI/VF in the guarded GL2 bank. Guest idle is always trapped and
     * emulated by a real wait in the lock-free EL2 exception path.
     */
    val = hv_wfx_apply_hcr(val);
    if (gxf_enabled() && !in_gl12())
        gl2_call(hv_write_hcr, val, 0, 0, 0);
    else
        msr(HCR_EL2, val);
}

u64 hv_get_spsr(void)
{
    if (in_gl12())
        return mrs(SYS_IMP_APL_SPSR_GL1);
    else
        return mrs(SPSR_EL2);
}

void hv_set_spsr(u64 val)
{
    if (in_gl12())
        return msr(SYS_IMP_APL_SPSR_GL1, val);
    else
        return msr(SPSR_EL2, val);
}

u64 hv_get_esr(void)
{
    if (in_gl12())
        return mrs(SYS_IMP_APL_ESR_GL1);
    else
        return mrs(ESR_EL2);
}

u64 hv_get_far(void)
{
    if (in_gl12())
        return mrs(SYS_IMP_APL_FAR_GL1);
    else
        return mrs(FAR_EL2);
}

u64 hv_get_afsr1(void)
{
    if (in_gl12())
        return mrs(SYS_IMP_APL_AFSR1_GL1);
    else
        return mrs(AFSR1_EL2);
}

u64 hv_get_elr(void)
{
    if (in_gl12())
        return mrs(SYS_IMP_APL_ELR_GL1);
    else
        return mrs(ELR_EL2);
}

void hv_set_elr(u64 val)
{
    if (in_gl12())
        return msr(SYS_IMP_APL_ELR_GL1, val);
    else
        return msr(ELR_EL2, val);
}

//
// Diagnostics only, run on EVERY core.
//
// hv_tick() runs solely on the interruptible CPU, so with a secondary online the PC sampler
// and the stuck/bugcheck detector simply never executed there - a two-core run produced zero
// HV SAMPLE lines and no bugcheck data, which is why IRQL_NOT_LESS_OR_EQUAL could not be
// attributed. This is deliberately NOT hv_tick(): that function also carries functional work
// tied to one core, and calling it everywhere would invent a new SMP bug instead of restoring
// observability.
//
// All state is per-CPU and only ever written by its owner, so no atomics are needed. The
// bugcheck dump is the one exception: KiBugCheckData is global, so it is latched once.
//
#define HV_DIAG_STUCK_TICKS 40000u  // ~8 s at the 5 kHz tick
#define HV_DIAG_X18_REPORT_LIMIT 16u

struct hv_diag_cpu {
    u64 fiq_count;
    u64 sample_count;
    u64 last_pc;
    u64 same_pc_ticks;
    u64 bcd_va;                 // cached once recovered
    u64 last_x18;
    u32 x18_reports;
    bool x18_seen;
    bool online_reported;
} __attribute__((aligned(64)));

static struct hv_diag_cpu hv_diag[MAX_CPUS];
static bool hv_bugcheck_dumped;

//
// Windows draws its stop screen spinning in HalpGitQueryCounter (mrs x0, CNTPCT_EL0 =
// 0xd53be040 in this build). That PC identifies the kernel base, and KiBugCheckData sits at a
// known offset from it - the same recovery the existing stuck reporter uses.
//
static bool hv_diag_read_bugcheck(struct hv_diag_cpu *d, u64 pc, u64 out[5])
{
    if (!d->bcd_va) {
        u64 pc_pa = hv_translate(pc & ~3UL, false, false, NULL);
        if (!pc_pa || read32(pc_pa) != 0xd53be040)
            return false;
        d->bcd_va = pc - 0x213a98UL + 0xdbb9a0UL;
    }

    u64 pa = hv_translate(d->bcd_va, false, false, NULL);
    if (!pa)
        return false;

    for (int i = 0; i < 5; i++)
        out[i] = read64(pa + 8 * i);

    return out[0] != 0 && out[0] < 0x1000;
}

void hv_percpu_diag_tick(struct exc_info *ctx)
{
    int cpu = smp_id();
    if (cpu < 0 || cpu >= MAX_CPUS)
        return;

    struct hv_diag_cpu *d = &hv_diag[cpu];
    d->fiq_count++;
    d->sample_count++;

    /*
     * No later vGIC read/modify/write may erase TWI/TWE. Check
     * sparsely to avoid changing guest timing; panic immediately with the
     * owning CPU and exact HCR if the invariant is ever broken again.
     */
    if ((d->sample_count & 0x3fff) == 0) {
        u64 hcr = mrs(HCR_EL2);
        if (!hv_wfx_policy_satisfied(hcr))
            hv_panic("HV: EL2 WFI/WFE HCR policy lost cpu=%d hcr=0x%lx\n", cpu,
                     hcr);
    }

    u64 x18 = ctx->regs[18];
    if (hv_runtime_diag_verbose_enabled() && (!d->x18_seen || x18 != d->last_x18) &&
        d->x18_reports < HV_DIAG_X18_REPORT_LIMIT) {
        printf("HV DIAG X18: cpu=%d sample=%lu pc=0x%lx x18=0x%lx x0=0x%lx sp0=0x%lx "
               "sp1=0x%lx spsr=0x%lx\n",
               cpu, d->sample_count, ctx->elr, x18, ctx->regs[0], ctx->sp[0], ctx->sp[1],
               ctx->spsr);
        d->x18_reports++;
    }
    d->last_x18 = x18;
    d->x18_seen = true;

    if (hv_runtime_diag_enabled() && !d->online_reported) {
        d->online_reported = true;
        printf("CPU_ENTRY cpu=%d mpidr=0x%lx\n", cpu, mrs(MPIDR_EL1));
        printf("HV DIAG ONLINE: cpu=%d mpidr=0x%lx pc=0x%lx spsr=0x%lx\n", cpu, mrs(MPIDR_EL1),
               ctx->elr, ctx->spsr);
    }

    if (ctx->elr == d->last_pc) {
        d->same_pc_ticks++;
    } else {
        d->last_pc = ctx->elr;
        d->same_pc_ticks = 0;
    }

    // Sparse on purpose: flooding the UART from here would itself change SMP timing.
    if (hv_runtime_diag_verbose_enabled() && (d->sample_count & 0x3fff) == 0)
        printf("HV DIAG: cpu=%d pc=0x%lx same=%lu spsr=0x%lx\n", cpu, ctx->elr,
               d->same_pc_ticks, ctx->spsr);

    if (hv_bugcheck_dumped || (ctx->elr >> 40) != 0xfffff8)
        return;

    //
    // Probe regularly rather than waiting for a long run at one PC. The stop-screen loop
    // alternates between two adjacent instructions, so same_pc_ticks kept resetting and the
    // long-spin trigger never fired even though the guest sat on the blue screen for seconds.
    // The opcode check inside the reader is what rejects false matches, so probing often is
    // safe; it is still only every 64th local tick.
    //
    bool probe = (d->sample_count & 0x3f) == 0;
    if (!probe)
        return;

    u64 a[5], b[5];
    if (!hv_diag_read_bugcheck(d, ctx->elr, a))
        return;
    sysop("dsb sy");
    if (!hv_diag_read_bugcheck(d, ctx->elr, b))
        return;
    for (int i = 0; i < 5; i++)
        if (a[i] != b[i])
            return; // caught mid-write by another core; try again next tick

    hv_bugcheck_dumped = true;
    printf("HV BUGCHECK: seen_by_cpu=%d code=0x%lx P1=0x%lx P2=0x%lx P3=0x%lx P4=0x%lx\n", cpu,
           a[0], a[1], a[2], a[3], a[4]);
    hv_watchdog_snapshot_dump();
    if (a[0] == 0xA)
        printf("HV BUGCHECK 0A: va=0x%lx irql=%lu access=%s fault_pc=0x%lx\n", a[1], a[2],
               (a[3] & 8) ? "EXECUTE" : (a[3] & 1) ? "WRITE" : "READ", a[4]);
}

void hv_arm_tick(bool secondary)
{
    int cpu = smp_id();
    if (cpu >= 0 && cpu < MAX_CPUS)
        HV_RUNTIME_DIAG_COUNT(hv_tick_arm_count[cpu]);
    if (secondary)
        msr(CNTP_TVAL_EL0, hv_secondary_tick_interval);
    else
        msr(CNTP_TVAL_EL0, __atomic_load_n(&hv_tick_interval, __ATOMIC_ACQUIRE));
    msr(CNTP_CTL_EL0, CNTx_CTL_ENABLE);
}

void hv_arm_guest_irq_recovery_tick(void)
{
    /*
     * HCR.VI is synthetic on Apple CPUs and does not itself guarantee a
     * physical wake from every idle state.  Keep one short EL2 wake pending
     * only while a guest timer delivery is outstanding; the normal sparse
     * cadence is restored as soon as Windows EOIs/rearms the timer.
     */
    if (!hv_guest_irq_recovery_tick_interval)
        return;

    int cpu = smp_id();
    if (cpu >= 0 && cpu < MAX_CPUS)
        HV_RUNTIME_DIAG_COUNT(hv_recovery_tick_arm_count[cpu]);
    msr(CNTP_TVAL_EL0, hv_guest_irq_recovery_tick_interval);
    msr(CNTP_CTL_EL0, CNTx_CTL_ENABLE);
}

void hv_mark_guest_runtime_ready(void)
{
    if (__atomic_exchange_n(&hv_guest_runtime_ready, true, __ATOMIC_ACQ_REL))
        return;

    __atomic_store_n(&hv_tick_interval, hv_runtime_tick_interval, __ATOMIC_RELEASE);
    printf("HV: guest runtime ready; boot tick %uHz -> %uHz\n",
           hv_boot_tick_rate(), hv_runtime_tick_rate());
}

void hv_maybe_exit(void)
{
    if (hv_should_exit[smp_id()]) {
        hv_exit_guest();
    }
}

//
// Sample where the guest is, from the hypervisor's own timer, and report a stop.
//
// Everything here obeys one hard rule, learned four times over: the FIQ handler runs on a
// small stack and a tight budget, and *any* extra work inside it kills the entire FIQ path.
// The failure is deceptive - PC samples and the heartbeat vanish together while
// synchronous-exception handling carries on, so it looks like an unrelated breakage.
//
// So the handler only ever copies. Collection is a fixed set of loads into static storage;
// printing is one line per tick, from that storage, long after the fact.
//
// This diagnostic walks mutable guest stacks from the timer FIQ. Keep it opt-in:
// a transient Windows spinlock is not a crash, and an unlucky cross-page diagnostic
// read must never take down the timer/event path used by the guest and framebuffer.
bool hv_pc_sampling = false;

#define STUCK_FRAMES 12
#define STUCK_CODE   12
#define STUCK_LINES  (4 + STUCK_CODE + STUCK_FRAMES + 1 + 1 + 1 + STUCK_CODE)

static struct {
    u64 pc, pa, spsr, sp;
    u64 regs[31];
    u32 code[STUCK_CODE];
    u64 frame_fp[STUCK_FRAMES];
    u64 frame_lr[STUCK_FRAMES];
    u64 frame_code[STUCK_FRAMES];   // the instruction that made the call
    u64 octx[4];        // Fp, Lr, Sp, Pc of the CONTEXT KiDispatchException built
    bool octx_ok;
    u64 far;            // guest FAR_EL1 at collect time; brk does not overwrite it, so
                        // it should still hold the original abort's faulting address
    //
    // Rather than guess where the KTRAP_FRAME is, scan the abort-dispatch stack for the
    // faulting address itself: KTRAP_FRAME.Pc == far for an instruction abort, so every
    // slot holding far is a candidate, and the words on either side are Lr/Fp/Sp. No
    // offset is trusted - the match locates the frame.
    //
    //
    // The abort's faulting instruction, resolved through the KTRAP_FRAME:
    // trap_ptr = [frame_fp[7]+0x28] (KiAbortException's saved x20 = its trap-frame arg),
    // fault_pc = [trap_ptr+0x148], and [trap_ptr+0x90] is the abort ESR (a self-check).
    // Offsets are from KiAbortException's disassembly, not guessed.
    //
    u64 trap_ptr;
    u64 fault_pc;
    u64 tf_esr;
    u64 fpc_pa;
    u32 fpc_code[STUCK_CODE];
    int fpc_read;
    int fpc_state;
    u64 bcd[5];         // KiBugCheckData: code + params 1..4, from x19
    u64 bcd_va;
    bool bcd_ok;
    u64 stack[32];      // filled a few words per tick, see hv_read_stack_slice()
    u64 stack_pa;
    u64 stack_base;
    int stack_read;
    //
    // Two EXCEPTION_RECORDs, one per pass through KiSynchronousException: index 0 is the
    // inner one (the bugcheck path's own debug break), index 1 the outer one - the
    // original abort that started it all. Same geometry both times: the record lives at
    // that pass's frame fp + 0x18.
    //
    u64 rec_base[2];
    u64 rec_pa[2];
    u64 rec[2][8];
    int rec_read[2];
    u64 exc_addr[2];    // ExceptionAddress out of each record
    u64 exc_pa[2];
    u32 exc_code[2][STUCK_CODE];
    int exc_read[2];
    int frames;
    int printed;
    bool valid;
    //
    // Per-CPU: the detector latches one guest stop at a time. With more than one core each
    // would otherwise overwrite the others' capture mid-collection and the report would mix
    // registers from different cores - worse than no report, because it looks plausible.
    //
} stuck_percpu[MAX_CPUS];

#define stuck (stuck_percpu[smp_id()])

static void __attribute__((noinline)) hv_collect_stuck(struct exc_info *ctx)
{
    u64 pa = hv_translate(ctx->elr & ~3UL, false, false, NULL);

    stuck.pc = ctx->elr;
    stuck.pa = pa;
    stuck.spsr = ctx->spsr;
    stuck.sp = ctx->sp[1];

    for (int i = 0; i < 31; i++)
        stuck.regs[i] = ctx->regs[i];

    for (int i = 0; i < STUCK_CODE; i++)
        stuck.code[i] = pa ? read32(pa + (i - 4) * 4) : 0;

    //
    // The kernel image is physically contiguous, so every address inside it is reachable
    // by arithmetic from the stop point. That matters: one hv_translate per frame is
    // affordable here, two was not.
    //
    u64 off = pa ? pa - (ctx->elr & ~3UL) : 0;
    u64 fp = ctx->regs[29];

    //
    // The frames all live on the same kernel stack, so cache the page translation:
    // twelve frames cost one to three hv_translate calls instead of twelve. A record
    // straddling a page boundary (fp ending in 0xff8) skips the cache.
    //
    u64 page_va = 1, page_pa = 0;

    stuck.frames = 0;
    for (int depth = 0; depth < STUCK_FRAMES && fp && off; depth++) {
        u64 va = fp & ~7UL;
        u64 fp_pa;
        if (page_pa && (va & ~0xFFFUL) == page_va && (va & 0xFFF) <= 0xFF0) {
            fp_pa = page_pa + (va & 0xFFF);
        } else {
            fp_pa = hv_translate(va, false, false, NULL);
            if (!fp_pa)
                break;
            page_va = va & ~0xFFFUL;
            page_pa = fp_pa - (va & 0xFFF);
        }

        u64 next_fp = read64(fp_pa);
        u64 lr = read64(fp_pa + 8);

        stuck.frame_fp[depth] = fp;
        stuck.frame_lr[depth] = lr;
        //
        // The instruction two words before the return address: for the frame that called
        // KeBugCheckEx that is the `mov w0, #<code>` setting the bugcheck code.
        //
        // Only if lr is plausibly inside the same physically-contiguous image as the
        // stop pc: the pa here comes from arithmetic, not from a translation, and an
        // lr from another module (or a garbage lr slot) sends the read into arbitrary
        // physical space - which faults at EL2 and takes the whole hypervisor down.
        // That is precisely how the vUART kept dying seconds after the guest stopped.
        //
        stuck.frame_code[depth] =
            (lr - stuck.pc + 0x800000UL) < 0x1000000UL ? read32(lr + off - 8) : 0;
        stuck.frames = depth + 1;

        if (next_fp <= fp)
            break;
        fp = next_fp;
    }

    //
    // Window the stack on frame 1 rather than on the stop's own SP.
    //
    // Frame 1 is the function that called KeBugCheckEx - the disassembly showed it loading
    // the parameters and setting w0 to 0x1e - so its frame is where those arguments live.
    // The window from SP showed only the breakpoint spin's own frame. Same cost: one
    // translation either way, and the reading is still spread over later ticks.
    //
    stuck.stack_base = stuck.frames > 1 ? stuck.frame_fp[1] - 0x40 : ctx->sp[1];
    stuck.stack_pa = hv_translate(stuck.stack_base & ~7UL, false, false, NULL);
    stuck.stack_read = 0;
    //
    // Frame 3's fp pair is the one KiSynchronousException saved (the lr chain runs one
    // deeper than the fp chain), and its prologue is `stp x29,x30,[sp,#-0xb0]!; mov
    // x29,sp` with the original EXCEPTION_RECORD built at sp+0x18 (`add x0,sp,#0x18`
    // just before `bl KiDispatchException`). Confirmed against the dump: the pointer
    // spilled at window+0x40 equals fp3+0x18.
    //
    stuck.rec_base[0] = stuck.frames > 3 ? stuck.frame_fp[3] + 0x18 : 0;
    stuck.rec_base[1] = stuck.frames > 9 ? stuck.frame_fp[9] + 0x18 : 0;
    for (int r = 0; r < 2; r++) {
        stuck.rec_pa[r] = 0;
        stuck.rec_read[r] = 0;
        stuck.exc_addr[r] = 0;
        stuck.exc_pa[r] = 0;
        stuck.exc_read[r] = 0;
    }
    //
    // Read the CONTEXT words (window+0x140..0x158: X29, X30, Sp, Pc at the moment of
    // the inner exception) here rather than on a later tick: four reads on a page
    // already translated. They have to be in the very first printed line - the vUART
    // link now dies within a tick or two of the guest stopping, so anything essential
    // must go out immediately.
    //
    stuck.octx_ok = stuck.stack_pa && ((stuck.stack_base & 0xFFF) + 0x158) < 0x1000;
    for (int i = 0; i < 4; i++)
        stuck.octx[i] = stuck.octx_ok ? read64(stuck.stack_pa + 0x140 + i * 8) : 0;

    stuck.far = mrs(FAR_EL12);

    //
    // KiBugCheckData: x19 held its address at the KeBugCheckEx call (x19 = nt+0xdbb9a0,
    // confirmed against symbols). Five qwords: BugCheckCode then Parameters 1..4. For
    // bugcheck 0xA that is (referenced address, IRQL, access type, faulting PC) - which
    // settles the 0xA-vs-0xC8 question and says whether the fault happened at raised
    // IRQL. One translation, five reads, done once here.
    //
    stuck.bcd_va = stuck.regs[19] & ~7UL;
    u64 bcd_pa = hv_translate(stuck.bcd_va, false, false, NULL);
    //
    // Once KiDisplayBlueScreen is running, x19 no longer points at KiBugCheckData. This
    // Windows 26100.8037 build spins in HalpGitQueryCounter at nt+0x213a98 while drawing
    // the blue screen; use that uniquely identified PC to recover nt base and the public
    // KiBugCheckData symbol (nt+0xdbb9a0). The image/PDB-specific offsets are already the
    // basis of the stack decoder above, and the opcode check prevents a false match.
    //
    if ((!bcd_pa || !read64(bcd_pa) || read64(bcd_pa) >= 0x1000) &&
        stuck.code[4] == 0xd53be040) {
        stuck.bcd_va = stuck.pc - 0x213a98UL + 0xdbb9a0UL;
        bcd_pa = hv_translate(stuck.bcd_va, false, false, NULL);
    }
    stuck.bcd_ok = bcd_pa != 0;
    for (int i = 0; i < 5; i++)
        stuck.bcd[i] = bcd_pa ? read64(bcd_pa + i * 8) : 0;

    stuck.trap_ptr = 0;
    stuck.fault_pc = 0;
    stuck.tf_esr = 0;
    stuck.fpc_pa = 0;
    stuck.fpc_read = 0;
    stuck.fpc_state = stuck.frames > 7 ? 0 : 9;

    stuck.printed = 0;
    stuck.valid = true;
}

//
// Four words per tick. The measured budget of the FIQ handler is about seven address
// translations and thirty reads; anything beyond that kills the whole FIQ path, silently
// and in a way that looks like an unrelated failure.
//
static void hv_read_stack_slice(void)
{
    if (!stuck.valid || !stuck.stack_pa)
        return;

    if (stuck.stack_read < 32) {
        for (int i = 0; i < 4; i++, stuck.stack_read++)
            stuck.stack[stuck.stack_read] = read64(stuck.stack_pa + stuck.stack_read * 8);
        return;
    }

    //
    // The stack words are in; next comes the original EXCEPTION_RECORD. One translation
    // on its own tick, then four words per tick - the same pacing the stack reads
    // survive. Everything here changes per boot with the kernel base, so it all has to
    // come from the walked frames, not from constants.
    //
    //
    // Each record, then the code around its ExceptionAddress (rec[2]). One phase per
    // tick, one translation per phase at most.
    //
    for (int r = 0; r < 2; r++) {
        if (stuck.rec_base[r] && !stuck.rec_pa[r]) {
            stuck.rec_pa[r] = hv_translate(stuck.rec_base[r] & ~7UL, false, false, NULL);
            if (!stuck.rec_pa[r])
                stuck.rec_base[r] = 0;  // translation failed; skip record and code dump
            return;
        }

        if (stuck.rec_pa[r] && stuck.rec_read[r] < 8) {
            for (int i = 0; i < 4; i++, stuck.rec_read[r]++)
                stuck.rec[r][stuck.rec_read[r]] =
                    read64(stuck.rec_pa[r] + stuck.rec_read[r] * 8);
            return;
        }

        if (!stuck.exc_addr[r]) {
            u64 addr = stuck.rec_base[r] ? stuck.rec[r][2] : 0;
            if ((addr >> 40) != 0xfffff8)   // not a kernel VA; leave exc_pa 0, give up
                addr = ~0UL;
            else
                stuck.exc_pa[r] = hv_translate(addr & ~3UL, false, false, NULL);
            stuck.exc_addr[r] = addr;
            return;
        }

        if (stuck.exc_pa[r] && stuck.exc_read[r] < STUCK_CODE) {
            for (int i = 0; i < 4; i++, stuck.exc_read[r]++)
                stuck.exc_code[r][stuck.exc_read[r]] =
                    read32(stuck.exc_pa[r] + (stuck.exc_read[r] - 4) * 4);
            return;
        }
    }

    //
    // Resolve the faulting instruction through the KTRAP_FRAME, one deref per tick.
    //
    if (stuck.fpc_state == 0) {   // trap_ptr = [frame_fp[7]+0x28]
        u64 pa = hv_translate((stuck.frame_fp[7] + 0x28) & ~7UL, false, false, NULL);
        stuck.trap_ptr = pa ? read64(pa) : 0;
        stuck.fpc_state = ((stuck.trap_ptr >> 40) == 0xfffff8) ? 1 : 9;
        return;
    }

    if (stuck.fpc_state == 1) {   // esr self-check and fault_pc
        u64 ea = hv_translate((stuck.trap_ptr + 0x90) & ~7UL, false, false, NULL);
        stuck.tf_esr = ea ? read64(ea + ((stuck.trap_ptr + 0x90) & 7)) : 0;
        u64 pa = hv_translate((stuck.trap_ptr + 0x148) & ~7UL, false, false, NULL);
        stuck.fault_pc = pa ? read64(pa + ((stuck.trap_ptr + 0x148) & 7)) : 0;
        stuck.fpc_state = ((stuck.fault_pc >> 40) == 0xfffff8) ? 2 : 9;
        return;
    }

    if (stuck.fpc_state == 2) {   // translate fault_pc
        stuck.fpc_pa = hv_translate(stuck.fault_pc & ~3UL, false, false, NULL);
        stuck.fpc_state = stuck.fpc_pa ? 3 : 9;
        return;
    }

    if (stuck.fpc_state == 3 && stuck.fpc_read < STUCK_CODE) {   // code around fault_pc
        for (int i = 0; i < 4; i++, stuck.fpc_read++)
            stuck.fpc_code[stuck.fpc_read] = read32(stuck.fpc_pa + (stuck.fpc_read - 4) * 4);
        if (stuck.fpc_read >= STUCK_CODE)
            stuck.fpc_state = 9;
        return;
    }
}

static void hv_print_stuck_line(void)
{
    int n = stuck.printed;

    if (!stuck.valid || n >= STUCK_LINES)
        return;

    stuck.printed++;

    if (n == 0)
        //
        // Everything essential in one line, printed on the collect tick itself: the
        // vUART link tends to die within a tick or two of the guest stopping. code= is
        // the `mov w0, #<bugcheck>` of the frame that called KeBugCheckEx; olr/opc are
        // Lr and Pc at the moment of the inner exception.
        //
        printf("HV STUCK: pc=0x%lx code=%08x f1lr=0x%lx olr=0x%lx opc=0x%lx far=0x%lx\n",
               stuck.pc, (u32)stuck.frame_code[1], stuck.frame_lr[1], stuck.octx[1],
               stuck.octx[3], stuck.far);
    else if (n == 1) {
        //
        // KiBugCheckData (code + params 1..4) - but x19 only points at it in the frame
        // that called KeBugCheckEx. Anywhere else (e.g. an idle or spinlock stop) x19 is
        // an unrelated register, so bcd[] is garbage. Only print it when x19 is a kernel
        // VA and bcd[0] is a plausible bugcheck code (small, non-zero); else say so.
        //
        if (stuck.bcd_ok && stuck.bcd[0] && stuck.bcd[0] < 0x1000)
            printf("HV STUCK: bugcheck=%lx P1=0x%lx P2=0x%lx P3=0x%lx P4=0x%lx\n", stuck.bcd[0],
                   stuck.bcd[1], stuck.bcd[2], stuck.bcd[3], stuck.bcd[4]);
        else
            printf("HV STUCK: bugcheck n/a (candidate=0x%lx x19=0x%lx)\n", stuck.bcd_va,
                   stuck.regs[19]);
    }
    else if (n <= 4) {
        int g = (n - 2) * 8;    // n = 2,3,4 -> x0-x7, x8-x15, x16-x23
        printf("HV STUCK: x%d-x%d = %016lx %016lx %016lx %016lx\n", g, g + 3,
               stuck.regs[g], stuck.regs[g + 1], stuck.regs[g + 2], stuck.regs[g + 3]);
    } else if (n < 5 + STUCK_CODE) {
        int i = n - 5;
        printf("HV STUCK:   %c0x%lx: %08x\n", i == 4 ? '*' : ' ', stuck.pc + (i - 4) * 4,
               stuck.code[i]);
    } else if (n < 5 + STUCK_CODE + STUCK_FRAMES) {
        int i = n - 5 - STUCK_CODE;
        if (i < stuck.frames)
            printf("HV STUCK: frame %d: lr=0x%lx setup=%08lx\n", i, stuck.frame_lr[i],
                   stuck.frame_code[i]);
    } else if (n < 5 + STUCK_CODE + STUCK_FRAMES + 1) {
        if (stuck.octx_ok)
            printf("HV STUCK: orig fp=0x%lx lr=0x%lx sp=0x%lx pc=0x%lx\n", stuck.octx[0],
                   stuck.octx[1], stuck.octx[2], stuck.octx[3]);
    } else {
        //
        // The faulting-instruction block. Wait here until the KTRAP_FRAME walk and the
        // code reads have finished (they run one step a tick). Rewind printed so this
        // same line index is retried next tick.
        //
        if (stuck.fpc_state != 9) {
            stuck.printed--;
            return;
        }
        int i = n - (5 + STUCK_CODE + STUCK_FRAMES + 1);
        if (i == 0)
            printf("HV STUCK: trap=0x%lx fault_pc=0x%lx tf_esr=0x%lx far=0x%lx\n",
                   stuck.trap_ptr, stuck.fault_pc, stuck.tf_esr, stuck.far);
        else {
            int j = i - 1;
            if (stuck.fpc_pa && j < stuck.fpc_read)
                printf("HV STUCK:   %c0x%lx: %08x\n", j == 4 ? '*' : ' ',
                       stuck.fault_pc + (j - 4) * 4, stuck.fpc_code[j]);
        }
    }
}

static void hv_sample_pc(struct exc_info *ctx)
{
    static u64 counter = 0;
    static u64 last_pc = 0;
    static u32 repeats = 0;

    if (!hv_pc_sampling)
        return;

    if (++counter % (HV_TICK_RATE / 2))
        return;

    //
    // Once a report is latched, keep draining it out on every tick regardless of where
    // the guest's PC wanders. A guest that is *looping* (a busy-wait) rather than spinning
    // on one instruction otherwise re-triggered collection endlessly and never finished
    // printing the frames - which is exactly where the bugcheck code lives.
    //
    if (stuck.valid) {
        hv_read_stack_slice();
        hv_print_stuck_line();
        //
        // Once the whole report is drained, release the latch and resume normal
        // sampling. A genuinely wedged guest re-collects the same report next tick
        // (confirming the hang); a guest that only spun briefly with interrupts masked
        // (a spinlock) is seen to move on. Without this, the latch blinds us to a guest
        // that is actually still making progress.
        //
        if (stuck.printed >= STUCK_LINES) {
            stuck.valid = false;
            last_pc = 0;
            repeats = 0;
        }
        return;
    }

    if (ctx->elr == last_pc) {
        //
        // The vUART link dies seconds after the Windows kernel stops, so for kernel
        // addresses the report cannot afford the usual four-second confirmation: two
        // repeats (one second) and collect. Firmware-time waits keep the longer fuse -
        // they resolve on their own and would otherwise spam false reports.
        //
        //
        // Only a PC that repeats *with interrupts masked* (SPSR.I = bit 7) is a real
        // stop - a bugcheck/halt spin runs with DAIF set. A microsecond delay/poll loop
        // (KeStallExecutionProcessor, UEFI waits) keeps interrupts enabled and must not
        // be mistaken for a hang; that false positive spammed the log before.
        //
        u32 need = ((ctx->elr >> 40) == 0xfffff8) ? 2 : 8;
        if (!repeats && (ctx->elr >> 40) == 0xfffff8) {
            u64 held_pa = hv_translate(ctx->elr & ~3UL, false, false, NULL);
            printf("HV SAMPLE: held pc=0x%lx insn=%08x\n", ctx->elr,
                   held_pa ? read32(held_pa) : 0);
        }
        //
        // Second path, for the blue screen. Once KeBugCheckEx has run, Windows draws the
        // stop screen spinning in HalpGitQueryCounter with interrupts ENABLED, so the
        // masked-interrupt gate above never fires and the bugcheck parameters are never
        // read - which is exactly what happened chasing IRQL_NOT_LESS_OR_EQUAL on a second
        // core. A kernel PC that repeats for a very long time is worth capturing even with
        // interrupts enabled: the collector below can recover KiBugCheckData from that PC.
        // The fuse is long (a few seconds) so ordinary delay/poll loops still do not trip it.
        //
        bool blue_screen_spin = repeats >= need * 32 && (ctx->elr >> 40) == 0xfffff8;
        if (++repeats >= need && ((ctx->spsr & BIT(7)) || blue_screen_spin)) {
            //
            // Skip the CPU idle loop. A repeating PC with interrupts masked, sitting one
            // instruction past a WFI, is HalProcessorIdle (dsb; isb; wfi; ...), not a
            // hang - the core is asleep with nothing to run and wakes each timer tick.
            //
            u64 pa = hv_translate((ctx->elr - 4) & ~3UL, false, false, NULL);
            if (!pa || read32(pa) != 0xd503207f /* wfi */)
                hv_collect_stuck(ctx);
        }
    } else {
        if (repeats)
            printf("HV SAMPLE: pc=0x%lx held for %u samples\n", last_pc, repeats + 1);
        printf("HV SAMPLE: pc=0x%lx spsr=0x%lx sp=0x%lx\n", ctx->elr, ctx->spsr, ctx->sp[1]);
        last_pc = ctx->elr;
        repeats = 0;
    }
}

void hv_tick(struct exc_info *ctx)
{
    hv_wdt_pet();
    struct hv_diag_sample_v1 diag_sample;
    if (hv_diag_tick(ctx, &diag_sample) && hv_runtime_diag_enabled())
        hv_diag_delivery_offer(&hv_diag_event_delivery, &diag_sample);
    if (hv_runtime_diag_enabled())
        hv_diag_delivery_flush(&hv_diag_event_delivery, hv_send_diag_sample, NULL);
    hv_sample_pc(ctx);
    iodev_handle_events(uartproxy_iodev);
    if (iodev_can_read(uartproxy_iodev)) {
        /*
         * Publish diagnostics before entering uartproxy.  A wedged secondary
         * may make the later time-stealing rendezvous fail, but these records
         * are lockless and remain readable from the interruptible CPU.
         */
        printf("HV: User interrupt; pre-rendezvous watchdog snapshot\n");
        hv_watchdog_snapshot_dump();
        iodev_console_flush();
        if (hv_pinned_cpu == -1 || hv_pinned_cpu == smp_id())
            hv_exc_proxy(ctx, START_HV, HV_USER_INTERRUPT, NULL);
    }
    hv_vuart_poll();
    hv_fb_stream_tick(&hv_framebuffer_stream);
#ifdef M1N1_J316S_SETUP
    hv_j316s_wifi_poll();
    hv_j316s_setup_draw(ctx->elr);
#endif
}

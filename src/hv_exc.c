/* SPDX-License-Identifier: MIT */

#include "hv.h"
#include "hv_diag.h"
#include "hv_exception_lower.h"
#include "hv_fiq_fast_path.h"
#include "hv_agx_power_broker.h"
#include "hv_guest_ipa_pa.h"
#include "hv_irq_routes.h"
#include "hv_runtime_diag.h"
#include "hv_sgi_diag.h"
#include "hv_sgi_pending.h"
#include "hv_watchdog_snapshot.h"
#include "hv_wfx_policy.h"
#include "assert.h"
#include "cpu_regs.h"
#include "exception.h"
#include "iodev.h"
#include "smp.h"
#include "string.h"
#include "uart.h"
#include "uartproxy.h"
#include "xnuboot.h"
#include "hv_vgic.h"
#include "hv_vgic_diag.h"
#include "aic.h"
#include "aic_regs.h"
#include "adt.h"

#define TIME_ACCOUNTING
//
// m1n1_windows change: when the vGIC is running in the guest - timer interrupts by virtue of coming from the generic timer are
// still going to come as FIQs to EL2 - we'll need to divert those to the guest as *IRQs* (to prevent Windows from crashing as it treats FIQs as
// errors).
//
extern bool vgic_inited;
extern spinlock_t bhl;

#define _SYSREG_ISS(_1, _2, op0, op1, CRn, CRm, op2)                                               \
    (((op0) << ESR_ISS_MSR_OP0_SHIFT) | ((op1) << ESR_ISS_MSR_OP1_SHIFT) |                         \
     ((CRn) << ESR_ISS_MSR_CRn_SHIFT) | ((CRm) << ESR_ISS_MSR_CRm_SHIFT) |                         \
     ((op2) << ESR_ISS_MSR_OP2_SHIFT))
#define SYSREG_ISS(...) _SYSREG_ISS(__VA_ARGS__)

//
// Note: trapped ID registers cannot be served through a helper macro. A register name
// expands to seven comma-separated tokens, and a macro parameter is expanded before
// substitution, so mrs() would receive seven arguments instead of one. Each case in
// hv_handle_msr_unlocked() therefore names its register literally.
//

#define PERCPU(x) pcpu[mrs(TPIDR_EL2)].x
#define PERCPU_N(x, y) pcpu[x].y

struct hv_pcpu_data {
    u32 ipi_queued;
    u32 ipi_pending;
    u32 pmc_pending;
    u64 pmc_irq_mode;
    u64 exc_entry_pmcr0_cnt;
    u64 watchdog_sample_ticks;
    u64 host_tick_fires;
    struct hv_watchdog_cpu_record watchdog_record;
#ifdef ENABLE_VGIC_MODULE
    virq_queue_t irq_queue;
    virq_queue_t timer_queue;
    u32 sgi_pending_mask;
    struct hv_sgi_diag_state sgi_diag;
    u32 last_sgi_from;
    u32 last_sgi_intid;
    u32 last_iar_intid;
    u32 last_eoi_intid;
    u64 last_iar_tick;
    u64 last_eoi_tick;
#endif
} ALIGNED(64);

struct hv_pcpu_data pcpu[MAX_CPUS];

void hv_exit_guest(void) __attribute__((noreturn));

static u64 stolen_time = 0;
static u64 exc_entry_time;

static u64 hv_guest_ipa_pa_translate(u64 ipa, void *opaque)
{
    UNUSED(opaque);
    return hv_ipa_to_pa(ipa);
}

static bool hv_guest_ipa_pa_is_ram(u64 pa, u64 size, void *opaque)
{
    u64 base = cur_boot_args.phys_base;
    u64 length = cur_boot_args.mem_size;

    UNUSED(opaque);
    return size != 0 && pa >= base && size <= length && pa - base <= length - size;
}

static bool hv_handle_guest_ipa_pa(struct exc_info *ctx)
{
    u32 status;

    if (hv_guest_arm_consumed_handle(FIELD_GET(ESR_ISS, ctx->esr),
                                     ctx->regs[0], &status)) {
        if (status) {
            printf("HV: GPUVA ARM_CONSUMED version=%u seq=%u\n",
                   HV_GPUVA_ARM_CONSUMED_VERSION, status);
            ctx->regs[0] = HV_GUEST_IPA_PA_STATUS_SUCCESS;
        } else {
            ctx->regs[0] = HV_GUEST_IPA_PA_STATUS_INVALID_REQUEST;
        }
        return true;
    }

    if (!hv_guest_ipa_pa_handle(FIELD_GET(ESR_ISS, ctx->esr), ctx->regs[0],
                                hv_guest_ipa_pa_translate, hv_guest_ipa_pa_is_ram, NULL, &status))
        return false;
    ctx->regs[0] = status;
    return true;
}
static int num_cpus;

extern u64 hv_cpus_in_guest;
extern bool hv_rendezvous_active;
extern int hv_pinned_cpu;
extern int hv_want_cpu;

static bool time_stealing = true;

void init_vgic_irq_queues(void) {
    for (int i = 0; i < MAX_CPUS; i++) {
        PERCPU_N(i, watchdog_sample_ticks) = 0;
        PERCPU_N(i, host_tick_fires) = 0;
        PERCPU_N(i, watchdog_record) = (struct hv_watchdog_cpu_record){0};
    }

#ifdef ENABLE_VGIC_MODULE
    int node = adt_path_offset(adt, "/cpus");
    num_cpus = adt_get_child_count(adt, node);
    for (int i = 0; i < MAX_CPUS; i++) {
        virq_queue_init(&PERCPU_N(i, irq_queue));
        virq_queue_init(&PERCPU_N(i, timer_queue));
        __atomic_store_n(&PERCPU_N(i, sgi_pending_mask), 0, __ATOMIC_RELAXED);
        PERCPU_N(i, sgi_diag) = (struct hv_sgi_diag_state){0};
        PERCPU_N(i, last_sgi_from) = ~0U;
        PERCPU_N(i, last_sgi_intid) = ~0U;
        PERCPU_N(i, last_iar_intid) = ~0U;
        PERCPU_N(i, last_eoi_intid) = ~0U;
        PERCPU_N(i, last_iar_tick) = 0;
        PERCPU_N(i, last_eoi_tick) = 0;
    }
#endif
}

#ifdef ENABLE_VGIC_MODULE
/*
 * GICv3 SGIs are pending bits, not FIFO messages.  A second send while the same SGI is
 * pending coalesces; a send while it is active changes the LR to active+pending.  Modelling
 * SGIs with a bounded FIFO created duplicate LRs for one INTID and could silently drop the
 * IPI that Windows was waiting for when the FIFO filled under SMP load.
 */
static void hv_vgic3_queue_sgi(int cpu, u32 intid)
{
    u32 bit = BIT(intid);
    u32 old = __atomic_fetch_or(&PERCPU_N(cpu, sgi_pending_mask), bit, __ATOMIC_RELEASE);
    __atomic_store_n(&PERCPU_N(cpu, last_sgi_from), smp_id(), __ATOMIC_RELAXED);
    __atomic_store_n(&PERCPU_N(cpu, last_sgi_intid), intid, __ATOMIC_RELAXED);
    hv_sgi_diag_note(&PERCPU_N(cpu, sgi_diag), HV_SGI_DIAG_QUEUE);
    static u32 trace_budget = 32;

    if (hv_runtime_trace_enabled() && trace_budget) {
        trace_budget--;
        printf("HV SGI QUEUE: from=%d to=%d intid=%u old=0x%x%s\n", smp_id(), cpu,
               intid, old, old & bit ? " coalesced" : "");
    }
    smp_send_ipi(cpu);
}

static bool hv_vgic3_repend_live_sgi(u32 intid)
{
    u64 lrs[HV_VGIC_DIAG_LR_COUNT] = {0};
    int lr_count = hv_vgic3_num_lrs();
    if (lr_count > HV_VGIC_DIAG_LR_COUNT)
        lr_count = HV_VGIC_DIAG_LR_COUNT;
    for (int lr = 0; lr < lr_count; lr++)
        lrs[lr] = hv_vgic3_read_lr(lr);

    int lr = hv_vgic_diag_find_live_intid(lrs, intid);
    if (lr < 0)
        return false;

    if (!(lrs[lr] & ICH_LR_STATE_PENDING)) {
        hv_vgic3_write_lr(lr, lrs[lr] | ICH_LR_STATE_PENDING);
        hv_vgic3_update_vi();
        sysop("isb");
    }
    return true;
}

static void hv_vgic3_drain_sgi_mask(u32 pending)
{
    if (pending)
        hv_sgi_diag_note(&PERCPU(sgi_diag), HV_SGI_DIAG_DRAIN);

    while (pending) {
        u32 intid = __builtin_ctz(pending);
        u32 bit = BIT(intid);
        pending &= ~bit;

        if (hv_vgic3_repend_live_sgi(intid)) {
            hv_sgi_diag_note(&PERCPU(sgi_diag), HV_SGI_DIAG_REPEND);
            continue;
        }
        if (hv_vgic3_get_free_lr() != -1) {
            hv_vgic3_inject_irq(intid, hv_vgic3_get_priority(intid), false, true, false, 0);
            hv_sgi_diag_note(&PERCPU(sgi_diag), HV_SGI_DIAG_INJECT);
            continue;
        }

        /* A maintenance interrupt will retry after an LR becomes free. */
        __atomic_fetch_or(&PERCPU(sgi_pending_mask), pending | bit, __ATOMIC_RELEASE);
        hv_sgi_diag_note(&PERCPU(sgi_diag), HV_SGI_DIAG_NO_LR);
        break;
    }
}

static void hv_vgic3_drain_sgis(void)
{
    u32 pending = __atomic_exchange_n(&PERCPU(sgi_pending_mask), 0, __ATOMIC_ACQ_REL);
    hv_vgic3_drain_sgi_mask(pending);
}

static void hv_ack_fast_ipi(void *opaque)
{
    (void)opaque;
    msr(SYS_IMP_APL_IPI_SR_EL1, IPI_SR_PENDING);
    sysop("isb");
}

/*
 * Drain only sources whose state is private to the current CPU.  This is safe
 * before the global hypervisor lock: the vGIC LRs, SGI mask and legacy proxy
 * IPI flags below are all per-CPU.  Keeping this small is intentional; device
 * IRQ routing and proxy work remain on the serialized slow path.
 */
static void hv_handle_local_ipi(void)
{
    if (!(mrs(SYS_IMP_APL_IPI_SR_EL1) & IPI_SR_PENDING))
        return;

#ifdef ENABLE_VGIC_MODULE
    hv_sgi_diag_note(&PERCPU(sgi_diag), HV_SGI_DIAG_IPI_RX);
    u32 pending_sgis = hv_sgi_ack_and_take_pending(&PERCPU(sgi_pending_mask),
                                                   hv_ack_fast_ipi, NULL);
    hv_vgic3_drain_sgi_mask(pending_sgis);
#else
    hv_ack_fast_ipi(NULL);
#endif
    if (PERCPU(ipi_queued)) {
        PERCPU(ipi_pending) = true;
        PERCPU(ipi_queued) = false;
    }
}

void hv_sgi_diag_vgic_event(enum hv_sgi_diag_event event)
{
    hv_sgi_diag_note(&PERCPU(sgi_diag), event);
}

void hv_irq_diag_vgic_iar(u32 intid)
{
    PERCPU(last_iar_intid) = intid;
    PERCPU(last_iar_tick) = mrs(CNTPCT_EL0);
}

void hv_irq_diag_vgic_eoi(u32 intid)
{
    PERCPU(last_eoi_intid) = intid;
    PERCPU(last_eoi_tick) = mrs(CNTPCT_EL0);
}

void hv_vgic3_drain_irq_queue(void)
{
    while (hv_vgic3_get_free_lr() != -1) {
        virq_t pending;
        if (!virq_queue_pop(&PERCPU(irq_queue), &pending))
            break;
        hv_vgic3_inject_irq(pending.vintid, pending.priority, pending.active,
                            pending.pending, pending.hw_status, pending.hw_irq);
    }
}
#endif

static void _hv_exc_proxy(struct exc_info *ctx, uartproxy_boot_reason_t reason, u32 type,
                          void *extra)
{
    int from_el = FIELD_GET(SPSR_M, ctx->spsr) >> 2;

    hv_wdt_breadcrumb('P');

    /*
     * Get all the CPUs into the HV before running the proxy, to make sure they all exit to
     * the guest with a consistent time offset.
     */
    if (time_stealing)
        hv_rendezvous();

    u64 entry_time = mrs(CNTPCT_EL0);

    ctx->elr_phys = hv_translate(ctx->elr, false, false, NULL);
    ctx->far_phys = hv_translate(ctx->far, false, false, NULL);
    ctx->sp_phys = hv_translate(from_el == 0 ? ctx->sp[0] : ctx->sp[1], false, false, NULL);
    ctx->extra = extra;

    struct uartproxy_msg_start start = {
        .reason = reason,
        .code = type,
        .info = ctx,
    };

    hv_wdt_suspend();
    int ret = uartproxy_run(&start);
    hv_wdt_resume();

    switch (ret) {
        case EXC_RET_HANDLED:
            hv_wdt_breadcrumb('p');
            if (time_stealing) {
                u64 lost = mrs(CNTPCT_EL0) - entry_time;
                stolen_time += lost;
            }
            break;
        case EXC_EXIT_GUEST:
            hv_rendezvous();
            spin_unlock(&bhl);
            hv_exit_guest(); // does not return
        default:
            printf("Guest exception not handled, rebooting.\n");
            print_regs(ctx->regs, 0);
            flush_and_reboot(); // does not return
    }
}

static void hv_maybe_switch_cpu(struct exc_info *ctx, uartproxy_boot_reason_t reason, u32 type,
                                void *extra)
{
    while (hv_want_cpu != -1) {
        if (hv_want_cpu == smp_id()) {
            hv_want_cpu = -1;
            _hv_exc_proxy(ctx, reason, type, extra);
        } else {
            // Unlock the HV so the target CPU can get into the proxy
            spin_unlock(&bhl);
            while (hv_want_cpu != -1)
                sysop("dmb sy");
            spin_lock(&bhl);
        }
    }
}

void hv_exc_proxy(struct exc_info *ctx, uartproxy_boot_reason_t reason, u32 type, void *extra)
{
    /*
     * Wait while another CPU is pinned or being switched to.
     * If a CPU switch is requested, handle it before actually handling the
     * exception. We still tell the host the real reason code, though.
     */
    while ((hv_pinned_cpu != -1 && hv_pinned_cpu != smp_id()) || hv_want_cpu != -1) {
        if (hv_want_cpu == smp_id()) {
            hv_want_cpu = -1;
            _hv_exc_proxy(ctx, reason, type, extra);
        } else {
            // Unlock the HV so the target CPU can get into the proxy
            spin_unlock(&bhl);
            while ((hv_pinned_cpu != -1 && hv_pinned_cpu != smp_id()) || hv_want_cpu != -1)
                sysop("dmb sy");
            spin_lock(&bhl);
        }
    }

    /* Handle the actual exception */
    _hv_exc_proxy(ctx, reason, type, extra);

    /*
     * If as part of handling this exception we want to switch CPUs, handle it without returning
     * to the guest.
     */
    hv_maybe_switch_cpu(ctx, reason, type, extra);
}

void hv_set_time_stealing(bool enabled, bool reset)
{
    time_stealing = enabled;
    if (reset)
        stolen_time = 0;
}

void hv_add_time(s64 time)
{
    stolen_time -= (u64)time;
}

//
// Temporary instrumentation for chasing timer delivery. Rate-limited so it cannot flood
// the log: we only need to know whether these paths are reached at all.
//
static int dbg_fiq_calls = 0;
static int dbg_timer_p = 0;
static int dbg_timer_v = 0;

//
// One timer injection per expiry, per CPU.
//
// hv_update_fiq() runs on every exception entry/exit, not just on timer FIQs. Until the
// guest reprograms its comparator, CNTx_CTL_EL02.ISTATUS stays set -- so without a latch
// every trapped access (and with ICH_HCR_EL2.TC|TALL1 that includes every ICC_IAR1 /
// ICC_EOIR1 / ICC_PMR touch the guest makes while servicing the very same interrupt)
// injected another timer interrupt.
//
// The guest counts interrupts as elapsed time, so its clock ran ~6x fast: it reported
// 00:09:02 for a 90-second run. UEFI arms a five-minute boot watchdog, which then fired
// after ~50 real seconds and called gRT->ResetSystem -> PSCI SYSTEM_RESET (0x84000009).
// That was the mysterious "reset with no guest exception".
//
u64 hv_fiq_count = 0;
u64 hv_fiq_ticks = 0;
static u64 watchdog_last_periodic_dump;

static bool timer_p_injected[MAX_CPUS];
static bool timer_v_injected[MAX_CPUS];

static bool hv_guest_timer_recovery_needed(int cpu)
{
    if (cpu < 0 || cpu >= MAX_CPUS)
        return false;
    if (!(mrs(HCR_EL2) & HCR_VI))
        return false;

    u64 vmcr = mrs(ICH_VMCR_EL2);
    u32 pmr = (vmcr >> 24) & 0xff;
    u32 running_priority = hv_vgic3_running_priority();
    int lr_count = hv_vgic3_num_lrs();

    for (int lr = 0; lr < lr_count; lr++) {
        u64 value = hv_vgic3_read_lr(lr);
        if (hv_vgic_diag_lr_needs_recovery_wake(value, 17, pmr,
                                                running_priority) ||
            hv_vgic_diag_lr_needs_recovery_wake(value, 18, pmr,
                                                running_priority))
            return true;
    }
    return false;
}

void hv_watchdog_snapshot_tick(struct exc_info *ctx)
{
    (void)ctx;

    /* RELEASE must not pay for monitor-only register/LR sampling on every
     * guest tick.  This check precedes even the per-CPU sample counter so the
     * production path compiles down to a constant false branch and return. */
    if (!hv_runtime_diag_enabled())
        return;

    int cpu = smp_id();
    if (cpu < 0 || cpu >= MAX_CPUS)
        return;

    u64 sample_tick = ++PERCPU(watchdog_sample_ticks);
    if (!hv_watchdog_snapshot_due(sample_tick))
        return;

    struct hv_watchdog_cpu_sample sample = {
        .cpu = cpu,
        .pc = hv_get_elr(),
        .spsr = hv_get_spsr(),
        .cntpct = mrs(CNTPCT_EL0),
        .cntvct = mrs(CNTVCT_EL0),
        .cntvoff = mrs(CNTVOFF_EL2),
        .host_cntp_ctl = mrs(CNTP_CTL_EL0),
        .host_cntp_cval = mrs(CNTP_CVAL_EL0),
        .tick_arm_count = __atomic_load_n(&hv_tick_arm_count[cpu],
                                          __ATOMIC_RELAXED),
        .recovery_tick_arm_count =
            __atomic_load_n(&hv_recovery_tick_arm_count[cpu],
                            __ATOMIC_RELAXED),
        .host_tick_fires = PERCPU(host_tick_fires),
        .cntp_ctl = mrs(CNTP_CTL_EL02),
        .cntp_cval = mrs(CNTP_CVAL_EL02),
        .cntv_ctl = mrs(CNTV_CTL_EL02),
        .cntv_cval = mrs(CNTV_CVAL_EL02),
        .vm_tmr_fiq_ena = mrs(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2),
        .hcr = mrs(HCR_EL2),
        .ich_hcr = mrs(ICH_HCR_EL2),
        .ich_vmcr = mrs(ICH_VMCR_EL2),
        .isr = mrs(ISR_EL1),
        .timer_p_injected = timer_p_injected[cpu],
        .timer_v_injected = timer_v_injected[cpu],
        .last_el2_marker = hv_wdt_get_breadcrumb(cpu),
    };

#ifdef ENABLE_VGIC_MODULE
    struct hv_sgi_diag_snapshot sgi = {0};
    hv_sgi_diag_snapshot(&PERCPU(sgi_diag), &sgi);
    sample.sgi_pending_mask =
        __atomic_load_n(&PERCPU(sgi_pending_mask), __ATOMIC_ACQUIRE);
    sample.sgi_queued = sgi.queued;
    sample.sgi_ipi_received = sgi.ipi_received;
    sample.sgi_drained = sgi.drained;
    sample.sgi_injected = sgi.injected;
    sample.sgi_repended = sgi.repended;
    sample.sgi_no_lr = sgi.no_lr;
    sample.sgi_iar = sgi.iars;
    sample.sgi_eoi = sgi.eois;
    sample.sgi_eoi_active_pending = sgi.eoi_active_pending;
    sample.last_sgi_from = PERCPU(last_sgi_from);
    sample.last_sgi_intid = PERCPU(last_sgi_intid);
    sample.last_iar_intid = PERCPU(last_iar_intid);
    sample.last_eoi_intid = PERCPU(last_eoi_intid);
    sample.last_iar_tick = PERCPU(last_iar_tick);
    sample.last_eoi_tick = PERCPU(last_eoi_tick);

    u32 timer_head = __atomic_load_n(&PERCPU(timer_queue).head, __ATOMIC_ACQUIRE);
    u32 timer_tail = __atomic_load_n(&PERCPU(timer_queue).tail, __ATOMIC_ACQUIRE);
    u32 irq_head = __atomic_load_n(&PERCPU(irq_queue).head, __ATOMIC_ACQUIRE);
    u32 irq_tail = __atomic_load_n(&PERCPU(irq_queue).tail, __ATOMIC_ACQUIRE);
    sample.timer_queue_depth = timer_head - timer_tail;
    sample.irq_queue_depth = irq_head - irq_tail;

    int lr_count = hv_vgic3_num_lrs();
    if (lr_count > HV_WATCHDOG_SNAPSHOT_LR_COUNT)
        lr_count = HV_WATCHDOG_SNAPSHOT_LR_COUNT;
    sample.lr_count = lr_count;
    for (int lr = 0; lr < lr_count; lr++)
        sample.lrs[lr] = hv_vgic3_read_lr(lr);
#endif

    hv_watchdog_snapshot_publish(&PERCPU(watchdog_record), &sample);
}

void hv_watchdog_snapshot_dump(void)
{
    /*
     * A CPU that stops immediately after the `X` breadcrumb is waiting in
     * hv_exc_entry() for the hypervisor-wide lock.  Publish the lock word
     * before the per-CPU records so a snapshot can identify the owner without
     * acquiring that same potentially wedged lock.
     */
    s64 bhl_owner = __atomic_load_n(&bhl.lock, __ATOMIC_ACQUIRE);
    int bhl_count = __atomic_load_n(&bhl.count, __ATOMIC_RELAXED);
    printf("HV WATCHDOG BHL: owner=%ld count=%d\n", bhl_owner, bhl_count);

    for (int cpu = 0; cpu < MAX_CPUS; cpu++) {
        struct hv_watchdog_cpu_sample s = {0};
        if (!hv_watchdog_snapshot_read(&PERCPU_N(cpu, watchdog_record), &s))
            continue;

        printf("HV WATCHDOG CPU: cpu=%lu pc=0x%lx spsr=0x%lx cntpct=0x%lx "
               "cntvct=0x%lx cntvoff=0x%lx host_pctl=0x%lx host_pcval=0x%lx "
               "tick_arm=%lu recovery_arm=%lu tick_fire=%lu "
               "pctl=0x%lx pcval=0x%lx "
               "vctl=0x%lx vcval=0x%lx vm_tmr=0x%lx hcr=0x%lx ich_hcr=0x%lx "
               "ich_vmcr=0x%lx isr=0x%lx pinj=%lu vinj=%lu "
               "tq=%lu iq=%lu pend=0x%lx q=%lu ipi=%lu drain=%lu inj=%lu "
               "repend=%lu no_lr=%lu iar=%lu eoi=%lu ap_eoi=%lu "
               "last_sgi=%lu<-%lu last_iar=%lu@0x%lx last_eoi=%lu@0x%lx "
               "marker=0x%lx ",
               s.cpu, s.pc, s.spsr, s.cntpct, s.cntvct, s.cntvoff,
               s.host_cntp_ctl, s.host_cntp_cval, s.tick_arm_count,
               s.recovery_tick_arm_count, s.host_tick_fires, s.cntp_ctl,
               s.cntp_cval, s.cntv_ctl, s.cntv_cval, s.vm_tmr_fiq_ena,
               s.hcr, s.ich_hcr, s.ich_vmcr, s.isr,
               s.timer_p_injected, s.timer_v_injected, s.timer_queue_depth,
               s.irq_queue_depth, s.sgi_pending_mask, s.sgi_queued,
               s.sgi_ipi_received, s.sgi_drained, s.sgi_injected, s.sgi_repended,
               s.sgi_no_lr, s.sgi_iar, s.sgi_eoi, s.sgi_eoi_active_pending,
               s.last_sgi_intid, s.last_sgi_from, s.last_iar_intid,
               s.last_iar_tick, s.last_eoi_intid, s.last_eoi_tick,
               s.last_el2_marker);
        printf("lrc=%lu lr0=0x%lx lr1=0x%lx lr2=0x%lx "
               "lr3=0x%lx lr4=0x%lx lr5=0x%lx lr6=0x%lx lr7=0x%lx\n",
               s.lr_count, s.lrs[0], s.lrs[1], s.lrs[2],
               s.lrs[3], s.lrs[4], s.lrs[5], s.lrs[6], s.lrs[7]);
    }
}

static bool timer_irq_outstanding(u32 intid)
{
    u64 lrs[HV_VGIC_DIAG_LR_COUNT] = {0};
    int lr_count = hv_vgic3_num_lrs();
    if (lr_count > HV_VGIC_DIAG_LR_COUNT)
        lr_count = HV_VGIC_DIAG_LR_COUNT;
    for (int lr = 0; lr < lr_count; lr++)
        lrs[lr] = hv_vgic3_read_lr(lr);
    if (hv_vgic_diag_has_live_intid(lrs, intid))
        return true;

    virq_queue_t *queue = &PERCPU(timer_queue);
    u32 tail = __atomic_load_n(&queue->tail, __ATOMIC_ACQUIRE);
    u32 head = __atomic_load_n(&queue->head, __ATOMIC_ACQUIRE);
    for (u32 pos = tail; pos != head; pos++) {
        if (queue->buf[pos & (VIRQ_QUEUE_SIZE - 1)].vintid == intid)
            return true;
    }
    return false;
}

static bool timer_sync_live_irq(u32 intid, bool asserted)
{
    u64 lrs[HV_VGIC_DIAG_LR_COUNT] = {0};
    int lr_count = hv_vgic3_num_lrs();
    if (lr_count > HV_VGIC_DIAG_LR_COUNT)
        lr_count = HV_VGIC_DIAG_LR_COUNT;
    for (int lr = 0; lr < lr_count; lr++)
        lrs[lr] = hv_vgic3_read_lr(lr);

    int lr = hv_vgic_diag_find_live_intid(lrs, intid);
    if (lr < 0)
        return false;

    struct hv_vgic_level_result next =
        hv_vgic_diag_sync_level_lr(lrs[lr], asserted);
    if (next.changed) {
        hv_vgic3_write_lr(lr, next.lr);
        hv_vgic3_update_vi();
        sysop("isb");
    }
    return true;
}

static bool timer_repend_live_irq(u32 intid)
{
    u64 lrs[HV_VGIC_DIAG_LR_COUNT] = {0};
    int lr_count = hv_vgic3_num_lrs();
    if (lr_count > HV_VGIC_DIAG_LR_COUNT)
        lr_count = HV_VGIC_DIAG_LR_COUNT;
    for (int lr = 0; lr < lr_count; lr++)
        lrs[lr] = hv_vgic3_read_lr(lr);

    int lr = hv_vgic_diag_repend_live_intid(lrs, intid);
    if (lr < 0)
        return false;

    hv_vgic3_write_lr(lr, lrs[lr]);
    hv_vgic3_update_vi();
    sysop("isb");
    return true;
}

//
// Measure the actual delivery rate. One line per 1024 injections, so the volume is
// negligible. CNTFRQ on M1 is 24 MHz, so ticks/CNTFRQ gives seconds: comparing the
// injection count against elapsed real time tells us directly whether the guest is being
// woken more often than it asked for, and the programmed interval shows what it asked for.
//
static u64 dbg_inj_count = 0;
static u64 dbg_inj_t0 = 0;

static void hv_update_fiq(void)
{
    u64 hcr = mrs(HCR_EL2);
    bool fiq_pending = false;
    int tcpu = smp_id();

    if (tcpu < 0 || tcpu >= MAX_CPUS)
        tcpu = 0;

    if (mrs(CNTP_CTL_EL02) == (CNTx_CTL_ISTATUS | CNTx_CTL_ENABLE)) {
        fiq_pending = true;
        reg_clr(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2, VM_TMR_FIQ_ENA_ENA_P);

#ifdef ENABLE_VGIC_MODULE
        if (timer_p_injected[tcpu]) {
            if (!timer_irq_outstanding(17)) {
                timer_p_injected[tcpu] = false;
                reg_set(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2, VM_TMR_FIQ_ENA_ENA_P);
            }
        } else if (timer_repend_live_irq(17)) {
            timer_p_injected[tcpu] = true;
        } else if (hv_vgic3_get_free_lr() != -1) {
            timer_p_injected[tcpu] = true;
            if (!dbg_inj_t0)
                dbg_inj_t0 = mrs(CNTPCT_EL0);
            if (hv_runtime_trace_enabled() && (++dbg_inj_count & 1023) == 0) {
                u64 now = mrs(CNTPCT_EL0);
                printf("TIMERRATE: inj=0x%lx elapsed_ticks=0x%lx cntfrq=0x%lx interval=0x%lx\n",
                       dbg_inj_count, now - dbg_inj_t0, mrs(CNTFRQ_EL0),
                       mrs(CNTP_CVAL_EL02) - now);
            }
            hv_vgic3_inject_irq(17, hv_vgic3_get_priority(17), false, true,
                                false, 0);
        } else {
            timer_p_injected[tcpu] = true;
            virq_t pending = {
                .vintid = 17,
                .priority = hv_vgic3_get_priority(17),
                .active = false,
                .pending = true,
                .hw_status = false,
                .hw_irq = 0,
            };
            virq_queue_push(&PERCPU(timer_queue), &pending);
        }
#endif
    } else {
        timer_p_injected[tcpu] = false;
        /* CNTP is hv_arm_tick()'s heartbeat; keep its Apple FIQ route alive. */
        reg_set(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2, VM_TMR_FIQ_ENA_ENA_P);
    }

    if (mrs(CNTV_CTL_EL02) == (CNTx_CTL_ISTATUS | CNTx_CTL_ENABLE)) {
        fiq_pending = true;
        reg_clr(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2, VM_TMR_FIQ_ENA_ENA_V);

#ifdef ENABLE_VGIC_MODULE
        if (timer_v_injected[tcpu]) {
            if (!timer_sync_live_irq(18, true) && !timer_irq_outstanding(18)) {
                timer_v_injected[tcpu] = false;
                reg_set(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2, VM_TMR_FIQ_ENA_ENA_V);
            }
        } else if (timer_repend_live_irq(18)) {
            timer_v_injected[tcpu] = true;
        } else if (hv_vgic3_get_free_lr() != -1) {
            timer_v_injected[tcpu] = true;
            hv_vgic3_inject_irq(18, hv_vgic3_get_priority(18), false, true,
                                false, 0);
        } else {
            timer_v_injected[tcpu] = true;
            virq_t pending = {
                .vintid = 18,
                .priority = hv_vgic3_get_priority(18),
                .active = false,
                .pending = true,
                .hw_status = false,
                .hw_irq = 0,
            };
            virq_queue_push(&PERCPU(timer_queue), &pending);
        }
#endif
    } else {
#ifdef ENABLE_VGIC_MODULE
        if (timer_v_injected[tcpu])
            timer_sync_live_irq(18, false);
#endif
        timer_v_injected[tcpu] = false;
        reg_set(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2, VM_TMR_FIQ_ENA_ENA_V);
    }

    fiq_pending |= PERCPU(ipi_pending) || PERCPU(pmc_pending);

    sysop("isb");
#ifndef ENABLE_VGIC_MODULE
    if ((hcr & HCR_VF) && !fiq_pending) {
        hv_write_hcr(hcr & ~HCR_VF);
    } else if (!(hcr & HCR_VF) && fiq_pending) {
        hv_write_hcr(hcr | HCR_VF);
    }
#endif
}

#define SYSREG_MAP(sr, to)                                                                         \
    case SYSREG_ISS(sr):                                                                           \
        if (is_read)                                                                               \
            regs[rt] = _mrs(sr_tkn(to));                                                           \
        else                                                                                       \
            _msr(sr_tkn(to), regs[rt]);                                                            \
        return true;

#define SYSREG_PASS(sr)                                                                            \
    case SYSREG_ISS(sr):                                                                           \
        if (is_read)                                                                               \
            regs[rt] = _mrs(sr_tkn(sr));                                                           \
        else                                                                                       \
            _msr(sr_tkn(sr), regs[rt]);                                                            \
        return true;

static bool hv_handle_msr_unlocked(struct exc_info *ctx, u64 iss)
{
    u64 reg = iss & (ESR_ISS_MSR_OP0 | ESR_ISS_MSR_OP2 | ESR_ISS_MSR_OP1 | ESR_ISS_MSR_CRn |
                     ESR_ISS_MSR_CRm);
    u64 rt = FIELD_GET(ESR_ISS_MSR_Rt, iss);
    bool is_read = iss & ESR_ISS_MSR_DIR;

    u64 *regs = ctx->regs;

    regs[31] = 0;

    switch (reg) {
        SYSREG_PASS(SYS_IMP_APL_CORE_NRG_ACC_DAT);
        SYSREG_PASS(SYS_IMP_APL_CORE_SRM_NRG_ACC_DAT);
        /* Architectural timer, for ECV */
        SYSREG_MAP(SYS_CNTV_CTL_EL0, SYS_CNTV_CTL_EL02)
        SYSREG_MAP(SYS_CNTV_CVAL_EL0, SYS_CNTV_CVAL_EL02)
        SYSREG_MAP(SYS_CNTV_TVAL_EL0, SYS_CNTV_TVAL_EL02)
        SYSREG_MAP(SYS_CNTP_CTL_EL0, SYS_CNTP_CTL_EL02)
        SYSREG_MAP(SYS_CNTP_CVAL_EL0, SYS_CNTP_CVAL_EL02)
        SYSREG_MAP(SYS_CNTP_TVAL_EL0, SYS_CNTP_TVAL_EL02)
        /* Spammy stuff seen on t600x p-cores */
        /* These are PMU/PMC registers */
        SYSREG_PASS(sys_reg(3, 2, 15, 12, 0));
        SYSREG_PASS(sys_reg(3, 2, 15, 13, 0));
        SYSREG_PASS(sys_reg(3, 2, 15, 14, 0));
        SYSREG_PASS(sys_reg(3, 2, 15, 15, 0));
        SYSREG_PASS(sys_reg(3, 1, 15, 7, 0));
        SYSREG_PASS(sys_reg(3, 1, 15, 8, 0));
        SYSREG_PASS(sys_reg(3, 1, 15, 9, 0));
        SYSREG_PASS(sys_reg(3, 1, 15, 10, 0));
        /* Noisy traps */
        SYSREG_PASS(SYS_IMP_APL_HID4)
        SYSREG_PASS(SYS_IMP_APL_EHID4)
        /* We don't normally trap these, but if we do, they're noisy */
        SYSREG_PASS(SYS_IMP_APL_GXF_STATUS_EL1)
        SYSREG_PASS(SYS_IMP_APL_CNTVCT_ALIAS_EL0)
        SYSREG_PASS(SYS_IMP_APL_TPIDR_GL1)
        SYSREG_MAP(SYS_IMP_APL_SPSR_GL1, SYS_IMP_APL_SPSR_GL12)
        SYSREG_MAP(SYS_IMP_APL_ASPSR_GL1, SYS_IMP_APL_ASPSR_GL12)
        SYSREG_MAP(SYS_IMP_APL_ELR_GL1, SYS_IMP_APL_ELR_GL12)
        SYSREG_MAP(SYS_IMP_APL_ESR_GL1, SYS_IMP_APL_ESR_GL12)
        SYSREG_MAP(SYS_IMP_APL_SPRR_PERM_EL1, SYS_IMP_APL_SPRR_PERM_EL12)
        SYSREG_MAP(SYS_IMP_APL_APCTL_EL1, SYS_IMP_APL_APCTL_EL12)
        SYSREG_MAP(SYS_IMP_APL_AMX_CTL_EL1, SYS_IMP_APL_AMX_CTL_EL12)
        /* FIXME:Might be wrong */
        SYSREG_PASS(SYS_IMP_APL_AMX_STATE_T)
        /* pass through PMU handling */
        SYSREG_PASS(SYS_IMP_APL_PMCR1)
        SYSREG_PASS(SYS_IMP_APL_PMCR2)
        SYSREG_PASS(SYS_IMP_APL_PMCR3)
        SYSREG_PASS(SYS_IMP_APL_PMCR4)
        SYSREG_PASS(SYS_IMP_APL_PMESR0)
        SYSREG_PASS(SYS_IMP_APL_PMESR1)
        SYSREG_PASS(SYS_IMP_APL_PMSR)
#ifndef DEBUG_PMU_IRQ
        SYSREG_PASS(SYS_IMP_APL_PMC0)
#endif
        SYSREG_PASS(SYS_IMP_APL_PMC1)
        SYSREG_PASS(SYS_IMP_APL_PMC2)
        SYSREG_PASS(SYS_IMP_APL_PMC3)
        SYSREG_PASS(SYS_IMP_APL_PMC4)
        SYSREG_PASS(SYS_IMP_APL_PMC5)
        SYSREG_PASS(SYS_IMP_APL_PMC6)
        SYSREG_PASS(SYS_IMP_APL_PMC7)
        SYSREG_PASS(SYS_IMP_APL_PMC8)
        SYSREG_PASS(SYS_IMP_APL_PMC9)

        //spammy ntoskrnl regs
        SYSREG_PASS(SYS_IMP_APL_L2C_ERR_STS)

        SYSREG_PASS(sys_reg(2, 0, 0, 1, 4))
        SYSREG_PASS(sys_reg(2, 0, 0, 1, 5))
        SYSREG_PASS(sys_reg(2, 0, 0, 1, 6))
        SYSREG_PASS(sys_reg(2, 0, 0, 1, 7))

        SYSREG_PASS(sys_reg(2, 0, 0, 2, 2))
        SYSREG_PASS(sys_reg(2, 0, 0, 2, 4))
        SYSREG_PASS(sys_reg(2, 0, 0, 2, 5))
        SYSREG_PASS(sys_reg(2, 0, 0, 2, 6))
        SYSREG_PASS(sys_reg(2, 0, 0, 2, 7))

        SYSREG_PASS(sys_reg(2, 0, 0, 3, 4))
        SYSREG_PASS(sys_reg(2, 0, 0, 3, 5))
        SYSREG_PASS(sys_reg(2, 0, 0, 3, 6))
        SYSREG_PASS(sys_reg(2, 0, 0, 3, 7))

        SYSREG_PASS(sys_reg(2, 0, 0, 4, 4))
        SYSREG_PASS(sys_reg(2, 0, 0, 4, 5))

        SYSREG_PASS(sys_reg(2, 0, 0, 5, 4))
        SYSREG_PASS(sys_reg(2, 0, 0, 5, 5))

        //these seem debugging related but looks like windbg/m1n1 debugger still works
        SYSREG_PASS(sys_reg(2, 0, 0, 0, 4))
        SYSREG_PASS(sys_reg(2, 0, 0, 0, 5))
        SYSREG_PASS(sys_reg(2, 0, 0, 0, 6))
        SYSREG_PASS(sys_reg(2, 0, 0, 0, 7))
        SYSREG_PASS(sys_reg(2, 0, 1, 1, 4))

        //
        // Enabled: ICH_HCR_EL2.TC | TALL1 are now set in
        // hv_vgicv3_enable_virtual_interrupts(), so these traps do arrive.
        // (This whole block used to sit inside a /* */ comment, which is why the
        // handlers below never ran even though the cases looked present.)
        //
        //
        // Common (group-agnostic) ICC registers, trapped by ICH_HCR_EL2.TC.
        //
        case SYSREG_ISS(ICC_SRE_EL1):
            //
            // Report SRE as enabled. EDK2's GicV3Supported() reads this back after
            // writing it and falls back to the GICv2 MMIO interface if the bit does not
            // stick, so it must read as 1. DFB/DIB are irrelevant here.
            //
            if(is_read) {
                regs[rt] = 0x1;
            }
            else {
            }
            return true;
        case SYSREG_ISS(ICC_PMR_EL1): {
            //
            // The guest's priority mask lives in ICH_VMCR_EL2.VPMR (bits 31:24). Keeping
            // these in sync is what lets the guest open the mask itself instead of
            // relying on the bring-up default set in hv_vgicv3_enable_virtual_interrupts.
            //
            u64 vmcr = mrs(ICH_VMCR_EL2);
            if(is_read) {
                regs[rt] = (vmcr >> 24) & 0xff;
            }
            else {
                vmcr &= ~(0xffUL << 24);
                vmcr |= (regs[rt] & 0xff) << 24;
                msr(ICH_VMCR_EL2, vmcr);
                //
                // The mask moved, so the line may need to rise (an interrupt held back
                // until now) or fall (the guest just raised its IRQL above a pending
                // interrupt's priority).
                //
                hv_vgic3_update_vi();
            }
            return true;
        }
        case SYSREG_ISS(ICC_CTLR_EL1): {
            u64 vmcr = mrs(ICH_VMCR_EL2);
            if(is_read) {
                //
                // PRIbits = 4 means five implemented priority bits, which matches what
                // the hardware reported back through VPMR (0xff written, 0xf8 read).
                // IDbits = 0 means 16-bit INTIDs. EOImode mirrors VMCR.VEOIM (bit 9).
                //
                regs[rt] = (4UL << 8) | (((vmcr >> 9) & 1) << 1);
            }
            else {
                vmcr &= ~BIT(9);
                if(regs[rt] & BIT(1))
                    vmcr |= BIT(9);
                msr(ICH_VMCR_EL2, vmcr);
            }
            return true;
        }
        case SYSREG_ISS(ICC_IAR1_EL1):
            if(is_read) {
                regs[rt] = hv_vgic3_do_iar1();
            }
            else{
            }
            return true;
        case SYSREG_ISS(ICC_IGRPEN1_EL1):
            if(is_read) {
                regs[rt] = hv_vgic3_get_igrpen1();
            }
            else{
                hv_vgic3_set_igrpen1(regs[rt]);
            }
            return true;
        case SYSREG_ISS(ICC_BPR1_EL1):
            if(is_read) {
                regs[rt] = 0;
            }
            else{
            }
            return true;
        case SYSREG_ISS(ICC_RPR_EL1):
            //
            // Running priority. Emulated from active LRs (see hv_vgic3_running_priority).
            // Was previously unhandled - the guest read whatever the trap left in the
            // register. The HAL reads this on interrupt entry; a wrong value makes it
            // compute an inconsistent IRQL and bugcheck 0xC8 IRQL_UNEXPECTED_VALUE.
            //
            if(is_read) {
                regs[rt] = hv_vgic3_running_priority();
            }
            return true;
        case SYSREG_ISS(ICC_EOIR1_EL1):
            if(is_read) {
                regs[rt] = 0;
            }
            else{
                hv_vgic3_do_eoir1(regs[rt]);
            }
            return true;


#ifdef ENABLE_VGIC_MODULE
        /* m1n1_windows change - emulate SGIs */
        case SYSREG_ISS(ICC_SGI1R_EL1):
            if(is_read) {
                regs[rt] = 0;
            }
            else{
                u64 sgir_value = regs[rt];
                u32 aff1, aff2, aff3;
                u32 aff0_targets;
                int virq, irm, rs;

                rs = (sgir_value >> ICH_SGI_RS_SHIFT) & ICH_SGI_RS_MASK;
                irm = (sgir_value >> ICH_SGI_IRQMODE_SHIFT) & ICH_SGI_IRQMODE_MASK;
                virq = (sgir_value >> ICH_SGI_IRQ_SHIFT ) & ICH_SGI_IRQ_MASK;
                aff1 = ICH_SGI_AFF1(sgir_value);
                aff2 = ICH_SGI_AFF2(sgir_value);
                aff3 = ICH_SGI_AFF3(sgir_value);
                aff0_targets = sgir_value & ICH_SGI_TARGETLIST_MASK;

                for(int cpu = 0; cpu < num_cpus; cpu++){
                    if(irm == ICH_SGI_TARGET_OTHERS){
                        if(smp_id() == cpu)
                            continue;
                    } else if(irm == ICH_SGI_TARGET_LIST){
                        if(aff0_targets == 0)
                            return false;
                        u64 mpidr =  smp_get_mpidr(cpu);

                        if(MPIDR_AFF3(mpidr) != aff3)
                            continue;
                        if(MPIDR_AFF2(mpidr) != aff2)
                            continue;
                        if(MPIDR_AFF1(mpidr) != aff1)
                            continue;
                        if(!(aff0_targets & BIT( MPIDR_AFF0(mpidr))))
                            continue;
                    } else{
                        return false;
                    }
                    //
                    // Use the priority the TARGET core programmed for this SGI. It was
                    // hardcoded to 0x20, but SGI priorities are per-redistributor and Windows
                    // derives IRQL from them, so an IPI delivered at a made-up priority is
                    // taken at the wrong IRQL - the shape of IRQL_NOT_LESS_OR_EQUAL.
                    //
                    hv_vgic3_queue_sgi(cpu, virq);
                }
            }
            return true;
        /* m1n1_windows change - advertise GIC */
        case SYSREG_ISS(ID_AA64PFR0_EL1):
            if(is_read) {
                u64 pfr0_value = mrs(ID_AA64PFR0_EL1);
                regs[rt] = pfr0_value | ((0x1 & 0xF) << 24);
            }
            else{
                msr(ID_AA64PFR0_EL1, regs[rt]);
            }
            return true;
        //
        // The rest of the ID register space. This only matters because the GIC
        // advertisement above requires HCR_EL2.TID3, and TID3 traps reads of *every*
        // register in this space -- without these cases the guest would take an
        // unhandled trap on the first unrelated ID register it reads.
        //
        // ID registers are read-only, so writes are simply swallowed.
        //
        case SYSREG_ISS(ID_PFR0_EL1):
            if (is_read)
                regs[rt] = mrs(ID_PFR0_EL1);
            return true;
        case SYSREG_ISS(ID_PFR1_EL1):
            if (is_read)
                regs[rt] = mrs(ID_PFR1_EL1);
            return true;
        case SYSREG_ISS(ID_DFR0_EL1):
            if (is_read)
                regs[rt] = mrs(ID_DFR0_EL1);
            return true;
        case SYSREG_ISS(ID_AFR0_EL1):
            if (is_read)
                regs[rt] = mrs(ID_AFR0_EL1);
            return true;
        case SYSREG_ISS(ID_MMFR0_EL1):
            if (is_read)
                regs[rt] = mrs(ID_MMFR0_EL1);
            return true;
        case SYSREG_ISS(ID_MMFR1_EL1):
            if (is_read)
                regs[rt] = mrs(ID_MMFR1_EL1);
            return true;
        case SYSREG_ISS(ID_MMFR2_EL1):
            if (is_read)
                regs[rt] = mrs(ID_MMFR2_EL1);
            return true;
        case SYSREG_ISS(ID_MMFR3_EL1):
            if (is_read)
                regs[rt] = mrs(ID_MMFR3_EL1);
            return true;
        case SYSREG_ISS(ID_ISAR0_EL1):
            if (is_read)
                regs[rt] = mrs(ID_ISAR0_EL1);
            return true;
        case SYSREG_ISS(ID_ISAR1_EL1):
            if (is_read)
                regs[rt] = mrs(ID_ISAR1_EL1);
            return true;
        case SYSREG_ISS(ID_ISAR2_EL1):
            if (is_read)
                regs[rt] = mrs(ID_ISAR2_EL1);
            return true;
        case SYSREG_ISS(ID_ISAR3_EL1):
            if (is_read)
                regs[rt] = mrs(ID_ISAR3_EL1);
            return true;
        case SYSREG_ISS(ID_ISAR4_EL1):
            if (is_read)
                regs[rt] = mrs(ID_ISAR4_EL1);
            return true;
        case SYSREG_ISS(ID_ISAR5_EL1):
            if (is_read)
                regs[rt] = mrs(ID_ISAR5_EL1);
            return true;
        case SYSREG_ISS(ID_MMFR4_EL1):
            if (is_read)
                regs[rt] = mrs(ID_MMFR4_EL1);
            return true;
        case SYSREG_ISS(ID_ISAR6_EL1):
            if (is_read)
                regs[rt] = mrs(ID_ISAR6_EL1);
            return true;
        case SYSREG_ISS(MVFR0_EL1):
            if (is_read)
                regs[rt] = mrs(MVFR0_EL1);
            return true;
        case SYSREG_ISS(MVFR1_EL1):
            if (is_read)
                regs[rt] = mrs(MVFR1_EL1);
            return true;
        case SYSREG_ISS(MVFR2_EL1):
            if (is_read)
                regs[rt] = mrs(MVFR2_EL1);
            return true;
        case SYSREG_ISS(ID_AA64PFR1_EL1):
            if (is_read)
                regs[rt] = mrs(ID_AA64PFR1_EL1);
            return true;
        case SYSREG_ISS(ID_AA64DFR0_EL1):
            if (is_read)
                regs[rt] = mrs(ID_AA64DFR0_EL1);
            return true;
        case SYSREG_ISS(ID_AA64DFR1_EL1):
            if (is_read)
                regs[rt] = mrs(ID_AA64DFR1_EL1);
            return true;
        case SYSREG_ISS(ID_AA64AFR0_EL1):
            if (is_read)
                regs[rt] = mrs(ID_AA64AFR0_EL1);
            return true;
        case SYSREG_ISS(ID_AA64AFR1_EL1):
            if (is_read)
                regs[rt] = mrs(ID_AA64AFR1_EL1);
            return true;
        case SYSREG_ISS(ID_AA64ISAR0_EL1):
            if (is_read)
                regs[rt] = mrs(ID_AA64ISAR0_EL1);
            return true;
        case SYSREG_ISS(ID_AA64ISAR1_EL1):
            if (is_read)
                regs[rt] = mrs(ID_AA64ISAR1_EL1);
            return true;
        case SYSREG_ISS(ID_AA64MMFR0_EL1):
            if (is_read)
                regs[rt] = mrs(ID_AA64MMFR0_EL1);
            return true;
        case SYSREG_ISS(ID_AA64MMFR1_EL1):
            if (is_read)
                regs[rt] = mrs(ID_AA64MMFR1_EL1);
            return true;
        case SYSREG_ISS(ID_AA64MMFR2_EL1):
            if (is_read)
                regs[rt] = mrs(ID_AA64MMFR2_EL1);
            return true;
#endif
        /* m1n1_windows change - Trap the ARM standard PMU regs */
        case SYSREG_ISS(SYS_PMCR_EL0):
            if(is_read) {
                u64 pmcr0_value = mrs(SYS_IMP_APL_PMCR0);
                u64 calculated = 0;
                u64 pmi_mask = PMCR0_IMODE_MASK;
                //
                // Are PMIs enabled? (affects bit 0 of PMCR equivalently)
                //
                if((pmcr0_value & pmi_mask) == (PMCR0_IMODE_FIQ)) {
                    calculated |= PMCR_E;
                }
                //
                // Bits 5:1 are 0 for now,  bits 6 and 7 are on always (long events always on)
                // and bit 9 is checked by PMCR0[20] (since it deals with freeze/overflow)
                //
                if((pmcr0_value & BIT(20)) != 0) {
                    calculated |= PMCR_FZO;
                }
                calculated |= ((BIT(6)) | (BIT(7)));
                HV_RUNTIME_TRACE("HV PMUv3 Redirect: mrs x%ld, PMCR_EL0 = 0x%lx\n", rt, calculated);
                regs[rt] = calculated;
            }
            else {
                //
                // Bits [63:10] will have writes discarded (mostly ARM spec, bit 32 due to lack of support)
                //
                u64 pmcr0_value = mrs(SYS_IMP_APL_PMCR0);
                int cycle_reset_requested = 0;
                //
                // Bit 9 (stop count on overflow) writes affect bit 20 of APL_PMCR0
                //
                if((regs[rt] & BIT(9)) != 0) {
                    pmcr0_value |= BIT(20);
                }
                else {
                    pmcr0_value &= ~(BIT(20));
                }
                
                //
                // Writes to bits [6:3] unsupported since no way of expressing either with Apple PMUs.
                //
                // Writing bit 2 (cycle counter reset) implies setting PMC0 to 0, so check if that's the case.
                //
                if((regs[rt] & PMCR_C) != 0) {
                    cycle_reset_requested = 1;
                }
                //
                // Bit 1 is the same as bit 2 but for event counters, unimplemented for now.
                //
                // Bit 0 controls whether event counters are enabled globally, if this is being set,
                // the closest thing on Apple platforms is the IRQ mode so set that if bit 0 is requested.
                //
                if((regs[rt] & PMCR_E) != 0) {
                    pmcr0_value &= ~(PMCR0_IMODE_MASK);
                    pmcr0_value |= PMCR0_IMODE_FIQ;
                }
                else {
                    pmcr0_value &= ~(PMCR0_IMODE_MASK);
                    pmcr0_value |= PMCR0_IMODE_OFF;
                }
                sysop("isb");
                if(cycle_reset_requested == 1) {
                    pmcr0_value &= ~(BIT(12));
                    pmcr0_value &= ~(BIT(0));
                }
                msr(SYS_IMP_APL_PMCR0, pmcr0_value);
                sysop("isb");
                if(cycle_reset_requested == 1) {
                    sysop("isb");
                    msr(SYS_IMP_APL_PMC0, 0);
                    sysop("isb");
                    pmcr0_value |= BIT(12);
                    pmcr0_value |= BIT(0);
                    sysop("isb");
                    msr(SYS_IMP_APL_PMCR0, pmcr0_value);
                    sysop("isb");
                }
                HV_RUNTIME_TRACE("HV PMUv3 Redirect (OK): msr PMCR_EL0, x%ld = 0x%lx\n", rt, regs[rt]);
            }
            return true;
        SYSREG_MAP(SYS_PMCCNTR_EL0, SYS_IMP_APL_PMC0)
        case SYSREG_ISS(SYS_PMCCFILTR_EL0):
            if(is_read) {
                u64 pmcr1_value = mrs(SYS_IMP_APL_PMCR1);
                u64 calculated_value = 0;
                //
                // If EL0/EL1 counting is disabled, set bit 30 of PMCCFILTR to 1.
                // (This is backwards from how I would've done it but okay I suppose...)
                //
                if((pmcr1_value & BIT(8)) == 0) {
                    calculated_value |= BIT(30);
                }
                if((pmcr1_value & BIT(16)) == 0) {
                    calculated_value |= BIT(31);
                }
                //
                // EL2 counting always happens as far as is known, so bit 27 is always set to 1
                // (bit set to 1 in this case means disable filtering...not sure why it's backwards.)
                //
                calculated_value |= BIT(27);
                HV_RUNTIME_TRACE("HV PMUv3 Redirect: mrs x%ld, PMCCFILTR_EL0 = 0x%lx\n", rt, calculated_value);
                regs[rt] = calculated_value;
            }
            else {
                u64 pmcr1_value = mrs(SYS_IMP_APL_PMCR1);
                //
                // If we're being asked to disable counting cycles for a given EL, set the appropriate bit for
                // PMCR1 respectively.
                //
                if((regs[rt] & PMCCFILTR_P) == 0) {
                    pmcr1_value |= BIT(16);
                }
                else {
                    pmcr1_value &= ~(BIT(16));
                }
                if((regs[rt] & PMCCFILTR_U) == 0) {
                    pmcr1_value |= BIT(8);
                }
                else {
                    pmcr1_value &= ~(BIT(16));
                }
                sysop("isb");
                msr(SYS_IMP_APL_PMCR1, pmcr1_value);
                sysop("isb");
                HV_RUNTIME_TRACE("HV PMUv3 Redirect (OK): msr PMCCFILTR_EL0, x%ld = 0x%lx\n", rt, regs[rt]);
            }
            return true;
        case SYSREG_ISS(SYS_PMCEID0_EL0):
            //
            // Unimplemented for now, return 0 for a read, discard writes.
            //
            if(is_read) {
                regs[rt] = 0;
                HV_RUNTIME_TRACE("HV PMUv3 Redirect: mrs x%ld, PMCEID0_EL0 = 0x%lx\n", rt, regs[rt]);
            }
            else {
                //
                // Do nothing here.
                //
                HV_RUNTIME_TRACE("HV PMUv3 Redirect (skipped write): msr PMCEID0_EL0, x%ld = 0x%lx\n", rt, regs[rt]);
            }
            return true;
        case SYSREG_ISS(SYS_PMCEID1_EL0):
            //
            // Unimplemented for now, return 0 for a read, discard writes.
            //
            if(is_read) {
                regs[rt] = 0;
                HV_RUNTIME_TRACE("HV PMUv3 Redirect: mrs x%ld, PMCEID1_EL0 = 0x%lx\n", rt, regs[rt]);
            }
            else {
                HV_RUNTIME_TRACE("HV PMUv3 Redirect (skipped write): msr PMCEID1_EL0, x%ld = 0x%lx\n", rt, regs[rt]);
            }
            return true;
        case SYSREG_ISS(SYS_PMCNTENCLR_EL0):
            if(is_read) {
                u64 pmcr0_value = mrs(SYS_IMP_APL_PMCR0);
                u64 calculated_value = 0;
                //
                // check what perf counters are enabled in PMCR0, and reflect that in the returned PMCNTENCLR/PMCNTENSET value
                //
                if((pmcr0_value & BIT(0)) != 0) {
                    calculated_value |= BIT(31);
                }
                HV_RUNTIME_TRACE("HV PMUv3 Redirect: mrs x%ld, PMCNTENCLR_EL0 = 0x%lx\n", rt, calculated_value);
                regs[rt] = calculated_value;
            }
            else {
                u64 pmcr0_value = mrs(SYS_IMP_APL_PMCR0);
                u64 counters_disable_mask = GENMASK(31,0);
                //
                // check if any counters are being requested to be disabled (cycle counter only for now)
                // and deal with those here.
                //
                if((regs[rt] & counters_disable_mask) != 0) {
                    if((regs[rt] & BIT(31)) != 0) {
                        pmcr0_value &= ~(BIT(0));
                    }
                    sysop("isb");
                    msr(SYS_IMP_APL_PMCR0, pmcr0_value);
                    sysop("isb");
                    HV_RUNTIME_TRACE("HV PMUv3 Redirect (OK): msr PMCNTENCLR_EL0, x%ld = 0x%lx\n", rt, regs[rt]);
                }
            }
            return true;
        case SYSREG_ISS(SYS_PMCNTENSET_EL0):
            if(is_read) {
                u64 pmcr0_value = mrs(SYS_IMP_APL_PMCR0);
                u64 calculated_value = 0;
                //
                // check what perf counters are enabled in PMCR0, and reflect that in the returned PMCNTENCLR/PMCNTENSET value
                //
                if((pmcr0_value & BIT(0)) != 0) {
                    calculated_value |= BIT(31);
                }
                HV_RUNTIME_TRACE("HV PMUv3 Redirect: mrs x%ld, PMCNTENSET_EL0 = 0x%lx\n", rt, calculated_value);
                regs[rt] = calculated_value;
            }
            else {
                u64 pmcr0_value = mrs(SYS_IMP_APL_PMCR0);
                u64 counters_enable_mask = GENMASK(31,0);
                //
                // check if any counters are being requested to be disabled (cycle counter only for now)
                // and deal with those here.
                //
                if((regs[rt] & counters_enable_mask) != 0) {
                    if((regs[rt] & BIT(31)) != 0) {
                        pmcr0_value |= BIT(0);
                    }
                    sysop("isb");
                    msr(SYS_IMP_APL_PMCR0, pmcr0_value);
                    sysop("isb");
                    HV_RUNTIME_TRACE("HV PMUv3 Redirect (OK): msr PMCNTENSET_EL0, x%ld = 0x%lx\n", rt, regs[rt]);
                }
            }
            return true;
        SYSREG_MAP(SYS_PMEVCNTR0_EL0, SYS_IMP_APL_PMC2)
        // case SYSREG_ISS(SYS_PMEVTYPER0_EL0):
        //     if(is_read) {
        //         printf("mrs(PMEVTYPER0_EL0)\n");
        //         regs[rt] = 0;
        //         int value = mrs(SYS_IMP_APL_PMCR1);
        //         if(value & GENMASK(23, 16)) {
        //             regs[rt] |= BIT(31); //privileged bit
        //         }
        //         if(value & GENMASK(15, 8)) {
        //             regs[rt] |= BIT(30); //user filter bit
        //         }
        //         regs[rt] |= (mrs(SYS_IMP_APL_PMESR0) & GENMASK(7, 0));
        //     }
        //     else {
        //         int val = mrs(SYS_IMP_APL_PMCR1);
        //         int event = mrs(SYS_IMP_APL_PMESR0) & GENMASK(7, 0);
        //         if(regs[rt] & PMEVTYPER_P) {
        //             printf("msr(PMEVTYPER0_EL0, 0x%08lx): enabling el1 counting of event\n", regs[rt]);
        //             val |= BIT(16);
        //         }
        //         if(regs[rt] & GENMASK(7, 0)) {
        //             printf("msr(PMEVTYPER0_EL0, 0x%08lx): setting event\n", regs[rt]);
        //             event |= regs[rt] & GENMASK(7, 0);
        //             msr(SYS_IMP_APL_PMESR0, event);
        //         }
        //         msr(SYS_IMP_APL_PMCR1, val);
        //     }
        //     return true;
        case SYSREG_ISS(SYS_PMINTENCLR_EL1):
            if(is_read) {
                u64 pmcr0_value = mrs(SYS_IMP_APL_PMCR0);
                u64 calculated_value = 0;
                //
                // As before, cycle counter only for now. (bit 12 = PMI enabled for cycle counter)
                //
                if((pmcr0_value & BIT(12)) != 0) {
                    calculated_value |= BIT(31);
                }
                HV_RUNTIME_TRACE("HV PMUv3 Redirect: mrs x%ld, PMINTENCLR_EL1 = 0x%lx\n", rt, calculated_value);
                regs[rt] = calculated_value;
            }
            else {
                u64 pmcr0_value = mrs(SYS_IMP_APL_PMCR0);
                u64 counter_irqs_disabled_mask = GENMASK(31,0);
                //
                // Cycle counter only for now (bits 19:12 control all PMIs for the PMUs)
                //
                if((regs[rt] & counter_irqs_disabled_mask) != 0) {
                    if((regs[rt] & (BIT(31))) != 0) {
                        pmcr0_value &= ~(BIT(12));
                    }
                    sysop("isb");
                    msr(SYS_IMP_APL_PMCR0, pmcr0_value);
                    sysop("isb");
                    HV_RUNTIME_TRACE("HV PMUv3 Redirect (OK): msr PMINTENCLR_EL1, x%ld = 0x%lx\n", rt, regs[rt]);
                }

            }
            return true;
        case SYSREG_ISS(SYS_PMINTENSET_EL1):
            if(is_read) {
                u64 pmcr0_value = mrs(SYS_IMP_APL_PMCR0);
                u64 calculated_value = 0;
                //
                // As before, cycle counter only for now. (bit 12 = PMI enabled for cycle counter)
                //
                if((pmcr0_value & BIT(12)) != 0) {
                    calculated_value |= BIT(31);
                }
                HV_RUNTIME_TRACE("HV PMUv3 Redirect: mrs x%ld, PMINTENSET_EL1 = 0x%lx\n", rt, calculated_value);
                regs[rt] = calculated_value;
            }
            else {
                u64 pmcr0_value = mrs(SYS_IMP_APL_PMCR0);
                u64 counter_irqs_enabled_mask = GENMASK(31,0);
                //
                // Cycle counter only for now (bits 19:12 control all PMIs for the PMUs)
                //
                if((regs[rt] & counter_irqs_enabled_mask) != 0) {
                    if((regs[rt] & BIT(31)) != 0) {
                        pmcr0_value |= BIT(12);
                    }
                    sysop("isb");
                    msr(SYS_IMP_APL_PMCR0, pmcr0_value);
                    sysop("isb");
                    HV_RUNTIME_TRACE("HV PMUv3 Redirect (OK): msr PMINTENSET_EL1, x%ld = 0x%lx\n", rt, regs[rt]);
                }
            }
            return true;
        case SYSREG_ISS(SYS_PMMIR_EL1):
            //
            // return 0 for now, discard writes.
            //
            if(is_read) {
                regs[rt] = 0;
                HV_RUNTIME_TRACE("HV PMUv3 Redirect: mrs x%ld, PMMIR_EL1 = 0x%lx\n", rt, regs[rt]);
            }
            else {
                HV_RUNTIME_TRACE("HV PMUv3 Redirect (skipped write): msr PMMIR_EL1, x%ld = 0x%lx\n", rt, regs[rt]);
            }
            return true;
        case SYSREG_ISS(SYS_PMOVSCLR_EL0):
            if(is_read) {
                //
                // Read the state of the PMSR register to see if a PMU has overflowed.
                // Cycle counter only for now.
                //
                u64 pmsr_value = mrs(SYS_IMP_APL_PMSR);
                u64 calculated_value = 0;
                u64 pmu_overflowed_mask = GENMASK(9, 0);
                if((pmsr_value & pmu_overflowed_mask) != 0) {
                    //
                    // bit 0 is for PMC 0
                    //
                    if((pmsr_value & BIT(0)) != 0) {
                        calculated_value |= BIT(31);
                    }
                }
                HV_RUNTIME_TRACE("HV PMUv3 Redirect: mrs x%ld, PMOVSCLR_EL0 = 0x%lx\n", rt, calculated_value);
                regs[rt] = calculated_value;
            }
            else {
                //
                // To clear the overflow bit requires a reset of the PMC (to clear PMSR)
                // so we need to disable the counter and re-enable it with the bit set to 0.
                //
                u64 pmcr0_value = mrs(SYS_IMP_APL_PMCR0);
                u64 counter_overflow_mask = GENMASK(31,0);
                if((regs[rt] & counter_overflow_mask) != 0) {
                    if((regs[rt] & BIT(31)) != 0) {
                        pmcr0_value &= ~(BIT(12));
                        pmcr0_value &= ~(BIT(0));
                        sysop("isb");
                        msr(SYS_IMP_APL_PMCR0, pmcr0_value);
                        sysop("isb");
                        sysop("isb");
                        msr(SYS_IMP_APL_PMC0, 0);
                        sysop("isb");
                        pmcr0_value |= BIT(12);
                        pmcr0_value |= BIT(0);
                        sysop("isb");
                        msr(SYS_IMP_APL_PMCR0, pmcr0_value);
                        sysop("isb");
                    }
                HV_RUNTIME_TRACE("HV PMUv3 Redirect (OK): msr PMOVSCLR_EL0, x%ld = 0x%lx\n", rt, regs[rt]);
                }
            }
            return true;
        case SYSREG_ISS(SYS_PMOVSSET_EL0):
            if(is_read) {
                //
                // Read the state of the PMSR register to see if a PMU has overflowed.
                // Cycle counter only for now.
                //
                u64 pmsr_value = mrs(SYS_IMP_APL_PMSR);
                u64 calculated_value = 0;
                u64 pmu_overflowed_mask = GENMASK(9, 0);
                if((pmsr_value & pmu_overflowed_mask) != 0) {
                    //
                    // bit 0 is for PMC 0
                    //
                    if((pmsr_value & BIT(0)) != 0) {
                        calculated_value |= BIT(31);
                    }
                }
                HV_RUNTIME_TRACE("HV PMUv3 Redirect: mrs x%ld, PMOVSSET_EL0 = 0x%lx\n", rt, calculated_value);
                regs[rt] = calculated_value;
            }
            else {
                //
                // For now, don't set the overflow bit. If this needs to change, reuse the code from before.
                //
            }
            return true;
        case SYSREG_ISS(SYS_PMSELR_EL0):
            //for now hardcode to set the cycle counter, this will very likely need to change
            if(is_read) {
                regs[rt] = 31;
                HV_RUNTIME_TRACE("HV PMUv3 Redirect: mrs x%ld, PMSELR_EL0 = 0x%lx\n", rt, regs[rt]);
            }
            else {
                HV_RUNTIME_TRACE("HV PMUv3 Redirect (skipped write): msr PMSELR_EL0, x%ld = 0x%lx\n", rt, regs[rt]);
            }
            return true;
        //SYSREG_MAP(SYS_PMSWINC_EL0, SYS_IMP_APL_PMC3)
        case SYSREG_ISS(SYS_PMUSERENR_EL0):
            if(is_read) {
                regs[rt] = 0;
                HV_RUNTIME_TRACE("HV PMUv3 Redirect: mrs x%ld, PMUSERENR_EL0 = 0x%lx\n", rt, regs[rt]);
            }
            else {
                HV_RUNTIME_TRACE("HV PMUv3 Redirect (skipped write): msr PMUSERENR_EL0, x%ld = 0x%lx\n", rt, regs[rt]);
            }
           return true;
        // SYSREG_MAP(SYS_PMXEVCNTR_EL0, SYS_IMP_APL_PMC2)
        // case SYSREG_ISS(SYS_PMXEVTYPER_EL0):
        //     if(is_read) {
        //         printf("mrs(PMXEVTYPER_EL0)\n");
        //         regs[rt] = 0;
        //         int value = mrs(SYS_IMP_APL_PMCR1);
        //         if(value & GENMASK(23, 16)) {
        //             regs[rt] |= BIT(31); //privileged bit
        //         }
        //         if(value & GENMASK(15, 8)) {
        //             regs[rt] |= BIT(30); //user filter bit
        //         }
        //         regs[rt] |= (mrs(SYS_IMP_APL_PMESR0) & GENMASK(7, 0));
        //     }
        //     else {
        //         int val = mrs(SYS_IMP_APL_PMCR1);
        //         int event = mrs(SYS_IMP_APL_PMESR0) & GENMASK(7, 0);
        //         if(regs[rt] & PMEVTYPER_P) {
        //             printf("msr(PMEVTYPER0_EL0, 0x%08lx): enabling el1 counting of event\n", regs[rt]);
        //             val |= BIT(16);
        //         }
        //         if(regs[rt] & GENMASK(7, 0)) {
        //             printf("msr(PMEVTYPER0_EL0, 0x%08lx): setting event\n", regs[rt]);
        //             event |= regs[rt] & GENMASK(7, 0);
        //             msr(SYS_IMP_APL_PMESR0, event);
        //         }
        //         msr(SYS_IMP_APL_PMCR1, val);
        //     }
        //     return true;

        /* Outer Sharable TLB maintenance instructions */
        SYSREG_PASS(sys_reg(1, 0, 8, 1, 0)) // TLBI VMALLE1OS
        SYSREG_PASS(sys_reg(1, 0, 8, 1, 1)) // TLBI VAE1OS
        SYSREG_PASS(sys_reg(1, 0, 8, 1, 2)) // TLBI ASIDE1OS
        SYSREG_PASS(sys_reg(1, 0, 8, 5, 1)) // TLBI RVAE1OS

        case SYSREG_ISS(SYS_ACTLR_EL1):
            if (is_read) {
                if (cpu_features->actlr_el2)
                    regs[rt] = mrs(SYS_ACTLR_EL12);
                else
                    regs[rt] = mrs(SYS_IMP_APL_ACTLR_EL12);
            } else {
                if (cpu_features->actlr_el2)
                    msr(SYS_ACTLR_EL12, regs[rt]);
                else
                    msr(SYS_IMP_APL_ACTLR_EL12, regs[rt]);
            }
            return true;

        case SYSREG_ISS(SYS_IMP_APL_IPI_SR_EL1):
            if (is_read)
                regs[rt] = PERCPU(ipi_pending) ? IPI_SR_PENDING : 0;
            else if (regs[rt] & IPI_SR_PENDING)
                PERCPU(ipi_pending) = false;
            return true;

        /* shadow the interrupt mode and state flag */
        case SYSREG_ISS(SYS_IMP_APL_PMCR0):
            if (is_read) {
                u64 val = (mrs(SYS_IMP_APL_PMCR0) & ~PMCR0_IMODE_MASK) | PERCPU(pmc_irq_mode);
                regs[rt] = val | (PERCPU(pmc_pending) ? PMCR0_IACT : 0);
            } else {
                PERCPU(pmc_pending) = !!(regs[rt] & PMCR0_IACT);
                PERCPU(pmc_irq_mode) = regs[rt] & PMCR0_IMODE_MASK;
                msr(SYS_IMP_APL_PMCR0, regs[rt]);
            }
            return true;

        /*
         * Handle this one here because m1n1/Linux (will) use it for explicit cpuidle.
         * We can pass it through; going into deep sleep doesn't break the HV since we
         * don't do any wfis that assume otherwise in m1n1. However, don't het macOS
         * disable WFI ret (when going into systemwide sleep), since that breaks things.
         */
        case SYSREG_ISS(SYS_IMP_APL_CYC_OVRD):
            if (is_read) {
                regs[rt] = mrs(SYS_IMP_APL_CYC_OVRD);
            } else {
                if (regs[rt] & (CYC_OVRD_DISABLE_WFI_RET | CYC_OVRD_FIQ_MODE_MASK))
                    return false;
                msr(SYS_IMP_APL_CYC_OVRD, regs[rt]);
            }
            return true;
            /* clang-format off */
        /* IPI handling */
        SYSREG_PASS(SYS_IMP_APL_IPI_CR_EL1)
        /* M1RACLES reg, handle here due to silly 12.0 "mitigation" */
        case SYSREG_ISS(sys_reg(3, 5, 15, 10, 1)):
            if (is_read)
                regs[rt] = 0;
            return true;
    }

#ifdef ENABLE_VGIC_MODULE
    //
    // Catch-all for the ID register space (op0=3, op1=0, CRn=0, CRm=1..7). HCR_EL2.TID3
    // traps every encoding in this space, including ones that are unallocated or that we
    // do not model. The architecture defines those as RAZ, so return zero instead of
    // letting the guest take an unhandled trap. Without this the guest dies on the first
    // ID register that is not listed explicitly above.
    //
    if (FIELD_GET(ESR_ISS_MSR_OP0, iss) == 3 && FIELD_GET(ESR_ISS_MSR_OP1, iss) == 0 &&
        FIELD_GET(ESR_ISS_MSR_CRn, iss) == 0) {
        u64 crm = FIELD_GET(ESR_ISS_MSR_CRm, iss);
        if (crm >= 1 && crm <= 7) {
            if (is_read)
                regs[rt] = 0;
            return true;
        }
    }
#endif

    return false;
}

static bool hv_handle_smc(struct exc_info *ctx) {
    // Per-SMC print removed: the guest polls PSCI in bursts, and the UART time spent
    // printing is stolen from the guest's virtual counter (hv_add_time), which distorts
    // CNTVCT-based delays like KeStallExecutionProcessor. Re-enable only when chasing PSCI.
    //
    // SYSTEM_RESET (0x84000009) kills the machine before BDS ever attempts a boot, and
    // neither the watchdog nor a failed StartImage explains it. Print the guest's return
    // address so the caller can be identified: subtract the FD base (PcdFdBaseAddress,
    // 0x8510B4000) and look the offset up in Build/.../FV/FVMAIN.Fv.map.
    //
    if (ctx->regs[0] == 0x84000009 || ctx->regs[0] == 0x84000008) {
        //
        // ELR lands inside ResetSystemRuntimeDxe (it issues the SMC), which tells us
        // nothing about who asked for the reset. Walk the guest stack and print any word
        // that looks like a DXE code address, so the real caller can be matched against
        // the "Loading driver at 0x..." lines in the firmware's own log.
        //
        printf("PSCI RESET FROM: ELR_EL2=0x%lx x30=0x%lx sp=0x%lx spsr=0x%lx\n", ctx->elr,
               ctx->regs[30], ctx->sp[1], ctx->spsr);
        printf("PSCI RESET REGS: x0-x3 = %016lx %016lx %016lx %016lx\n", ctx->regs[0],
               ctx->regs[1], ctx->regs[2], ctx->regs[3]);
        printf("PSCI RESET REGS: x4-x7 = %016lx %016lx %016lx %016lx\n", ctx->regs[4],
               ctx->regs[5], ctx->regs[6], ctx->regs[7]);
        //
        // A reset now comes from the Windows kernel (bugcheck -> ResetSystem), not just
        // DXE. Print any stack word that looks like kernel text (0xfffff8...) or a DXE
        // image (0x9d...) so the call chain that led to the reset - and, we hope, the
        // bugcheck path - can be reconstructed. Flush before the reset actually fires.
        //
        u64 sp = ctx->sp[1];
        for (int i = 0; i < 96; i++) {
            u64 va = sp + i * 8;
            u64 pa = hv_translate(va, false, false, NULL);
            if (!pa)
                continue;
            u64 val = read64(pa);
            bool kern = (val >> 40) == 0xfffff8;
            bool dxe = val >= 0x9d0000000UL && val < 0x9e2000000UL;
            if (kern || dxe)
                printf("PSCI RESET STACK: [sp+0x%x] = 0x%lx%s\n", i * 8, val, kern ? " (kern)" : "");
        }
        iodev_console_flush();
    }
    bool handled_smc = hv_handle_psci_smc(ctx);
    return handled_smc;
}

static bool hv_handle_msr(struct exc_info *ctx, u64 iss)
{
    u64 reg = iss & (ESR_ISS_MSR_OP0 | ESR_ISS_MSR_OP2 | ESR_ISS_MSR_OP1 | ESR_ISS_MSR_CRn |
                     ESR_ISS_MSR_CRm);
    u64 rt = FIELD_GET(ESR_ISS_MSR_Rt, iss);
    bool is_read = iss & ESR_ISS_MSR_DIR;

    u64 *regs = ctx->regs;

    regs[31] = 0;

    switch (reg) {
        /* clang-format on */
        case SYSREG_ISS(SYS_IMP_APL_IPI_RR_LOCAL_EL1): {
            assert(!is_read);
            u64 mpidr = (regs[rt] & 0xff) | (mrs(MPIDR_EL1) & 0xffff00);
            for (int i = 0; i < MAX_CPUS; i++)
                if (mpidr == smp_get_mpidr(i)) {
                    pcpu[i].ipi_queued = true;
                    msr(SYS_IMP_APL_IPI_RR_LOCAL_EL1, regs[rt]);
                    return true;
                }
            return false;
        }
        case SYSREG_ISS(SYS_IMP_APL_IPI_RR_GLOBAL_EL1):
            assert(!is_read);
            u64 mpidr = (regs[rt] & 0xff) | ((regs[rt] & 0xff0000) >> 8);
            for (int i = 0; i < MAX_CPUS; i++) {
                if (mpidr == (smp_get_mpidr(i) & 0xffff)) {
                    pcpu[i].ipi_queued = true;
                    msr(SYS_IMP_APL_IPI_RR_GLOBAL_EL1, regs[rt]);
                    return true;
                }
            }
            return false;
#ifdef DEBUG_PMU_IRQ
        case SYSREG_ISS(SYS_IMP_APL_PMC0):
            if (is_read) {
                regs[rt] = mrs(SYS_IMP_APL_PMC0);
            } else {
                msr(SYS_IMP_APL_PMC0, regs[rt]);
                printf("msr(SYS_IMP_APL_PMC0, 0x%04lx_%08lx)\n", regs[rt] >> 32,
                       regs[rt] & 0xFFFFFFFF);
            }
            return true;
#endif
    }

    return false;
}

static void hv_get_context(struct exc_info *ctx)
{
    ctx->spsr = hv_get_spsr();
    ctx->elr = hv_get_elr();
    ctx->esr = hv_get_esr();
    ctx->far = hv_get_far();
    ctx->afsr1 = hv_get_afsr1();
    ctx->sp[0] = mrs(SP_EL0);
    ctx->sp[1] = mrs(SP_EL1);
    ctx->sp[2] = (u64)ctx;
    ctx->cpu_id = smp_id();
    ctx->mpidr = mrs(MPIDR_EL1);

    sysop("isb");
}

static void hv_exc_entry(void)
{
    // Enable SErrors in the HV, but only if not already pending
    if (!(mrs(ISR_EL1) & 0x100))
        sysop("msr daifclr, 4");

    __atomic_and_fetch(&hv_cpus_in_guest, ~BIT(smp_id()), __ATOMIC_ACQUIRE);
    spin_lock(&bhl);
    hv_wdt_breadcrumb('X');
    exc_entry_time = mrs(CNTPCT_EL0);
    /* disable PMU counters in the hypervisor */
    u64 pmcr0 = mrs(SYS_IMP_APL_PMCR0);
    PERCPU(exc_entry_pmcr0_cnt) = pmcr0 & PMCR0_CNT_MASK;
    msr(SYS_IMP_APL_PMCR0, pmcr0 & ~PMCR0_CNT_MASK);
}

static void hv_exc_exit(struct exc_info *ctx)
{
    hv_wdt_breadcrumb('x');
    hv_nvme_poll_irq();
    hv_update_fiq();
    /* reenable PMU counters */
    reg_set(SYS_IMP_APL_PMCR0, PERCPU(exc_entry_pmcr0_cnt));
    msr(CNTVOFF_EL2, stolen_time);
    spin_unlock(&bhl);
    /* Exactly one bounded scanout state-machine step.  This is deliberately
     * outside the global BHL and never runs from the broker MMIO handler. */
    hv_agx_scanout_service_run_once();
    hv_maybe_exit();
    __atomic_or_fetch(&hv_cpus_in_guest, BIT(smp_id()), __ATOMIC_ACQUIRE);

    hv_set_spsr(ctx->spsr);
    hv_set_elr(ctx->elr);
    msr(SP_EL0, ctx->sp[0]);
    msr(SP_EL1, ctx->sp[1]);
}

void hv_exc_sync(struct exc_info *ctx)
{
    hv_wdt_breadcrumb('S');
    hv_get_context(ctx);
    bool handled = false;
    bool advance_elr = true;
    u32 ec = FIELD_GET(ESR_EC, ctx->esr);

    switch (ec) {
        case ESR_EC_WFI:
            /*
             * Virtualize idle without holding bhl.  Let EL2 perform the real
             * physical WFI while no virtual IRQ is deliverable; a hardware
             * timer or IPI wakes this core, after which the guest resumes at
             * the following instruction.  Guest WFE remains untrapped because
             * Windows uses WFE/SEV as a synchronization primitive, not as a
             * vCPU halt.  If a WFE trap is nevertheless observed, resume it
             * defensively instead of sleeping in EL2.  If VI is already set,
             * never sleep: Windows must observe the pending interrupt first.
             */
            hv_update_fiq();
            switch (hv_wfx_trap_action(FIELD_GET(ESR_ISS, ctx->esr),
                                       !!(mrs(HCR_EL2) & HCR_VI))) {
                case HV_WFX_WAIT_WFI:
                    sysop("wfi");
                    break;
                case HV_WFX_RESUME_GUEST:
                    break;
            }
            ctx->elr = hv_wfx_resume_pc(ctx->elr);
            hv_set_elr(ctx->elr);
            /* Close the wait-to-return race before the guest can idle again. */
            hv_update_fiq();
            hv_wdt_breadcrumb('w');
            return;
        case ESR_EC_DABORT_LOWER: {
            bool nvme_matched = false;
            handled = hv_nvme_try_handle_dabort(ctx, &nvme_matched);
            if (nvme_matched) {
                if (!handled)
                    break;
                ctx->elr += 4;
                hv_set_elr(ctx->elr);
                hv_update_fiq();
                hv_wdt_breadcrumb('s');
                return;
            }
            break;
        }
        case ESR_EC_BRK: {
            /*
             * Windows uses BRK #0xf002/#0xf802 as an internal exception path.
             * Lowering it only touches the current CPU's EL1 exception state;
             * serializing thousands of these through bhl caused other vCPUs to
             * miss their clock deadlines under load.
             */
            u64 immediate = FIELD_GET(ESR_ISS, ctx->esr);
            struct hv_exception_lower_plan plan;
            if (hv_exception_lower_brk_is_fast(immediate) &&
                hv_exception_lower_plan(ctx->spsr, mrs(VBAR_EL12), &plan)) {
                msr(ELR_EL12, ctx->elr);
                msr(SPSR_EL12, ctx->spsr);
                msr(ESR_EL12, ctx->esr);
                msr(FAR_EL12, ctx->far);
                hv_set_spsr(plan.target_spsr);
                hv_set_elr(plan.target_elr);
                hv_update_fiq();
                hv_wdt_breadcrumb('s');
                return;
            }
            break;
        }
        case ESR_EC_MSR:
            hv_wdt_breadcrumb('m');
            handled = hv_handle_msr_unlocked(ctx, FIELD_GET(ESR_ISS, ctx->esr));
            break;
        //
        // for Blizzard/Avalanche and later - we need to explicitly check for SMC EC to handle SMCs
        //
        case ESR_EC_SMC:
            hv_wdt_breadcrumb('s');
            handled = hv_handle_smc(ctx);
            break;
        case ESR_EC_HVC:
            handled = hv_handle_guest_ipa_pa(ctx);
            break;
        case ESR_EC_IMPDEF:
            hv_wdt_breadcrumb('a');
            if(ctx->afsr1 == 0x1c00000) {
                /**
                 * m1n1_windows change: add SMC handling support.
                 * 
                 * right now the only reason a guest OS would fire an SMC is due to 
                 * requesting a PSCI service.
                */
                handled = hv_handle_smc(ctx);
                break;
            }
            switch (FIELD_GET(ESR_ISS, ctx->esr)) {
                case ESR_ISS_IMPDEF_MSR:
                    handled = hv_handle_msr_unlocked(ctx, ctx->afsr1);
                    break;
            }
            break;
    }

    if (handled) {
        hv_wdt_breadcrumb('#');
        /* HVC saves the following instruction, unlike trapped MSR/SMC. */
        if (FIELD_GET(ESR_EC, ctx->esr) != ESR_EC_HVC)
            ctx->elr += 4;
        hv_set_elr(ctx->elr);
        hv_update_fiq();
        hv_wdt_breadcrumb('s');
        return;
    }

    hv_exc_entry();

    switch (ec) {
        case ESR_EC_DABORT_LOWER:
            hv_wdt_breadcrumb('D');
            handled = hv_handle_dabort(ctx);
            break;
        case ESR_EC_MSR:
            hv_wdt_breadcrumb('M');
            handled = hv_handle_msr(ctx, FIELD_GET(ESR_ISS, ctx->esr));
            break;
        case ESR_EC_IMPDEF:
            hv_wdt_breadcrumb('A');
            switch (FIELD_GET(ESR_ISS, ctx->esr)) {
                case ESR_ISS_IMPDEF_MSR:
                    handled = hv_handle_msr(ctx, ctx->afsr1);
                    break;
            }
            break;
        case ESR_EC_BRK:
            /*
             * BRK #0x4242 is the assisted hypervisor call ABI and still needs a
             * proxy.  Every other BRK belongs to the guest.  In particular,
             * Windows uses BRK #0xf002 during normal kernel startup.
             */
            if (hv_exception_lower_brk_is_fast(FIELD_GET(ESR_ISS, ctx->esr))) {
                struct hv_exception_lower_plan plan;
                u64 guest_vbar = mrs(VBAR_EL12);

                if (hv_exception_lower_plan(ctx->spsr, guest_vbar, &plan)) {
                    msr(ELR_EL12, ctx->elr);
                    msr(SPSR_EL12, ctx->spsr);
                    msr(ESR_EL12, ctx->esr);
                    msr(FAR_EL12, ctx->far);
                    ctx->spsr = plan.target_spsr;
                    ctx->elr = plan.target_elr;
                    handled = true;
                    advance_elr = false;
                    printf("HV LOWER BRK: cpu=%d imm=0x%lx vector=0x%lx\n", smp_id(),
                           FIELD_GET(ESR_ISS, ctx->esr), ctx->elr);
                }
            }
            break;
    }

    if (handled) {
        hv_wdt_breadcrumb('+');
        if (advance_elr)
            ctx->elr += 4;
    } else {
        hv_wdt_breadcrumb('-');
        printf("HV UNHANDLED SYNC: cpu=%d ec=0x%x elr=0x%lx esr=0x%lx far=0x%lx "
               "afsr1=0x%lx spsr=0x%lx x0=0x%lx x1=0x%lx x18=0x%lx sp1=0x%lx\n",
               smp_id(), ec, ctx->elr, ctx->esr, ctx->far, ctx->afsr1, ctx->spsr,
               ctx->regs[0], ctx->regs[1], ctx->regs[18], ctx->sp[1]);
        iodev_console_kick();
        // VM code can forward a nested SError exception here
        if (FIELD_GET(ESR_EC, ctx->esr) == ESR_EC_SERROR)
            hv_exc_proxy(ctx, START_EXCEPTION_LOWER, EXC_SERROR, NULL);
        else
            hv_exc_proxy(ctx, START_EXCEPTION_LOWER, EXC_SYNC, NULL);
    }

    hv_exc_exit(ctx);
    hv_wdt_breadcrumb('s');
}

void hv_exc_irq(struct exc_info *ctx)
{
#ifdef ENABLE_VGIC_MODULE
    u32 reason = aic_ack();
    int irq = FIELD_GET(AIC_EVENT_NUM, reason);
    int type = FIELD_GET(AIC_EVENT_TYPE, reason);

    hv_diag_count_hw_irq(irq);

    u64 misr = mrs(ICH_MISR_EL2);
    u64 eisr = mrs(ICH_EISR_EL2);

    if(irq == 0 || type == 0){//maintenance IRQ?
        if(misr != 0 && eisr != 0){
            for(int lr = 0; lr < hv_vgic3_num_lrs(); lr++){
                if(eisr & BIT(lr)){
                    u64 lr_val = hv_vgic3_read_lr(lr);
                    u64 intd = (lr_val >> ICH_LR_VIRTUAL_SHIFT) & ICH_LR_VIRTUAL_MASK;
                    hv_vgic3_write_lr(lr, 0);
                    const struct hv_irq_route *route = hv_irq_route_from_vintid(intd);
                    if (route)
                        aic_set_mask(route->hw_irq, false);
                }
            }
        }

        while (hv_vgic3_get_free_lr() != -1) {
            virq_t pending;
            if (!virq_queue_pop(&PERCPU(timer_queue), &pending))
                break;
            hv_vgic3_inject_irq(pending.vintid, pending.priority,
                                pending.active, pending.pending,
                                pending.hw_status, pending.hw_irq);
        }
        hv_vgic3_drain_sgis();
        hv_vgic3_drain_irq_queue();
        /*
         * The maintenance path is the one LR mutation that does not pass
         * through inject/IAR/EOI, all of which already update HCR.VI.  Recompute
         * here after clearing EISR-selected LRs and draining deferred work.
         * Doing this at every serialized exception exit scans all LRs for every
         * trapped ICC access and causes cumulative Windows DPC latency.
         */
        hv_vgic3_update_vi();
        sysop("isb");
        return;
    }

    const struct hv_irq_route *route = hv_irq_route_from_hw(irq);
    u32 vintid;
    if (!hv_irq_route_resolve_incoming(irq, hv_pci_intx_irq(), &vintid)) {
        static u32 synthetic_collision_logs;
        if (synthetic_collision_logs++ < 16)
            printf("HV: dropping AIC IRQ %u: collides with synthetic NVMe vINTID %u\n",
                   irq, hv_pci_intx_irq());
        aic_set_mask(irq, true);
        return;
    }
    if (route) {
        static u32 routed_irq_trace_count;
        if (routed_irq_trace_count < 16)
            HV_RUNTIME_VERBOSE_TRACE(
                "HV: HW IRQ route AIC=%u vINTID=%u type=%u reason=0x%x count=%u\n",
                route->hw_irq, route->vintid, type, reason, routed_irq_trace_count + 1);
        routed_irq_trace_count++;
    }
    if (route && route->level)
        aic_set_mask(route->hw_irq, true);

    if(hv_vgic3_get_free_lr() != -1){
        hv_vgic3_inject_irq(
            vintid,                         //vintid
            hv_vgic3_get_priority(vintid),  //priority
            false,                       //active
            true,                        //pending
            false,                       //hw_status
            0                            //hw_irq
        );
    }
    else{
        virq_t pending = {
            .vintid = vintid,
            .priority = hv_vgic3_get_priority(vintid),
            .active = false, 
            .pending = true,
            .hw_status = false,
            .hw_irq = 0,
        };
        if (!virq_queue_push(&PERCPU(irq_queue), &pending) && route && route->level)
            aic_set_mask(route->hw_irq, false);
    }
#else
    hv_wdt_breadcrumb('I');
    hv_get_context(ctx);
    hv_exc_entry();
    hv_exc_proxy(ctx, START_EXCEPTION_LOWER, EXC_IRQ, NULL);
    hv_exc_exit(ctx);
    hv_wdt_breadcrumb('i');
#endif
}

void hv_exc_fiq(struct exc_info *ctx)
{
    bool tick = false;

    hv_maybe_exit();

    //
    // TODO: inject the FIQ to the guest as an IRQ if vGIC is enabled.
    //

    //
    // Detect our own expired tick by testing the bits, NOT by comparing the whole register.
    //
    // The tick is masked here and only re-armed later, and only when `tick` is true. If the
    // handler ever leaves without reaching hv_arm_tick() - the budget note below is explicit
    // that extra work in this path does exactly that - the register is left as
    // ISTATUS|IMASK|ENABLE. An exact-equality test never matches that value again, so `tick`
    // stays false forever, the EL2 timer is never re-armed, and every EL2 tick stops: the
    // heartbeat and PC samples freeze while the hardware is still fine. Matching on the bits
    // lets the very next FIQ from any other source (the guest timer keeps running) re-arm it
    // and recover instead of wedging permanently.
    //
    u64 cntp_ctl = mrs(CNTP_CTL_EL0);
    if ((cntp_ctl & (CNTx_CTL_ISTATUS | CNTx_CTL_ENABLE)) ==
        (CNTx_CTL_ISTATUS | CNTx_CTL_ENABLE)) {
        HV_RUNTIME_DIAG_COUNT(PERCPU(host_tick_fires));
        //
        // Re-arm immediately instead of masking and deferring the re-arm to the end of the
        // handler. Masking first was the deadlock: any path that leaves before hv_arm_tick()
        // strands the timer as ISTATUS|IMASK|ENABLE, and once EL2's own tick is gone the only
        // remaining wake-up is the guest timer - so if the guest also stops rearming, every
        // FIQ source is silent and the hypervisor goes quiet with the hardware still healthy
        // (heartbeat, PC samples, USB and vUART service all stop together). Re-arming here
        // costs nothing: the next tick is still a full interval away, the same budget the
        // handler already had.
        //
        int tick_cpu = hv_pinned_cpu;
        if (tick_cpu == -1)
            tick_cpu = boot_cpu_idx;
        hv_arm_tick(smp_id() != tick_cpu);
        tick = true;
    }

    //
    // Heartbeat. Counting only - printing here is not safe, this runs before
    // hv_get_context()/hv_exc_entry() and a printf at this point silences the whole FIQ
    // path, which is how the previous attempt at this lost both instruments at once.
    //
    hv_fiq_count++;
    if (tick)
        hv_fiq_ticks++;

    int interruptible_cpu = hv_pinned_cpu;
    if (interruptible_cpu == -1)
        interruptible_cpu = boot_cpu_idx;

    bool secondary_fast =
        hv_fiq_secondary_fast_eligible(
            smp_id(), interruptible_cpu, hv_want_cpu,
            __atomic_load_n(&hv_rendezvous_active, __ATOMIC_ACQUIRE));
    bool snapshot_sampled = false;
    if (secondary_fast) {
        /*
         * Service the two high-rate, per-CPU sources before taking bhl.  The old
         * path made every guest clock interrupt on all eight vCPUs wait for one
         * global lock.  Under load a secondary could wait in EL2 long enough for
         * Windows to report CLOCK_WATCHDOG_TIMEOUT.
         */
        hv_update_fiq();
        hv_handle_local_ipi();
        /* HCR.VI is a software-cached output.  The accepted timer latch may
         * retain a live Pending LR without passing through an inject/repend
         * helper, so rebuild VI from the final local LR/VMCR state before the
         * fast-return decision can put an idle vCPU back to sleep. */
        hv_vgic3_update_vi();
        sysop("isb");
        hv_watchdog_snapshot_tick(ctx);
        snapshot_sampled = true;

        /* Preserve the accepted baseline exit rule: a secondary may use the
         * abbreviated path only when neither a physical FIQ nor HCR.VI remains.
         * This prevents returning to an idle guest before its clock IRQ is
         * observed, without adding another LR scan or recovery timer. */
        if (hv_fiq_secondary_fast_complete(true, !!(mrs(ISR_EL1) & 0x40),
                                           !!(mrs(HCR_EL2) & HCR_VI))) {
            hv_arm_tick(true);
            return;
        }
    }

    // Slow (single threaded) path
    hv_wdt_breadcrumb('F');
    hv_get_context(ctx);
    hv_exc_entry();
    if (!snapshot_sampled)
        hv_watchdog_snapshot_tick(ctx);

    //
    // The PC samples stop the moment the Windows kernel is reached, and so does the
    // hypervisor's servicing of USB and the vUART. Two candidates: EL2 stops taking FIQs
    // at all, or FIQs keep arriving but hv_tick is no longer reached - it runs only when
    // the EL2 physical timer fired and this is the interruptible CPU. The counts separate
    // those, and the registers printed alongside decide between them.
    //
    if (hv_runtime_diag_verbose_enabled() && (hv_fiq_count % 20000) == 0) {
        printf("HV FIQ: total=%lu ticks=%lu cpu=%d boot=%d pinned=%d vm_tmr=0x%lx\n",
               hv_fiq_count, hv_fiq_ticks, smp_id(), boot_cpu_idx, hv_pinned_cpu,
               mrs(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2));

#ifdef ENABLE_VGIC_MODULE
        // A frozen Windows desktop with every vCPU in its idle loop can still leave EL2's
        // own tick alive.  Capture the complete per-CPU guest-timer delivery state here:
        // comparator, one-shot latch, LR state, PPI enable and deferred queue depth.  This
        // is deliberately tied to the existing rate-limited heartbeat; printing on every
        // timer FIQ perturbs the exact path being measured.
        int diag_cpu = smp_id();
        int p_lr = -1;
        int v_lr = -1;
        u64 p_lr_val = 0;
        u64 v_lr_val = 0;
        for (int lr = 0; lr < hv_vgic3_num_lrs(); lr++) {
            u64 lr_val = hv_vgic3_read_lr(lr);
            u32 intid = (lr_val >> ICH_LR_VIRTUAL_SHIFT) & ICH_LR_VIRTUAL_MASK;
            if (intid == 17 &&
                ((lr_val >> ICH_LR_STATE_SHIFT) & ICH_LR_STATE_MASK)) {
                p_lr = lr;
                p_lr_val = lr_val;
            }
            if (intid == 18 &&
                ((lr_val >> ICH_LR_STATE_SHIFT) & ICH_LR_STATE_MASK)) {
                v_lr = lr;
                v_lr_val = lr_val;
            }
        }
        u32 tq_head = __atomic_load_n(&PERCPU(timer_queue).head, __ATOMIC_ACQUIRE);
        u32 tq_tail = __atomic_load_n(&PERCPU(timer_queue).tail, __ATOMIC_ACQUIRE);
        u64 p_now = mrs(CNTPCT_EL0);
        u64 v_now = mrs(CNTVCT_EL0);
        printf("HV TIMER: cpu=%d pctl=0x%lx pdelta=%ld pinj=%d pen=%d plr=%d "
               "pstate=%lu vctl=0x%lx vdelta=%ld vinj=%d ven=%d vlr=%d "
               "vstate=%lu tq=%u\n",
               diag_cpu, mrs(CNTP_CTL_EL02), (s64)(mrs(CNTP_CVAL_EL02) - p_now),
               timer_p_injected[diag_cpu], hv_vgic3_irq_enabled(17), p_lr,
               (p_lr_val >> ICH_LR_STATE_SHIFT) & ICH_LR_STATE_MASK,
               mrs(CNTV_CTL_EL02), (s64)(mrs(CNTV_CVAL_EL02) - v_now),
               timer_v_injected[diag_cpu], hv_vgic3_irq_enabled(18), v_lr,
               (v_lr_val >> ICH_LR_STATE_SHIFT) & ICH_LR_STATE_MASK,
               tq_head - tq_tail);

        struct hv_sgi_diag_snapshot sgi = {0};
        u64 sgi_lrs[HV_VGIC_DIAG_LR_COUNT] = {0};
        for (int lr = 0; lr < hv_vgic3_num_lrs() && lr < HV_VGIC_DIAG_LR_COUNT; lr++)
            sgi_lrs[lr] = hv_vgic3_read_lr(lr);
        int sgi_lr = hv_vgic_diag_find_live_intid(sgi_lrs, 0);
        u64 sgi_lr_value = sgi_lr < 0 ? 0 : sgi_lrs[sgi_lr];
        hv_sgi_diag_snapshot(&PERCPU(sgi_diag), &sgi);
        u64 diag_vmcr = mrs(ICH_VMCR_EL2);
        printf("HV SGI DIAG: cpu=%d pend=0x%x lr=%d state=%lu q=%lu ipi=%lu "
               "drain=%lu inject=%lu repend=%lu no_lr=%lu iar=%lu eoi=%lu ap_eoi=%lu "
               "sgi_prio=0x%x timer_prio=0x%x pmr=0x%lx rpr=0x%x vmcr=0x%lx "
               "last_sgi=%u<-%u last_iar=%u/%ld last_eoi=%u/%ld\n",
               diag_cpu, __atomic_load_n(&PERCPU(sgi_pending_mask), __ATOMIC_ACQUIRE),
               sgi_lr, (sgi_lr_value >> ICH_LR_STATE_SHIFT) & ICH_LR_STATE_MASK,
               sgi.queued, sgi.ipi_received, sgi.drained, sgi.injected, sgi.repended,
               sgi.no_lr, sgi.iars, sgi.eois, sgi.eoi_active_pending,
               hv_vgic3_get_priority(0), hv_vgic3_get_priority(18),
               (diag_vmcr >> 24) & 0xff, hv_vgic3_running_priority(), diag_vmcr,
               PERCPU(last_sgi_intid), PERCPU(last_sgi_from), PERCPU(last_iar_intid),
               PERCPU(last_iar_tick) ? (s64)(p_now - PERCPU(last_iar_tick)) : -1,
               PERCPU(last_eoi_intid),
               PERCPU(last_eoi_tick) ? (s64)(p_now - PERCPU(last_eoi_tick)) : -1);
#endif
        hv_trace_j313_xhci_tick();
    }

    /*
     * A CLOCK_WATCHDOG_TIMEOUT is often preceded by a long freeze without a
     * bugcheck, so waiting for KiBugCheckData loses the state that caused it.
     * In monitor builds, let the interruptible CPU periodically print the
     * latest lockless snapshot published by every vCPU.  The subtraction-based
     * cadence deliberately tolerates the shared FIQ counter skipping values.
     */
    if (hv_runtime_diag_verbose_enabled() && smp_id() == interruptible_cpu &&
        hv_watchdog_snapshot_dump_due(hv_fiq_count, watchdog_last_periodic_dump,
                                      2000000)) {
        watchdog_last_periodic_dump = hv_fiq_count;
        printf("HV WATCHDOG PERIODIC: fiq=%lu\n", hv_fiq_count);
        hv_watchdog_snapshot_dump();
    }

    // Only poll for HV events in the interruptible CPU
    if (tick) {
        //
        // Diagnostics first, on EVERY core. hv_tick() below stays bound to the interruptible
        // CPU because it also does functional work; running that everywhere would create a new
        // SMP bug rather than restore visibility. hv_diag_tick() only reads and prints.
        //
        hv_percpu_diag_tick(ctx);

        if (smp_id() == interruptible_cpu) {
            hv_tick(ctx);
            hv_arm_tick(false);
        } else {
            hv_arm_tick(true);
        }
    }

    if (mrs(CNTV_CTL_EL0) == (CNTx_CTL_ISTATUS | CNTx_CTL_ENABLE)) {
        msr(CNTV_CTL_EL0, CNTx_CTL_ISTATUS | CNTx_CTL_IMASK | CNTx_CTL_ENABLE);
        hv_exc_proxy(ctx, START_HV, HV_VTIMER, NULL);
    }

    u64 reg = mrs(SYS_IMP_APL_PMCR0);
    if ((reg & (PMCR0_IMODE_MASK | PMCR0_IACT)) == (PMCR0_IMODE_FIQ | PMCR0_IACT)) {
#ifdef DEBUG_PMU_IRQ
        printf("[FIQ] PMC IRQ, masking and delivering to the guest\n");
#endif
        reg_clr(SYS_IMP_APL_PMCR0, PMCR0_IACT | PMCR0_IMODE_MASK);
        PERCPU(pmc_pending) = true;
    }

    reg = mrs(SYS_IMP_APL_UPMCR0);
    if (FIELD_GET(UPMCR0_IMODE_T8020, reg) == UPMCR0_IMODE_FIQ &&
        (mrs(SYS_IMP_APL_UPMSR) & UPMSR_IACT)) {
        printf("[FIQ] UPMC IRQ, masking");
        reg_clr(SYS_IMP_APL_UPMCR0, UPMCR0_IMODE_T8020);
        hv_exc_proxy(ctx, START_EXCEPTION_LOWER, EXC_FIQ, NULL);
    }

    hv_handle_local_ipi();

    hv_maybe_switch_cpu(ctx, START_HV, HV_CPU_SWITCH, NULL);

    // Handles guest timers
    hv_exc_exit(ctx);
    hv_wdt_breadcrumb('f');
}

void hv_exc_serr(struct exc_info *ctx)
{
    hv_wdt_breadcrumb('E');
    hv_get_context(ctx);
    hv_exc_entry();
    hv_exc_proxy(ctx, START_EXCEPTION_LOWER, EXC_SERROR, NULL);
    hv_exc_exit(ctx);
    hv_wdt_breadcrumb('e');
}

/**
 * Copyright (c) 2025, amarioguy (AppleWOA authors).
 * 
 * Module Name:
 *     hv_vgic.c
 * 
 * Abstract:
 *     The vGIC virtual device code for the m1n1 hypervisor.
 * 
 * 
 * Environment:
 *     m1n1 in hypervisor mode.
 * 
 * License:
 *     SPDX-License-Identifier: (BSD-2-Clause-Patent OR MIT)
 * 
 *     Inspiration borrowed from the KVM vGIC driver in the Linux source tree. Original copyright notice below.
 *     
*/

#include "hv.h"
#include "hv_runtime_diag.h"
#include "hv_diag.h"
#include "hv_irq_routes.h"
#include "hv_xhci_handoff.h"
#include "hv_vgic.h"
#include "hv_vgic_diag.h"
#include "hv_vgic_redist.h"
#include "assert.h"
#include "cpu_regs.h"
#include "display.h"
#include "dart.h"
#include "memory.h"
#include "pcie.h"
#include "smp.h"
#include "string.h"
#include "usb.h"
#include "utils.h"
#include "aic.h"
#include "malloc.h"
#include "heapblock.h"
#include "smp.h"
#include "string.h"
#include "types.h"
#include "uartproxy.h"

#define J313_XHCI1_BASE 0x502280000ULL
#define J313_XHCI_LOW_DMA_END 0x40000000ULL
#define J313_XHCI_DART_IOVA_END 0x100000000ULL
#define J313_DART_USB1_0_BASE 0x502f00000ULL
#define J313_DART_USB1_1_BASE 0x502f80000ULL

static u32 j313_xhci_tick_trace_budget;
static struct hv_xhci_dma_trace j313_xhci_dma_trace;
static struct hv_xhci_cap_state j313_xhci_caps;
static u32 j313_xhci_erdp_trace_budget = 16;
static u32 j313_xhci_erdp_last_upper = 0xffffffff;
static u32 irq_route_transition_count;

struct hv_xhci_stage2_view {
    u64 direct_end;
    u64 high_window_base;
};

static u64 hv_xhci_stage2_translate(u64 ipa, void *opaque)
{
    struct hv_xhci_stage2_view *view = opaque;
    u64 guest_ipa = hv_xhci_dma_guest_ipa(ipa, view->direct_end, view->high_window_base);

    return hv_ipa_to_pa(guest_ipa);
}

static bool hv_xhci_map_stage2_span(dart_dev_t *dart, u64 start, u64 end, u64 *pages,
                                    u32 *runs, struct hv_xhci_stage2_view *view)
{
    struct hv_xhci_dma_run run;
    u64 cursor = start;

    while (hv_xhci_dma_next_run(cursor, end, hv_xhci_stage2_translate, view, &run)) {
        if (dart_map(dart, run.iova, (void *)(uintptr_t)run.paddr, run.size)) {
            printf("HV: xHCI DART map failed IOVA=0x%lx PA=0x%lx size=0x%lx\n", run.iova,
                   run.paddr, run.size);
            return false;
        }
        *pages += run.size / HV_XHCI_DMA_PAGE_SIZE;
        (*runs)++;
        cursor = run.next_iova;
    }

    return true;
}

static bool hv_prepare_j313_xhci_darts(void)
{
    static int state;
    dart_dev_t *darts[2];
    struct hv_xhci_stage2_view view = {
        .direct_end = J313_XHCI_LOW_DMA_END,
        .high_window_base = ram_base & ~0xffffffffULL,
    };
    u64 pages = 0;
    u32 runs = 0;

    if (state)
        return state > 0;
    state = -1;

    /*
     * Windows sees the LOW_MEM stage-2 alias as ordinary physical RAM.  A bypassed DART
     * cannot see that translation: it would send e.g. IOVA 0x01000000 to physical
     * 0x01000000 instead of its 0x8a... backing.  Program both DWC3 DMA paths with the
     * same IPA -> PA view as stage-2 before USBXHCI starts the controller. T8103
     * DART has a 32-bit input address space, so high guest addresses are exposed
     * through their low-32-bit aliases (e.g. 0x8f392f000 at IOVA 0xf392f000).
     */
    darts[0] = dart_init_adt("/arm-io/dart-usb1", 0, 0, false);
    darts[1] = dart_init_adt("/arm-io/dart-usb1", 1, 1, false);
    if (!darts[0] || !darts[1]) {
        printf("HV: xHCI DART init failed dart0=%p dart1=%p\n", darts[0], darts[1]);
        return false;
    }

    for (u32 i = 0; i < ARRAY_SIZE(darts); i++) {
        if (!hv_xhci_map_stage2_span(darts[i], 0, J313_XHCI_DART_IOVA_END, &pages, &runs,
                                     &view))
            return false;
    }

    state = 1;
    printf("HV: xHCI DART 32-bit stage-2 view ready pages=%lu runs=%u high_base=0x%lx\n",
           pages, runs, view.high_window_base);
    return true;
}

static bool handle_j313_xhci_mmio(struct exc_info *ctx, u64 addr, u64 *val, bool write,
                                  int width)
{
    enum hv_xhci_dma_reg reg = HV_XHCI_DMA_REG_NONE;
    u64 offset = addr - J313_XHCI1_BASE;

    if (write && hv_xhci_cap_state_ready(&j313_xhci_caps))
        reg = hv_xhci_dma_trace_write(&j313_xhci_dma_trace, j313_xhci_caps.caplen,
                                      j313_xhci_caps.rtsoff, offset, *val, width);

    /* Install the guest IPA -> PA DART view before the first DMA pointer reaches xHCI. */
    if (write && reg != HV_XHCI_DMA_REG_NONE && !hv_prepare_j313_xhci_darts())
        return false;

    if (!hv_pa_rw(ctx, addr, val, write, width))
        return false;

    if (!write) {
        u64 hardware_value = *val;
        bool caps_were_ready = hv_xhci_cap_state_ready(&j313_xhci_caps);

        hv_xhci_cap_state_observe_read(&j313_xhci_caps, offset, hardware_value, width);
        if (!caps_were_ready && hv_xhci_cap_state_ready(&j313_xhci_caps))
            printf("HV: xHCI capabilities discovered caplen=0x%x rtsoff=0x%x\n",
                   j313_xhci_caps.caplen, j313_xhci_caps.rtsoff);

        *val = hv_xhci_cap_read_for_guest(offset, *val, width);
        if (*val != hardware_value) {
            static bool logged;
            if (!logged) {
                printf("HV: xHCI guest capability AC64 masked hardware=0x%lx guest=0x%lx "
                       "offset=0x%lx width=%u\n",
                       hardware_value, *val, offset, 1U << width);
                logged = true;
            }
        }
        return true;
    }

    if (reg != HV_XHCI_DMA_REG_NONE) {
        u64 guest_value;
        u64 reg_offset;
        bool log = true;

        switch (reg) {
            case HV_XHCI_DMA_REG_CRCR:
                guest_value = j313_xhci_dma_trace.crcr;
                reg_offset = j313_xhci_caps.caplen + 0x18;
                break;
            case HV_XHCI_DMA_REG_DCBAAP:
                guest_value = j313_xhci_dma_trace.dcbaap;
                reg_offset = j313_xhci_caps.caplen + 0x30;
                break;
            case HV_XHCI_DMA_REG_ERSTBA:
                guest_value = j313_xhci_dma_trace.erstba;
                reg_offset = j313_xhci_caps.rtsoff + 0x30;
                break;
            case HV_XHCI_DMA_REG_ERDP: {
                guest_value = j313_xhci_dma_trace.erdp;
                reg_offset = j313_xhci_caps.rtsoff + 0x38;
                u32 upper = guest_value >> 32;
                log = upper != j313_xhci_erdp_last_upper || j313_xhci_erdp_trace_budget;
                j313_xhci_erdp_last_upper = upper;
                if (j313_xhci_erdp_trace_budget)
                    j313_xhci_erdp_trace_budget--;
                break;
            }
            default:
                return true;
        }

        if (log)
            HV_RUNTIME_VERBOSE_TRACE(
                "HV: xHCI guest W %s+0x%lx width=%u fragment=0x%lx guest=0x%lx hw=0x%lx\n",
                hv_xhci_dma_reg_name(reg), (u64)(offset - reg_offset), 1U << width, *val,
                guest_value, read64(J313_XHCI1_BASE + reg_offset));
    }

    return true;
}

bool hv_vgic_rearm_j313_xhci_trace(void)
{
    memset(&j313_xhci_caps, 0, sizeof(j313_xhci_caps));
    memset(&j313_xhci_dma_trace, 0, sizeof(j313_xhci_dma_trace));

    int ret = hv_map_hook(J313_XHCI1_BASE, handle_j313_xhci_mmio, HV_XHCI_DMA_PAGE_SIZE);
    printf("HV: xHCI MMIO trace %s lazily base=0x%lx size=0x%lx\n",
           ret ? "FAILED" : "armed", (u64)J313_XHCI1_BASE,
           (u64)HV_XHCI_DMA_PAGE_SIZE);
    return ret == 0;
}

static void hv_trace_j313_xhci(const char *where)
{
    u32 cap = read32(J313_XHCI1_BASE);
    u32 caplen = cap & 0xff;
    u32 dboff = read32(J313_XHCI1_BASE + 0x14) & ~3U;
    u32 rtsoff = read32(J313_XHCI1_BASE + 0x18) & ~0x1fU;
    u64 op = J313_XHCI1_BASE + caplen;
    u64 ir0 = J313_XHCI1_BASE + rtsoff + 0x20;

    printf("HV: xHCI %s cap=0x%x caplen=0x%x dboff=0x%x rtsoff=0x%x "
           "USBCMD=0x%x USBSTS=0x%x CRCR=0x%lx DCBAAP=0x%lx CONFIG=0x%x "
           "IMAN0=0x%x IMOD0=0x%x ERSTSZ0=0x%x ERSTBA=0x%lx ERDP=0x%lx "
           "DART0_ERR=0x%x@0x%x%08x/TCR0=0x%x DART1_ERR=0x%x@0x%x%08x/TCR1=0x%x\n",
           where, cap, caplen, dboff, rtsoff, read32(op), read32(op + 4),
           read64(op + 0x18), read64(op + 0x30), read32(op + 0x38), read32(ir0),
           read32(ir0 + 4), read32(ir0 + 8), read64(ir0 + 0x10), read64(ir0 + 0x18),
           read32(J313_DART_USB1_0_BASE + 0x40), read32(J313_DART_USB1_0_BASE + 0x54),
           read32(J313_DART_USB1_0_BASE + 0x50), read32(J313_DART_USB1_0_BASE + 0x100),
           read32(J313_DART_USB1_1_BASE + 0x40), read32(J313_DART_USB1_1_BASE + 0x54),
           read32(J313_DART_USB1_1_BASE + 0x50), read32(J313_DART_USB1_1_BASE + 0x104));
}

void hv_trace_j313_xhci_tick(void)
{
    if (!j313_xhci_tick_trace_budget)
        return;
    j313_xhci_tick_trace_budget--;
    hv_trace_j313_xhci("tick");
}

static void hv_prepare_j313_xhci_handoff(void)
{
    u32 caplen = read32(J313_XHCI1_BASE) & 0xff;
    u32 rtsoff = read32(J313_XHCI1_BASE + 0x18) & ~0x1fU;
    u64 op = J313_XHCI1_BASE + caplen;
    u64 ir0 = J313_XHCI1_BASE + rtsoff + 0x20;
    u32 usbcmd = read32(op);
    u32 usbsts = read32(op + 4);
    u32 iman = read32(ir0);
    struct hv_xhci_handoff_clear clear;

    bool dma_programmed = j313_xhci_dma_trace.crcr || j313_xhci_dma_trace.dcbaap ||
                          j313_xhci_dma_trace.erstba || j313_xhci_dma_trace.erdp;
    if (!hv_xhci_handoff_clear_plan(usbcmd, usbsts, iman, dma_programmed, &clear))
        return;

    if (clear.reset) {
        write32(op, usbcmd | BIT(1));
        sysop("dsb sy");
        if (poll32(op, BIT(1), 0, 1000000))
            printf("HV: xHCI handoff reset timed out USBCMD=0x%x USBSTS=0x%x\n", read32(op),
                   read32(op + 4));
        else
            printf("HV: xHCI handoff reset complete USBCMD=0x%x USBSTS=0x%x\n", read32(op),
                   read32(op + 4));
    }
    if (clear.usbsts_w1c)
        write32(op + 4, clear.usbsts_w1c);
    if (clear.iman_w1c)
        write32(ir0, clear.iman_w1c);
    sysop("dsb sy");
    printf("HV: xHCI handoff cleared USBSTS=0x%x IMAN0=0x%x\n", clear.usbsts_w1c,
           clear.iman_w1c);
}

/**
 * General idea of how this should work:
 * 
 * Apple Silicon chips since the M1 implement the GIC CPU interface registers in hardware, meaning
 * only the distributor, the core specific redistributors, and (potentially) an ITS need to be emulated by m1n1.
 * 
 * As such, this file implements most of the code needed to make this possible. The emulated distributor/redistributors
 * will need to meet a few constraints (namely it's limited by what the GIC CPU interface supports)
 * 
 * Apple's vGIC CPU interface has the following characteristics (on M1 and M2):
 * - 32 levels of virtual priority and preemption priority (5 preemption/priority bits)
 * - 16 bits of virtual interrupt ID bits (meaning up to 65535 interrupts are supported theoretically, however practically limited by the number of IRQs the AIC supports)
 * - supports guest-generated SEIs upon writing to GIC registers in a bad way 
 *   (note that an errata here exists on pre-M3 SoCs that can result in a host SError - we implement special handling for this.)
 * - 3 level affinity (aff2/aff1/aff0 valid, aff3 invalid/reserved as 0)
 * - legacy operation is not supported (ICC_SRE_EL2.SRE is reserved, set to 1) (no GICv2 operations)
 * - TDIR bit is supported (FEAT_GICv3_TDIR)
 * - extended SPI and PPI ranges are *not* supported on M1/M2 (and their Pro counterparts, even if the SoC itself has > 16 cores)
 * - 8 list registers
 * - direct injection of virtual interrupts are not supported (not a GICv4, and by extension, no NMIs supported)
 * - IRQ/FIQ bypass are not supported
 * 
 * 
 * The mappings are different for platforms with 36-bit vs 42-bit physical addressing, with 36-bit platforms tentatively
 * having the distributor being mapped to 0xF00000000, redistributors at offset +0x10000000
 * and 42-bit platforms having the distributor at 0x5000000000, redistributors at offset +0x100000000
 * 
 * A major note about processor affinities: since AICv2 platforms don't support setting core affinities easily,
 * the tentative solution is to do routing to any virtual CPU once we receive an IRQ, we can't assume
 * that the core that got the IRQ is the one that needs to be signaled. (for FIQs, because they're core specific,
 * we'll know which core needs to be signaled in those cases.)
 * 
 */
#ifdef ENABLE_VGIC_MODULE
#define DIST_BASE_36_BIT 0xF00000000
#define REDIST_BASE_36_BIT 0xF10000000
#define DIST_BASE_42_BIT 0x5000000000
#define REDIST_BASE_42_BIT 0x5100000000
//
// This is tentative - depends on if direct MSIs or ITS translated IRQs end up being easier to implement.
//
#define ITS_BASE_36_BIT 0xF20000000
#define ITS_BASE_42_BIT 0x5200000000

// Per-register tracing is ~5000 lines per boot and slows the guest by an order of
// magnitude, which distorts every timing measurement. Turn on only when debugging the
// distributor itself.
#define ENABLE_VGIC_LOGGING 0

#if ENABLE_VGIC_LOGGING
#define vgic_log(...) printf(__VA_ARGS__)
#else
#define vgic_log(...)                                                                               \
    do {                                                                                           \
    } while (0)
#endif


vgicv3_dist *distributor;
vgicv3_vcpu_redist *redistributors;
vgicv3_its *interrupt_translation_service;
static u64 dist_base, redist_base, its_base;
static u16 num_cpus;
static bool vgic_inited;
static u64 igrpen1;

static bool handle_vgic_its_access(struct exc_info *ctx, u64 addr, u64 *val, bool write, int width)
{
    u64 relative_addr;
    bool register_handled;
    bool unimplemented_reg_accessed;
    u32 reg_num;
    relative_addr = addr - its_base;
    register_handled = false;
    unimplemented_reg_accessed = false;
    reg_num = 0;
    if(write) {
        switch(relative_addr) {
            case GITS_CTLR:
                interrupt_translation_service->its_ctl_region.gits_ctl_reg = *val;
                register_handled = true;
                break;
            case GITS_BASER0 ... GITS_BASER7:
                reg_num = (relative_addr - GITS_BASER0) / 8;
                interrupt_translation_service->its_ctl_region.gits_baser[reg_num] = *val;
                register_handled = true;
                break;
            default:
                //
                // we're dealing with a register that is banked n times, we need to get to the if statements.
                //
                break;
        }

    }
    else {
        switch(relative_addr) {
            case GITS_CTLR:
                *val = interrupt_translation_service->its_ctl_region.gits_ctl_reg;
                register_handled = true;
                break;
            case GITS_BASER0 ... GITS_BASER7:
                reg_num = (relative_addr - GITS_BASER0) / 8;
                *val = interrupt_translation_service->its_ctl_region.gits_baser[reg_num];
                register_handled = true;
                break;
            default:
                //
                // we're dealing with a register that is banked n times, we need to get to the if statements.
                //
                break;
        }
    }

    vgic_log("HV vGIC DEBUG [INFO] [ITS]: 0x%llx = 0x%llx ", relative_addr, *val);
    if(write) {
        vgic_log("[Written]");
    }
    else {
        vgic_log("[Read]");
    }
    if(unimplemented_reg_accessed) {
        vgic_log("[Unimplemented]\n");
    }
    else {
        vgic_log("\n");
    }
    return register_handled;
}


//
// Description:
//   the vGIC guest interrupt handler for distributor writes.
//
// Return values:
//   true - access has been handled successfully, even if the access itself is either bad or not permitted.
//   false - access was not handled successfully.
//
static bool handle_vgic_dist_access(struct exc_info *ctx, u64 addr, u64 *val, bool write, int width)
{
    u64 relative_addr;
    bool register_handled;
    bool unimplemented_reg_accessed;
    relative_addr = addr - dist_base;
    register_handled = false;
    unimplemented_reg_accessed = false;
    if(write) {
        //
        // The guest attempted to write a register.
        // Handle it based on what they're trying to write, and preserve the value if
        // the value is going to a RW register.
        // Emit a warning (to become an error later) if the guest is attempting to write a register that doesn't exist or is read only.
        //

        //
        // This switch statement covers all the unique one of a kind registers.
        //
        switch(relative_addr) {
            case GIC_DIST_CTLR:
                //
                // GICD_CTLR has fields that we cannot change (due to the underlying physical environment or constraints)
                // and fields we can change, so check for RO fields here first.
                //
                u32 gicd_ctlr_new_val = (u32)(*val);
                vgic_log("HV vGIC DEBUG: guest writing GICD_CTLR = 0x%x, old value 0x%x\n", gicd_ctlr_new_val, distributor->gicd_ctl_reg);
                bool is_rwp_to_be_set = false;
                if(((gicd_ctlr_new_val & GENMASK(30, 8)) != 0) || ((gicd_ctlr_new_val & BIT(5)) != 0) || ((gicd_ctlr_new_val & GENMASK(3, 2)) != 0)) {
                    //
                    // these bits are RES0 - clear out this bitmask.
                    //
                    gicd_ctlr_new_val &= ~(GENMASK(30, 8));
                    gicd_ctlr_new_val &= ~(GENMASK(3, 2));
                    gicd_ctlr_new_val &= ~(BIT(5));
                    vgic_log("HV vGIC DEBUG [WARN]: guest attempted to write RES0 bits in GICD_CTLR, discarding\n");
                }
                
                if((gicd_ctlr_new_val & BIT(6)) == 0) {
                    //
                    // the guest is trying to set DS = 0. we do not support this so ensure that bit 6 is always written,
                    // however we need to emit a warning because this means that our GIC configuration is wrong.
                    //
                    gicd_ctlr_new_val |= BIT(6);
                    vgic_log("HV vGIC DEBUG [WARN]: guest attempted to set DS = 0, discarding\n");
                }
                if((gicd_ctlr_new_val & BIT(4)) == 0) {
                    //
                    // the guest is trying to set ARE = 0. we do not support this so ensure that bit 4 is always written,
                    // however we need to emit a warning because this means that our GIC configuration is wrong.
                    //
                    gicd_ctlr_new_val |= BIT(4);
                    vgic_log("HV vGIC DEBUG [WARN]: guest attempted to set ARE = 0, discarding\n");
                }
                if((((gicd_ctlr_new_val & BIT(7)) != 0) && ((distributor->gicd_ctl_reg & BIT(7)) == 0)) 
                || (((gicd_ctlr_new_val & BIT(7)) == 0) && ((distributor->gicd_ctl_reg & BIT(7)) != 0))) {
                    //
                    // the guest is trying to set EN1WF either way. we need to know about any attempt to change this, as it affects IRQ behavior.
                    // we also need to flag that RWP needs to be set to 1.
                    //
                    is_rwp_to_be_set = true;
                    vgic_log("HV vGIC DEBUG [INFO]: guest is changing EN1WF\n");
                }
                if(((gicd_ctlr_new_val & BIT(1)) == 0) && ((distributor->gicd_ctl_reg & BIT(1)) != 0)) {
                    //
                    // the guest is trying to set EnableGrp1 = 0. we need to know about any attempt to set this, as it affects IRQ behavior.
                    // we also need to flag that RWP needs to be set to 1.
                    //
                    is_rwp_to_be_set = true;
                    vgic_log("HV vGIC DEBUG [INFO]: guest is setting EnableGrp1 = 0\n");
                }
                if(((gicd_ctlr_new_val & BIT(0)) == 0) && ((distributor->gicd_ctl_reg & BIT(0)) != 0)) {
                    //
                    // the guest is trying to set EnableGrp1 = 0. we need to know about any attempt to set this, as it affects IRQ behavior.
                    // we also need to flag that RWP needs to be set to 1.
                    //
                    is_rwp_to_be_set = true;
                    vgic_log("HV vGIC DEBUG [INFO]: guest is setting EnableGrp0 = 0\n");
                }

                //
                // RWP (Register Write Pending bit) - this bit is a tad bit special - it's RO, but it has to be set if bits 0 or 1 are transitioning
                // from 1 to 0.
                //
                if(is_rwp_to_be_set == true) {
                    //
                    // set RWP here - then start propagating the effects immediately after.
                    //
                    gicd_ctlr_new_val |= BIT(31);
                }

                distributor->gicd_ctl_reg = gicd_ctlr_new_val;
                if(is_rwp_to_be_set == true) {
                    //
                    // TODO: start the changes signaled by RWP.
                    //
                    //hv_vgicv3_apply_gic_dist_changes(gicd_ctlr_new_val);
                }
                register_handled = true;
                break;
            case GIC_DIST_TYPER:
            case GIC_DIST_TYPER2:
            case GIC_DIST_IIDR:
                //
                // these registers are totally RO, so leave their values unchanged.
                //
                vgic_log("HV vGIC DEBUG [WARN]: guest attempted to change a read-only register (0x%x), discarding\n", relative_addr);
                register_handled = true;
                break;
            case GIC_DIST_STATUSR:
                //
                // GICD_STATUSR is a bit special, software must write 1 to ack an error, which then *clears* the bit.
                // Note that [31:4] are always RES0.
                //
                u32 gicd_statusr_new_val = (u32)(*val);
                u32 gicd_statusr_current_val = distributor->gicd_err_sts;
                if((gicd_statusr_new_val & GENMASK(31, 4)) != 0) {
                    gicd_statusr_new_val &= ~(GENMASK(31, 4));
                    vgic_log("HV vGIC DEBUG [WARN]: guest attempted to write RES0 bits in GICD_STATUSR, discarding\n");
                }
                if(((gicd_statusr_new_val & BIT(3)) != 0) & ((gicd_statusr_current_val & BIT(3)) != 0)) {
                    gicd_statusr_current_val &= ~(BIT(3));
                    vgic_log("HV vGIC DEBUG [INFO]: clearing WROD bit in GICD_STATUSR\n");
                }
                if(((gicd_statusr_new_val & BIT(2)) != 0) & ((gicd_statusr_current_val & BIT(2)) != 0)) {
                    gicd_statusr_current_val &= ~(BIT(2));
                    vgic_log("HV vGIC DEBUG [INFO]: clearing RWOD bit in GICD_STATUSR\n");
                }
                if(((gicd_statusr_new_val & BIT(1)) != 0) & ((gicd_statusr_current_val & BIT(1)) != 0)) {
                    gicd_statusr_current_val &= ~(BIT(1));
                    vgic_log("HV vGIC DEBUG [INFO]: clearing WRD bit in GICD_STATUSR\n");
                }
                if(((gicd_statusr_new_val & BIT(0)) != 0) & ((gicd_statusr_current_val & BIT(0)) != 0)) {
                    gicd_statusr_current_val &= ~(BIT(0));
                    vgic_log("HV vGIC DEBUG [INFO]: clearing RRD bit in GICD_STATUSR\n");
                }
                distributor->gicd_err_sts = gicd_statusr_current_val;
                register_handled = true;
                break;
            //
            // right now, MBIS is disabled - so these four registers are reserved.
            //
            case GIC_DIST_SETSPI_NSR:
            case GIC_DIST_CLRSPI_NSR:
            case GIC_DIST_CLRSPI_SR:
            case GIC_DIST_SETSPI_SR:
                register_handled = true;
                break;
            
            case GIC_DIST_SGIR:
                //
                // This register is reserved too, since affinity routing is always enabled.
                //
                register_handled = true;
                break;

            case GIC_DIST_IROUTER32 ... GIC_DIST_IROUTER1019:
                u32 reg_num;
                u64 mpidr = 0;
                u32 cpu_num;
                reg_num = (relative_addr - GIC_DIST_IROUTER32) / 8;
                distributor->gicd_interrupt_router_regs[reg_num] = *val;
                
                mpidr |= (u64)MPIDR_AFF0(*val);
                mpidr |= (u64)MPIDR_AFF1(*val) << 8;
                mpidr |= (u64)MPIDR_AFF2(*val) << 16;
                mpidr |= (u64)MPIDR_AFF3(*val) << 32;
                cpu_num = smp_get_id(mpidr);

                const struct hv_irq_route *route = hv_irq_route_from_vintid(reg_num + 32);
                if (route)
                    aic_set_affinity(route->hw_irq, cpu_num);
                vgic_log("HV vGIC DEBUG [INFO] [Distributor]: interrupt routing register %d = %d\n", reg_num, cpu_num);
                register_handled = true;
                break;
            default:
                //
                // we're dealing with a register that is banked n times, we need to get to the if statements.
                //
                break;
        }

        //
        // Fair warning this code is probably dicey...
        //
        if((register_handled == false) && (relative_addr >= GIC_DIST_IGROUPR0) && (relative_addr <= GIC_DIST_IGROUPR31) ) {
            //
            // the guest is trying to change the group of a given interrupt.
            //
            u32 reg_num;
            reg_num = (relative_addr - GIC_DIST_IGROUPR0) / 4;

            //
            // TODO: bank GICD_IGROUPR0 for cores 0-7 - GIC spec requires it - but since we're booting with 1 core atm, we can ignore
            // this for now.
            //

            distributor->gicd_interrupt_group_regs[reg_num] = *val;
            register_handled = true;

        }
        else if ( (register_handled == false) && (relative_addr >= GIC_DIST_ISENABLER0) && (relative_addr <= GIC_DIST_ISENABLER31) ) {
            //
            // enables an IRQ to be forwarded to a CPU interface.
            //
            u32 reg_num;
            reg_num = (relative_addr - GIC_DIST_ISENABLER0) / 4;
            u32 value_is_enabler, value_ic_enabler, current_val;
            u32 irq_num;
            value_is_enabler = distributor->gicd_interrupt_set_enable_regs[reg_num];
            value_ic_enabler = distributor->gicd_interrupt_clear_enable_regs[reg_num];
            current_val = *val;

            //
            // if 1 is written to the bits in these registers, they need to read 1 in GICD_ICENABLER[0:31] as well.
            // also this is banked for the first 8 processor cores - so changes must reflect across all of them.
            //
            // There has to be a way more efficient way of doing this...
            //

            for(u32 i = 0; i < 32; i++) {
                if( ( (current_val & BIT(i)) != 0 ) && ( ( value_is_enabler & BIT(i) ) == 0) ) {
                    value_is_enabler |= BIT(i);
                    value_ic_enabler |= BIT(i);      
                    irq_num = (32 * reg_num) + i;

                    const struct hv_irq_route *route = hv_irq_route_from_vintid(irq_num);
                    if (route) {
                        if (route->hw_irq == 857) {
                            hv_prepare_j313_xhci_handoff();
                            hv_prepare_j313_xhci_darts();
                            j313_xhci_tick_trace_budget = 24;
                            hv_mark_guest_runtime_ready();
                        }
                        aic_set_mask(route->hw_irq, false);
                        u32 transition = __atomic_add_fetch(&irq_route_transition_count, 1,
                                                            __ATOMIC_RELAXED);
                        if (hv_vgic_diag_should_log_route_transition(transition))
                            printf("HV: IRQ route enabled vINTID=%u AIC=%u count=%u\n",
                                   route->vintid, route->hw_irq, transition);
                        hv_vgic3_trace_intid(route->vintid, 48);
                        if (route->hw_irq == 857)
                            hv_trace_j313_xhci("route-enable");
                    }
                }
            }
            if(reg_num == 0) {

            }
            else {
                distributor->gicd_interrupt_set_enable_regs[reg_num] = value_is_enabler;
                distributor->gicd_interrupt_clear_enable_regs[reg_num] = value_ic_enabler;
            }

            register_handled = true;

        }
        else if ( (register_handled == false) && (relative_addr >= GIC_DIST_ICENABLER0) && (relative_addr <= GIC_DIST_ICENABLER31) ) {
            //
            // disables an IRQ to be forwarded to a CPU interface.
            //
            u32 reg_num;
            reg_num = (relative_addr - GIC_DIST_ICENABLER0) / 4;
            u32 irq_num;
            u32 value_is_enabler, value_ic_enabler, current_val;
            value_is_enabler = distributor->gicd_interrupt_set_enable_regs[reg_num];
            value_ic_enabler = distributor->gicd_interrupt_clear_enable_regs[reg_num];
            current_val = *val;

            //
            // if 1 is written to the bits in these registers, they need to read 0 in GICD_ISENABLER[0:31] as well.
            // also this is banked for the first 8 processor cores - so changes must reflect across all of them.
            //
            // There has to be a way more efficient way of doing this...
            //
            for(u32 i = 0; i < 32; i++) {
                if( ( (current_val & BIT(i)) != 0 ) && ( ( value_ic_enabler & BIT(i) ) != 0) ) {
                    value_is_enabler &= ~BIT(i);
                    value_ic_enabler &= ~BIT(i);      
                    irq_num = (32 * reg_num) + i;

                    const struct hv_irq_route *route = hv_irq_route_from_vintid(irq_num);
                    if (route) {
                        aic_set_mask(route->hw_irq, true);
                        u32 transition = __atomic_add_fetch(&irq_route_transition_count, 1,
                                                            __ATOMIC_RELAXED);
                        if (hv_vgic_diag_should_log_route_transition(transition))
                            printf("HV: IRQ route disabled vINTID=%u AIC=%u count=%u\n",
                                   route->vintid, route->hw_irq, transition);
                    }
                }
            }
            if(reg_num == 0) {

            }
            else {
                distributor->gicd_interrupt_set_enable_regs[reg_num] = value_is_enabler;
                distributor->gicd_interrupt_clear_enable_regs[reg_num] = value_ic_enabler;
            }
            register_handled = true;
            //
            // ICENABLER register writes require RWP dependent things to be updated, set the bit.
            //
            distributor->gicd_ctl_reg |= BIT(31);
            //
            // TODO: propagate the changes
            //

        }
        else if ( (register_handled == false) && (relative_addr >= GIC_DIST_ISPENDR0) && (relative_addr <= GIC_DIST_ISPENDR31) ) {
            //
            // sets an IRQ to pending
            //
            u32 reg_num, irq_num;
            reg_num = (relative_addr - GIC_DIST_ISPENDR0) / 4;
            u32 value_is_enabler, value_ic_enabler, current_val;
            value_is_enabler = distributor->gicd_interrupt_set_pending_regs[reg_num];
            value_ic_enabler = distributor->gicd_interrupt_clear_pending_regs[reg_num];
            current_val = *val;

            //
            // if 1 is written to the bits in these registers, they need to read 1 in GICD_ICENABLER[1:31] as well.
            // also this is banked for the first 8 processor cores - so changes must reflect across all of them.
            //
            // There has to be a way more efficient way of doing this...
            //

            for (u32 i = 0; i < 32; i++) {
                if( ( (current_val & BIT(i)) != 0 ) && ( ( value_is_enabler & BIT(i) ) == 0) ) {
                    value_is_enabler |= BIT(i);
                    value_ic_enabler |= BIT(i);
                    irq_num = (32 * reg_num) + i;
                    //
                    // TODO: do this
                    //
                    vgic_log("HV vGIC DEBUG [ERROR]: ISPENDR not implemented for irq %d\n", irq_num);
                }
            }
            if(reg_num == 0) {
                //
                // don't attempt to write these registers, since affinity routing is always on.
                //
            }
            else {
                distributor->gicd_interrupt_set_pending_regs[reg_num] = value_is_enabler;
                distributor->gicd_interrupt_clear_pending_regs[reg_num] = value_ic_enabler;
            }
            register_handled = true;

        }
        else if ( (register_handled == false) && (relative_addr >= GIC_DIST_ICPENDR0) && (relative_addr <= GIC_DIST_ICPENDR31) ) {
            //
            // clears the pending state from an IRQ
            //
            u32 reg_num, irq_num;
            reg_num = (relative_addr - GIC_DIST_ICPENDR0) / 4;
            u32 value_is_enabler, value_ic_enabler, current_val;
            value_is_enabler = distributor->gicd_interrupt_set_pending_regs[reg_num];
            value_ic_enabler = distributor->gicd_interrupt_clear_pending_regs[reg_num];
            current_val = *val;

            //
            // if 1 is written to the bits in these registers, they need to read 0 in GICD_ISENABLER[0:31] as well.
            // also this is banked for the first 8 processor cores - so changes must reflect across all of them.
            //
            // There has to be a way more efficient way of doing this...
            //
            for (u32 i = 0; i < 32; i++) {
                if( ( (current_val & BIT(i)) != 0 ) && ( ( value_ic_enabler & BIT(i) ) != 0) ) {
                    value_is_enabler &= ~BIT(i);
                    value_ic_enabler &= ~BIT(i);
                    irq_num = (32 * reg_num) + i;
                    //
                    // TODO: do this
                    //
                    vgic_log("HV vGIC DEBUG [ERROR]: ICPENDR not implemented for irq %d\n", irq_num);
                }  
            }
            if(reg_num == 0) {

            }
            else {
                distributor->gicd_interrupt_set_pending_regs[reg_num] = value_is_enabler;
                distributor->gicd_interrupt_clear_pending_regs[reg_num] = value_ic_enabler;
            }
            register_handled = true;

        }
        else if ( (register_handled == false) && (relative_addr >= GIC_DIST_ISACTIVER0) && (relative_addr <= GIC_DIST_ISACTIVER31) ) {
            //
            // clears the pending state from an IRQ
            //
            u32 reg_num, irq_num;
            reg_num = (relative_addr - GIC_DIST_ISACTIVER0) / 4;
            u32 value_is_enabler, value_ic_enabler, current_val;
            value_is_enabler = distributor->gicd_interrupt_set_active_regs[reg_num];
            value_ic_enabler = distributor->gicd_interrupt_clear_active_regs[reg_num];
            current_val = *val;

            //
            // if 1 is written to the bits in these registers, they need to read 0 in GICD_ISACTIVER[0:31] as well.
            // also this is banked for the first 8 processor cores - so changes must reflect across all of them.
            //
            // There has to be a way more efficient way of doing this...
            //
            for (u32 i = 0; i < 32; i++) {
                if( ( (current_val & BIT(i)) != 0 ) && ( ( value_ic_enabler & BIT(i) ) == 0) ) {
                    value_is_enabler &= ~BIT(i);
                    value_ic_enabler &= ~BIT(i);
                    irq_num = (32 * reg_num) + i;
                    //
                    // TODO: do this
                    //
                    vgic_log("HV vGIC DEBUG [ERROR]: ISACTIVER not implemented for irq %d\n", irq_num);
                }  
            }
            if(reg_num == 0) {

            }
            else {
                distributor->gicd_interrupt_set_active_regs[reg_num] = value_is_enabler;
                distributor->gicd_interrupt_clear_active_regs[reg_num] = value_ic_enabler;
            }
            register_handled = true;
        }
        else if ( (register_handled == false) && (relative_addr >= GIC_DIST_ICACTIVER0) && (relative_addr <= GIC_DIST_ICACTIVER31) ) {
            //
            // clears the pending state from an IRQ
            //
            u32 reg_num, irq_num;
            reg_num = (relative_addr - GIC_DIST_ICACTIVER0) / 4;
            u32 value_is_enabler, value_ic_enabler, current_val;
            value_is_enabler = distributor->gicd_interrupt_set_active_regs[reg_num];
            value_ic_enabler = distributor->gicd_interrupt_clear_active_regs[reg_num];
            current_val = *val;

            //
            // if 1 is written to the bits in these registers, they need to read 0 in GICD_ISACTIVER[0:31] as well.
            // also this is banked for the first 8 processor cores - so changes must reflect across all of them.
            //
            // There has to be a way more efficient way of doing this...
            //
            for (u32 i = 0; i < 32; i++) {
                if( ( (current_val & BIT(i)) != 0 ) && ( ( value_ic_enabler & BIT(i) ) != 0) ) {
                    value_is_enabler &= ~BIT(i);
                    value_ic_enabler &= ~BIT(i);
                    irq_num = (32 * reg_num) + i;
                    //
                    // TODO: do this
                    //
                    vgic_log("HV vGIC DEBUG [ERROR]: ICACTIVER not implemented for irq %d\n", irq_num);
                }  
            }
            if(reg_num == 0) {

            }
            else {
                distributor->gicd_interrupt_set_active_regs[reg_num] = value_is_enabler;
                distributor->gicd_interrupt_clear_active_regs[reg_num] = value_ic_enabler;
            }
            register_handled = true;
        }
        else if ( (register_handled == false) && (relative_addr >= GIC_DIST_IPRIORITYR0) && (relative_addr <= GIC_DIST_IPRIORITYR254) ) {
            u32 reg_num;
            reg_num = (relative_addr - GIC_DIST_IPRIORITYR0) / 4;
            // Windows writes one byte per INTID: merge into its lane.
            distributor->gicd_interrupt_priority_regs[reg_num] =
                hv_vgic_diag_priority_word_write(distributor->gicd_interrupt_priority_regs[reg_num],
                                                 relative_addr & 3, width, *val);
            vgic_log("HV vGIC DEBUG [INFO] [Distributor]: interrupt priority register %d = 0x%llx\n", reg_num, *val);
            register_handled = true;
            //unimplemented_reg_accessed = true;
        }
        else if ( (register_handled == false) && (relative_addr >= GIC_DIST_ITARGETSR0) && (relative_addr <= GIC_DIST_ITARGETSR254) ) {
            //
            // These are RES0 - since affinity routing is always enabled on Apple platforms.
            //
            vgic_log("HV vGIC DEBUG [WARN]: GICD_ITARGETS registers are RES0 - discarding write\n");
            register_handled = true;

        }
        else if ( (register_handled == false) && (relative_addr >= GIC_DIST_ICFGR0) && (relative_addr <= GIC_DIST_ICFGR63) ) {
            u32 reg_num;
            reg_num = (relative_addr - GIC_DIST_ICFGR0) / 4;
            //
            // Unimplemented for now (we only support the timer interrupt right now - and those are managed by the redistributors)
            //
            distributor->gicd_interrupt_config_regs[reg_num] = *val;
            vgic_log("HV vGIC DEBUG [INFO] [Distributor]: interrupt configuration register %d = 0x%llx\n", reg_num, *val);
            register_handled = true;
            //unimplemented_reg_accessed = true;
        }
        else if(register_handled == false){
            //
            // the register is unknown (or unimplemented) - print a warning.
            //
            vgic_log("HV vGIC DEBUG [ERR] - guest attempted to access unknown register 0x%llx\n", relative_addr);
            register_handled = true;
            unimplemented_reg_accessed = true;
        }
    }
    else {
        //
        // The guest is attempting to read a register.
        // Handle it appropriately. Emit a warning (to become an error later) if a register is write only or doesn't exist
        //
        switch(relative_addr) {
            case GIC_DIST_CTLR:
                *val = distributor->gicd_ctl_reg;
                register_handled = true;
                break;
            case GIC_DIST_TYPER:
                *val = distributor->gicd_type_reg;
                register_handled = true;
                break;
            case GIC_DIST_TYPER2:
                *val = distributor->gicd_type_reg_2;
                register_handled = true;
                break;
            case GIC_DIST_IIDR:
                *val = distributor->gicd_imp_id_reg;
                register_handled = true;
                break;
            case GIC_DIST_STATUSR:
                *val = distributor->gicd_err_sts;
                register_handled = true;
                break;
            case GIC_DIST_SETSPI_NSR:
            case GIC_DIST_CLRSPI_NSR:
            case GIC_DIST_CLRSPI_SR:
            case GIC_DIST_SETSPI_SR:
            case GIC_DIST_SGIR:
                *val = 0; // these registers are write only so force return 0 to the guest.
                register_handled = true;
                break;
            case 0xffe8: // make Hal happy
                *val = 0xff;
                register_handled = true;
                break;
            case GIC_DIST_IROUTER32 ... GIC_DIST_IROUTER1019:
                u32 reg_num;
                reg_num = (relative_addr - GIC_DIST_IROUTER32) / 8;
                *val = distributor->gicd_interrupt_router_regs[reg_num];
                register_handled = true;
                break;
            default:
                //
                // we're dealing with a register that is banked n times, we need to get to the if statements.
                //
                break;
        }
        if((register_handled == false) && (relative_addr >= GIC_DIST_IGROUPR0) && (relative_addr <= GIC_DIST_IGROUPR31) ) {
            //
            // the guest is trying to change the group of a given interrupt.
            //
            u32 reg_num;
            reg_num = (relative_addr - GIC_DIST_IGROUPR0) / 4;

            //
            // TODO: bank GICD_IGROUPR0 for cores 0-7 - GIC spec requires it - but since we're booting with 1 core atm, we can ignore
            // this for now.
            //

            *val = distributor->gicd_interrupt_group_regs[reg_num];
            register_handled = true;

        }
        else if ( (register_handled == false) && (relative_addr >= GIC_DIST_ISENABLER0) && (relative_addr <= GIC_DIST_ISENABLER31) ) {
            //
            // enables an IRQ to be forwarded to a CPU interface.
            //
            u32 reg_num;
            reg_num = (relative_addr - GIC_DIST_ISENABLER0) / 4;
            *val = distributor->gicd_interrupt_set_enable_regs[reg_num];
            register_handled = true;

        }
        else if ( (register_handled == false) && (relative_addr >= GIC_DIST_ICENABLER0) && (relative_addr <= GIC_DIST_ICENABLER31) ) {
            //
            // disables an IRQ to be forwarded to a CPU interface.
            //
            u32 reg_num;
            reg_num = (relative_addr - GIC_DIST_ICENABLER0) / 4;
            *val = distributor->gicd_interrupt_clear_enable_regs[reg_num];
            register_handled = true;

        }
        else if ( (register_handled == false) && (relative_addr >= GIC_DIST_ISPENDR0) && (relative_addr <= GIC_DIST_ISPENDR31) ) {
            //
            // sets an IRQ to pending
            //
            u32 reg_num, irq_num;
            reg_num = (relative_addr - GIC_DIST_ISPENDR0) / 4;
            distributor->gicd_interrupt_set_pending_regs[reg_num];
            register_handled = true;

        }
        else if ( (register_handled == false) && (relative_addr >= GIC_DIST_ICPENDR0) && (relative_addr <= GIC_DIST_ICPENDR31) ) {
            //
            // clears the pending state from an IRQ
            //
            u32 reg_num, irq_num;
            reg_num = (relative_addr - GIC_DIST_ICPENDR0) / 4;
            *val = distributor->gicd_interrupt_clear_pending_regs[reg_num];
            register_handled = true;

        }
        else if ( (register_handled == false) && (relative_addr >= GIC_DIST_ISACTIVER0) && (relative_addr <= GIC_DIST_ISACTIVER31) ) {
            //
            // 
            //
            // clears the pending state from an IRQ
            //
            u32 reg_num, irq_num;
            reg_num = (relative_addr - GIC_DIST_ISACTIVER0) / 4;
            *val = distributor->gicd_interrupt_set_active_regs[reg_num];
            register_handled = true;
        }
        else if ( (register_handled == false) && (relative_addr >= GIC_DIST_ICACTIVER0) && (relative_addr <= GIC_DIST_ICACTIVER31) ) {
            //
            // clears the pending state from an IRQ
            //
            u32 reg_num, irq_num;
            reg_num = (relative_addr - GIC_DIST_ICACTIVER0) / 4;
            *val = distributor->gicd_interrupt_clear_active_regs[reg_num];
            register_handled = true;
        }
        else if ( (register_handled == false) && (relative_addr >= GIC_DIST_IPRIORITYR0) && (relative_addr <= GIC_DIST_IPRIORITYR254) ) {
            u32 reg_num;
            reg_num = (relative_addr - GIC_DIST_IPRIORITYR0) / 4;
            *val = hv_vgic_diag_priority_word_read(distributor->gicd_interrupt_priority_regs[reg_num],
                                                   relative_addr & 3, width);
            vgic_log("HV vGIC DEBUG [INFO] [Distributor]: interrupt priority register %d = 0x%llx\n", reg_num, *val);
            register_handled = true;
        }
        else if ( (register_handled == false) && (relative_addr >= GIC_DIST_ITARGETSR0) && (relative_addr <= GIC_DIST_ITARGETSR254) ) {
            //
            // These are RES0 - since affinity routing is always enabled on Apple platforms.
            //
            *val = 0;
            register_handled = true;

        }
        else if ( (register_handled == false) && (relative_addr >= GIC_DIST_ICFGR0) && (relative_addr <= GIC_DIST_ICFGR63) ) {
            u32 reg_num;
            reg_num = (relative_addr - GIC_DIST_ICFGR0) / 4;
            //
            // Unimplemented for now (we only support the timer interrupt right now - and those are managed by the redistributors)
            //
            *val = distributor->gicd_interrupt_config_regs[reg_num];
            vgic_log("HV vGIC DEBUG [INFO] [Distributor]: interrupt configuration register %d = 0x%llx\n", reg_num, *val);
            register_handled = true;
            //unimplemented_reg_accessed = true;
        }
        else if (register_handled == false) {
            //
            // the register is unknown (or unimplemented) - print a warning.
            //
            vgic_log("HV vGIC DEBUG [ERR] - guest attempted to access unknown register 0x%llx\n", relative_addr);
            register_handled = true;
            unimplemented_reg_accessed = true;
        }
    }
    vgic_log("HV vGIC DEBUG [INFO] [Distributor]: 0x%llx = 0x%llx ", relative_addr, *val);
    if(write) {
        vgic_log("[Written]");
    }
    else {
        vgic_log("[Read]");
    }
    if(unimplemented_reg_accessed) {
        vgic_log("[Unimplemented]\n");
    }
    else {
        vgic_log("\n");
    }
    return register_handled;
}



//
// Description:
//   the vGIC guest interrupt handler for redistributor writes.
//
// Return values:
//   true - access has been handled successfully, even if the access itself is either bad or not permitted.
//   false - access was not handled successfully.
//
static bool handle_vgic_redist_access(struct exc_info *ctx, u64 addr, u64 *val, bool write, int width)
{
    u64 relative_addr;
    bool register_handled;
    bool unimplemented_reg_accessed;
    struct hv_vgic_redist_addr decoded;
    if (!hv_vgic_redist_decode(addr, redist_base, num_cpus, &decoded))
        return false;

    // A GICv3 redistributor is one 128-KiB frame per vCPU (64-KiB RD + 64-KiB
    // SGI/PPI).  The frame encoded in the MMIO address selects the target CPU;
    // it is not necessarily the CPU executing the access.  Keeping the full
    // offset here made every frame after CPU0 look like an unknown register, so
    // Windows could not enable or prioritize the timer PPIs on secondary CPUs.
    relative_addr = decoded.reg;
    static u32 traced_redist_frames;
    u32 frame_bit = BIT(decoded.cpu);
    if (!(__atomic_fetch_or(&traced_redist_frames, frame_bit, __ATOMIC_RELAXED) & frame_bit))
        printf("HV vGIC: first redistributor access frame=%u reg=0x%lx from_cpu=%lu\n",
               decoded.cpu, relative_addr, ctx->cpu_id);
    register_handled = false;
    unimplemented_reg_accessed = false;
    u32 cpu_num;
    u32 value_is_enabler, value_ic_enabler, current_val;
    u32 irq_num;
    u32 reg_num;
    u32 reg_offset;
    value_ic_enabler = 0;
    value_is_enabler = 0;
    current_val = 0;
    irq_num = 0;
    reg_num = 0;
    reg_offset = 0;

    cpu_num = decoded.cpu;
    if(write) {
        //
        // The guest attempted to write a register.
        // Handle it based on what they're trying to write, and preserve the value if
        // the value is going to a RW register.
        // Emit a warning (to become an error later) if the guest is attempting to write a register that doesn't exist or is read only.
        //
        
        switch(relative_addr) {
            //
            // RD region
            //
            case GIC_REDIST_CTLR:
                u32 gicr_ctlr_new_val = (u32)(*val);
                vgic_log("HV vGIC DEBUG: guest writing GICR_CTLR = 0x%x, old value 0x%x\n", gicr_ctlr_new_val, redistributors[cpu_num].rd_region.gicr_ctl_reg);
                bool is_uwp_to_be_set = false;
                bool is_rwp_to_be_set = false;
                //
                // like RWP in the distributor's case, the redistributor has it's own version of this type of bit (UWP),
                // where certain actions will trigger updates (IPIs in this case.)
                // we have to deal with this once the CPU interface is brought up.
                // the redistributors also have their own RWP bits which need to be handled similarly

                //
                // bits 30-27 and 23-4 are RES0 so discard writes.
                //
                if(((gicr_ctlr_new_val & GENMASK(30, 27)) != 0) || ((gicr_ctlr_new_val & GENMASK(23, 4)) != 0)) {
                    //
                    // these bits are RES0 - clear out this bitmask.
                    //
                    gicr_ctlr_new_val &= ~(GENMASK(30, 27));
                    gicr_ctlr_new_val &= ~(GENMASK(23, 4));
                    vgic_log("HV vGIC DEBUG [WARN]: guest attempted to write RES0 bits in GICR_CTLR, discarding\n");
                }

                //
                // since DS = 1 - bit 26 (DPG1S) is RAZ/WI
                //
                if( ( (gicr_ctlr_new_val) & BIT(26) ) != 0 ) {
                    //
                    // clear the bit
                    //
                    gicr_ctlr_new_val &= ~BIT(26);
                }

                //
                // setting or clearing bits 25 and 24 (DPG1NS and DPG0) will trigger an RWP change.
                //
                if( ( ( (gicr_ctlr_new_val) & BIT(25) ) != 0 ) && ( (redistributors[cpu_num].rd_region.gicr_ctl_reg) & BIT(25) == 0 ) 
                 || ( ( (gicr_ctlr_new_val) & BIT(24) ) != 0 ) && ( (redistributors[cpu_num].rd_region.gicr_ctl_reg) & BIT(24) == 0 ) 
                 || ( ( (gicr_ctlr_new_val) & BIT(25) ) == 0 ) && ( (redistributors[cpu_num].rd_region.gicr_ctl_reg) & BIT(25) != 0 ) 
                 || ( ( (gicr_ctlr_new_val) & BIT(24) ) == 0 ) && ( (redistributors[cpu_num].rd_region.gicr_ctl_reg) & BIT(24) != 0 ) ) {
                    //
                    // signal that RWP is going to be changed.
                    //
                    is_rwp_to_be_set = true;
                }

                //
                // bits 2 and 1 are RO - so discard writes to those bits.
                //
                if(((gicr_ctlr_new_val & BIT(2)) == 0) || ((gicr_ctlr_new_val & BIT(1)) == 0)) {
                    //
                    // guest is attempting to clear these RO bits - discard the write.
                    //
                    gicr_ctlr_new_val |= (BIT(2) | BIT(1));
                    vgic_log("HV vGIC DEBUG [WARN]: guest attempted to write read-only bits in GICR_CTLR, discarding\n");
                }

                //
                // EnableLPIs if cleared will trigger an RWP write.
                //
                if(((gicr_ctlr_new_val & BIT(0)) == 0) || ((redistributors[cpu_num].rd_region.gicr_ctl_reg & BIT(0)) != 0)) {
                    is_rwp_to_be_set = true;
                }

                //
                // start propagating the effects of the RWP changes.
                //
                if(is_rwp_to_be_set == true) {
                    //
                    // set RWP here - then start propagating the effects immediately after.
                    //
                    gicr_ctlr_new_val |= BIT(31);
                }

                redistributors[cpu_num].rd_region.gicr_ctl_reg = gicr_ctlr_new_val;
                if(is_rwp_to_be_set == true) {
                    //
                    // TODO: start the changes signaled by RWP.
                    //
                    //hv_vgicv3_apply_gic_redist_changes(gicr_ctlr_new_val);
                }

                redistributors[cpu_num].rd_region.gicr_ctl_reg = gicr_ctlr_new_val;
                register_handled = true;
                break;
            case GIC_REDIST_IIDR:
            case GIC_REDIST_TYPER:
            case GIC_REDIST_MPAMIDR:
                //
                // these are simple - the registers are read only so discard any write attempts.
                //
                vgic_log("HV vGIC DEBUG [WARN]: guest attempted to change a read-only register (0x%x), discarding\n", relative_addr);
                register_handled = true;
                break;
            case GIC_REDIST_STATUSR:
                //
                // GICR_STATUSR is a bit special, software must write 1 to ack an error, which then *clears* the bit.
                // Note that [31:4] are always RES0.
                //
                u32 gicr_statusr_new_val = (u32)(*val);
                u32 gicr_statusr_current_val = redistributors[cpu_num].rd_region.gicr_status_reg;
                if((gicr_statusr_new_val & GENMASK(31, 4)) != 0) {
                    gicr_statusr_new_val &= ~(GENMASK(31, 4));
                    vgic_log("HV vGIC DEBUG [WARN]: guest attempted to write RES0 bits in GICD_STATUSR, discarding\n");
                }
                if(((gicr_statusr_new_val & BIT(3)) != 0) & ((gicr_statusr_current_val & BIT(3)) != 0)) {
                    gicr_statusr_current_val &= ~(BIT(3));
                    vgic_log("HV vGIC DEBUG [INFO]: clearing WROD bit in GICD_STATUSR\n");
                }
                if(((gicr_statusr_new_val & BIT(2)) != 0) & ((gicr_statusr_current_val & BIT(2)) != 0)) {
                    gicr_statusr_current_val &= ~(BIT(2));
                    vgic_log("HV vGIC DEBUG [INFO]: clearing RWOD bit in GICD_STATUSR\n");
                }
                if(((gicr_statusr_new_val & BIT(1)) != 0) & ((gicr_statusr_current_val & BIT(1)) != 0)) {
                    gicr_statusr_current_val &= ~(BIT(1));
                    vgic_log("HV vGIC DEBUG [INFO]: clearing WRD bit in GICD_STATUSR\n");
                }
                if(((gicr_statusr_new_val & BIT(0)) != 0) & ((gicr_statusr_current_val & BIT(0)) != 0)) {
                    gicr_statusr_current_val &= ~(BIT(0));
                    vgic_log("HV vGIC DEBUG [INFO]: clearing RRD bit in GICD_STATUSR\n");
                }
                redistributors[cpu_num].rd_region.gicr_status_reg = gicr_statusr_current_val;
                register_handled = true;
                break;
            case GIC_REDIST_WAKER:
                redistributors[cpu_num].rd_region.gicr_wake_reg = *val;
                register_handled = true;
                break;
            case GIC_REDIST_PARTIDR:
                redistributors[cpu_num].rd_region.gicr_partidr = *val;
                register_handled = true;
                break;
            case GIC_REDIST_SETLPIR:
                redistributors[cpu_num].rd_region.gicr_setlpir = *val;
                //
                // TODO: actually do the action here.
                //
                vgic_log("HV vGIC DEBUG [WARN]: GICR_SETLPIR is currently unimplemented!\n");
                unimplemented_reg_accessed = true;
                register_handled = true;
                break;
            case GIC_REDIST_CLRLPIR:
                redistributors[cpu_num].rd_region.gicr_clrlpir = *val;
                //
                // TODO: actually do the action here.
                //
                vgic_log("HV vGIC DEBUG [WARN]: GICR_CLRLPIR is currently unimplemented!\n");
                unimplemented_reg_accessed = true;
                register_handled = true;
                break;
            case GIC_REDIST_PROPBASER:
                redistributors[cpu_num].rd_region.gicr_propbaser = *val;
                register_handled = true;
                break;
            case GIC_REDIST_PENDBASER:
                redistributors[cpu_num].rd_region.gicr_pendbaser = *val;
                register_handled = true;
                break;
            case GIC_REDIST_INVLPIR:
                redistributors[cpu_num].rd_region.gicr_invlpir = *val;
                //
                // TODO: implement this. note that for INTID bits, bits 31:16 are unused since IDbits = 16 for us.
                //
                vgic_log("HV vGIC DEBUG [WARN]: GICR_INVLPIR is currently unimplemented!\n");
                unimplemented_reg_accessed = true;
                register_handled = true;
                break;
            case GIC_REDIST_INVALLR:
                //
                // Any write to this register will invalidate all LPI config data - but the bits themselves are RES0.
                // TODO: implement this.
                //
                redistributors[cpu_num].rd_region.gicr_invallr = 0;
                vgic_log("HV vGIC DEBUG [WARN]: GICR_INVALLR is currently unimplemented!\n");
                unimplemented_reg_accessed = true;
                register_handled = true;
                break;
            case GIC_REDIST_SYNCR:
                //
                // this register is read only - but has special handling. currently unimplemented.
                //
                vgic_log("HV vGIC DEBUG [WARN]: GICR_SYNCR is currently unimplemented!\n");
                unimplemented_reg_accessed = true;
                register_handled = true;
                break;
            //
            // SGI region
            //
            case GIC_REDIST_IGROUPR0:
                redistributors[cpu_num].sgi_region.gicr_igroupr0 = *val;
                register_handled = true;
                break;
            case GIC_REDIST_ISENABLER0:
                // u32 value_is_enabler, value_ic_enabler, current_val;
                // u32 irq_num;
                value_is_enabler = redistributors[cpu_num].sgi_region.gicr_isactiver0;
                value_ic_enabler = redistributors[cpu_num].sgi_region.gicr_icactiver0;
                current_val = *val;

                //
                // if 1 is written to the bits in these registers, they need to read 1 in GICR_ICENABLER0 as well.
                // also this is banked for the first 8 processor cores - so changes must reflect across all of them.
                //
                // There has to be a way more efficient way of doing this...
                //

                for(u32 i = 0; i < 32; i++) {
                    if( ( (current_val & BIT(i)) != 0 ) && ( ( value_is_enabler & BIT(i) ) == 0) ) {
                        value_is_enabler |= BIT(i);
                        value_ic_enabler |= BIT(i);      
                        irq_num = i;
                        //
                        // TODO: do the AIC operation associated with this.
                        //        
                    }
                }
                redistributors[cpu_num].sgi_region.gicr_isactiver0 = value_is_enabler;
                redistributors[cpu_num].sgi_region.gicr_icactiver0 = value_ic_enabler;
                //
                // GICR_ISENABLER0 is set-enable: writing 1 enables an interrupt, writing
                // 0 has no effect (disabling is done through GICR_ICENABLER0). A plain
                // assignment cleared every other interrupt's enable - e.g. enabling the
                // virtual timer (INTID 18) silently disabled the physical timer (17).
                //
                redistributors[cpu_num].sgi_region.gicr_isenabler0 |= *val;
                register_handled = true;
                break;
            case GIC_REDIST_ICENABLER0:
                // u32 value_is_enabler, value_ic_enabler, current_val;
                // u32 irq_num;
                value_is_enabler = redistributors[cpu_num].sgi_region.gicr_isactiver0;
                value_ic_enabler = redistributors[cpu_num].sgi_region.gicr_icactiver0;
                current_val = *val;

                //
                // if 1 is written to the bits in these registers, they need to read 1 in GICD_ICENABLER[0:31] as well.
                // also this is banked for the first 8 processor cores - so changes must reflect across all of them.
                //
                // There has to be a way more efficient way of doing this...
                //

                for(u32 i = 0; i < 32; i++) {
                    if( ( (current_val & BIT(i)) != 0 ) && ( ( value_is_enabler & BIT(i) ) != 0) ) {
                        value_is_enabler &= ~BIT(i);
                        value_ic_enabler &= ~BIT(i);      
                        irq_num = i;
                        //
                        // TODO: do the AIC operation associated with this.
                        //        
                    }
                }
                redistributors[cpu_num].sgi_region.gicr_isactiver0 = value_is_enabler;
                redistributors[cpu_num].sgi_region.gicr_icactiver0 = value_ic_enabler;
                register_handled = true;
                break;
            case GIC_REDIST_ISPENDR0:
                // u32 value_is_enabler, value_ic_enabler, current_val;
                // u32 irq_num;
                value_is_enabler = redistributors[cpu_num].sgi_region.gicr_ispendr0;
                value_ic_enabler = redistributors[cpu_num].sgi_region.gicr_icpendr0;
                current_val = *val;

                //
                // if 1 is written to the bits in these registers, they need to read 1 in GICD_ICENABLER[0:31] as well.
                // also this is banked for the first 8 processor cores - so changes must reflect across all of them.
                //
                // There has to be a way more efficient way of doing this...
                //

                for(u32 i = 0; i < 32; i++) {
                    if( ( (current_val & BIT(i)) != 0 ) && ( ( value_is_enabler & BIT(i) ) == 0) ) {
                        value_is_enabler |= BIT(i);
                        value_ic_enabler |= BIT(i);      
                        irq_num = i;
                        //
                        // TODO: do the AIC operation associated with this.
                        //        
                    }
                }
                register_handled = true;
                redistributors[cpu_num].sgi_region.gicr_isactiver0 = value_is_enabler;
                redistributors[cpu_num].sgi_region.gicr_icactiver0 = value_ic_enabler;
                break;
            case GIC_REDIST_ICPENDR0:
                // u32 value_is_enabler, value_ic_enabler, current_val;
                // u32 irq_num;
                value_is_enabler = redistributors[cpu_num].sgi_region.gicr_ispendr0;
                value_ic_enabler = redistributors[cpu_num].sgi_region.gicr_icpendr0;
                current_val = *val;

                //
                // if 1 is written to the bits in these registers, they need to read 1 in GICD_ICENABLER[0:31] as well.
                // also this is banked for the first 8 processor cores - so changes must reflect across all of them.
                //
                // There has to be a way more efficient way of doing this...
                //

                for(u32 i = 0; i < 32; i++) {
                    if( ( (current_val & BIT(i)) != 0 ) && ( ( value_is_enabler & BIT(i) ) != 0) ) {
                        value_is_enabler &= ~BIT(i);
                        value_ic_enabler &= ~BIT(i);      
                        irq_num = i;
                        //
                        // TODO: do the AIC operation associated with this.
                        //        
                    }
                }
                register_handled = true;
                redistributors[cpu_num].sgi_region.gicr_isactiver0 = value_is_enabler;
                redistributors[cpu_num].sgi_region.gicr_icactiver0 = value_ic_enabler;
                break;
            case GIC_REDIST_ISACTIVER0:
                // u32 value_is_enabler, value_ic_enabler, current_val;
                // u32 irq_num;
                value_is_enabler = redistributors[cpu_num].sgi_region.gicr_isactiver0;
                value_ic_enabler = redistributors[cpu_num].sgi_region.gicr_icactiver0;
                current_val = *val;

                //
                // if 1 is written to the bits in these registers, they need to read 1 in GICD_ICENABLER[0:31] as well.
                // also this is banked for the first 8 processor cores - so changes must reflect across all of them.
                //
                // There has to be a way more efficient way of doing this...
                //

                for(u32 i = 0; i < 32; i++) {
                    if( ( (current_val & BIT(i)) != 0 ) && ( ( value_is_enabler & BIT(i) ) == 0) ) {
                        value_is_enabler |= BIT(i);
                        value_ic_enabler |= BIT(i);      
                        irq_num = i;
                        //
                        // TODO: do the AIC operation associated with this.
                        //        
                    }
                }
                register_handled = true;
                redistributors[cpu_num].sgi_region.gicr_isactiver0 = value_is_enabler;
                redistributors[cpu_num].sgi_region.gicr_icactiver0 = value_ic_enabler;
                break;
            case GIC_REDIST_ICACTIVER0:
                // u32 value_is_enabler, value_ic_enabler, current_val;
                // u32 irq_num;
                value_is_enabler = redistributors[cpu_num].sgi_region.gicr_isactiver0;
                value_ic_enabler = redistributors[cpu_num].sgi_region.gicr_icactiver0;
                current_val = *val;

                //
                // if 1 is written to the bits in these registers, they need to read 1 in GICD_ICENABLER[0:31] as well.
                // also this is banked for the first 8 processor cores - so changes must reflect across all of them.
                //
                // There has to be a way more efficient way of doing this...
                //

                for(u32 i = 0; i < 32; i++) {
                    if( ( (current_val & BIT(i)) != 0 ) && ( ( value_is_enabler & BIT(i) ) != 0) ) {
                        value_is_enabler &= ~BIT(i);
                        value_ic_enabler &= ~BIT(i);      
                        irq_num = i;
                        //
                        // TODO: do the AIC operation associated with this.
                        //        
                    }
                }
                register_handled = true;
                redistributors[cpu_num].sgi_region.gicr_isactiver0 = value_is_enabler;
                redistributors[cpu_num].sgi_region.gicr_icactiver0 = value_ic_enabler;
                break;
            case GIC_REDIST_ICFGR0:
                redistributors[cpu_num].sgi_region.gicr_icfgr0 = *val;
                register_handled = true;
                break;
            case GIC_REDIST_ICFGR1:
                redistributors[cpu_num].sgi_region.gicr_icfgr1 = *val;
                register_handled = true;
                break;
            case GIC_REDIST_IGRPMODR0:
                redistributors[cpu_num].sgi_region.gicr_igrpmodr0 = *val;
                register_handled = true;
                break;
            case GIC_REDIST_NSACR:
                redistributors[cpu_num].sgi_region.gicr_nsacr = *val;
                register_handled = true;
                break;
            case GIC_REDIST_IPRIORITYR0 ... GIC_REDIST_IPRIORITYR3 + 3:
                // u32 reg_num;
                reg_num = (relative_addr - GIC_REDIST_IPRIORITYR0) / 4;
                reg_offset = (relative_addr - GIC_REDIST_IPRIORITYR0) % 4;

                redistributors[cpu_num].sgi_region.gicr_sgi_ipriority_reg[reg_num] =
                    hv_vgic_diag_priority_word_write(
                        redistributors[cpu_num].sgi_region.gicr_sgi_ipriority_reg[reg_num],
                        reg_offset, width, *val);
                register_handled = true;
                break;
            case GIC_REDIST_IPRIORITYR4 ... GIC_REDIST_IPRIORITYR7 + 3:
                // u32 reg_num;
                reg_num = (relative_addr - GIC_REDIST_IPRIORITYR4) / 4;
                reg_offset = (relative_addr - GIC_REDIST_IPRIORITYR4) % 4;
                redistributors[cpu_num].sgi_region.gicr_ppi_ipriority_reg[reg_num] =
                    hv_vgic_diag_priority_word_write(
                        redistributors[cpu_num].sgi_region.gicr_ppi_ipriority_reg[reg_num],
                        reg_offset, width, *val);
                register_handled = true;
                break;
            default:
                //
                // an unimplemented register.
                //
                unimplemented_reg_accessed = true;
                break;
        }
    }
    else {
        //
        // The guest is attempting to read a register.
        // Handle it appropriately. Emit a warning (to become an error later) if a register is write only or doesn't exist
        //
        
        
        switch(relative_addr) {
            //
            // RD region
            //
            case GIC_REDIST_CTLR:
                *val = redistributors[cpu_num].rd_region.gicr_ctl_reg;
                register_handled = true;
                break;
            case GIC_REDIST_IIDR:
                *val = redistributors[cpu_num].rd_region.gicr_iidr;
                register_handled = true;
                break;
            case GIC_REDIST_TYPER:
                *val = redistributors[cpu_num].rd_region.gicr_type_reg;
                register_handled = true;
                break;
            case GIC_REDIST_STATUSR:
                *val = redistributors[cpu_num].rd_region.gicr_status_reg;
                register_handled = true;
                break;
            case GIC_REDIST_WAKER:
                *val = redistributors[cpu_num].rd_region.gicr_wake_reg;
                register_handled = true;
                break;
            case GIC_REDIST_MPAMIDR:
                *val = redistributors[cpu_num].rd_region.gicr_mpamidr;
                register_handled = true;
                break;
            case GIC_REDIST_PARTIDR:
                *val = redistributors[cpu_num].rd_region.gicr_partidr;
                register_handled = true;
                break;
            case GIC_REDIST_SETLPIR:
            case GIC_REDIST_CLRLPIR:
            case GIC_REDIST_INVLPIR:
            case GIC_REDIST_INVALLR:
                //
                // these registers are write-only so reads in our case will return 0 (only meaningful action here is writes)
                *val = 0;
                register_handled = true;
                break;
            case GIC_REDIST_PROPBASER:
                *val = redistributors[cpu_num].rd_region.gicr_propbaser;
                register_handled = true;
                break;
            case GIC_REDIST_PENDBASER:
                *val = redistributors[cpu_num].rd_region.gicr_pendbaser;
                register_handled = true;
                break;
            case GIC_REDIST_SYNCR:
                *val = redistributors[cpu_num].rd_region.gicr_iidr;
                register_handled = true;
                break;
            //
            // SGI region
            //
            case GIC_REDIST_IGROUPR0:
                *val = redistributors[cpu_num].sgi_region.gicr_igroupr0;
                register_handled = true;
                break;
            case GIC_REDIST_ISENABLER0:
                *val = redistributors[cpu_num].sgi_region.gicr_isenabler0;
                register_handled = true;
                break;
            case GIC_REDIST_ICENABLER0:
                *val = redistributors[cpu_num].sgi_region.gicr_icenabler0;
                register_handled = true;
                break;
            case GIC_REDIST_ISPENDR0:
                *val = redistributors[cpu_num].sgi_region.gicr_ispendr0;
                register_handled = true;
                break;
            case GIC_REDIST_ICPENDR0:
                *val = redistributors[cpu_num].sgi_region.gicr_icpendr0;
                register_handled = true;
                break;
            case GIC_REDIST_ISACTIVER0:
                *val = redistributors[cpu_num].sgi_region.gicr_isactiver0;
                register_handled = true;
                break;
            case GIC_REDIST_ICACTIVER0:
                *val = redistributors[cpu_num].sgi_region.gicr_icactiver0;
                register_handled = true;
                break;
            case GIC_REDIST_ICFGR0:
                *val = redistributors[cpu_num].sgi_region.gicr_icfgr0;
                register_handled = true;
                break;
            case GIC_REDIST_ICFGR1:
                *val = redistributors[cpu_num].sgi_region.gicr_icfgr1;
                register_handled = true;
                break;
            case GIC_REDIST_IGRPMODR0:
                *val = redistributors[cpu_num].sgi_region.gicr_igrpmodr0;
                register_handled = true;
                break;
            case GIC_REDIST_NSACR:
                *val = redistributors[cpu_num].sgi_region.gicr_nsacr;
                register_handled = true;
                break;
            case GIC_REDIST_IPRIORITYR0 ... GIC_REDIST_IPRIORITYR3 + 3:
                // u32 reg_num;
                reg_num = (relative_addr - GIC_REDIST_IPRIORITYR0) / 4;
                reg_offset = (relative_addr - GIC_REDIST_IPRIORITYR0) % 4;

                *val = hv_vgic_diag_priority_word_read(
                    redistributors[cpu_num].sgi_region.gicr_sgi_ipriority_reg[reg_num],
                    reg_offset, width);
                register_handled = true;
                break;
            case GIC_REDIST_IPRIORITYR4 ... GIC_REDIST_IPRIORITYR7 + 3:
                // u32 reg_num;
                reg_num = (relative_addr - GIC_REDIST_IPRIORITYR4) / 4;
                reg_offset = (relative_addr - GIC_REDIST_IPRIORITYR4) % 4;
                //
                // Read path: return the stored PPI priorities. This case used to assign
                // *val INTO the register (a copy-paste of the write case) - so a guest
                // read of GICR_IPRIORITYR4-7 corrupted the byte and returned garbage.
                // Windows programs priorities with a read-modify-write, so every RMW read
                // came back 0 and wrote back a single byte, clobbering the others - the
                // timer PPI (INTID 17) ended up priority 0, which the injected LR then
                // carried, and the HAL bugchecked 0xC8 IRQL_UNEXPECTED_VALUE.
                //
                *val = hv_vgic_diag_priority_word_read(
                    redistributors[cpu_num].sgi_region.gicr_ppi_ipriority_reg[reg_num],
                    reg_offset, width);
                register_handled = true;
                break;
            default:
                //
                // an unimplemented register.
                //
                unimplemented_reg_accessed = true;
                break;
        }
        
    }
    vgic_log("HV vGIC DEBUG [INFO] [Redistributor]: 0x%llx = 0x%llx ", relative_addr, *val);
    if(write) {
        vgic_log("[Written]");
    }
    else {
        vgic_log("[Read]");
    }
    if(unimplemented_reg_accessed) {
        vgic_log("[Unimplemented]\n");
    }
    else {
        vgic_log("\n");
    }
    return register_handled;
}

/**
 * @brief hv_vgicv3_init_dist_registers
 * 
 * Sets up the initial values for the distributor registers.
 * 
 * For registers that deal with unsupported features, set them to 0 and just never interact with them
 * 
 * For write only registers, set them to 0, and emulate the effect upon attempting to write that register.
 * Read-only registers, set their value here and don't let the guest touch their values.
 * 
 */
void hv_vgicv3_init_dist_registers(void)
{
    memset(distributor, 0, sizeof(vgicv3_dist));
    //
    // For now - taking the easy route of saying that at least 1024 IRQs are supported on all platforms.
    //
    distributor->gicd_ctl_reg = (BIT(6) | BIT(4) | BIT(1) | BIT(0));
    //
    // GIC type will be defined as the following:
    // - No extended SPIs (Update on 6/10/2025: maz in Asahi IRC says we can probably expose extended SPIs? could also look into some other hacks for > 1024 IRQ platforms)
    // - Affinity level 0 can go up to 15
    // - 1 of N SPI interrupts are supported (kind of how AIC2 can behave?)
    // - Affinity 3 invalid
    // - 16 interrupt ID bits (to match what the CPU interface supports)
    // - LPIs/MSIs supported (MSIs not using an ITS)
    //
    distributor->gicd_type_reg = (BIT(22) | BIT(21) | BIT(20) | BIT(19) | BIT(17) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0));
    distributor->gicd_imp_id_reg = (BIT(10) | BIT(5) | BIT(4) | BIT(3) | BIT(1) | BIT(0));
    distributor->gicd_type_reg_2 = 0; 
    distributor->gicd_err_sts = 0;
    return;

}

void hv_vgicv3_assign_redist_affinity_value(u16 cpu_num, bool last_cpu) {
    u32 cpu_affinity_value;
    uint64_t mpidr_val;
    uint64_t gicr_typer;
    mpidr_val = smp_get_mpidr(cpu_num);
    //
    // Affinity level 3 is always 0.
    //
    cpu_affinity_value = (0 << 24);
    //
    // Affinity level 2 signifies if we're targeting a P-core or E-core cluster.
    // (0x0 for an E-core, 0x1 for a P-core)
    //
    cpu_affinity_value |= ((mpidr_val >> 16) & 0xFF) << 16;
    //
    // Affinity level 1 signifies the cluster number on the local die (for multi-die systems it's cluster_num + (die_num * 8)).
    //
    cpu_affinity_value |= ((mpidr_val >> 8) & 0xFF) << 8;
    //
    // Affinity level 0 is the core number on the local cluster.
    //
    cpu_affinity_value |= ((mpidr_val) & 0xFF);
    gicr_typer = (((uint64_t)cpu_affinity_value) << 32);
    //
    // Apple silicon platforms (at least the M1 and M2 and the Pro counterparts) do not support the extended PPI/SPI ranges
    // so bits 31:27 remain 0. If M3 or M4 do support the extended ranges, check the Chip ID here and toggle those bits.
    // (Unlikely, as even though M1 Ultra has > 16 cores, we do not have those ranges on that platform, which means we probably will need
    // to have a solution for those platforms.)
    //
    // We're also sharing a common LPI configuration table across all the vCPUs.
    //
    // Put the processor/CPU number in the right field here, 
    // the way we're doing it here ensures we have a one to one mapping with how m1n1 identifies the CPUs.
    //
    gicr_typer |= (cpu_num << 8);

    //
    // Leave out MPAM support for now - we can't assume the CPU supports it.
    // Let processors opt out of interrupts though. (bit 5, GICR_TYPER.DPGS bit)
    //
    gicr_typer |= BIT(5);
    if(last_cpu == true) {
        //
        // this is the last redistributor, set bit 4 to indicate this.
        //
        gicr_typer |= BIT(4);
    }

    //
    // If we need an ITS, we need to comment out the bottom line, since we wouldn't be supporting direct LPI injection to
    // redistributors.
    //
    gicr_typer |= BIT(3);
    //
    // say that we have physical LPIs to be safe.
    //
    gicr_typer |= BIT(0);

    //
    // write back this feature set to our vGIC registers.
    //
    redistributors[cpu_num].rd_region.gicr_type_reg = gicr_typer;

    return;

}

void hv_vgicv3_init_redist_registers(void) {
    memset(redistributors, 0, (sizeof(vgicv3_vcpu_redist) * num_cpus));
    for(u16 i = 0; i < num_cpus; i++) {
        bool last_cpu = (i + 1 == num_cpus) ? true : false;
        redistributors[i].rd_region.gicr_ctl_reg = (BIT(2) | BIT(1));
        redistributors[i].rd_region.gicr_iidr = (BIT(10) | BIT(5) | BIT(4) | BIT(3) | BIT(1) | BIT(0));
        //
        // assign affinity values to redistributors.
        //
        hv_vgicv3_assign_redist_affinity_value(i, last_cpu);
        redistributors[i].rd_region.gicr_status_reg = 0;
        redistributors[i].rd_region.gicr_wake_reg = (BIT(2) | BIT(1)); //GICR_WAKER reset values, currently not using bits 31 or 0.
        //
        // Generate and set the LPI configuration table here.
        // (Right now this is ignored just to test if stuff is working since we have no MSIs and LPIs are disabled right now.)
        //
        

    }
}


/**
 * @brief hv_vgicv3_init_list_registers
 * 
 * Enables the platform's list registers for use by the guest OS.
 * 
 */
//
// How many list registers this core actually has, read from ICH_VTR_EL2.ListRegs (bits 4:0,
// encoded as count-1). Per-CPU on purpose: the M1 is heterogeneous (4 E cores in cluster 0,
// 4 P cores in cluster 1) and nothing guarantees both clusters expose the same number. The
// code used to loop to a hardcoded 8, which silently reads and writes registers a core may
// not implement - the kind of assumption that breaks on exactly one cluster and is then very
// hard to attribute.
//
static u8 vgic_num_lrs[MAX_CPUS];

int hv_vgic3_num_lrs(void)
{
    int n = vgic_num_lrs[smp_id()];
    return n ? n : 8; // before per-CPU init has run, keep the historical assumption
}

void hv_vgicv3_init_list_registers(void)
{
    vgic_num_lrs[smp_id()] = (mrs(ICH_VTR_EL2) & 0x1f) + 1;

    msr(ICH_LR0_EL2, 0);
    msr(ICH_LR1_EL2, 0);
    msr(ICH_LR2_EL2, 0);
    msr(ICH_LR3_EL2, 0);
    msr(ICH_LR4_EL2, 0);
    msr(ICH_LR5_EL2, 0);
    msr(ICH_LR6_EL2, 0);
    msr(ICH_LR7_EL2, 0);
}


/**
 * @brief hv_vgicv3_enable_virtual_interrupts
 * 
 * Enables virtual interrupts for the guest.
 * 
 * Note that actual interrupts are always handled by m1n1, then passed onto the vGIC which will signal the virtual interrupt to the OS.
 * 
 * @return
 * 0 - success
 * -1 - there was an error.
 */

int hv_vgicv3_enable_virtual_interrupts(void)
{
    //set VMCR to reset values, then enable virtual group 0 and 1 interrupts
    msr(ICH_VMCR_EL2, 0);
    //
    // VPMR (bits 31:24) is the guest's virtual priority mask. Leaving it at 0 blocks
    // *every* virtual interrupt: the GIC only signals an interrupt whose priority is
    // numerically lower than the mask, and nothing is lower than 0. Measured symptom:
    // timer interrupts were injected into list registers (free_lr climbing 0,1,2,...)
    // and never acknowledged, because the guest could not be signalled at all.
    //
    // The architectural reset value really is 0, and a guest that programs ICC_PMR_EL1
    // itself would fix it -- but EDK2 never got that far here, so open the mask during
    // bring-up.
    //
    // VENG0/VENG1 stay clear: IAR/EOIR are emulated, so the hardware never sees an
    // interrupt become active, its ICH_AP1R stays empty and, with VENG1 set, it signals
    // every Pending LR under VPMR regardless of the emulated running priority.  EXP1894:
    // a pending timer LR blocked by an ACTIVE SGI was signalled, the emulated IAR
    // returned spurious and five vCPUs re-entered the IRQ vector forever.  The virtual
    // IRQ is driven only through HCR_EL2.VI by hv_vgic3_update_vi().
    //
    msr(ICH_VMCR_EL2, 0xFFUL << 24);
    //bit 0 enables the virtual CPU interface registers
    //AMO/IMO/FMO set by m1n1 on boot
    //
    // TALL1 (bit 12) traps the guest's group 1 ICC_* accesses to EL2. Without it the
    // guest talks straight to the hardware virtual CPU interface and hv_vgic3_do_iar1()
    // / hv_vgic3_do_eoir1() -- which exist in this tree already -- are never reached, so
    // an injected interrupt is never acknowledged and never completed. Measured: list
    // registers filled up (free_lr climbing 0,1,2,...) with no IAR/EOIR from the guest.
    //
    // TC (bit 10) traps the registers common to both groups -- SRE, CTLR, PMR -- which
    // TALL1 does not cover. All three are served in hv_handle_msr_unlocked().
    msr(ICH_HCR_EL2, (BIT(0) | BIT(2) | BIT(10) | BIT(12)));


    return 0;
}

//
// SGI and PPI priorities live in each core's own redistributor, so a cross-core SGI has to be
// injected with the priority the TARGET core programmed, not the sender's. Windows maps GIC
// priority onto IRQL; delivering an IPI at the wrong priority makes the kernel take it at an
// IRQL it did not expect, which is exactly how IRQL_NOT_LESS_OR_EQUAL appears.
//
u8 hv_vgic3_get_priority_cpu(u64 intd, int cpu){
    u64 reg_num = 0;
    u64 reg_offset = 0;
    u8 *reg_val = NULL;

    if(cpu < 0 || cpu >= (int)num_cpus)
        cpu = smp_id();

    if(intd <= 15){
        reg_num = intd / 4;
        reg_offset = intd % 4;
        reg_val = (u8 *)&redistributors[cpu].sgi_region.gicr_sgi_ipriority_reg[reg_num];
    }
    else if(intd >= 16 && intd <= 31){
        reg_num = (intd - 16) / 4;
        reg_offset = (intd - 16) % 4;
        reg_val = (u8 *)&redistributors[cpu].sgi_region.gicr_ppi_ipriority_reg[reg_num];
    }
    else if(intd < 1020){
        // GICD_IPRIORITYR<n> holds INTIDs 4n..4n+3 counted from INTID 0, exactly as the
        // distributor write handler stores them.  Indexing SPIs from INTID 32 gave every
        // SPI another line's priority (EXP1898: vINTID 889 read 0x00 instead of 0x60).
        return hv_vgic_diag_spi_priority(distributor->gicd_interrupt_priority_regs, intd);
    }
    else{
        // Special INTIDs have no priority register: report the idle priority.
        return 0xff;
    }
    reg_val += reg_offset;

    return *reg_val;
}

u8 hv_vgic3_get_priority(u64 intd){
    return hv_vgic3_get_priority_cpu(intd, smp_id());
}

bool hv_vgic3_irq_enabled(u32 intid)
{
    if (intid < 32)
        return redistributors[smp_id()].sgi_region.gicr_isenabler0 & BIT(intid);
    if (intid >= 1024)
        return false;
    return distributor->gicd_interrupt_set_enable_regs[intid / 32] & BIT(intid % 32);
}

int hv_vgic3_get_free_lr(void)
{
    u64 elrsr = mrs(ICH_ELRSR_EL2);
    if (!elrsr)
        return -1;
    return __builtin_ctzll(elrsr);
}

u64 hv_vgic3_read_lr(u32 lr_num){
    switch(lr_num){
        case 0:
            return mrs(ICH_LR0_EL2);
            break;
        case 1:
            return mrs(ICH_LR1_EL2);
            break;
        case 2:
            return mrs(ICH_LR2_EL2);
            break;
        case 3:
            return mrs(ICH_LR3_EL2);
            break;
        case 4:
            return mrs(ICH_LR4_EL2);
            break;
        case 5:
            return mrs(ICH_LR5_EL2);
            break;
        case 6:
            return mrs(ICH_LR6_EL2);
            break;
        case 7:
            return mrs(ICH_LR7_EL2);
            break;
    }
    // Out-of-range LR: return an invalid (State=0) entry rather than fall off the end
    // (which was undefined behaviour and returned garbage).
    return 0;
}

void hv_vgic3_get_diag_snapshot(struct hv_vgic_diag_snapshot *out)
{
    u64 lrs[HV_VGIC_DIAG_LR_COUNT];

    for (u32 i = 0; i < HV_VGIC_DIAG_LR_COUNT; i++)
        lrs[i] = hv_vgic3_read_lr(i);
    hv_vgic_diag_classify_lrs(lrs, out);
}

void hv_vgic3_write_lr(u32 lr_num, u64 lr_val){
    switch(lr_num){
        case 0:
            msr(ICH_LR0_EL2, lr_val);
            break;
        case 1:
            msr(ICH_LR1_EL2, lr_val);
            break;
        case 2:
            msr(ICH_LR2_EL2, lr_val);
            break;
        case 3:
            msr(ICH_LR3_EL2, lr_val);
            break;
        case 4:
            msr(ICH_LR4_EL2, lr_val);
            break;
        case 5:
            msr(ICH_LR5_EL2, lr_val);
            break;
        case 6:
            msr(ICH_LR6_EL2, lr_val);
            break;
        case 7:
            msr(ICH_LR7_EL2, lr_val);
            break;
    }
    sysop("isb");
}


static u32 trace_intid = 0xffffffff;
static u32 trace_budget;

void hv_vgic3_trace_intid(u32 intid, u32 budget)
{
    trace_intid = intid;
    trace_budget = budget;
}

static bool trace_take(u32 intid)
{
    if (intid != trace_intid || !trace_budget)
        return false;
    trace_budget--;
    return true;
}

static u32 timer_inject_count[MAX_CPUS][2];
static u32 timer_iar_count[MAX_CPUS][2];
static u32 timer_eoi_count[MAX_CPUS][2];

static bool timer_trace_count(u32 intid, u32 counts[MAX_CPUS][2], u32 *count)
{
    if (intid != 17 && intid != 18)
        return false;
    int cpu = smp_id();
    if (cpu < 0 || cpu >= MAX_CPUS)
        cpu = 0;
    *count = ++counts[cpu][intid - 17];
    return *count <= 8 || ((*count & 1023) == 0);
}

void hv_vgic3_inject_irq(u32 vintid, u8 priority, bool active, bool pending, bool hw_status, u64 hw_irq){
    u64 val = 0;
    val |= (u64)(vintid & ICH_LR_VIRTUAL_MASK) << ICH_LR_VIRTUAL_SHIFT;
    val |= (u64)(priority & ICH_LR_PRIORITY_MASK) << ICH_LR_PRIORITY_SHIFT;
    val |= ICH_LR_GRP1;

    if(active)
        val |= ICH_LR_STATE_ACTIVE;
    if(pending)
        val |= ICH_LR_STATE_PENDING;
    if(hw_status){
        val |= ICH_LR_HW;
        val |= hw_irq << ICH_LR_PHYSICAL_SHIFT;
    }
    else{
        val |= ICH_LR_MAINTENANCE_IRQ;
    }
    

    u64 elrsr = mrs(ICH_ELRSR_EL2);
    int free_lr = hv_vgic3_get_free_lr();
    hv_vgic3_write_lr(free_lr, val);
    hv_diag_count_vgic_irq(HV_DIAG_IRQ_INJECT, vintid, hv_pci_intx_irq());

    hv_vgic3_update_vi();
    sysop("isb");
    u32 timer_count;
    if (hv_runtime_diag_verbose_enabled() &&
        timer_trace_count(vintid, timer_inject_count, &timer_count))
        printf("HV TIMER INJECT: cpu=%d intid=%u count=%u lr=%d prio=0x%x "
               "VMCR=0x%lx HCR=0x%lx\n",
               smp_id(), vintid, timer_count, free_lr, priority,
               mrs(ICH_VMCR_EL2), mrs(HCR_EL2));
    if (hv_runtime_diag_verbose_enabled() && trace_take(vintid))
        HV_RUNTIME_VERBOSE_TRACE("HV: NVMe IRQ inject intid=%u prio=0x%x lr=%d value=0x%lx "
                                 "ELRSR=0x%lx VMCR=0x%lx HCR=0x%lx\n",
                                 vintid, priority, free_lr, val, elrsr, mrs(ICH_VMCR_EL2),
                                 mrs(HCR_EL2));
}

//
// Decide whether the virtual IRQ line should be raised, honouring the guest's priority
// mask. HCR_EL2.VI is used because the M1's GIC virtual CPU interface does not signal on
// its own (measured: valid pending LR, group 1 enabled, VPMR open - and the guest never
// takes the interrupt). But VI bypasses the GIC's signalling logic entirely, so the
// masking the hardware would have done must be reproduced here: a pending group-1
// interrupt signals only if group 1 is enabled and its priority is numerically lower
// than the guest's priority mask.
//
// Found the hard way: raising VI unconditionally delivers interrupts that the guest's
// IRQL says are masked (Windows maps IRQL onto ICC_PMR_EL1, which this hypervisor traps
// straight into VMCR.VPMR). The HAL checks that invariant on interrupt entry and
// bugchecks 0xC8 IRQL_UNEXPECTED_VALUE - which then nested into the 0xA abort and the
// 0x1E stop this took a week to unpick.
//
//
// Running priority = the highest-priority (numerically lowest) currently-active virtual
// interrupt, or 0xff (idle) when none is active. We emulate IAR/EOI in software, so the
// hardware ICH_AP1R/ICV_RPR are never populated; this derives the same value from the
// active LR states instead. The HAL reads ICC_RPR_EL1 on interrupt entry to sanity-check
// the IRQL it is about to set; returning garbage (0) there is a direct cause of 0xC8.
//
u8 hv_vgic3_running_priority(void){
    u8 rp = 0xff;
    for(int lr = 0; lr < hv_vgic3_num_lrs(); lr++){
        u64 v = hv_vgic3_read_lr(lr);
        if(v & ICH_LR_STATE_ACTIVE){
            u8 prio = (v >> ICH_LR_PRIORITY_SHIFT) & ICH_LR_PRIORITY_MASK;
            if(prio < rp)
                rp = prio;
        }
    }
    return rp;
}

void hv_vgic3_update_vi(void){
    u64 vmcr = mrs(ICH_VMCR_EL2);
    u8 vpmr = (vmcr >> 24) & 0xff;
    u8 running_priority = hv_vgic3_running_priority();
    bool signal = false;

    // Group 1 is always enabled: VMCR.VENG1 is kept clear so the hardware interface
    // cannot signal behind the emulated running priority
    // (see hv_vgicv3_enable_virtual_interrupts).
    for(int lr = 0; lr < hv_vgic3_num_lrs(); lr++){
        u64 lr_val = hv_vgic3_read_lr(lr);
        // Exact State == Pending (0b01), not merely the pending bit: a
        // pending+active LR (0b11) must not re-signal while it is active.
        if(((lr_val >> ICH_LR_STATE_SHIFT) & ICH_LR_STATE_MASK) != 1)
            continue;
        u8 priority = (lr_val >> ICH_LR_PRIORITY_SHIFT) & ICH_LR_PRIORITY_MASK;
        if(hv_vgic_diag_priority_deliverable(priority, vpmr, running_priority)){
            signal = true;
        }
    }

    if(signal)
        hv_write_hcr(mrs(HCR_EL2) | HCR_VI);
    else
        hv_write_hcr(mrs(HCR_EL2) & ~HCR_VI);
}

int hv_vgic3_do_iar1(void){
    //
    // found_lr must be a signed int: it used to be `u8 found_lr = -1`, which holds 255,
    // and `found_lr != -1` promotes to `255 != -1` == always true - so the spurious path
    // below was unreachable and a no-pending IAR fell into hv_vgic3_read_lr(255) (a
    // switch with no default -> undefined behaviour, a garbage INTID). Windows re-enters
    // the IRQ because VI was left asserted after the first ack (see below), reads IAR a
    // second time with nothing Pending, and that garbage INTID is what the HAL bugchecks
    // 0xC8 on.
    //
    int found_lr = -1;
    u8 found_priority = 0xff;
    u8 vpmr = (mrs(ICH_VMCR_EL2) >> 24) & 0xff;
    u8 running_priority = hv_vgic3_running_priority();
    for(int lr = 0; lr < hv_vgic3_num_lrs(); lr++){
        u64 lr_val = hv_vgic3_read_lr(lr);
        // Only a purely Pending (0b01) group-1 LR may be acknowledged; an
        // Active+Pending (0b11) one must not be re-acknowledged while it is active.
        if(((lr_val >> ICH_LR_STATE_SHIFT) & ICH_LR_STATE_MASK) != 1)
            continue;
        if(!(lr_val & ICH_LR_GRP1))
            continue;
        u8 priority = (lr_val >> ICH_LR_PRIORITY_SHIFT) & ICH_LR_PRIORITY_MASK;
        if(!hv_vgic_diag_priority_deliverable(priority, vpmr, running_priority))
            continue;
        if(found_lr < 0 || priority < found_priority){
            found_lr = lr;
            found_priority = priority;
        }
    }

    if(found_lr < 0){
        // Nothing to acknowledge: return the architectural spurious INTID and re-evaluate
        // the line. Diagnostic: if this fires while VI is asserted, the re-entry chain is
        // confirmed.
        // Atomic: this counter is shared across cores once SMP is on, and a plain ++ there
        // would both race and let the print fire far more than the intended 16 times.
        static u32 spurious_iar = 0;
        if(hv_runtime_trace_enabled() &&
           __atomic_fetch_add(&spurious_iar, 1, __ATOMIC_RELAXED) < 16)
            printf("VGIC IAR1 SPURIOUS: HCR=%lx VMCR=%lx\n", mrs(HCR_EL2), mrs(ICH_VMCR_EL2));
        hv_vgic3_update_vi();
        return 0x3FF;
    }

    u64 lr_val = hv_vgic3_read_lr(found_lr);
    u64 before = lr_val;
    u32 intid = (lr_val >> ICH_LR_VIRTUAL_SHIFT) & ICH_LR_VIRTUAL_MASK;
    lr_val &= ~ICH_LR_STATE_PENDING;
    lr_val |= ICH_LR_STATE_ACTIVE;
    hv_vgic3_write_lr(found_lr, lr_val);
    hv_diag_count_vgic_irq(HV_DIAG_IRQ_IAR, intid, hv_pci_intx_irq());
    hv_irq_diag_vgic_iar(intid);
    if (intid < 16)
        hv_sgi_diag_vgic_event(HV_SGI_DIAG_IAR);

    //
    // Acknowledging the only Pending LR must deassert VI immediately - not wait until
    // EOIR. Leaving VI asserted after the ack lets the guest re-enter the IRQ and take a
    // second, spurious IAR.
    //
    hv_vgic3_update_vi();
    sysop("isb");
    u32 timer_count;
    if (hv_runtime_diag_verbose_enabled() &&
        timer_trace_count(intid, timer_iar_count, &timer_count))
        printf("HV TIMER IAR: cpu=%d intid=%u count=%u lr=%d before=0x%lx "
               "after=0x%lx\n",
               smp_id(), intid, timer_count, found_lr, before, lr_val);
    if (hv_runtime_diag_verbose_enabled() && trace_take(intid))
        HV_RUNTIME_VERBOSE_TRACE(
            "HV: NVMe IRQ IAR intid=%u lr=%d before=0x%lx after=0x%lx HCR=0x%lx\n", intid,
            found_lr, before, lr_val, mrs(HCR_EL2));

    return intid;
}

void hv_vgic3_do_eoir1(u64 reg){
    u32 intd = reg & ICH_LR_VIRTUAL_MASK;
    int trace_lr = -1;
    u64 trace_before = 0;
    for(int lr = 0; lr < hv_vgic3_num_lrs(); lr++){
        u64 lr_val = hv_vgic3_read_lr(lr);
        //vgic_log("CHECKING LR: 0x%lx %d %d %d\n", lr_val, intd, (lr_val >> ICH_LR_VIRTUAL_SHIFT) & ICH_LR_VIRTUAL_MASK, lr_val & ICH_LR_STATE_ACTIVE);
        if( ((lr_val >> ICH_LR_VIRTUAL_SHIFT) & ICH_LR_VIRTUAL_MASK) == intd && (lr_val & ICH_LR_STATE_ACTIVE)){
            //vgic_log("DOING EOIR 0x%lx, found LR%d: 0x%lx, setting to 0\n", reg, lr, lr_val);
            trace_lr = lr;
            trace_before = lr_val;
            if (intd < 16 && (lr_val & ICH_LR_STATE_PENDING))
                hv_sgi_diag_vgic_event(HV_SGI_DIAG_EOI_ACTIVE_PENDING);
            hv_vgic3_write_lr(lr, hv_vgic_diag_eoi_lr(lr_val));
        }
    }

    /*
     * LR state is emulated in software, so clearing an LR here does not produce
     * the hardware maintenance interrupt that used to be the queue's only
     * drain point. Without this, a level IRQ queued while all LRs were busy
     * remained physically masked forever (observed as xHCI IMAN.IP stuck at 1).
     */
    hv_vgic3_drain_irq_queue();
    hv_vgic3_update_vi();
    hv_nvme_irq_eoi(intd);
    u32 hw_irq;
    bool enabled = distributor->gicd_interrupt_set_enable_regs[intd / 32] & BIT(intd % 32);
    if (hv_irq_route_level_eoi_target(intd, enabled, &hw_irq))
        aic_set_mask(hw_irq, false);
    if (trace_lr >= 0)
        hv_diag_count_vgic_irq(HV_DIAG_IRQ_EOI, intd, hv_pci_intx_irq());
    if (trace_lr >= 0)
        hv_irq_diag_vgic_eoi(intd);
    if (trace_lr >= 0 && intd < 16)
        hv_sgi_diag_vgic_event(HV_SGI_DIAG_EOI);
    u32 timer_count;
    if (hv_runtime_diag_verbose_enabled() && trace_lr >= 0 &&
        timer_trace_count(intd, timer_eoi_count, &timer_count))
        printf("HV TIMER EOI: cpu=%d intid=%u count=%u lr=%d before=0x%lx\n",
               smp_id(), intd, timer_count, trace_lr, trace_before);
    if (hv_runtime_diag_verbose_enabled() && trace_lr >= 0 && trace_take(intd))
        HV_RUNTIME_VERBOSE_TRACE(
            "HV: NVMe IRQ EOI intid=%u lr=%d before=0x%lx after=0 HCR=0x%lx\n", intd,
            trace_lr, trace_before, mrs(HCR_EL2));
}

void hv_vgic3_set_igrpen1(u64 reg){
    igrpen1 = reg;
    if(reg == 0){
        for(int lr = 0; lr < hv_vgic3_num_lrs(); lr++)
            hv_vgic3_write_lr(lr, 0);
    }
    hv_vgic3_update_vi();
}

u64 hv_vgic3_get_igrpen1(void){
    return igrpen1;
}

#endif

/**
 * @brief hv_vgicv3_init
 * 
 * Initializes the vGIC and prepares it for use by the guest OS.
 * 
 * Note that this function is only expected to be called once.
 * 
 * @return 
 * 
 * 0 - success, vGIC is ready for use by the guest
 * -1 - an error has occurred during vGIC initialization, refer to m1n1 output log for details on the specific error 
 */

void hv_vgicv3_init(void)
{
#ifdef ENABLE_VGIC_MODULE
    printf("HV vGIC DEBUG: start\n");
    vgic_inited = false;
    //
    // First things first - set the parameters appropriately based on whether
    // we're running on a 36-bit or 42-bit platform.
    // Also for now, we are catering to "lowest common denominator" for all chips,
    // so on more powerful systems we may not be using all cores.
    //
    switch(chip_id) {
        case T8103:
        case T8112:
            dist_base = DIST_BASE_36_BIT;
            redist_base = REDIST_BASE_36_BIT;
            its_base = ITS_BASE_36_BIT;
            num_cpus = 8;
            break;
        // case T8010:
        // case T8015:
        // case T8011:
        // case T8012:
        //     dist_base = DIST_BASE_36_BIT;
        //     redist_base = REDIST_BASE_36_BIT;
        //     its_base = ITS_BASE_36_BIT;
        //     break;
        case T6000:
            dist_base = DIST_BASE_42_BIT;
            redist_base = REDIST_BASE_42_BIT;
            its_base = ITS_BASE_42_BIT;
            num_cpus = 8; // cannot assume that we have 10 cores for M1 Pro
        case T6001:
            dist_base = DIST_BASE_42_BIT;
            redist_base = REDIST_BASE_42_BIT;
            its_base = ITS_BASE_42_BIT;
            num_cpus = 10;
        case T6002:
            dist_base = DIST_BASE_42_BIT;
            redist_base = REDIST_BASE_42_BIT;
            its_base = ITS_BASE_42_BIT;
            num_cpus = 20;
        case T6020:
            dist_base = DIST_BASE_42_BIT;
            redist_base = REDIST_BASE_42_BIT;
            its_base = ITS_BASE_42_BIT;
            num_cpus = 10;
        case T6021:
            dist_base = DIST_BASE_42_BIT;
            redist_base = REDIST_BASE_42_BIT;
            its_base = ITS_BASE_42_BIT;
            num_cpus = 12;
        case T6022:
            dist_base = DIST_BASE_42_BIT;
            redist_base = REDIST_BASE_42_BIT;
            its_base = ITS_BASE_42_BIT;
            num_cpus = 24;
        // case T6030:
        // case T6031:
        // case 0x6032:
        // case T6034:
        // case 0x6040:
        // case 0x6041:
        // case T8122:
        //     dist_base = DIST_BASE_42_BIT;
        //     redist_base = REDIST_BASE_42_BIT;
        //     its_base = ITS_BASE_42_BIT;
        //     break;
    }
    //
    // Step 1 - distributor setup.
    //
    printf("HV vGIC DEBUG: setting up distributor\n");
    distributor = heapblock_alloc(sizeof(vgicv3_dist));
    hv_vgicv3_init_dist_registers();
    //
    // Map the vGIC distributor into unoccupied MMIO space.
    //
    printf("HV vGIC DEBUG: mapping distributor into guest space\n");
    hv_map_hook(dist_base, handle_vgic_dist_access, 0x10000);


    /* Redistributor setup */
    printf("HV vGIC DEBUG: setting up redistributors\n");
    redistributors = heapblock_alloc(sizeof(vgicv3_vcpu_redist) * num_cpus);
    hv_vgicv3_init_redist_registers();
    printf("HV vGIC DEBUG: mapping redistributors into guest space\n");
    hv_map_hook(redist_base, handle_vgic_redist_access,
                HV_VGIC_REDIST_STRIDE * num_cpus);

    //
    // ITS setup (for MSIs - PCIe devices usually signal via these.)
    // Disabled for now, seems like direct injection into the guest is easier.
    //
    interrupt_translation_service = heapblock_alloc(sizeof(vgicv3_its));
    hv_map_hook(its_base, handle_vgic_its_access, 0x10000);

    //vGIC setup is complete.
    if (chip_id == T8103)
        hv_vgic_rearm_j313_xhci_trace();
    vgic_inited = true;
    return;
#else
    printf("HV vGIC DEBUG: Disabled\n");
    return;
#endif //ENABLE_VGIC_MODULE
}

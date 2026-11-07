/* SPDX-License-Identifier: MIT */

// Standards-facing NVMe controller for the Windows/UEFI guest. The guest queue engine is
// kept in hv_nvme_queue.c so its command and PRP semantics can be tested on a host. This
// file owns the BAR register model, Apple ANS backend connection, and virtual INTx line.

#include "hv.h"
#include "hv_runtime_diag.h"
#include "hv_nvme_fast_path.h"
#include "hv_nvme_gpt.h"
#include "hv_nvme_queue.h"
#include "hv_vgic.h"
#include "malloc.h"
#include "nvme.h"
#include "smp.h"
#include "string.h"
#include "types.h"
#include "utils.h"

#define NVME_CAP   0x00
#define NVME_VS    0x08
#define NVME_INTMS 0x0c
#define NVME_INTMC 0x10
#define NVME_CC    0x14
#define NVME_CSTS  0x1c
#define NVME_AQA   0x24
#define NVME_ASQ   0x28
#define NVME_ACQ   0x30
#define NVME_DBS   0x1000

#define CC_EN         BIT(0)
#define CC_CSS        GENMASK(6, 4)
#define CC_MPS        GENMASK(10, 7)
#define CC_SHN        GENMASK(15, 14)
#define CC_IOSQES     GENMASK(19, 16)
#define CC_IOCQES     GENMASK(23, 20)
#define CC_SHN_NORMAL FIELD_PREP(CC_SHN, 1)

#define CSTS_RDY           BIT(0)
#define CSTS_CFS           BIT(1)
#define CSTS_SHST          GENMASK(3, 2)
#define CSTS_SHST_COMPLETE FIELD_PREP(CSTS_SHST, 2)

// 256-entry queues, contiguous queues required, five-second ready timeout, 4 KiB pages,
// and the NVM command set. DSTRD is zero, so doorbells have a four-byte stride.
#define NVME_CAP_VALUE    ((VNVME_MAX_QSIZE - 1ULL) | BIT(16) | (10ULL << 24) | (1ULL << 37))
#define NVME_VS_VALUE     0x00010300
#define NVME_BAR_SIZE     0x1000000
#define NVME_TRACE_BUDGET 192

extern int hv_pci_intx_irq(void);

static u64 bar_base;
static DECLARE_SPINLOCK(nvme_lock);
static bool backend_ready;
static u64 backend_blocks;
static u64 backend_minimum_bytes;
static struct hv_nvme_disk_slice backend_slice;
static bool backend_bounded;
static bool backend_installation;
static struct vnvme_intx_delivery irq_delivery;
static u32 nvme_trace_budget;
static struct vnvme_ctrl queue_ctrl;
static bool irq_eoi_pending;
static u64 irq_owner_kicks;
static u64 irq_owner_polls;
/* Receipt-only: CNTPCT of the last guest CQ-head write and the last INTx injection. */
static u64 last_cq_doorbell_ticks;
static u64 last_inject_ticks;
static u64 irq_renotifies;

static struct {
    u32 intms;
    u32 cc;
    u32 csts;
    u32 aqa;
    u64 asq;
    u64 acq;
} regs;

static bool nvme_trace_take(void)
{
    if (!hv_runtime_diag_enabled())
        return false;
    if (!nvme_trace_budget)
        return false;
    nvme_trace_budget--;
    return true;
}

static bool backend_read(void *opaque, u64 lba, void *buffer)
{
    UNUSED(opaque);
    u64 physical;
    return hv_nvme_slice_translate(&backend_slice, lba, 1, &physical) &&
           nvme_read(1, physical, buffer);
}

static bool backend_write(void *opaque, u64 lba, const void *buffer)
{
    UNUSED(opaque);
    u64 physical;
    return hv_nvme_slice_translate(&backend_slice, lba, 1, &physical) &&
           nvme_write(1, physical, buffer);
}

static bool backend_read_blocks(void *opaque, u64 lba, u32 blocks, void *buffer)
{
    UNUSED(opaque);
    u64 physical;
    return hv_nvme_slice_translate(&backend_slice, lba, blocks, &physical) &&
           nvme_read_blocks(1, physical, blocks, buffer);
}

static bool backend_write_blocks(void *opaque, u64 lba, u32 blocks, const void *buffer)
{
    UNUSED(opaque);
    u64 physical;
    return hv_nvme_slice_translate(&backend_slice, lba, blocks, &physical) &&
           nvme_write_blocks(1, physical, blocks, buffer);
}

static bool backend_flush(void *opaque)
{
    UNUSED(opaque);
    return nvme_flush(1);
}

static void backend_publish(void *opaque)
{
    UNUSED(opaque);
    dma_wmb();
}

static void try_raise_intx(void)
{
    /* Until vGIC affinity routing is implemented, keep this INTx and its LR on CPU0. */
    int current_cpu = smp_id();
    if (current_cpu != boot_cpu_idx) {
        /*
         * The CQ doorbell that publishes the next synchronous completion can trap on any
         * guest CPU.  The synthetic INTx LR, however, belongs to the boot vCPU.  Wake that
         * owner explicitly instead of waiting for an unrelated CPU0 exit from the guest;
         * otherwise a completed read can sit unnoticed until stornvme's ten-second reset.
         */
        if (vnvme_intx_delivery_should_kick_owner(&irq_delivery, queue_ctrl.irq_asserted,
                                                   regs.intms, current_cpu, boot_cpu_idx)) {
            irq_owner_kicks++;
            if (hv_runtime_diag_verbose_enabled() && nvme_trace_take())
                HV_RUNTIME_VERBOSE_TRACE(
                    "HV: NVMe INTx owner kick source=%d owner=%d count=%lu\n", current_cpu,
                    boot_cpu_idx, irq_owner_kicks);
            smp_send_ipi(boot_cpu_idx);
        }
        return;
    }

    vnvme_intx_delivery_owner_polled(&irq_delivery);
    irq_owner_polls++;

    int irq = hv_pci_intx_irq();
    bool enabled = hv_vgic3_irq_enabled(irq);
    int free_lr = hv_vgic3_get_free_lr();
    if (!vnvme_intx_delivery_can_inject(&irq_delivery, queue_ctrl.irq_asserted, regs.intms,
                                        enabled, free_lr)) {
        if (queue_ctrl.irq_asserted && !regs.intms && !irq_delivery.outstanding && !enabled) {
            static u32 disabled_logs;
            if (disabled_logs++ < 8)
                printf("HV: NVMe INTx %d pending: GICD disabled\n", irq);
        } else if (queue_ctrl.irq_asserted && !regs.intms && !irq_delivery.outstanding &&
                   free_lr < 0) {
            static u32 no_lr_logs;
            if (no_lr_logs++ < 8)
                printf("HV: NVMe INTx %d delayed: no free vGIC LR\n", irq);
        }
        return;
    }

    hv_vgic3_inject_irq(irq, hv_vgic3_get_priority(irq), false, true, false, 0);
    vnvme_intx_delivery_mark_injected(&irq_delivery);
    last_inject_ticks = mrs(CNTPCT_EL0);
}

static u64 ticks_ago_ms(u64 then)
{
    u64 freq = mrs(CNTFRQ_EL0);
    return then && freq ? (mrs(CNTPCT_EL0) - then) * 1000 / freq : ~0ULL;
}

/*
 * Receipt-only: the guest disabled a running controller (stornvme "Reset to device",
 * System 129, follows a ten-second I/O timeout).  Record the queue and INTx delivery
 * state it gave up on before the reset clears it.
 */
static void reset_receipt(void)
{
    static u32 receipts;
    const struct vnvme_queue *q0 = &queue_ctrl.queues[0], *q1 = &queue_ctrl.queues[1];
    int irq = hv_pci_intx_irq();
    if (receipts++ >= 32)
        return;
    printf("HV: NVMe guest reset #%u irq=%d intms=0x%x out=%d notified=%d kick=%d eoipend=%d "
           "gicd=%d kicks=%lu polls=%lu renotify=%lu cqdb_ms=%lu inj_ms=%lu q0 sq=%u/%u cq=%u/%u/%u "
           "q1 sq=%u/%u cq=%u/%u/%u ie=%d cmds=%lu cpl=%lu sqdb=%lu cqdb=%lu cpu=%d\n",
           receipts, queue_ctrl.irq_asserted, regs.intms, irq_delivery.outstanding,
           irq_delivery.assertion_notified, irq_delivery.owner_kick_pending, irq_eoi_pending,
           hv_vgic3_irq_enabled(irq), irq_owner_kicks, irq_owner_polls, irq_renotifies,
           ticks_ago_ms(last_cq_doorbell_ticks), ticks_ago_ms(last_inject_ticks), q0->sq_head,
           q0->sq_tail, q0->cq_head, q0->cq_tail, q0->cq_pending, q1->sq_head, q1->sq_tail,
           q1->cq_head, q1->cq_tail, q1->cq_pending, q1->irq_enabled, queue_ctrl.stats.commands,
           queue_ctrl.stats.completions, queue_ctrl.stats.sq_doorbells,
           queue_ctrl.stats.cq_doorbells, smp_id());
}

static void drain_deferred_eoi(void)
{
    if (__atomic_exchange_n(&irq_eoi_pending, false, __ATOMIC_ACQ_REL))
        vnvme_intx_delivery_eoi(&irq_delivery);
}

bool hv_nvme_try_handle_dabort(struct exc_info *ctx, bool *matched)
{
    bool is_write = ctx->esr & ESR_ISS_DABORT_WnR;
    u64 ipa = hv_translate(ctx->far, true, is_write, NULL);
    u64 base = __atomic_load_n(&bar_base, __ATOMIC_ACQUIRE);

    if (matched)
        *matched = false;
    if (!ipa || !hv_nvme_bar_contains(base, NVME_BAR_SIZE, ipa))
        return false;

    /*
     * The queue engine performs synchronous ANS I/O.  Serializing that work
     * with the hypervisor-wide bhl stopped every vCPU during a busy doorbell,
     * eventually tripping Windows' clock watchdog.  NVMe state needs
     * serialization, but unrelated CPU timers and IPIs do not.
     */
    spin_lock(&nvme_lock);
    base = __atomic_load_n(&bar_base, __ATOMIC_ACQUIRE);
    if (!hv_nvme_bar_contains(base, NVME_BAR_SIZE, ipa)) {
        spin_unlock(&nvme_lock);
        return false;
    }

    if (matched)
        *matched = true;
    bool handled = hv_handle_dabort(ctx);
    drain_deferred_eoi();
    try_raise_intx();
    spin_unlock(&nvme_lock);
    return handled;
}

/* Renotify a still-asserted, EOIed line after 10 ms (see vnvme_intx_delivery_renotify_due). */
#define NVME_RENOTIFY_HOLDOFF_US 10000

void hv_nvme_poll_irq(void)
{
    /* Never make a bhl owner wait behind synchronous storage I/O. */
    if (!spin_try_lock(&nvme_lock))
        return;
    drain_deferred_eoi();
    if (vnvme_intx_delivery_renotify_due(&irq_delivery, queue_ctrl.irq_asserted, regs.intms,
                                         mrs(CNTPCT_EL0), last_inject_ticks,
                                         mrs(CNTFRQ_EL0) / 1000000 * NVME_RENOTIFY_HOLDOFF_US)) {
        irq_renotifies++;
        vnvme_intx_delivery_acknowledged(&irq_delivery);
        /* Receipt-only, logarithmic: how often the holdoff renotification fires. */
        if (!(irq_renotifies & (irq_renotifies - 1)))
            printf("HV: NVMe renotify count=%lu q0 cq=%u/%u q1 cq=%u/%u cpu=%d\n", irq_renotifies,
                   queue_ctrl.queues[0].cq_head, queue_ctrl.queues[0].cq_tail,
                   queue_ctrl.queues[1].cq_head, queue_ctrl.queues[1].cq_tail, smp_id());
    }
    try_raise_intx();
    spin_unlock(&nvme_lock);
}

void hv_nvme_irq_eoi(u32 intid)
{
    if (intid != (u32)hv_pci_intx_irq())
        return;
    if (!spin_try_lock(&nvme_lock)) {
        __atomic_store_n(&irq_eoi_pending, true, __ATOMIC_RELEASE);
        return;
    }
    vnvme_intx_delivery_eoi(&irq_delivery);
    try_raise_intx();
    spin_unlock(&nvme_lock);
}

void hv_nvme_get_diag_snapshot(struct vnvme_snapshot *out, bool *ready)
{
    if (!spin_try_lock(&nvme_lock)) {
        if (ready)
            *ready = false;
        return;
    }
    vnvme_get_snapshot(&queue_ctrl, out);
    if (ready)
        *ready = backend_ready && (regs.csts & CSTS_RDY);
    spin_unlock(&nvme_lock);
}

static void backend_irq(void *opaque, bool asserted)
{
    UNUSED(opaque);
    /* Track the effective wire level, including the controller's INTM mask. */
    vnvme_intx_delivery_update_line(&irq_delivery, asserted && !regs.intms);
    if (hv_runtime_diag_verbose_enabled() && nvme_trace_take())
        HV_RUNTIME_VERBOSE_TRACE("HV: NVMe INTx logical=%d masked=0x%x injected=%d\n", asserted,
                                 regs.intms, irq_delivery.outstanding);
    try_raise_intx();
}

static void backend_trace(void *opaque, const struct vnvme_trace_event *event)
{
    UNUSED(opaque);
    if (!hv_runtime_diag_verbose_enabled() || !nvme_trace_take())
        return;

    if (event->type == VNVME_TRACE_SUBMISSION) {
        printf("HV: NVMe SQ q=%u cq=%u slot=%u head=%u tail=%u opc=0x%x cid=0x%x status=0x%x "
               "irq=%d\n",
               event->qid, event->cqid, event->slot, event->head, event->tail, event->opcode,
               event->cid, event->status, event->irq_asserted);
    } else {
        printf("HV: NVMe CQE cq=%u sq=%u slot=%u head=%u tail=%u phase=%u cid=0x%x "
               "status=0x%x irq=%d\n",
               event->cqid, event->qid, event->slot, event->head, event->tail, event->phase,
               event->cid, event->status, event->irq_asserted);
    }
}

static const struct vnvme_backend_ops backend_ops = {
    .read = backend_read,
    .write = backend_write,
    .read_blocks = backend_read_blocks,
    .write_blocks = backend_write_blocks,
    .flush = backend_flush,
    .publish = backend_publish,
    .irq = backend_irq,
    .trace = backend_trace,
};

static bool read_physical_gpt(u64 lba, void *block)
{
    return nvme_read(1, lba, block);
}

static bool init_backend(bool bounded, u64 minimum_bytes, bool installation)
{
    if (backend_ready)
        return backend_bounded == bounded && backend_minimum_bytes == minimum_bytes &&
               backend_installation == installation;
    if (!nvme_init()) {
        printf("HV: NVMe backend: Apple ANS initialization failed\n");
        return false;
    }

    u32 lba_size = 0;
    u64 disk_blocks;
    if (!nvme_get_namespace_info(1, &disk_blocks, &lba_size)) {
        printf("HV: NVMe backend: Identify Namespace 1 failed\n");
        return false;
    }
    if (lba_size != VNVME_LBA_SIZE) {
        printf("HV: NVMe backend: unsupported physical LBA size %u (expected %u)\n", lba_size,
               VNVME_LBA_SIZE);
        return false;
    }

    if (bounded) {
        void *scratch = memalign(HV_NVME_GPT_BLOCK_SIZE, HV_NVME_GPT_SCRATCH_SIZE);
        if (!scratch)
            return false;
        bool found = installation ? hv_nvme_gpt_find_j316s_install(
            disk_blocks, read_physical_gpt, scratch, &backend_slice) :
            hv_nvme_gpt_find_windows_minimum(disk_blocks, read_physical_gpt, scratch,
                &backend_slice, minimum_bytes / HV_NVME_GPT_BLOCK_SIZE);
        free(scratch);
        if (!found) {
            printf("HV: NVMe backend: no verified Windows slice\n");
            return false;
        }
    } else {
        backend_slice = (struct hv_nvme_disk_slice){0, disk_blocks};
    }
    backend_blocks = backend_slice.blocks;
    backend_bounded = bounded;
    backend_installation = installation;
    backend_minimum_bytes = minimum_bytes;
    backend_ready = true;
    printf("HV: NVMe backend: namespace 1, %lu x %u-byte blocks (%lu MiB)\n", backend_blocks,
           lba_size, backend_blocks / 256);
    return true;
}

bool hv_nvme_init_backend(void)
{
    return init_backend(false, 0, false);
}

bool hv_nvme_init_windows_slice(void)
{
    return init_backend(true, 64ULL << 30, true);
}

bool hv_nvme_init_diagnostics_slice(void)
{
    return init_backend(true, 1ULL << 30, false);
}

static void reset_frontend(void)
{
    memset(&regs, 0, sizeof(regs));
    irq_delivery = (struct vnvme_intx_delivery){0};
    irq_eoi_pending = false;
    nvme_trace_budget = NVME_TRACE_BUDGET;
    vnvme_init(&queue_ctrl, backend_blocks, &backend_ops, NULL);
    hv_vgic3_trace_intid(hv_pci_intx_irq(), NVME_TRACE_BUDGET);
}

static bool valid_cc(u32 cc)
{
    return !FIELD_GET(CC_CSS, cc) && !FIELD_GET(CC_MPS, cc) && FIELD_GET(CC_IOSQES, cc) == 6 &&
           FIELD_GET(CC_IOCQES, cc) == 4;
}

static void cc_write(u32 value)
{
    u32 old = regs.cc;
    regs.cc = value;

    if ((value & CC_EN) && !(old & CC_EN)) {
        u16 asq_size = (regs.aqa & 0xfff) + 1;
        u16 acq_size = ((regs.aqa >> 16) & 0xfff) + 1;
        if (nvme_trace_take())
            printf("HV: NVMe enable CC=0x%x ASQS=%u ACQS=%u max=%u backend=%d\n", value, asq_size,
                   acq_size, VNVME_MAX_QSIZE, backend_ready);
        if (!backend_ready || !valid_cc(value) ||
            !vnvme_set_admin_queue(&queue_ctrl, regs.asq, regs.acq, asq_size, acq_size)) {
            regs.csts = CSTS_CFS;
            printf("HV: NVMe enable rejected: backend=%d CC=0x%x AQA=0x%x ASQ=0x%lx ACQ=0x%lx\n",
                   backend_ready, value, regs.aqa, regs.asq, regs.acq);
            return;
        }
        regs.csts = CSTS_RDY;
        printf("HV: NVMe ready: AQA=0x%x ASQ=0x%lx ACQ=0x%lx\n", regs.aqa, regs.asq, regs.acq);
    } else if (!(value & CC_EN) && (old & CC_EN)) {
        reset_receipt();
        vnvme_init(&queue_ctrl, backend_blocks, &backend_ops, NULL);
        irq_delivery = (struct vnvme_intx_delivery){0};
        regs.csts = 0;
    }

    if ((value & CC_SHN) == CC_SHN_NORMAL)
        regs.csts = (regs.csts & ~CSTS_SHST) | CSTS_SHST_COMPLETE;
    else if (!(value & CC_SHN))
        regs.csts &= ~CSTS_SHST;
}

static u64 register_value(u32 off)
{
    switch (off) {
        case NVME_CAP:
            return NVME_CAP_VALUE;
        case NVME_VS:
            return NVME_VS_VALUE;
        case NVME_INTMS:
        case NVME_INTMC:
            return regs.intms;
        case NVME_CC:
            return regs.cc;
        case NVME_CSTS:
            return regs.csts;
        case NVME_AQA:
            return regs.aqa;
        case NVME_ASQ:
            return regs.asq;
        case NVME_ACQ:
            return regs.acq;
        default:
            return 0;
    }
}

static u64 reg_read(u32 off, int width)
{
    u32 base = off & ~3u;
    if ((off & ~7u) == NVME_CAP || (off & ~7u) == NVME_ASQ || (off & ~7u) == NVME_ACQ)
        base = off & ~7u;
    u64 value = register_value(base);
    u32 byte_offset = off - base;
    u32 bytes = 1u << width;
    if (bytes >= 8)
        return value;
    u64 mask = (1ULL << (bytes * 8)) - 1;
    return (value >> (byte_offset * 8)) & mask;
}

static u64 merge_write(u64 old, u32 byte_offset, int width, u64 value)
{
    u32 bytes = 1u << width;
    if (bytes >= 8)
        return value;
    u64 mask = ((1ULL << (bytes * 8)) - 1) << (byte_offset * 8);
    return (old & ~mask) | ((value << (byte_offset * 8)) & mask);
}

static void reg_write(u32 off, int width, u64 value)
{
    if ((off & ~7u) == NVME_ASQ) {
        regs.asq = merge_write(regs.asq, off - NVME_ASQ, width, value);
        return;
    }
    if ((off & ~7u) == NVME_ACQ) {
        regs.acq = merge_write(regs.acq, off - NVME_ACQ, width, value);
        return;
    }

    u32 base = off & ~3u;
    u32 byte_offset = off - base;
    u32 merged = merge_write(register_value(base), byte_offset, width, value);
    switch (base) {
        case NVME_INTMS:
            if (hv_runtime_diag_verbose_enabled() && nvme_trace_take())
                HV_RUNTIME_VERBOSE_TRACE("HV: NVMe INTMS old=0x%x set=0x%x new=0x%x\n",
                                         regs.intms, merged, regs.intms | merged);
            regs.intms |= merged;
            vnvme_intx_delivery_update_line(&irq_delivery,
                                             queue_ctrl.irq_asserted && !regs.intms);
            break;
        case NVME_INTMC:
            if (hv_runtime_diag_verbose_enabled() && nvme_trace_take())
                HV_RUNTIME_VERBOSE_TRACE("HV: NVMe INTMC old=0x%x clear=0x%x new=0x%x\n",
                                         regs.intms, merged, regs.intms & ~merged);
            regs.intms &= ~merged;
            vnvme_intx_delivery_update_line(&irq_delivery,
                                             queue_ctrl.irq_asserted && !regs.intms);
            try_raise_intx();
            break;
        case NVME_CC:
            cc_write(merged);
            break;
        case NVME_AQA:
            regs.aqa = merged;
            break;
        default:
            break;
    }
}

static bool doorbell_write(u32 index, u32 value)
{
    if (!(regs.csts & CSTS_RDY))
        return true;
    u16 qid = index / 2;
    if (hv_runtime_diag_verbose_enabled() && nvme_trace_take())
        HV_RUNTIME_VERBOSE_TRACE(
            "HV: NVMe doorbell q=%u %s=%u irq=%d injected=%d masked=0x%x\n", qid,
            (index & 1) ? "CQH" : "SQT", value, queue_ctrl.irq_asserted,
            irq_delivery.outstanding, regs.intms);
    bool ok;
    if (index & 1) {
        last_cq_doorbell_ticks = mrs(CNTPCT_EL0);
        ok = vnvme_cq_doorbell(&queue_ctrl, qid, value);
        /*
         * A synchronous batch raises INTx at its first CQE and may still be posting CQEs
         * on this or another CPU when the guest scans.  If the guest's head write leaves
         * CQEs it has not seen, notify again (level-triggered INTx stays asserted).
         */
        if (ok && queue_ctrl.cq_ack_left_pending) {
            vnvme_intx_delivery_acknowledged(&irq_delivery);
            try_raise_intx();
        }
    } else {
        dma_rmb();
        ok = vnvme_sq_doorbell(&queue_ctrl, qid, value);
        try_raise_intx();
    }
    if (!ok)
        printf("HV: NVMe invalid doorbell q=%u %s=%u\n", qid, (index & 1) ? "CQH" : "SQT", value);
    return true;
}

static bool handle_nvme_bar(struct exc_info *ctx, u64 addr, u64 *value, bool write, int width)
{
    UNUSED(ctx);
    u32 off = addr - bar_base;
    if (off >= NVME_DBS) {
        u32 index = (off - NVME_DBS) / 4;
        if (write)
            return doorbell_write(index, *value);
        *value = 0;
        return true;
    }

    if (write)
        reg_write(off, width, *value);
    else
        *value = reg_read(off, width);
    return true;
}

void hv_nvme_map_bar(u64 base)
{
    if (bar_base && bar_base != base)
        hv_unmap(bar_base, NVME_BAR_SIZE);
    __atomic_store_n(&bar_base, base, __ATOMIC_RELEASE);
    reset_frontend();
    hv_map_hook(base, handle_nvme_bar, NVME_BAR_SIZE);
    printf("HV: NVMe BAR0 MMIO live at 0x%lx (backend=%d)\n", base, backend_ready);
}

void hv_nvme_unmap_bar(void)
{
    if (!bar_base)
        return;
    hv_unmap(bar_base, NVME_BAR_SIZE);
    __atomic_store_n(&bar_base, 0, __ATOMIC_RELEASE);
    reset_frontend();
}

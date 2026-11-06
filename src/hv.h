/* SPDX-License-Identifier: MIT */

#ifndef HV_H
#define HV_H

#include "exception.h"
#include "hv_sgi_diag.h"
#include "iodev.h"
#include "types.h"
#include "uartproxy.h"

typedef bool(hv_hook_t)(struct exc_info *ctx, u64 addr, u64 *val, bool write, int width);

#define MMIO_EVT_ATTR  GENMASK(31, 24)
#define MMIO_EVT_CPU   GENMASK(23, 16)
#define MMIO_EVT_SH    GENMASK(15, 14)
#define MMIO_EVT_MULTI BIT(6)
#define MMIO_EVT_WRITE BIT(5)
#define MMIO_EVT_WIDTH GENMASK(4, 0)

struct hv_evt_mmiotrace {
    u32 flags;
    u32 reserved;
    u64 pc;
    u64 addr;
    u64 data;
};

struct hv_evt_irqtrace {
    u32 flags;
    u16 type;
    u16 num;
};

#define HV_MAX_RW_SIZE  64
#define HV_MAX_RW_WORDS (HV_MAX_RW_SIZE >> 3)

struct hv_vm_proxy_hook_data {
    u32 flags;
    u32 id;
    u64 addr;
    u64 data[HV_MAX_RW_WORDS];
};

typedef enum _hv_entry_type {
    HV_HOOK_VM = 1,
    HV_VTIMER,
    HV_USER_INTERRUPT,
    HV_WDT_BARK,
    HV_CPU_SWITCH,
    HV_VIRTIO,
    HV_PANIC,
} hv_entry_type;


/* VM */
void hv_pt_init(void);
int hv_map(u64 from, u64 to, u64 size, u64 incr);
int hv_unmap(u64 from, u64 size);
int hv_map_hw(u64 from, u64 to, u64 size);
int hv_map_sw(u64 from, u64 to, u64 size);
int hv_map_hook(u64 from, hv_hook_t *hook, u64 size);
bool hv_window_prepare(u64 base, u64 size);
bool hv_window_map_page(u64 ipa, u64 pa);
void hv_window_sync(bool invalidated);
u64 hv_translate(u64 addr, bool s1only, bool w, u64 *par_out);
u64 hv_pt_walk(u64 addr);
bool hv_handle_dabort(struct exc_info *ctx);
bool hv_pa_write(struct exc_info *ctx, u64 addr, u64 *val, int width);
bool hv_pa_read(struct exc_info *ctx, u64 addr, u64 *val, int width);
bool hv_pa_rw(struct exc_info *ctx, u64 addr, u64 *val, bool write, int width);

/* AIC events through tracing the MMIO event address */
bool hv_trace_irq(u32 type, u32 num, u32 count, u32 flags);

/* Virtual peripherals */
void hv_vuart_poll(void);
bool hv_map_vuart(u64 base, int irq, iodev_id_t iodev);
bool pl011_reg(u64 off, u64 *val, bool write);
bool pl011_kd_live(void);
bool hv_pci_init(u64 ecam, u64 bar_window, int irq);
bool hv_pci_init_windows_slice(u64 ecam, u64 bar_window, int irq);
bool hv_pci_init_diagnostics_slice(u64 ecam, u64 bar_window, int irq);
int hv_pci_intx_irq(void);
u64 hv_pci_bar_window(void);
u64 hv_ipa_to_pa(u64 ipa);
bool hv_ipa_is_normal_rw(u64 ipa);
bool hv_configure_fb_stream(u64 ipa, u64 size, u64 width, u64 height, u64 stride);
bool hv_nvme_init_backend(void);
bool hv_nvme_init_windows_slice(void);
bool hv_nvme_init_diagnostics_slice(void);
void hv_nvme_map_bar(u64 base);
void hv_nvme_unmap_bar(void);
bool hv_nvme_try_handle_dabort(struct exc_info *ctx, bool *matched);
void hv_nvme_poll_irq(void);
void hv_nvme_irq_eoi(u32 intid);
struct vnvme_snapshot;
void hv_nvme_get_diag_snapshot(struct vnvme_snapshot *out, bool *ready);
struct virtio_conf;
void hv_map_virtio(u64 base, struct virtio_conf *conf);
void virtio_put_buffer(u64 base, int qu, u32 id, u32 len);

/* Exceptions */
void hv_exc_proxy(struct exc_info *ctx, uartproxy_boot_reason_t reason, u32 type, void *extra);
void hv_set_time_stealing(bool enabled, bool reset);
void hv_add_time(s64 time);

/* WDT */
void hv_wdt_pet(void);
void hv_wdt_suspend(void);
void hv_wdt_resume(void);
void hv_wdt_init(void);
void hv_wdt_start(int cpu);
void hv_wdt_stop(void);
void hv_wdt_breadcrumb(char c);
u64 hv_wdt_get_breadcrumb(int cpu);
void hv_do_panic(void);

#define hv_panic(fmt, ...)                                                                         \
    do {                                                                                           \
        debug_printf("HV panic:" fmt, ##__VA_ARGS__);                                              \
        hv_do_panic();                                                                             \
        flush_and_reboot();                                                                        \
    } while (0)

/* Utilities */
void hv_write_hcr(u64 val);
u64 hv_get_spsr(void);
void hv_set_spsr(u64 val);
u64 hv_get_esr(void);
u64 hv_get_far(void);
u64 hv_get_elr(void);
u64 hv_get_afsr1(void);
void hv_set_elr(u64 val);

/* HV main */
bool hv_init(void);
#ifdef M1N1_J316S_SETUP
bool hv_init_j316s_setup(void);
#endif
void hv_prepare_guest_cpu_state(void);
void hv_start(void *entry, u64 regs[4]);
void hv_start_secondary(int cpu, void *entry, u64 regs[4]);
void hv_exit_cpu(int cpu);
void hv_rendezvous(void);
bool hv_switch_cpu(int cpu);
void hv_pin_cpu(int cpu);
void hv_arm_tick(bool secondary);
void hv_arm_guest_irq_recovery_tick(void);
void hv_mark_guest_runtime_ready(void);
void hv_percpu_diag_tick(struct exc_info *ctx);
void hv_watchdog_snapshot_tick(struct exc_info *ctx);
void hv_watchdog_snapshot_dump(void);
void hv_rearm(void);
void hv_maybe_exit(void);
void hv_tick(struct exc_info *ctx);
extern u64 hv_fiq_count;
extern u64 hv_fiq_ticks;
extern u64 hv_tick_arm_count[];
extern u64 hv_recovery_tick_arm_count[];

//
// PSCI init
//

void hv_psci_init(void);
#ifdef ENABLE_VGIC_MODULE
bool hv_vgicv3_init(void);
void init_vgic_irq_queues(void);
void hv_vgicv3_init_list_registers(void);
int hv_vgicv3_enable_virtual_interrupts(void);
void hv_sgi_diag_vgic_event(enum hv_sgi_diag_event event);
void hv_irq_diag_vgic_iar(u32 intid);
void hv_irq_diag_vgic_eoi(u32 intid);
#endif
bool hv_handle_psci_smc(struct exc_info *ctx);
int hv_handle_psci_smc_python_entry(uint64_t regs[4]);

#endif

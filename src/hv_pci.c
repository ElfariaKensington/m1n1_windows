/* SPDX-License-Identifier: MIT */

//
// Minimal emulated PCIe host: one device 00:00.0 (an NVMe controller) exposed through a
// trapped ECAM window, so a stock Windows/UEFI PCI enumerator finds it and stornvme binds
// to PCI class 010802. Only what an enumerator actually touches is modelled; every other
// BDF reads back all-ones ("no device"). Config-space accesses are honoured at 1/2/4-byte
// width and arbitrary offset (byte/word/dword config reads are real - see hv_vgic IPRIORITYR
// for why this matters). Interrupt delivery is legacy INTx (a wired SPI via the vGIC), NOT
// MSI: MSI on GICv3 needs an ITS we do not emulate, and a device with no MSI-capability makes
// Windows fall back to the _PRT-routed pin interrupt (the Raspberry Pi 4 PCIe precedent).
//
// ECAM offset layout (one bus): dev[19:15] | fn[14:12] | reg[11:0]; bus is implicit 0.
//

#include "hv.h"
#include "hv_runtime_diag.h"
#include "hv_pci_state.h"
#include "iodev.h"
#include "types.h"
#include "utils.h"

// --- identity of the emulated controller ---
#define PCI_VENDOR_ID  0x1B36   // Red Hat / QEMU vendor
#define PCI_DEVICE_ID  0x0010   // QEMU NVMe - stornvme knows this pair
#define PCI_CLASS_CODE 0x010802 // base 01 (mass storage), sub 08 (NVM), prog-if 02 (NVMe)
#define PCI_REVISION   0x02

// BAR0: 64-bit, non-prefetchable memory, 16 MB. Low-dword type bits are read-only.
#define BAR0_SIZE      0x1000000
#define BAR0_TYPE_BITS 0x4 // bit0=0 memory, bits[2:1]=10 (64-bit), bit3=0 non-prefetch
#define BAR0_ADDR_MASK (~(u32)(BAR0_SIZE - 1)) // 0xFF000000 for 16 MB

// Command register bits we care about.
#define CMD_MEM_SPACE  BIT(1) // MSE - decode memory BARs
#define CMD_BUS_MASTER BIT(2) // BME - allow DMA

// Provided by hv_nvme.c (Layer 2): install/remove the BAR0 MMIO trap.
extern void hv_nvme_map_bar(u64 bar_base);
extern void hv_nvme_unmap_bar(void);

static u64 ecam_base;
static u64 bar_window_base;
static int intx_irq;

// Writable config state of 00:00.0.
static u16 cfg_command;
static u32 cfg_bar0_lo; // stores guest-written value (address bits only)
static u32 cfg_bar0_hi;
static u8 cfg_intr_line = 0xff;
static bool bar_mapped;
static bool bounded_bar;

// Assembled current BAR0 address (0 until both dwords hold a real, non-sizing value).
static u64 bar0_addr(void)
{
    u64 lo = cfg_bar0_lo & BAR0_ADDR_MASK;
    return ((u64)cfg_bar0_hi << 32) | lo;
}

// Re-evaluate whether the BAR0 trap should be installed: needs a plausible address in the
// MMIO window and Memory-Space-Enable set. Idempotent.
static void bar_reeval(void)
{
    u64 addr = bar0_addr();
    // Never arm on the all-ones BAR-sizing pattern. The enumerator writes 0xFFFFFFFF to BOTH
    // dwords of a 64-bit BAR during sizing; checking only the low dword (old addr != 0xFF000000)
    // let bar0_addr()==0xFFFFFFFFFF000000 through and would map the trap at a bogus address.
    bool sizing = (cfg_bar0_hi == 0xFFFFFFFF) || ((cfg_bar0_lo & BAR0_ADDR_MASK) == BAR0_ADDR_MASK);
    bool want =
        (cfg_command & CMD_MEM_SPACE) && addr != 0 && !sizing && (addr & (BAR0_SIZE - 1)) == 0;
    if (bounded_bar)
        want = want && addr == bar_window_base;

    if (want && !bar_mapped) {
        printf("HV: PCI BAR0 programmed at 0x%lx, arming NVMe MMIO trap\n", addr);
        hv_nvme_map_bar(addr);
        bar_mapped = true;
    } else if (!want && bar_mapped) {
        hv_nvme_unmap_bar();
        bar_mapped = false;
    }

    hv_pci_state_record_config(cfg_command, bar0_addr(), bar_mapped);
}

// Return the full 32-bit config dword at register offset `reg` (reg is 4-byte aligned).
static u32 cfg_read_dword(u32 reg)
{
    switch (reg) {
        case 0x00:
            return (PCI_DEVICE_ID << 16) | PCI_VENDOR_ID;
        case 0x04:
            // Status high half = 0 (no capabilities list -> no MSI advertised).
            return cfg_command;
        case 0x08:
            return (PCI_CLASS_CODE << 8) | PCI_REVISION;
        case 0x0c:
            return 0x00000000; // cacheline/latency/header-type 0/BIST
        case 0x10:             // BAR0 low
            return (cfg_bar0_lo & BAR0_ADDR_MASK) | BAR0_TYPE_BITS;
        case 0x14: // BAR0 high
            return cfg_bar0_hi;
        case 0x2c:
            return (PCI_DEVICE_ID << 16) | PCI_VENDOR_ID; // subsystem id
        case 0x34:
            return 0x00000000; // capabilities pointer = 0: deliberately no MSI/MSI-X
        case 0x3c:
            // Interrupt Pin = INTA (1), Interrupt Line = OS-programmed.
            return (0x01 << 8) | cfg_intr_line;
        default:
            return 0x00000000; // unused BARs, expansion ROM, reserved
    }
}

// Apply a write of `bytes` bytes at byte offset `boff` within register dword `reg`.
static void cfg_write(u32 reg, u32 boff, u32 bytes, u64 data)
{
    // Merge the written bytes into the current dword value.
    u32 cur = cfg_read_dword(reg);
    u32 mask = (bytes >= 4) ? 0xFFFFFFFFu : (((1u << (bytes * 8)) - 1) << (boff * 8));
    u32 merged = (cur & ~mask) | ((u32)(data << (boff * 8)) & mask);

    switch (reg) {
        case 0x04:
            cfg_command = merged & 0xFFFF;
            bar_reeval();
            break;
        case 0x10:
            cfg_bar0_lo = merged & 0xFFFFFFF0; // type bits are read-only
            bar_reeval();
            break;
        case 0x14:
            cfg_bar0_hi = merged;
            bar_reeval();
            break;
        case 0x3c:
            cfg_intr_line = merged & 0xFF; // only Interrupt Line is writable
            break;
        default:
            break; // everything else is read-only; drop the write
    }
}

// Stage-2 hook over the whole ECAM window.
static bool handle_pci_cfg(struct exc_info *ctx, u64 addr, u64 *val, bool write, int width)
{
    UNUSED(ctx);

    u64 off = addr - ecam_base;
    u32 bus = (off >> 20) & 0xff;
    u32 dev = (off >> 15) & 0x1f;
    u32 fn = (off >> 12) & 0x07;
    u32 reg = off & 0xffc;
    u32 boff = off & 0x3;
    u32 bytes = 1u << width;

    // One-shot: proves the enumerator is actually reading our ECAM (hook not clobbered).
    static bool announced = false;
    if (!announced) {
        announced = true;
        printf("HV: PCI ECAM first access bdf=%02x:%02x.%x reg=0x%x w=%d\n", bus, dev, fn, reg,
               write);
    }
    // Trace every access to our device so the enumerator's walk is visible.
    if (bus == 0 && dev == 0 && fn == 0)
        HV_RUNTIME_TRACE("HV: PCI cfg 00:00.0 reg=0x%03x w=%d width=%d val=0x%lx\n", reg,
                         write, bytes, write ? *val : 0);

    // Only 00:00.0 exists. Any other BDF reads as all-ones ("no device present"); a missing
    // all-ones default here makes the enumerator probe empty space forever.
    if (bus != 0 || dev != 0 || fn != 0) {
        if (!write)
            *val = 0xFFFFFFFFFFFFFFFFULL;
        return true;
    }

    if (write) {
        cfg_write(reg, boff, bytes, *val);
    } else {
        u32 dw = cfg_read_dword(reg);
        u32 m = (bytes >= 4) ? 0xFFFFFFFFu : ((1u << (bytes * 8)) - 1);
        *val = (dw >> (boff * 8)) & m;
    }
    return true;
}

static bool init_pci(u64 ecam, u64 bar_window, int irq, bool windows_slice,
                     bool diagnostics_slice)
{
    ecam_base = ecam;
    bar_window_base = bar_window;
    intx_irq = irq;
    cfg_command = 0;
    cfg_bar0_lo = cfg_bar0_hi = 0;
    bar_mapped = false;
    bounded_bar = windows_slice || diagnostics_slice;
    hv_pci_state_reset(ecam, bar_window, (u32)irq);

    // m1n1 retains ownership of the native Apple ANS controller. Initialize it before the
    // guest enumerates PCI so Identify can return the real namespace geometry immediately.
    bool bounded = windows_slice || diagnostics_slice;
    bool backend = diagnostics_slice ? hv_nvme_init_diagnostics_slice() :
                   windows_slice ? hv_nvme_init_windows_slice() : hv_nvme_init_backend();
    if (bounded && !backend)
        return false;

    int r = hv_map_hook(ecam_base, handle_pci_cfg, 0x100000); // 1 MB = one bus
    // Self-check: confirm our SPTE_HOOK actually replaced the arm-io pass-through mapping at
    // the ECAM base (vs. failing to split a HW block, which would leave the pass-through and
    // send guest config reads to real hardware instead of us).
    u64 pte = hv_pt_walk(ecam_base);
    hv_pci_state_record_init(r == 0, backend);
    printf("HV: emulated PCIe ECAM at 0x%lx (map_hook=%d pte=0x%lx), NVMe 00:00.0 "
           "(INTx SPI %d backend=%d)\n",
           ecam_base, r, pte, intx_irq, backend);
    return r == 0 && backend;
}

bool hv_pci_init(u64 ecam, u64 bar_window, int irq)
{
    return init_pci(ecam, bar_window, irq, false, false);
}

bool hv_pci_init_windows_slice(u64 ecam, u64 bar_window, int irq)
{
    return init_pci(ecam, bar_window, irq, true, false);
}

bool hv_pci_init_diagnostics_slice(u64 ecam, u64 bar_window, int irq)
{
    return init_pci(ecam, bar_window, irq, false, true);
}

// Exposed so hv_nvme.c can raise/lower the wired INTx line and know the window base.
int hv_pci_intx_irq(void)
{
    return intx_irq;
}
u64 hv_pci_bar_window(void)
{
    return bar_window_base;
}

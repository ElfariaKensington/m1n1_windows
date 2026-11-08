/* SPDX-License-Identifier: MIT */

#include "../build/build_cfg.h"
#include "hv_j316s_setup.h"
#include "hv_j316s_wifi.h"

#include "adt.h"
#include "aic.h"
#include "arm_cpu_regs.h"
#include "cpu_regs.h"
#include "dart.h"
#include "fb.h"
#include "heapblock.h"
#include "hv.h"
#include "hv_apple_input.h"
#include "hv_irq_routes.h"
#include "hv_stage2_state.h"
#include "hv_vgic_redist.h"
#include "hv_xhci_handoff.h"
#include "mcc.h"
#include "memory.h"
#include "pmgr.h"
#include "smp.h"
#include "soc.h"
#include "string.h"
#include "tinf/tinf.h"
#include "usb.h"
#include "usb_dwc3_regs.h"
#include "utils.h"
#include "xnuboot.h"

/* One guest pixel covers a 2-by-2 block on the 3456-by-2234 J316s panel. */
#define FRAME_WIDTH ((u32)J316S_SETUP_FB_WIDTH)
#define FRAME_HEIGHT ((u32)J316S_SETUP_FB_HEIGHT)
#define FRAME_STRIDE (FRAME_WIDTH * 4u)
#define EL2_HEAP_HEADROOM (512ULL * 1024 * 1024)
#define J316S_SETUP_XHCI_COUNT 3u

struct j316s_setup_xhci {
    const char *controller;
    const char *darts;
    u64 base;
    u64 dart0;
    u64 dart1;
    u32 aic_irq;
    u32 vintid;
    u32 index;
};

static const struct j316s_setup_xhci setup_xhci[] = {
    {"/arm-io/usb-drd0", "/arm-io/dart-usb0", J316S_SETUP_XHCI0,
     J316S_SETUP_USB0_DART0, J316S_SETUP_USB0_DART1,
     J316S_SETUP_XHCI_AIC_IRQ0, J316S_SETUP_XHCI_VINTID0, 0},
    {"/arm-io/usb-drd1", "/arm-io/dart-usb1", J316S_SETUP_XHCI1,
     J316S_SETUP_USB1_DART0, J316S_SETUP_USB1_DART1,
     J316S_SETUP_XHCI_AIC_IRQ1, J316S_SETUP_XHCI_VINTID1, 1},
    {"/arm-io/usb-drd2", "/arm-io/dart-usb2", J316S_SETUP_XHCI2,
     J316S_SETUP_USB2_DART0, J316S_SETUP_USB2_DART1,
     J316S_SETUP_XHCI_AIC_IRQ2, J316S_SETUP_XHCI_VINTID2, 2},
};

_Static_assert(ARRAY_SIZE(setup_xhci) == J316S_SETUP_XHCI_COUNT,
               "J316s USB controller table");

static u64 backing;
static u64 next_frame_tick;
static bool draw_setup;
static bool frame_seen;
static bool owns_usb;
static u32 setup_usb_mask;

bool hv_j316s_setup_owns_usb(void)
{
    return owns_usb;
}

static void *guest_ptr(u64 ipa)
{
    return (void *)(backing + (ipa - J316S_SETUP_RAM_BASE));
}

static bool overlaps(u64 start, u64 end, u64 other_start, u64 other_size)
{
    return other_size && other_start < end &&
           (other_start >= start || other_size > start - other_start);
}

static bool reserve_backing(void)
{
    u64 start = (u64)heapblock_alloc_aligned(0, SZ_16K);
    u64 host_base = cur_boot_args.phys_base;
    u64 host_size = cur_boot_args.mem_size;
    const u64 required = J316S_SETUP_RAM_SIZE + EL2_HEAP_HEADROOM;
    if (start < host_base || host_size > UINT64_MAX - host_base ||
        start > host_base + host_size || required > host_base + host_size - start)
        return false;

    for (size_t index = 0; index < mcc_carveout_count; index++)
        if (overlaps(start, start + required, mcc_carveouts[index].base,
                     mcc_carveouts[index].size))
            return false;

    /* Preserve the live iBoot framebuffer, which the host will use to draw
     * the guest's copy. No framebuffer or peripheral PA enters stage 2. */
    if (overlaps(start, start + required, cur_boot_args.video.base, fb.size))
        return false;

    backing = (u64)heapblock_alloc_aligned(J316S_SETUP_RAM_SIZE, SZ_16K);
    return backing == start;
}

bool hv_j316s_setup_present(void)
{
    u64 payload = (u64)_payload_start;
    u64 top = cur_boot_args.top_of_kernel_data;
    return payload <= top && top - payload >= sizeof(struct j316s_setup_header) &&
           !memcmp(_payload_start, J316S_SETUP_MAGIC, 8);
}

static void write_boot_args(void)
{
    struct boot_args guest = cur_boot_args;
    guest.phys_base = J316S_SETUP_RAM_BASE;
    guest.mem_size = J316S_SETUP_RAM_SIZE;
    guest.virt_base = 0xfffffe0010000000ULL;
    guest.devtree = (void *)(guest.virt_base + J316S_SETUP_ADT - guest.phys_base);
    guest.top_of_kernel_data = J316S_SETUP_BOOT_ARGS + SZ_16K;
    guest.video.base = J316S_SETUP_FB;
    guest.video.width = FRAME_WIDTH;
    guest.video.height = FRAME_HEIGHT;
    guest.video.stride = FRAME_STRIDE;
    guest.video.depth = 32;
    guest.video.display = 1;
    if (guest.revision <= 1)
        guest.rv1.mem_size_actual = guest.mem_size;
    else if (guest.revision == 2)
        guest.rv2.mem_size_actual = guest.mem_size;
    else
        guest.rv3.mem_size_actual = guest.mem_size;

    memcpy(guest_ptr(J316S_SETUP_BOOT_ARGS), &guest, sizeof(guest));
    dc_civac_range(guest_ptr(J316S_SETUP_BOOT_ARGS), sizeof(guest));
}

static bool verify_mappings(bool usb, bool input, bool storage)
{
    struct hv_stage2_mapping allowed[6 + J316S_SETUP_XHCI_COUNT + 3 + 2] = {
        {J316S_SETUP_GIC_DIST, 0, 0x10000, 0, HV_STAGE2_MAPPING_HOOK},
        {J316S_SETUP_GIC_REDIST, 0, HV_VGIC_T6000_CPU_COUNT * HV_VGIC_REDIST_STRIDE,
         0, HV_STAGE2_MAPPING_HOOK},
        {J316S_SETUP_GIC_ITS, 0, 0x10000, 0, HV_STAGE2_MAPPING_HOOK},
        {J316S_SETUP_RAM_BASE, backing, J316S_SETUP_RAM_SIZE, 1, HV_STAGE2_MAPPING_HARDWARE},
        {J316S_SETUP_LOW_IPA, (u64)guest_ptr(J316S_SETUP_LOW_BACKING),
         J316S_SETUP_LOW_SIZE, 1, HV_STAGE2_MAPPING_HARDWARE},
        {J316S_SETUP_UART, 0, 0x2000, 0, HV_STAGE2_MAPPING_HOOK},
    };

    size_t count = 6;
    if (usb)
        for (size_t i = 0; i < ARRAY_SIZE(setup_xhci); i++)
            allowed[count++] = (struct hv_stage2_mapping){
                setup_xhci[i].base, 0, J316S_SETUP_XHCI_SIZE, 0, HV_STAGE2_MAPPING_HOOK};
    if (input) {
        allowed[count++] = (struct hv_stage2_mapping){
            J316S_SETUP_INPUT_SPI, 0, SZ_16K, 0, HV_STAGE2_MAPPING_HOOK};
        allowed[count++] = (struct hv_stage2_mapping){
            J316S_SETUP_INPUT_AP_GPIO, 0, SZ_16K, 0, HV_STAGE2_MAPPING_HOOK};
        allowed[count++] = (struct hv_stage2_mapping){
            J316S_SETUP_INPUT_NUB_GPIO, 0, SZ_16K, 0, HV_STAGE2_MAPPING_HOOK};
    }
    if (storage)
        allowed[count++] = (struct hv_stage2_mapping){
            J316S_SETUP_ECAM, 0, J316S_SETUP_ECAM_SIZE, 0, HV_STAGE2_MAPPING_HOOK};
    /* Stage-2 mappings are compared in creation order; the bridge is mapped last. */
    if (hv_j316s_wifi_mapped())
        allowed[count++] = (struct hv_stage2_mapping){
            J316S_SETUP_WIFI_BASE, hv_j316s_wifi_window_pa(), J316S_SETUP_WIFI_SIZE, 1,
            HV_STAGE2_MAPPING_HARDWARE};
    return hv_stage2_state_matches(allowed, count);
}

static bool input_mmio(struct exc_info *ctx, u64 addr, u64 *val, bool write, int width)
{
    /* Restrict the two GPIO controllers to SPI enable, input IRQ and its W1C
     * bit. All other pins and the other interrupt groups stay with the host. */
    const u64 ap_pin = J316S_SETUP_INPUT_AP_GPIO + 4 * 194;
    const u64 nub_pin = J316S_SETUP_INPUT_NUB_GPIO + 4 * 6;
    const u64 nub_ack = J316S_SETUP_INPUT_NUB_GPIO + 0x800;
    if (width != 2)
        return false;
    if (addr == ap_pin || addr == nub_pin) {
        if (!write)
            return hv_pa_rw(ctx, addr, val, false, width);
        u32 mask = addr == ap_pin ? 0xf : (0xf | (3 << 5) | BIT(9) | (7 << 16));
        u64 old;
        if (!hv_pa_rw(ctx, addr, &old, false, width))
            return false;
        u64 data = (old & ~mask) | (*val & mask);
        if (addr == nub_pin)
            data &= ~(7u << 16); /* The only routed GPIO group is group zero. */
        return hv_pa_rw(ctx, addr, &data, true, width);
    }
    if (addr == nub_ack)
        return write && *val == BIT(6) && hv_pa_rw(ctx, addr, val, true, width);
    if (addr < J316S_SETUP_INPUT_SPI || addr >= J316S_SETUP_INPUT_SPI + SZ_16K)
        return false;
    /* PIO registers used by AppleInput. SPI DMA and other registers are absent. */
    switch (addr - J316S_SETUP_INPUT_SPI) {
        case 0x000: case 0x004: case 0x00c: case 0x010: case 0x020:
        case 0x030: case 0x034: case 0x04c: case 0x10c: case 0x130:
        case 0x134: case 0x138: case 0x13c: case 0x150: case 0x154:
        case 0x160: case 0x168:
            return hv_pa_rw(ctx, addr, val, write, width);
    }
    return false;
}

static bool prepare_input(void)
{
    const u32 pin_function = GENMASK(6, 5);
    const u32 input_enable = BIT(9);
    struct hv_apple_input_observed input;
    if (!hv_apple_input_observe_adt(&input) || !input.spi_compatible || !input.hid_compatible ||
        input.spi_base != J316S_SETUP_INPUT_SPI || input.spi_size != SZ_16K ||
        input.ap_gpio_base != J316S_SETUP_INPUT_AP_GPIO || input.ap_gpio_size != SZ_16K ||
        input.nub_gpio_base != J316S_SETUP_INPUT_NUB_GPIO || input.nub_gpio_size != SZ_16K ||
        input.ap_gpio_pin != 194 || input.nub_gpio_pin != 6 || input.parent_irq != 623)
        return false;
    aic_set_mask(input.parent_irq, true);
    /* The AP/Nub GPIO pin controllers are inherited live from firmware and
     * their J316s ADT nodes do not expose PMGR clock-gates. Preserve that
     * state and power only SPI3, whose ADT entry owns the required gate. */
    if (pmgr_adt_power_enable("/arm-io/spi3") < 0 ||
        !hv_irq_route_register(input.parent_irq, J316S_SETUP_INPUT_VINTID, true) ||
        hv_map_hook(J316S_SETUP_INPUT_SPI, input_mmio, SZ_16K) ||
        hv_map_hook(J316S_SETUP_INPUT_AP_GPIO, input_mmio, SZ_16K) ||
        hv_map_hook(J316S_SETUP_INPUT_NUB_GPIO, input_mmio, SZ_16K))
        return false;
    if (!verify_mappings(true, true, false))
        return false;

    /* T600x SPI3 uses AP pins 52–55, function 1. Configure these in EL2;
     * the guest is allowed to touch only SPI enable and the Nub IRQ pin. */
    for (u32 pin = 52; pin <= 55; pin++)
        mask32(input.ap_gpio_base + 4 * pin, pin_function | input_enable,
               FIELD_PREP(pin_function, 1) | input_enable);
    /* SPI enable is an active-high GPIO, not a peripheral signal. */
    clear32(input.ap_gpio_base + 4 * input.ap_gpio_pin, pin_function);
    sysop("dsb sy");
    return true;
}

static bool xhci_mmio(struct exc_info *ctx, u64 addr, u64 *val, bool write, int width)
{
    for (size_t i = 0; i < ARRAY_SIZE(setup_xhci); i++) {
        u64 base = setup_xhci[i].base;
        if (addr < base || addr - base >= J316S_SETUP_XHCI_SIZE)
            continue;
        u64 offset = addr - base;
        if (width < 0 || width > 3 || (1ULL << width) > J316S_SETUP_XHCI_SIZE - offset ||
            !hv_pa_rw(ctx, addr, val, write, width))
            return false;
        if (!write)
            *val = hv_xhci_cap_read_for_guest(offset, *val, width);
        return true;
    }
    return false;
}

static bool usb_reg_matches(const char *path, int index, u64 expected, u64 min_size)
{
    int trace[8];
    u64 base, size;
    return adt_path_offset_trace(adt, path, trace) >= 0 &&
           adt_get_reg(adt, trace, "reg", index, &base, &size) >= 0 &&
           base == expected && size >= min_size;
}

static void reset_setup_usb(void)
{
    for (size_t i = 0; i < ARRAY_SIZE(setup_xhci); i++)
        if (setup_usb_mask & BIT(setup_xhci[i].index))
            set32(setup_xhci[i].base + DWC3_GCTL, DWC3_GCTL_CORESOFTRESET);
    setup_usb_mask = 0;
}

static bool prepare_usb_port(const struct j316s_setup_xhci *port)
{
    const u64 base = port->base;

    /* Aurora's USB2 host initialization uses the existing ATC PHY and resets
     * the Synopsys core and PHYs before switching to host mode. */
    if ((read32(base + DWC3_GSNPSID) & DWC3_GSNPSID_MASK) != 0x33310000)
        return false;
    set32(base + DWC3_GCTL, DWC3_GCTL_CORESOFTRESET);
    set32(base + DWC3_GUSB2PHYCFG(0), DWC3_GUSB2PHYCFG_PHYSOFTRST);
    set32(base + DWC3_GUSB3PIPECTL(0), DWC3_GUSB3PIPECTL_PHYSOFTRST);
    sysop("dsb sy");
    mdelay(100);

    u64 low_pa = (u64)guest_ptr(J316S_SETUP_LOW_BACKING);
    if (!dart_t6000_map_exclusive(port->dart0, 0, J316S_SETUP_LOW_IPA,
                                  low_pa, J316S_SETUP_LOW_SIZE) ||
        !dart_t6000_map_exclusive(port->dart1, 1, J316S_SETUP_LOW_IPA,
                                  low_pa, J316S_SETUP_LOW_SIZE))
        return false;

    clear32(base + DWC3_GUSB2PHYCFG(0), DWC3_GUSB2PHYCFG_PHYSOFTRST);
    clear32(base + DWC3_GUSB3PIPECTL(0), DWC3_GUSB3PIPECTL_PHYSOFTRST);
    sysop("dsb sy");
    mdelay(100);
    write32(base + 0xcd38, 0x0f800f80);
    write32(base + 0xcd3c, 0x0fc00fc0);
    mask32(base + 0xcd40, 0x00ffffff, 0x00140a10);
    mask32(base + DWC3_GSBUSCFG0, 0xff, 0x0f);
    set32(base + DWC3_GSBUSCFG1, 0xf00);
    set32(base + 0xc630, 0xa0); /* DWC3 frame-length adjustment */
    u32 control = read32(base + DWC3_GCTL);
    control &= ~(DWC3_GCTL_CORESOFTRESET | DWC3_GCTL_SCALEDOWN_MASK |
                 DWC3_GCTL_DISSCRAMBLE | DWC3_GCTL_PRTCAPDIR(DWC3_GCTL_PRTCAP_OTG));
    if (DWC3_GHWPARAMS1_EN_PWROPT(read32(base + DWC3_GHWPARAMS1)) ==
        DWC3_GHWPARAMS1_EN_PWROPT_CLK)
        control &= ~DWC3_GCTL_DSBLCLKGTNG;
    write32(base + DWC3_GCTL, control | DWC3_GCTL_PRTCAPDIR(DWC3_GCTL_PRTCAP_HOST));
    sysop("dsb sy");

    u32 caps = read32(base);
    if ((caps & 0xff) < 0x20 || caps >> 16 < 0x100 ||
        !hv_irq_route_register(port->aic_irq, port->vintid, true) ||
        hv_map_hook(base, xhci_mmio, J316S_SETUP_XHCI_SIZE))
        return false;

    printf("J316s Setup: usb-drd%u enabled at 0x%lx, IRQ %u -> %u\n",
           port->index, port->base, port->aic_irq, port->vintid);
    return true;
}

static bool prepare_usb(void)
{
    for (size_t i = 0; i < ARRAY_SIZE(setup_xhci); i++) {
        const struct j316s_setup_xhci *port = &setup_xhci[i];
        int node = adt_path_offset(adt, port->controller);
        int dart_node = adt_path_offset(adt, port->darts);
        u32 irq_size = 0;
        u32 port_number = 0;
        const u32 *irqs = node < 0 ? NULL : adt_getprop(adt, node, "interrupts", &irq_size);
        if (!irqs || irq_size < sizeof(*irqs) || irqs[0] != port->aic_irq ||
            ADT_GETPROP(adt, node, "port-number", &port_number) < 0 ||
            port_number != port->index + 1 ||
            !adt_is_compatible(adt, node, "usb-drd,t6000") || dart_node < 0 ||
            !adt_is_compatible(adt, dart_node, "dart,t6000") ||
            !usb_reg_matches(port->controller, 0, port->base, 0xcd44) ||
            !usb_reg_matches(port->controller, 2, port->base + 0xc000, 0xd44) ||
            !usb_reg_matches(port->darts, 0, port->dart0, SZ_16K) ||
            !usb_reg_matches(port->darts, 1, port->dart1, SZ_16K) ||
            iodev_get_usage(IODEV_USB0 + port->index)) {
            printf("J316s Setup: USB port %u ADT or ownership preflight failed\n", port->index);
            return false;
        }
    }

    owns_usb = true;
    setup_usb_mask = 0;
    for (size_t i = 0; i < ARRAY_SIZE(setup_xhci); i++) {
        const struct j316s_setup_xhci *port = &setup_xhci[i];
        aic_set_mask(port->aic_irq, true);
        setup_usb_mask |= BIT(port->index);
        if (usb_phy_bringup(port->index) < 0 || !prepare_usb_port(port)) {
            printf("J316s Setup: USB port %u initialization failed\n", port->index);
            reset_setup_usb();
            return false;
        }
    }
    if (!verify_mappings(true, false, false)) {
        printf("J316s Setup: USB stage-2 mapping allowlist mismatch\n");
        reset_setup_usb();
        return false;
    }
    printf("J316s Setup: all three J316s USB-C xHCI controllers enabled with guest-only DMA\n");
    return true;
}

static void dump_stage2_mappings(void)
{
    struct hv_stage2_mapping mappings[HV_STAGE2_STATE_MAX_MAPPINGS];
    size_t count = 0;

    if (!hv_stage2_state_snapshot(mappings, ARRAY_SIZE(mappings), &count)) {
        printf("J316s Setup: stage-2 mapping snapshot unavailable\n");
        return;
    }

    printf("J316s Setup: observed %lu stage-2 mappings\n", (u64)count);
    for (size_t index = 0; index < count; index++) {
        printf("J316s Setup: map[%lu] ipa=0x%lx pa=0x%lx size=0x%lx incr=%lu kind=%u\n",
               (u64)index, mappings[index].ipa, mappings[index].pa,
               mappings[index].size, mappings[index].increment, mappings[index].kind);
    }
}

void hv_j316s_setup_draw(u64 guest_pc)
{
    if (!draw_setup || smp_id() != boot_cpu_idx || get_ticks() < next_frame_tick)
        return;

    if (!next_frame_tick)
        printf("J316s Setup: first guest timer tick pc=0x%lx\n", guest_pc);
    next_frame_tick = get_ticks() + mrs(CNTFRQ_EL0) / 4;
    const volatile u32 *pixels = guest_ptr(J316S_SETUP_FB);
    if (!frame_seen) {
        for (size_t index = 0; index < FRAME_WIDTH * FRAME_HEIGHT; index++) {
            if (pixels[index] & 0x00ffffff) {
                frame_seen = true;
                break;
            }
        }
        if (!frame_seen)
            return;
        fb_set_active(false);
        fb_fill(0, 0, fb.width, fb.height, (rgb_t){0});
    }
    u32 scale = fb.width >= 2 * FRAME_WIDTH && fb.height >= 2 * FRAME_HEIGHT ? 2 : 1;
    fb_blit_scaled((fb.width - FRAME_WIDTH * scale) / 2,
                   (fb.height - FRAME_HEIGHT * scale) / 2,
                   FRAME_WIDTH, FRAME_HEIGHT, guest_ptr(J316S_SETUP_FB),
                   FRAME_WIDTH, PIX_FMT_XBGR, scale);
}

bool hv_j316s_setup_run(void)
{
    if (!hv_j316s_setup_present() || chip_id != T6000 ||
        !adt_is_compatible(adt, 0, "J316sAP")) {
        printf("J316s Setup: unsupported board\n");
        return false;
    }

    const struct j316s_setup_header *header = (const void *)_payload_start;
    const u64 fixed_size = J316S_SETUP_PAYLOAD_OFFSET + J316S_SETUP_FIRMWARE_SIZE;
    u64 size_available = cur_boot_args.top_of_kernel_data - (u64)_payload_start;
    bool usb = header->flags & J316S_SETUP_USB_FLAG;
    bool input = header->flags & J316S_SETUP_INPUT_FLAG;
    bool diagnostics_storage = header->flags & J316S_SETUP_DIAGNOSTICS_STORAGE_FLAG;
    bool storage = header->flags & (J316S_SETUP_STORAGE_FLAG |
                                    J316S_SETUP_DIAGNOSTICS_STORAGE_FLAG);
    if ((header->flags & ~(J316S_SETUP_USB_FLAG | J316S_SETUP_INPUT_FLAG |
                          J316S_SETUP_STORAGE_FLAG |
                          J316S_SETUP_DIAGNOSTICS_STORAGE_FLAG | J316S_SETUP_WIFI_FLAG)) ||
        (diagnostics_storage && (!input || (header->flags & J316S_SETUP_STORAGE_FLAG))) ||
        ((input || storage) && !usb) || size_available < fixed_size ||
        header->media_size < SZ_1M || header->media_size % 512 ||
        header->media_size > J316S_SETUP_RAMDISK_SIZE - J316S_SETUP_RAMDISK_HEADER_SIZE ||
        header->media_size > size_available - fixed_size) {
        printf("J316s Setup: invalid payload size\n");
        return false;
    }

    const u8 *firmware = (const u8 *)_payload_start + J316S_SETUP_PAYLOAD_OFFSET;
    const u8 *media = firmware + J316S_SETUP_FIRMWARE_SIZE;
    if (memcmp(firmware + 0x38, "ARM\x64", 4) ||
        tinf_crc32(firmware, J316S_SETUP_FIRMWARE_SIZE) != header->firmware_crc ||
        tinf_crc32(media, header->media_size) != header->media_crc) {
        printf("J316s Setup: invalid firmware or media checksum\n");
        return false;
    }
    if (!cur_boot_args.devtree_size || cur_boot_args.devtree_size > J316S_SETUP_ADT_MAX ||
        !fb.hwptr || !fb.ptr || fb.width < FRAME_WIDTH || fb.height < FRAME_HEIGHT ||
        fb.stride < fb.width || (u64)fb.stride * fb.height * 4 != fb.size ||
        !reserve_backing()) {
        printf("J316s Setup: unsupported display or unavailable guest RAM\n");
        return false;
    }

    memset64(guest_ptr(J316S_SETUP_RAM_BASE), 0, J316S_SETUP_RAM_SIZE);
    memcpy(guest_ptr(J316S_SETUP_ADT), adt, cur_boot_args.devtree_size);
    dc_civac_range(guest_ptr(J316S_SETUP_ADT), cur_boot_args.devtree_size);
    memcpy128(guest_ptr(J316S_SETUP_FIRMWARE), (void *)firmware, J316S_SETUP_FIRMWARE_SIZE);
    dc_cvau_range(guest_ptr(J316S_SETUP_FIRMWARE), J316S_SETUP_FIRMWARE_SIZE);
    ic_ivau_range(guest_ptr(J316S_SETUP_FIRMWARE), J316S_SETUP_FIRMWARE_SIZE);
    write_boot_args();

    u64 *ramdisk = guest_ptr(J316S_SETUP_RAMDISK);
    memcpy(ramdisk, "ASIRAMDK", 8);
    ramdisk[1] = header->media_size;
    memcpy128((u8 *)ramdisk + J316S_SETUP_RAMDISK_HEADER_SIZE, (void *)media, header->media_size);
    dc_civac_range(ramdisk, J316S_SETUP_RAMDISK_HEADER_SIZE + header->media_size);

    if (!hv_init_j316s_setup()) {
        printf("J316s Setup: isolated hypervisor initialization failed\n");
        return false;
    }

    int ram_map_result = hv_map_hw(J316S_SETUP_RAM_BASE, backing, J316S_SETUP_RAM_SIZE);
    int low_map_result = hv_map_hw(J316S_SETUP_LOW_IPA,
                                   (u64)guest_ptr(J316S_SETUP_LOW_BACKING),
                                   J316S_SETUP_LOW_SIZE);
    bool uart_map_result = hv_map_vuart(J316S_SETUP_UART, 0, IODEV_UART);
    bool mappings_match = !ram_map_result && !low_map_result && uart_map_result &&
                          verify_mappings(false, false, false);
    if (ram_map_result || low_map_result || !uart_map_result || !mappings_match) {
        printf("J316s Setup: map results ram=%d low=%d uart=%u allowlist=%u\n",
               ram_map_result, low_map_result, uart_map_result, mappings_match);
        dump_stage2_mappings();
        printf("J316s Setup: isolated mappings failed\n");
        return false;
    }

    if (usb && !prepare_usb()) {
        printf("J316s Setup: USB preflight failed; guest not started\n");
        return false;
    }
    if (input && !prepare_input()) {
        reset_setup_usb();
        printf("J316s Setup: Apple input preflight failed; guest not started\n");
        return false;
    }
    if (storage &&
        (!(diagnostics_storage ?
             hv_pci_init_diagnostics_slice(J316S_SETUP_ECAM, J316S_SETUP_NVME_BAR,
                                           J316S_SETUP_NVME_INTID) :
             hv_pci_init_windows_slice(J316S_SETUP_ECAM, J316S_SETUP_NVME_BAR,
                                       J316S_SETUP_NVME_INTID)) ||
         !verify_mappings(true, input, true))) {
        reset_setup_usb();
        printf("J316s Setup: bounded Windows disk unavailable; guest not started\n");
        return false;
    }

    if (!hv_j316s_wifi_init(header->flags & J316S_SETUP_WIFI_FLAG) ||
        !verify_mappings(true, input, storage)) {
        printf("J316s Setup: Wi-Fi bridge mapping rejected\n");
        return false;
    }
    hv_prepare_guest_cpu_state();
    msr(SYS_IMP_APL_SPRR_CONFIG_EL1, 1);
    msr(SYS_IMP_APL_GXF_CONFIG_EL1, 1);
    sysop("isb");
    u64 regs[4] = {J316S_SETUP_BOOT_ARGS, 0, 0, 0};
    printf("J316s Setup: entering RAM-backed guest\n");
    next_frame_tick = 0;
    frame_seen = false;
    draw_setup = true;
    hv_start((void *)J316S_SETUP_FIRMWARE, regs);
    draw_setup = false;
    fb_set_active(true);
    return true;
}

/* SPDX-License-Identifier: MIT */

#include "../build/build_cfg.h"
#include "hv_j316s_setup.h"

#include "adt.h"
#include "arm_cpu_regs.h"
#include "cpu_regs.h"
#include "fb.h"
#include "heapblock.h"
#include "hv.h"
#include "hv_stage2_state.h"
#include "hv_vgic_redist.h"
#include "mcc.h"
#include "memory.h"
#include "smp.h"
#include "soc.h"
#include "string.h"
#include "tinf/tinf.h"
#include "utils.h"
#include "xnuboot.h"

#define FRAME_WIDTH 1280u
#define FRAME_HEIGHT 720u
#define FRAME_STRIDE (FRAME_WIDTH * 4u)
#define FRAME_SIZE (FRAME_HEIGHT * FRAME_STRIDE)
#define EL2_HEAP_HEADROOM (512ULL * 1024 * 1024)

static u64 backing;
static u64 next_frame_tick;
static bool draw_setup;

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

static bool verify_mappings(void)
{
    const struct hv_stage2_mapping allowed[] = {
        {J316S_SETUP_GIC_DIST, 0, 0x10000, 0, HV_STAGE2_MAPPING_HOOK},
        {J316S_SETUP_GIC_REDIST, 0, HV_VGIC_T6000_CPU_COUNT * HV_VGIC_REDIST_STRIDE,
         0, HV_STAGE2_MAPPING_HOOK},
        {J316S_SETUP_GIC_ITS, 0, 0x10000, 0, HV_STAGE2_MAPPING_HOOK},
        {J316S_SETUP_RAM_BASE, backing, J316S_SETUP_RAM_SIZE, 1, HV_STAGE2_MAPPING_HARDWARE},
        {J316S_SETUP_LOW_IPA, (u64)guest_ptr(J316S_SETUP_LOW_BACKING),
         J316S_SETUP_LOW_SIZE, 1, HV_STAGE2_MAPPING_HARDWARE},
        {J316S_SETUP_UART, 0, 0x2000, 0, HV_STAGE2_MAPPING_HOOK},
    };
    return hv_stage2_state_matches(allowed, ARRAY_SIZE(allowed));
}

void hv_j316s_setup_draw(void)
{
    if (!draw_setup || smp_id() != boot_cpu_idx || get_ticks() < next_frame_tick)
        return;

    next_frame_tick = get_ticks() + mrs(CNTFRQ_EL0) / 4;
    fb_blit((fb.width - FRAME_WIDTH) / 2, (fb.height - FRAME_HEIGHT) / 2,
            FRAME_WIDTH, FRAME_HEIGHT, guest_ptr(J316S_SETUP_FB),
            FRAME_WIDTH, PIX_FMT_XBGR);
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
    if (header->reserved || size_available < fixed_size ||
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

    if (!hv_init_j316s_setup() ||
        hv_map_hw(J316S_SETUP_RAM_BASE, backing, J316S_SETUP_RAM_SIZE) ||
        hv_map_hw(J316S_SETUP_LOW_IPA, (u64)guest_ptr(J316S_SETUP_LOW_BACKING),
                  J316S_SETUP_LOW_SIZE) ||
        !hv_map_vuart(J316S_SETUP_UART, 0, IODEV_UART) ||
        !verify_mappings()) {
        printf("J316s Setup: isolated mappings failed\n");
        return false;
    }

    hv_prepare_guest_cpu_state();
    msr(SYS_IMP_APL_SPRR_CONFIG_EL1, 1);
    msr(SYS_IMP_APL_GXF_CONFIG_EL1, 1);
    sysop("isb");
    u64 regs[4] = {J316S_SETUP_BOOT_ARGS, 0, 0, 0};
    printf("J316s Setup: entering RAM-backed guest\n");
    fb_set_active(false);
    draw_setup = true;
    hv_start((void *)J316S_SETUP_FIRMWARE, regs);
    draw_setup = false;
    return true;
}

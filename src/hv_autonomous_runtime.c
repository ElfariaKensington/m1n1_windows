/* SPDX-License-Identifier: MIT */

#include "hv_autonomous.h"

#include "adt.h"
#include "arm_cpu_regs.h"
#include "cpu_regs.h"
#include "cpufreq.h"
#include "heapblock.h"
#include "hv.h"
#include "hv_agx_power_broker.h"
#include "hv_assisted_layout.h"
#include "hv_launch_golden_j313.h"
#include "hv_launch_j313.h"
#include "hv_launch_preflight.h"
#include "hv_autonomous_layout.generated.h"
#include "hv_autonomous_memory.h"
#include "hv_autonomous_profile.h"
#include "hv_vgic.h"
#include "iodev.h"
#include "display.h"
#include "memory.h"
#include "minilzlib/minlzma.h"
#include "soc.h"
#include "string.h"
#include "tinf/tinf.h"
#include "types.h"
#include "utils.h"
#include "xnuboot.h"

struct hv_autonomous_runtime {
    void *firmware;
    u32 firmware_size;
    u64 ram_end;
    struct hv_autonomous_profile profile;
    struct hv_assisted_layout guest_layout;
    struct hv_contract_snapshot golden[HV_LAUNCH_PREFLIGHT_BLOCKING_CHECKPOINTS];
    struct hv_contract_snapshot actual;
    struct hv_launch_j313_host_state base_state;
    struct hv_launch_preflight preflight;
    u64 firmware_entry;
};

static bool publish_runtime_launch_state(struct hv_autonomous_runtime *runtime,
                                         const struct hv_autonomous_payload *payload)
{
    const struct hv_autonomous_layout *layout = &J313_AUTONOMOUS_LAYOUT;
    const struct hv_contract_snapshot *reference = &runtime->golden[0];
    struct hv_launch_j313_host_state *state = &runtime->base_state;

    memset(state, 0, sizeof(*state));

    state->identity = reference->identity;
    state->boot = reference->boot;
    state->boot.ram_base = layout->phys_base;
    state->boot.ram_size = runtime->ram_end - layout->phys_base;
    state->boot.guest_entry = layout->firmware_base;
    state->boot.args[0] = layout->boot_args_base;
    state->adt_size = cur_boot_args.devtree_size;
    state->region_count = reference->region_count;
    for (u32 i = 0; i < state->region_count; i++)
        state->regions[i] = reference->regions[i];
    state->regions[HV_CONTRACT_REGION_GUEST_RAM].base = layout->phys_base;
    state->regions[HV_CONTRACT_REGION_GUEST_RAM].size = state->boot.ram_size;
    state->regions[HV_CONTRACT_REGION_HEAP].base = layout->phys_base;
    state->regions[HV_CONTRACT_REGION_FIRMWARE].base = layout->firmware_base;
    state->regions[HV_CONTRACT_REGION_FIRMWARE].size = ALIGN_UP(payload->uncompressed_size,
                                                                HV_ASSISTED_ALIGNMENT);
    state->regions[HV_CONTRACT_REGION_ADT].base = layout->adt_base;
    state->regions[HV_CONTRACT_REGION_ADT].size = ALIGN_UP(cur_boot_args.devtree_size,
                                                           HV_ASSISTED_ALIGNMENT);
    state->regions[HV_CONTRACT_REGION_BOOT_ARGS].base = layout->boot_args_base;
    state->regions[HV_CONTRACT_REGION_FRAMEBUFFER].base = layout->virtual_fb_base;
    state->regions[HV_CONTRACT_REGION_FRAMEBUFFER].size =
        (u64)layout->virtual_fb_stride * layout->virtual_fb_height;
    state->regions[HV_CONTRACT_REGION_LOW_MEMORY].base = layout->low_mem_pa;
    state->regions[HV_CONTRACT_REGION_LOW_MEMORY].size = layout->low_mem_size;
    state->devices = reference->devices;
    state->devices.pci_ecam_base = layout->pci_ecam;
    state->devices.xhci_base = layout->xhci_base;
    state->devices.display_base = layout->virtual_fb_base;
    state->devices.display_width = layout->virtual_fb_width;
    state->devices.display_height = layout->virtual_fb_height;
    state->devices.display_stride = layout->virtual_fb_stride;
    state->cpu_count = reference->cpu_count;
    for (u32 i = 0; i < state->cpu_count; i++)
        state->cpus[i].mpidr = reference->cpus[i].mpidr;
    return hv_launch_j313_set_base_state(state);
}

static bool check_launch_checkpoint(struct hv_autonomous_runtime *runtime,
                                    enum hv_contract_checkpoint checkpoint)
{
    static const char *const checkpoint_names[] = {
        [HV_CONTRACT_PRE_HV_INIT] = "PRE_HV_INIT",
        [HV_CONTRACT_POST_HV_INIT] = "POST_HV_INIT",
        [HV_CONTRACT_POST_MAPS] = "POST_MAPS",
        [HV_CONTRACT_PRE_GUEST] = "PRE_GUEST",
    };
    struct hv_contract_snapshot *actual = &runtime->actual;
    u32 sequence = checkpoint + 1;
    const char *name = checkpoint < HV_CONTRACT_CHECKPOINT_COUNT ?
                           checkpoint_names[checkpoint] : "INVALID";

    bool captured = checkpoint == HV_CONTRACT_PRE_HV_INIT ?
                        hv_launch_j313_capture_base(checkpoint, sequence, actual) :
                        hv_launch_j313_capture(checkpoint, sequence, actual);
    if (!captured) {
        printf("PREFLIGHT FAIL checkpoint=%s reason=capture\n", name);
        iodev_console_kick();
        return false;
    }
    const struct hv_contract_schema *schema =
        checkpoint == HV_CONTRACT_PRE_HV_INIT ? &HV_J313_STANDALONE_PRE_INIT_SCHEMA :
                                                &HV_J313_STANDALONE_CONTRACT_SCHEMA;
    if (!hv_launch_preflight_check_schema(&runtime->preflight, actual, schema)) {
        const struct hv_contract_failure *failure = &runtime->preflight.failure;
        printf("PREFLIGHT FAIL checkpoint=%s field=%u index=%u rule=%u "
               "expected=0x%lx actual=0x%lx\n",
               name, failure->field, failure->index, failure->rule,
               failure->expected, failure->actual);
        iodev_console_kick();
        return false;
    }
    printf("PREFLIGHT PASS checkpoint=%s\n", name);
    iodev_console_kick();
    return true;
}

static bool enter_validated_guest(void *opaque)
{
    struct hv_autonomous_runtime *runtime = opaque;
    u64 regs[4] = {runtime->guest_layout.boot_args_base, 0, 0, 0};

    msr(SYS_IMP_APL_SPRR_CONFIG_EL1, 1);
    msr(SYS_IMP_APL_GXF_CONFIG_EL1, 1);
    sysop("isb");
    printf("PREFLIGHT PASS; entering guest entry=0x%lx arg0=0x%lx\n",
           runtime->firmware_entry, regs[0]);
    iodev_console_kick();
    hv_start((void *)runtime->firmware_entry, regs);
    return false;
}

static bool read_memory_map_region(const void *tree, int node, const char *name,
                                   u64 region[2], bool required)
{
    u32 size = 0;
    const u64 *value = adt_getprop(tree, node, name, &size);

    if (!value)
        return !required;
    if (size != sizeof(u64) * 2)
        return false;
    region[0] = value[0];
    region[1] = value[1];
    return true;
}

static bool prepare_secondary_rvbars(u64 entry)
{
    int cpus = adt_path_offset(adt, "/cpus");
    u64 rvbar = entry & ~0xfffULL;

    if (cpus < 0)
        return false;
    ADT_FOREACH_CHILD(adt, cpus)
    {
        const char *state = adt_getprop(adt, cpus, "state", NULL);
        u64 cpu_impl_reg[2];

        if (state && strcmp(state, "running") == 0)
            continue;
        if (ADT_GETPROP_ARRAY(adt, cpus, "cpu-impl-reg", cpu_impl_reg) < 0)
            return false;
        printf("Standalone: RVBAR [0x%lx] = 0x%lx\n", cpu_impl_reg[0], rvbar);
        write64(cpu_impl_reg[0], rvbar);
    }
    sysop("dmb sy");
    return true;
}

static bool map_arm_io_ranges(void)
{
    int node = adt_path_offset(adt, "/arm-io");
    u32 ranges_len = 0;
    const u32 *ranges;

    if (node < 0)
        return false;
    ranges = adt_getprop(adt, node, "ranges", &ranges_len);
    if (!ranges || ranges_len % 24)
        return false;

    for (u32 offset = 0; offset < ranges_len; offset += 24, ranges += 6) {
        u64 base = ranges[2] | ((u64)ranges[3] << 32);
        u64 size = ranges[4] | ((u64)ranges[5] << 32);

        if (!size || hv_map_hw(base, base, size))
            return false;
    }
    return true;
}

static bool prepare_boot_data(struct hv_autonomous_runtime *runtime,
                              struct hv_autonomous_status *status)
{
    const struct hv_autonomous_layout *layout = &J313_AUTONOMOUS_LAYOUT;
    struct boot_args guest_args = cur_boot_args;
    u64 trust_cache[2] = {0};
    u64 sepfw[2] = {0};
    u64 preoslog[2] = {0};
    int source_memory_map = adt_path_offset(adt, "/chosen/memory-map");
    void *guest_adt = (void *)layout->adt_base;
    int guest_memory_map;

    if (source_memory_map < 0 ||
        !read_memory_map_region(adt, source_memory_map, "TrustCache", trust_cache, true) ||
        !read_memory_map_region(adt, source_memory_map, "SEPFW", sepfw, true) ||
        !read_memory_map_region(adt, source_memory_map, "preoslog", preoslog, false) ||
        cur_boot_args.devtree_size > layout->adt_max_size ||
        runtime->firmware_size > layout->firmware_max_size ||
        !hv_assisted_layout_compute(layout->phys_base, cur_boot_args.devtree_size,
                                    trust_cache[1], runtime->firmware_size, sepfw[1],
                                    preoslog[1], &runtime->guest_layout) ||
        runtime->guest_layout.adt_base != layout->adt_base ||
        runtime->guest_layout.firmware_base != layout->firmware_base ||
        runtime->guest_layout.boot_args_base != layout->boot_args_base ||
        runtime->guest_layout.top_of_kernel_data > layout->ramdisk_base)
        return false;

    memcpy(guest_adt, adt, cur_boot_args.devtree_size);
    memcpy((void *)runtime->guest_layout.trust_cache_base, (void *)trust_cache[0], trust_cache[1]);
    memcpy((void *)runtime->guest_layout.firmware_base, runtime->firmware,
           runtime->firmware_size);
    memcpy((void *)runtime->guest_layout.sepfw_base, (void *)sepfw[0], sepfw[1]);
    if (preoslog[1])
        memcpy((void *)runtime->guest_layout.preoslog_base, (void *)preoslog[0], preoslog[1]);

    guest_memory_map = adt_path_offset(guest_adt, "/chosen/memory-map");
    u64 guest_trust_cache[2] = {runtime->guest_layout.trust_cache_base, trust_cache[1]};
    u64 guest_sepfw[2] = {runtime->guest_layout.sepfw_base, sepfw[1]};
    u64 guest_preoslog[2] = {runtime->guest_layout.preoslog_base, preoslog[1]};
    u64 guest_device_tree[2] = {runtime->guest_layout.adt_base,
                                ALIGN_UP(cur_boot_args.devtree_size, HV_ASSISTED_ALIGNMENT)};
    u64 guest_boot_args[2] = {runtime->guest_layout.boot_args_base,
                              HV_ASSISTED_BOOTARGS_SIZE};
    u64 guest_kernel[2] = {runtime->guest_layout.firmware_base, 0};
    if (guest_memory_map < 0 ||
        adt_setprop(guest_adt, guest_memory_map, "TrustCache", guest_trust_cache,
                    sizeof(guest_trust_cache)) < 0 ||
        adt_setprop(guest_adt, guest_memory_map, "SEPFW", guest_sepfw,
                    sizeof(guest_sepfw)) < 0 ||
        adt_setprop(guest_adt, guest_memory_map, "DeviceTree", guest_device_tree,
                    sizeof(guest_device_tree)) < 0 ||
        adt_setprop(guest_adt, guest_memory_map, "BootArgs", guest_boot_args,
                    sizeof(guest_boot_args)) < 0 ||
        (preoslog[1] && adt_setprop(guest_adt, guest_memory_map, "preoslog",
                                   guest_preoslog, sizeof(guest_preoslog)) < 0))
        return false;
    if (adt_getprop(guest_adt, guest_memory_map, "Kernel_mach__header", NULL) &&
        adt_setprop(guest_adt, guest_memory_map, "Kernel_mach__header", guest_kernel,
                    sizeof(guest_kernel)) < 0)
        return false;

    guest_args.phys_base = layout->phys_base;
    guest_args.mem_size = runtime->ram_end - layout->phys_base;
    guest_args.virt_base = 0xfffffe0010000000ULL + (layout->phys_base & (SZ_32M - 1));
    guest_args.devtree = (void *)(guest_args.virt_base + runtime->guest_layout.adt_base -
                                  layout->phys_base);
    guest_args.devtree_size = cur_boot_args.devtree_size;
    guest_args.top_of_kernel_data = runtime->guest_layout.top_of_kernel_data;
    guest_args.video.base = layout->virtual_fb_base;
    guest_args.video.display = 1;
    guest_args.video.stride = layout->virtual_fb_stride;
    guest_args.video.width = layout->virtual_fb_width;
    guest_args.video.height = layout->virtual_fb_height;
    guest_args.video.depth = 32;
    if (guest_args.revision <= 1)
        guest_args.rv1.mem_size_actual = guest_args.mem_size;
    else if (guest_args.revision == 2)
        guest_args.rv2.mem_size_actual = guest_args.mem_size;
    else
        guest_args.rv3.mem_size_actual = guest_args.mem_size;
    memcpy((void *)runtime->guest_layout.boot_args_base, &guest_args, sizeof(guest_args));

    dc_cvau_range((void *)runtime->guest_layout.firmware_base, runtime->firmware_size);
    ic_ivau_range((void *)runtime->guest_layout.firmware_base, runtime->firmware_size);
    dc_cvau_range(guest_adt, cur_boot_args.devtree_size);
    dc_cvau_range((void *)runtime->guest_layout.trust_cache_base, trust_cache[1]);
    dc_cvau_range((void *)runtime->guest_layout.sepfw_base, sepfw[1]);
    if (preoslog[1])
        dc_cvau_range((void *)runtime->guest_layout.preoslog_base, preoslog[1]);
    dc_cvau_range((void *)runtime->guest_layout.boot_args_base, sizeof(guest_args));
    /* M1: standalone never preloads a boot medium and publishes no ramdisk
     * region, so GPU backing pages may use the window. Clear any "ASIRAMDK"
     * header an earlier assisted launch left without a SoC reset: Mu's PEI
     * reads it with the MMU off and would otherwise keep the window from the
     * OS while this m1n1 lets GPU DMA reach it. */
    memset((void *)layout->ramdisk_base, 0, 0x1000);
    dc_civac_range((void *)layout->ramdisk_base, 0x1000);
    status->firmware_entry = runtime->guest_layout.firmware_base;
    printf("Standalone: assisted layout ADT=0x%lx TC=0x%lx FW=0x%lx SEPFW=0x%lx "
           "preoslog=0x%lx bootargs=0x%lx top=0x%lx\n",
           runtime->guest_layout.adt_base, runtime->guest_layout.trust_cache_base,
           runtime->guest_layout.firmware_base, runtime->guest_layout.sepfw_base,
           runtime->guest_layout.preoslog_base, runtime->guest_layout.boot_args_base,
           runtime->guest_layout.top_of_kernel_data);
    iodev_console_kick();
    return prepare_secondary_rvbars(runtime->guest_layout.firmware_base);
}

static bool map_stage2(const struct hv_autonomous_runtime *runtime)
{
    const struct hv_autonomous_layout *layout = &J313_AUTONOMOUS_LAYOUT;

    if (!hv_init())
        return false;
    /* Assisted HV.init() establishes the complete guest EL1 register image
     * before PCI and ADT setup. Waiting until hv_start() lets secondary/PSCI
     * bring-up observe host defaults and caused the standalone-only CPU1
     * failure caught by the POST_HV_INIT contract. */
    hv_prepare_guest_cpu_state();
    if (hv_map_hw(layout->phys_base, layout->phys_base,
                  runtime->ram_end - layout->phys_base))
        return false;
    if (hv_map_hw(layout->low_mem_ipa, layout->low_mem_pa, layout->low_mem_size))
        return false;
    return map_arm_io_ranges() && hv_agx_g2_resources_map();
}

static bool map_vuart_from_adt(void)
{
    int path[8];
    int node = adt_path_offset_trace(adt, "/arm-io/uart0", path);
    u64 base;
    const u32 *interrupts;
    u32 length;

    if (node < 0 || adt_get_reg(adt, path, "reg", 0, &base, NULL))
        return false;
    interrupts = adt_getprop(adt, node, "interrupts", &length);
    if (!interrupts || length < sizeof(*interrupts))
        return false;
    return hv_map_vuart(base, interrupts[0], IODEV_UART);
}

static bool runtime_stage(enum hv_autonomous_stage stage,
                          const struct hv_autonomous_payload *payload,
                          struct hv_autonomous_status *status, void *opaque)
{
    struct hv_autonomous_runtime *runtime = opaque;
    const struct hv_autonomous_layout *layout = &J313_AUTONOMOUS_LAYOUT;

    switch (stage) {
        case HV_AUTONOMOUS_STAGE_VALIDATE: {
            if (chip_id != T8103 || !adt_is_compatible(adt, 0, "J313AP")) {
                printf("Standalone: J313 firmware requires a J313/T8103 Mac\n");
                iodev_console_kick();
                return false;
            }
            if (!hv_autonomous_profile_decode(payload->flags, &runtime->profile) ||
                payload->layout_version != layout->layout_version ||
                payload->uncompressed_size > layout->firmware_max_size ||
                payload->compressed_size > UINT32_MAX ||
                payload->uncompressed_size > UINT32_MAX)
                return false;

            if (!hv_autonomous_resolve_ram_end(
                    layout->phys_base, layout->ram_end, cur_boot_args.phys_base,
                    cur_boot_args.mem_size, &runtime->ram_end)) {
                printf("Standalone: invalid RAM bounds guest=0x%lx configured=0x%lx "
                       "platform=0x%lx+0x%lx\n",
                       layout->phys_base, layout->ram_end, cur_boot_args.phys_base,
                       cur_boot_args.mem_size);
                return false;
            }

            printf("Standalone: RAM configured=0x%lx platform=0x%lx effective=0x%lx "
                   "size=0x%lx\n",
                   layout->ram_end, cur_boot_args.phys_base + cur_boot_args.mem_size,
                   runtime->ram_end, runtime->ram_end - layout->phys_base);
            iodev_console_kick();
            printf("PREFLIGHT INIT step=golden begin\n");
            iodev_console_kick();
            /* Some J313 ADTs do not publish the SPI-HID child.  Only require
             * its dynamic IRQ route when the device is actually declared.
             * A declared but malformed device remains fail-closed: hv_init()
             * will omit the route and POST_HV_INIT will reject the mismatch. */
            bool apple_input_declared =
                adt_path_offset(adt, "/arm-io/spi3/ipd") >= 0;
            if (!hv_launch_golden_j313_init(runtime->golden,
                                            apple_input_declared)) {
                printf("PREFLIGHT FAIL checkpoint=PRE_HV_INIT reason=golden\n");
                iodev_console_kick();
                return false;
            }
            printf("PREFLIGHT INIT step=golden ready\n");
            iodev_console_kick();
            if (!hv_launch_preflight_init(&runtime->preflight, runtime->golden,
                                          HV_LAUNCH_PREFLIGHT_BLOCKING_CHECKPOINTS,
                                          &HV_J313_STANDALONE_CONTRACT_SCHEMA)) {
                printf("PREFLIGHT FAIL checkpoint=PRE_HV_INIT reason=gate\n");
                iodev_console_kick();
                return false;
            }
            printf("PREFLIGHT INIT step=gate ready\n");
            iodev_console_kick();
            if (!publish_runtime_launch_state(runtime, payload)) {
                printf("PREFLIGHT FAIL checkpoint=PRE_HV_INIT reason=publish\n");
                iodev_console_kick();
                return false;
            }
            printf("PREFLIGHT INIT step=publish ready\n");
            iodev_console_kick();
            return check_launch_checkpoint(runtime, HV_CONTRACT_PRE_HV_INIT);
        }
        case HV_AUTONOMOUS_STAGE_PLATFORM: {
            if (runtime->profile.monitor)
                cpufreq_print_state("standalone-before");
            int result = cpufreq_init();
            if (runtime->profile.monitor)
                cpufreq_print_state("standalone-after");
            if (result) {
                printf("PREFLIGHT FAIL checkpoint=PLATFORM reason=cpufreq\n");
                iodev_console_kick();
                return false;
            }
            printf("PREFLIGHT PASS checkpoint=PLATFORM component=cpufreq\n");
            iodev_console_kick();
            return true;
        }
        case HV_AUTONOMOUS_STAGE_DECOMPRESS: {
            u32 source_size = payload->compressed_size;
            u32 destination_size = payload->uncompressed_size;

            runtime->firmware = heapblock_alloc_aligned(destination_size, SZ_16K);
            if (!runtime->firmware ||
                !XzDecode((u8 *)payload->compressed, &source_size, runtime->firmware,
                          &destination_size) ||
                source_size != payload->compressed_size ||
                destination_size != payload->uncompressed_size ||
                tinf_crc32(runtime->firmware, destination_size) != payload->crc32)
                return false;
            runtime->firmware_size = destination_size;
            return true;
        }
        case HV_AUTONOMOUS_STAGE_BOOT_DATA:
            if (!prepare_boot_data(runtime, status))
                return false;
            runtime->firmware_entry = status->firmware_entry;
            return true;
        case HV_AUTONOMOUS_STAGE_STAGE2:
            return map_stage2(runtime);
        case HV_AUTONOMOUS_STAGE_VGIC:
            return true; // hv_init() owns vGIC/PSCI initialization.
        case HV_AUTONOMOUS_STAGE_PCI_NVME:
            return hv_pci_init(layout->pci_ecam, layout->pci_bar_window,
                               layout->nvme_vintid);
        case HV_AUTONOMOUS_STAGE_XHCI:
            // Broad /arm-io pass-through is installed after hv_init(); restore
            // the diagnostic/DART hook over the physical xHCI page last.
            return hv_vgic_rearm_j313_xhci_trace();
        case HV_AUTONOMOUS_STAGE_VUART:
            return map_vuart_from_adt() &&
                   check_launch_checkpoint(runtime, HV_CONTRACT_POST_HV_INIT);
        case HV_AUTONOMOUS_STAGE_READY:
            memset((void *)layout->virtual_fb_base, 0,
                   (u64)layout->virtual_fb_stride * layout->virtual_fb_height);
            if (runtime->profile.physical_display &&
                display_prepare_guest_surface(
                    layout->virtual_fb_base,
                    (u64)layout->virtual_fb_stride * layout->virtual_fb_height,
                    layout->virtual_fb_width, layout->virtual_fb_height,
                    layout->virtual_fb_stride, 32) != 1)
                return false;
            if (runtime->profile.virtual_display &&
                !hv_configure_fb_stream(
                    layout->virtual_fb_base,
                    (u64)layout->virtual_fb_stride * layout->virtual_fb_height,
                    layout->virtual_fb_width, layout->virtual_fb_height,
                    layout->virtual_fb_stride))
                return false;
            printf("Standalone: display physical=%u virtual=%u telemetry=%u\n",
                   runtime->profile.physical_display, runtime->profile.virtual_display,
                   runtime->profile.telemetry);
            return check_launch_checkpoint(runtime, HV_CONTRACT_POST_MAPS);
        case HV_AUTONOMOUS_STAGE_ENTERED: {
            if (!check_launch_checkpoint(runtime, HV_CONTRACT_PRE_GUEST))
                return false;
            return hv_launch_preflight_enter(&runtime->preflight, enter_validated_guest,
                                             runtime);
        }
        case HV_AUTONOMOUS_STAGE_COUNT:
            return false;
    }
    return false;
}

enum hv_autonomous_result hv_autonomous_prepare(const struct hv_autonomous_payload *payload,
                                                struct hv_autonomous_status *status)
{
    static const struct hv_autonomous_ops ops = {
        .callbacks = {
            runtime_stage, runtime_stage, runtime_stage, runtime_stage, runtime_stage,
            runtime_stage, runtime_stage, runtime_stage, runtime_stage, runtime_stage,
            runtime_stage,
        },
    };
    /* Golden snapshots, live capture and the published base state do not fit
     * safely in m1n1's early stack. Preparation is single-shot, so put the
     * workspace in BSS and clear it explicitly for deterministic retries. */
    static struct hv_autonomous_runtime runtime;
    memset(&runtime, 0, sizeof(runtime));

    return hv_autonomous_prepare_with_ops(payload, status, &ops, &runtime);
}

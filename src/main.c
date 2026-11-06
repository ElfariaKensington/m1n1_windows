/* SPDX-License-Identifier: MIT */

#include "../build/build_cfg.h"
#include "../build/build_tag.h"

#include "../config.h"

#include "adt.h"
#include "aic.h"
#include "boot_options.h"
#include "cpufreq.h"
#include "display.h"
#include "exception.h"
#include "fb.h"
#include "firmware.h"
#include "gxf.h"
#include "heapblock.h"
#include "hv_autonomous_boot_runtime.h"
#include "hv_bootstrap.h"
#include "hv_stage_role.h"
#include "mcc.h"
#include "memory.h"
#include "nvme.h"
#include "payload.h"
#ifdef M1N1_J316S_SETUP
#include "hv_j316s_setup.h"
#endif
#include "pcie.h"
#include "pmgr.h"
#include "proxy_boot_identity.h"
#include "sep.h"
#include "smp.h"
#include "string.h"
#include "uart.h"
#include "uartproxy.h"
#include "usb.h"
#include "utils.h"
#include "wdt.h"
#include "xnuboot.h"

#if defined(DCP_IOMFB_START_OBSERVER) || defined(DCP_IOMFB_EARLY_PIODMA_OBSERVER) || \
    defined(DCP_IOMFB_SET_SHMEM_OBSERVER) || defined(DCP_IOMFB_A401_OBSERVER) || \
    defined(DCP_IOMFB_A426_OBSERVER) || defined(DCP_IOMFB_A449_OBSERVER) || \
    defined(DCP_IOMFB_A456_OBSERVER) || defined(DCP_IOMFB_A411_OBSERVER) || \
    defined(DCP_IOMFB_A472_OBSERVER) || defined(DCP_IOMFB_A410_OBSERVER) || \
    defined(DCP_IOMFB_A412_OBSERVER) || defined(DCP_IOMFB_SWAP_OBSERVER)
#define DCP_IOMFB_TERMINAL_OBSERVER 1
#else
#define DCP_IOMFB_TERMINAL_OBSERVER 0
#endif

struct vector_args next_stage;

const char version_tag[] = "##m1n1_ver##" BUILD_TAG;
const char *const m1n1_version = version_tag + 12;

u32 board_id = ~0, chip_id = ~0;
static bool boot_display_initialized;

static const char *boot_cmdline(void)
{
    switch (cur_boot_args.revision) {
        case 1:
            return cur_boot_args.rv1.cmdline;
        case 2:
            return cur_boot_args.rv2.cmdline;
        default:
            return cur_boot_args.rv3.cmdline;
    }
}

void get_device_info(void)
{
    const char *model = (const char *)adt_getprop(adt, 0, "model", NULL);
    const char *target = (const char *)adt_getprop(adt, 0, "target-type", NULL);

    printf("Device info:\n");

    if (model)
        printf("  Model: %s\n", model);

    if (target)
        printf("  Target: %s\n", target);

    is_mac = !!strstr(model, "Mac");

    int chosen = adt_path_offset(adt, "/chosen");
    if (chosen > 0) {
        if (ADT_GETPROP(adt, chosen, "board-id", &board_id) < 0)
            printf("Failed to find board-id\n");
        if (ADT_GETPROP(adt, chosen, "chip-id", &chip_id) < 0)
            printf("Failed to find chip-id\n");

        printf("  Board-ID: 0x%x\n", board_id);
        printf("  Chip-ID: 0x%x\n", chip_id);
    } else {
        printf("No chosen node!\n");
    }

    printf("\n");
}

void run_actions(void)
{
    bool usb_up = false;

#ifndef BRINGUP
#ifdef EARLY_PROXY_TIMEOUT
    int node = adt_path_offset(adt, "/chosen/asmb");
    u64 lp_sip0 = 0;

    if (node >= 0) {
        ADT_GETPROP(adt, node, "lp-sip0", &lp_sip0);
        printf("Boot policy: sip0 = %ld\n", lp_sip0);
    }

    if (!cur_boot_args.video.display && lp_sip0 == 127) {
        printf("Bringing up USB for early debug...\n");

        usb_init();
        usb_iodev_init();

        usb_up = true;

        printf("Waiting for proxy connection... ");
        for (int i = 0; i < EARLY_PROXY_TIMEOUT * 100; i++) {
            for (int j = 0; j < USB_IODEV_COUNT; j++) {
                iodev_id_t iodev = IODEV_USB0 + j;

                if (!(iodev_get_usage(iodev) & USAGE_UARTPROXY))
                    continue;

                usb_iodev_vuart_setup(iodev);
                iodev_handle_events(iodev);
                if (iodev_can_write(iodev) || iodev_can_write(IODEV_USB_VUART)) {
                    printf(" Connected!\n");
                    uartproxy_run(NULL);
                    return;
                }
            }

            mdelay(10);
            if (i % 100 == 99)
                printf(".");
        }
        printf(" Timed out\n");
    }
#endif
#endif

    printf("Checking for payloads...\n");

#ifdef M1N1_J316S_SETUP
    if (hv_j316s_setup_present()) {
        if (hv_j316s_setup_run())
            return;
        /* Do not bring up the device-mode proxy after touching host-mode USB. */
        usb_up |= hv_j316s_setup_owns_usb();
        goto proxy_fallback;
    }
#endif

    enum hv_stage_role stage_role = hv_stage_role_current();
    if (hv_stage_role_allows_bootstrap(stage_role)) {
        enum hv_bootstrap_attempt bootstrap = hv_bootstrap_chainload_if_present(&usb_up);
        if (bootstrap == HV_BOOTSTRAP_HANDLED)
            return;
        if (bootstrap == HV_BOOTSTRAP_ATTEMPT_FAILED)
            goto proxy_fallback;
    }

    if (hv_stage_role_allows_autonomous(stage_role)) {
        enum hv_autonomous_boot_attempt autonomous = hv_autonomous_boot_if_present(&usb_up);
        if (autonomous == HV_AUTONOMOUS_BOOT_HANDLED)
            return;
        if (autonomous == HV_AUTONOMOUS_BOOT_ATTEMPT_FAILED)
            goto proxy_fallback;
    }

    if (payload_run() == 0) {
        printf("Valid payload found\n");
        return;
    }
    fb_set_active(true);

    printf("No valid payload found\n");

proxy_fallback:

#ifndef BRINGUP
    if (!usb_up) {
        usb_init();
        usb_iodev_init();
    }
#endif

    printf("Running proxy...\n");

    uartproxy_run(NULL);
}

void m1n1_main(void)
{
    /* Capture one immutable identity before host-visible proxy work starts. */
    proxy_boot_identity_init(get_ticks());

    printf("\n\nm1n1 %s\n", m1n1_version);
    printf("Copyright The Asahi Linux Contributors\n");
    printf("Licensed under the MIT license\n\n");

    printf("Running in EL%lu\n\n", mrs(CurrentEL) >> 2);

    firmware_init();

    heapblock_init();

#ifndef BRINGUP
    if (supports_gxf())
        gxf_init();
    mcc_init();
    mmu_init();
    aic_init();
#endif
    wdt_disable();
#ifndef BRINGUP
    pmgr_init();
#ifdef USE_FB
    if (boot_option_skip_display(boot_cmdline())) {
        printf("display: Device initialization explicitly disabled by boot option\n");
    } else {
        display_init();
        boot_display_initialized = true;
#if defined(DCP_IOMFB_FULL_OWNER) && !DCP_IOMFB_TERMINAL_OBSERVER
        /* The HV scanout owner is the same logical owner initialized here.
         * Closing and immediately reconstructing it can strand the DCP
         * endpoint in the same boot stage.  Keep it live until the explicit
         * next-stage or rollback shutdown path below. */
        printf("display: retaining initial IOMFB owner for HV scanout\n");
#else
        // Kick DCP to sleep, so dodgy monitors which cause reconnect cycles don't cause us to lose
        // the framebuffer.
        display_shutdown(DCP_SLEEP_IF_EXTERNAL);
#endif
#if defined(DCP_IOMFB_A401_OBSERVER) || defined(DCP_IOMFB_A426_OBSERVER) || \
    defined(DCP_IOMFB_A449_OBSERVER) || defined(DCP_IOMFB_A456_OBSERVER) || \
    defined(DCP_IOMFB_A411_OBSERVER) || defined(DCP_IOMFB_A472_OBSERVER) || \
    defined(DCP_IOMFB_A410_OBSERVER) || defined(DCP_IOMFB_A412_OBSERVER) || \
    defined(DCP_IOMFB_SWAP_OBSERVER)
        if (!display_shutdown_complete())
            panic("IOMFB admission observer could not prove DCP quiesce\n");
        printf("IOMFB admission observer complete; payload execution disabled\n");
        usb_init();
        usb_iodev_init();
        printf("Running proxy...\n");
        uartproxy_run(NULL);
        panic("IOMFB admission observer proxy returned\n");
#endif
        // On idevice we need to always clear, because otherwise it looks scuffed on white devices
        fb_init(!is_mac);
        fb_display_logo();
#ifdef FB_SILENT_MODE
        fb_set_active(!cur_boot_args.video.display);
#else
        fb_set_active(true);
#endif
    }
#endif

    cpufreq_fixup();
    sep_init();
#endif

    printf("Initialization complete.\n");

    run_actions();

    if (!next_stage.entry) {
        panic("Nothing to do!\n");
    }

    printf("Preparing to run next stage at %p...\n", next_stage.entry);

    nvme_shutdown();
    exception_shutdown();
#ifndef BRINGUP
    usb_iodev_shutdown();
#ifdef USE_FB
    if (boot_display_initialized) {
        display_shutdown(DCP_SLEEP_IF_EXTERNAL);
        fb_shutdown(next_stage.restore_logo);
    }
#endif
    mmu_shutdown();
#endif

    printf("Vectoring to next stage...\n");

    next_stage.entry(next_stage.args[0], next_stage.args[1], next_stage.args[2], next_stage.args[3],
                     next_stage.args[4]);

    panic("Next stage returned!\n");
}

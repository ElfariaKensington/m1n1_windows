/* SPDX-License-Identifier: MIT */

#include "../build/build_cfg.h"

#include "display.h"
#include "display_dcp_frontend.h"
#include "display_guest.h"
#include "adt.h"
#include "assert.h"
#include "dcp.h"
#include "dcp_iboot.h"
#include "fb.h"
#include "firmware.h"
#include "memory.h"
#include "soc.h"
#include "string.h"
#include "utils.h"
#include "xnuboot.h"

#define DISPLAY_STATUS_DELAY         100
#define DISPLAY_STATUS_RETRIES(dptx) ((dptx) ? 100 : 20)

#define COMPARE(a, b)                                                                              \
    if ((a) > (b)) {                                                                               \
        *best = modes[i];                                                                          \
        continue;                                                                                  \
    } else if ((a) < (b)) {                                                                        \
        continue;                                                                                  \
    }

static dcp_dev_t *dcp;
static dcp_iboot_if_t *iboot;
static dcp_ib_swap_async_t scanout_swap;
static u64 scanout_cookie;
static u64 scanout_quiesce_after;
static bool scanout_swap_active;
static bool scanout_quiescing;
static u32 scanout_owner_swap_id;
static u64 scanout_owner_surface_iova;
static u64 fb_dva;
static u64 fb_size;
static u64 guest_fb_dva;
static u64 guest_fb_size;
bool has_dcp;
bool display_is_external;
bool display_is_dptx;
bool display_needs_power_cycle;

static const display_config_t display_config_m1 = {
    .dcp = "/arm-io/dcp",
    .dcp_dart = "/arm-io/dart-dcp",
    .disp_dart = "/arm-io/dart-disp0",
    .pmgr_dev = "DISP0_CPU0",
    .dcp_alias = "dcp",
};

#define USE_DCPEXT 1

static const display_config_t display_config_m2 = {
#if USE_DCPEXT
    .dcp = "/arm-io/dcpext",
    .dcp_dart = "/arm-io/dart-dcpext",
    .disp_dart = "/arm-io/dart-dispext0",
    .pmgr_dev = "DISPEXT_CPU0",
    .dcp_alias = "dcpext",
    .dcp_index = 1,
#else
    .dcp = "/arm-io/dcp",
    .dcp_dart = "/arm-io/dart-dcp",
    .disp_dart = "/arm-io/dart-disp0",
    .dp2hdmi_gpio = "/arm-io/dp2hdmi-gpio",
    .dptx_phy = "/arm-io/dptx-phy",
    .pmgr_dev = "DISP0_CPU0",
    .dcp_alias = "dcp",
    .dcp_index = 0,
#endif
    .dp2hdmi_gpio = "/arm-io/dp2hdmi-gpio",
    .dptx_phy = "/arm-io/dptx-phy",
    .num_dptxports = 2,
};

static const display_config_t display_config_m2_pro_max = {
#if USE_DCPEXT
    .dcp = "/arm-io/dcpext0",
    .dcp_dart = "/arm-io/dart-dcpext0",
    .disp_dart = "/arm-io/dart-dispext0",
    .pmgr_dev = "DISPEXT0_CPU0",
    .dcp_alias = "dcpext0",
    .dcp_index = 1,
    .num_dptxports = 2,
#else
    .dcp = "/arm-io/dcp0",
    .dcp_dart = "/arm-io/dart-dcp0",
    .disp_dart = "/arm-io/dart-disp0",
    .pmgr_dev = "DISP0_CPU0",
    .dcp_alias = "dcp",
    .dcp_index = 0,
    .num_dptxports = 1,
#endif
    .dp2hdmi_gpio = "/arm-io/dp2hdmi-gpio0",
    .dptx_phy = "/arm-io/lpdptx-phy0",
};

static const display_config_t display_config_m2_ultra = {
    .dcp = "/arm-io/dcpext4",
    .dcp_dart = "/arm-io/dart-dcpext4",
    .disp_dart = "/arm-io/dart-dispext4",
    .dp2hdmi_gpio = "/arm-io/dp2hdmi-gpio1",
    .dptx_phy = "/arm-io/lpdptx-phy1",
    .pmgr_dev = "DISPEXT0_CPU0",
    .dcp_alias = "dcpext4",
    .dcp_index = 1,
    .num_dptxports = 2,
    .die = 1,
};

#define abs(x) ((x) >= 0 ? (x) : -(x))

u64 display_mode_fb_size(dcp_timing_mode_t *mode)
{
    // assume 4 byte per pixel (either BGRA x2r10b10g10)
    return mode->width * mode->height * 4;
}

static void display_choose_timing_mode(dcp_timing_mode_t *modes, int cnt, dcp_timing_mode_t *best,
                                       dcp_timing_mode_t *want)
{
    *best = modes[0];

    for (int i = 1; i < cnt; i++) {
        COMPARE(modes[i].valid, best->valid);
        if (want && want->valid) {
            COMPARE(modes[i].width == want->width && modes[i].height == want->height,
                    best->width == want->width && best->height == want->height);
            COMPARE(-abs((long)modes[i].fps - (long)want->fps),
                    -abs((long)best->fps - (long)want->fps));
        } else {
            COMPARE(display_mode_fb_size(&modes[i]) <= fb_size,
                    display_mode_fb_size(best) <= fb_size);
        }

        COMPARE(modes[i].width <= 1920, best->width <= 1920);
        COMPARE(modes[i].height <= 1200, best->height <= 1200);
        COMPARE(modes[i].fps <= 60 << 16, best->fps <= 60 << 16);
        COMPARE(modes[i].width, best->width);
        COMPARE(modes[i].height, best->height);
        COMPARE(modes[i].fps, best->fps);
    }

    printf("display: timing mode: valid=%d %dx%d %d.%02d Hz\n", best->valid, best->width,
           best->height, best->fps >> 16, ((best->fps & 0xffff) * 100 + 0x7fff) >> 16);
}

static void display_choose_color_mode(dcp_color_mode_t *modes, int cnt, dcp_color_mode_t *best)
{
    *best = modes[0];

    for (int i = 1; i < cnt; i++) {
        COMPARE(modes[i].valid, best->valid);
        COMPARE(modes[i].bpp <= 32, best->bpp <= 32);
        COMPARE(modes[i].bpp, best->bpp);
        COMPARE(-modes[i].colorimetry, -best->colorimetry);
        COMPARE(-modes[i].encoding, -best->encoding);
        COMPARE(-modes[i].eotf, -best->eotf);
    }

    printf("display: color mode: valid=%d colorimetry=%d eotf=%d encoding=%d bpp=%d\n", best->valid,
           best->colorimetry, best->eotf, best->encoding, best->bpp);
}

int display_get_vram(u64 *paddr, u64 *size)
{
    int ret = 0;
    int adt_path[4];
    int node = adt_path_offset_trace(adt, "/vram", adt_path);

    if (node < 0) {
        printf("display: '/vram' not found\n");
        return -1;
    }

    int pp = 0;
    while (adt_path[pp])
        pp++;
    adt_path[pp + 1] = 0;

    ret = adt_get_reg(adt, adt_path, "reg", 0, paddr, size);
    if (ret < 0) {
        printf("display: failed to read /vram/reg\n");
        return -1;
    }

    if (*paddr != cur_boot_args.video.base) {
        printf("display: vram does not match boot_args.video.base\n");
        return -1;
    }

    return 0;
}

static uintptr_t display_map_fb(uintptr_t iova, u64 paddr, u64 size)
{
    if (iova == 0) {
        u64 iova_disp0 = 0;
        u64 iova_dcp = 0;

        // start scanning for free iova space on vm-base
        iova_dcp = dart_find_iova(dcp->dart_dcp, dart_vm_base(dcp->dart_dcp) + SZ_16K, size);
        if (DART_IS_ERR(iova_dcp)) {
            printf("display: failed to find IOVA for fb of %06zx bytes (dcp)\n", size);
            return iova_dcp;
        }

        // try to map the fb to the same IOVA on disp0
        iova_disp0 = dart_find_iova(dcp->dart_disp, iova_dcp, size);
        if (DART_IS_ERR(iova_disp0)) {
            printf("display: failed to find IOVA for fb of %06zx bytes (disp0)\n", size);
            return iova_disp0;
        }

        // try to find the same IOVA on DCP again
        if (iova_disp0 != iova_dcp) {
            iova_dcp = dart_find_iova(dcp->dart_dcp, iova_disp0, size);
            if (DART_IS_ERR(iova_dcp)) {
                printf("display: failed to find IOVA for fb of %06zx bytes (dcp)\n", size);
                return iova_dcp;
            }
        }

        // assume this results in the same IOVA, not sure if this is required but matches what iboot
        // does on other models.
        if (iova_disp0 != iova_dcp) {
            printf("display: IOVA mismatch for fb between dcp (%08lx) and disp0 (%08lx)\n",
                   (u64)iova_dcp, (u64)iova_disp0);
            return DART_PTR_ERR;
        }

        iova = iova_dcp;
    }

    int ret = dart_map(dcp->dart_disp, iova, (void *)paddr, size);
    if (ret < 0) {
        printf("display: failed to map fb to dart-disp0\n");
        return DART_PTR_ERR;
    }

    ret = dart_map(dcp->dart_dcp, iova, (void *)paddr, size);
    if (ret < 0) {
        printf("display: failed to map fb to dart-dcp\n");
        dart_unmap(dcp->dart_disp, iova, size);
        return DART_PTR_ERR;
    }

    return iova;
}

const display_config_t *display_get_config(void)
{
    const display_config_t *conf = NULL;

    if (adt_is_compatible(adt, 0, "J473AP"))
        conf = &display_config_m2;
    else if (adt_is_compatible(adt, 0, "J474sAP") || adt_is_compatible(adt, 0, "J475cAP"))
        conf = &display_config_m2_pro_max;
    else if (adt_is_compatible(adt, 0, "J180dAP") || adt_is_compatible(adt, 0, "J475dAP"))
        conf = &display_config_m2_ultra;
    else
        conf = &display_config_m1;

    has_dcp = adt_path_offset(adt, conf->dcp) > 0;
    if (!has_dcp) {
        return NULL;
    }

    return conf;
}

int display_start_dcp(void)
{
    enum display_dcp_frontend frontend;

#ifdef DCP_IOMFB_FULL_OWNER
    if (dcp_iomfb_owner_active(dcp))
        return 0;
    if (dcp)
        return -1;
#else
    if (iboot)
        return dcp_ib_is_ready(iboot) ? 0 : -1;
    if (dcp) {
        printf("display: refusing DCP reopen while retained owner is live\n");
        return -1;
    }
#endif

#ifdef NO_DISPLAY
    printf("display: NO_DISPLAY!\n");
    return 0;
#endif

    const display_config_t *disp_cfg = display_get_config();

    if (!has_dcp) {
        printf("display: device has no DCP. Display will not be initialised.\n");
        return -1;
    }

    display_is_dptx = !!disp_cfg->dptx_phy[0];

    dcp = dcp_init(disp_cfg);
    if (!dcp) {
        printf("display: failed to initialize DCP\n");
        return -1;
    }

    // determine frame buffer PA and size from "/vram"
    u64 pa, size;
    if (display_get_vram(&pa, &size)) {
        // use a safe fb_size
        fb_size = cur_boot_args.video.stride * cur_boot_args.video.height *
                  ((cur_boot_args.video.depth + 7) / 8);
    } else {
        fb_size = size;
    }

    // Find the framebuffer DVA
    fb_dva = dart_search(dcp->dart_disp, (void *)cur_boot_args.video.base);
    // framebuffer is not mapped on the M1 Ultra Mac Studio
    if (DART_IS_ERR(fb_dva) || !fb_dva)
        fb_dva = display_map_fb(0, pa, size);
    if (DART_IS_ERR(fb_dva)) {
        printf("display: failed to find display DVA\n");
        fb_dva = 0;
        dcp_shutdown(dcp, false);
        return -1;
    }

#ifdef DCP_IOMFB_FULL_OWNER
    frontend = display_dcp_frontend_select(true, display_is_external);
#else
    frontend = display_dcp_frontend_select(false, display_is_external);
#endif
    if (frontend == DISPLAY_DCP_FRONTEND_UNSUPPORTED) {
        printf("display: IOMFB full owner only supports the internal panel\n");
        dcp_shutdown(dcp, false);
        dcp = NULL;
        return -1;
    }
    if (frontend == DISPLAY_DCP_FRONTEND_IBOOT) {
        iboot = dcp_ib_init(dcp);
        if (!dcp_ib_is_ready(iboot)) {
            printf("display: failed to initialize DCP iBoot interface\n");
            if (iboot) {
                printf("display: retaining failed iBoot owner until reset\n");
                return -1;
            }
            if (dcp_shutdown(dcp, false) == 0)
                dcp = NULL;
            else
                printf("display: DCP shutdown failed; retaining owner until reset\n");
            return -1;
        }
    }

#ifdef DCP_IOMFB_FULL_OWNER
    /* The J313/13.5 Asahi contract starts endpoint 0x37 directly after RTKit.
     * Neither the external iBoot frontend nor the optional system AFK service
     * belongs to this reduced internal-panel ownership path. */
    else if (!dcp_iomfb_owner_supported()) {
        printf("display: refusing IOMFB ownership outside exact J313/13.5 profile\n");
        dcp_shutdown(dcp, false);
        dcp = NULL;
        return -1;
    } else if (!dcp_iomfb_owner_start(dcp)) {
        printf("display: IOMFB single-owner bootstrap failed closed\n");
        return -1;
    }
#endif

#ifdef DCP_IOMFB_LATCH_OBSERVER
    if (!dcp_iomfb_observer_start(dcp))
        printf("display: IOMFB latch observer unavailable; trace remains fail-closed\n");
#endif

    return 0;
}

#ifdef DCP_IOMFB_SWAP_OBSERVER
static bool display_iomfb_swap_observer_run(void)
{
    int swap_id;
    int latch = 0;

    if (!dcp_iomfb_owner_active(dcp) || !fb_dva ||
        !cur_boot_args.video.width || !cur_boot_args.video.height ||
        (u64)cur_boot_args.video.stride <
            (u64)cur_boot_args.video.width * 4)
        return false;
    swap_id = dcp_iomfb_owner_present(dcp, fb_dva,
                                      cur_boot_args.video.width,
                                      cur_boot_args.video.height,
                                      cur_boot_args.video.stride,
                                      DCP_IOMFB_FORMAT_BGRA);
    if (swap_id <= 0) {
        printf("display: one-swap observer rejected before A408\n");
        return false;
    }
    for (unsigned int attempt = 0; attempt < 5000 && latch == 0; ++attempt) {
        latch = dcp_iomfb_owner_poll_latch(dcp, (u32)swap_id);
        if (latch == 0)
            udelay(100);
    }
    printf("display: one-swap observer swap_id=%d D589=%s\n", swap_id,
           latch > 0 ? "MATCHED" : latch == 0 ? "TIMEOUT" : "FAILED");
    return latch > 0;
}
#endif

struct display_options {
    bool retina;
};

int display_parse_mode(const char *config, dcp_timing_mode_t *mode, struct display_options *opts)
{
    memset(mode, 0, sizeof(*mode));

    if (!config || !strcmp(config, "auto"))
        return 0;

    const char *s_w = config;
    const char *s_h = strchr(config, 'x');
    const char *s_fps = strchr(config, '@');

    if (s_w && s_h) {
        mode->width = atol(s_w);
        mode->height = atol(s_h + 1);
        mode->valid = mode->width && mode->height;
    }

    if (s_fps) {
        mode->fps = atol(s_fps + 1) << 16;

        const char *s_fps_frac = strchr(s_fps + 1, '.');
        if (s_fps_frac) {
            // Assumes two decimals...
            mode->fps += (atol(s_fps_frac + 1) << 16) / 100;
        }
    }

    const char *option = config;
    while (option && opts) {
        if (!strncmp(option + 1, "retina", 6))
            opts->retina = true;
        option = strchr(option + 1, ',');
    }

    printf("display: want mode: valid=%d %dx%d %d.%02d Hz\n", mode->valid, mode->width,
           mode->height, mode->fps >> 16, ((mode->fps & 0xffff) * 100 + 0x7fff) >> 16);

    return mode->valid;
}

static int display_swap(u64 iova, u32 stride, u32 width, u32 height)
{
    int ret;

    dcp_layer_t layer = {
        .planes = {{
            .addr = iova,
            .stride = stride,
            .addr_format = ADDR_PLANAR,
        }},
        .plane_cnt = 1,
        .width = width,
        .height = height,
        .surface_fmt = FMT_BGRA,
        .colorspace = 2,
        .eotf = EOTF_GAMMA_SDR,
        .transform = XFRM_NONE,
    };

    if ((ret = dcp_ib_set_surface(iboot, &layer)) < 0) {
        printf("display: failed to set surface\n");
        return -1;
    }

    return 0;
}

static u64 display_guest_map(void *opaque, u64 base, u64 size)
{
    (void)opaque;
    u64 iova = display_map_fb(0, base, size);
    return DART_IS_ERR(iova) ? 0 : iova;
}

static u32 display_guest_owner_present_begin(void *opaque, u64 iova,
                                             u32 width, u32 height,
                                             u32 stride)
{
    int swap_id;

    (void)opaque;
    swap_id = dcp_iomfb_owner_present(dcp, iova, width, height, stride,
                                      DCP_IOMFB_FORMAT_BGRA);
    return swap_id > 0 ? (u32)swap_id : 0;
}

static int display_guest_owner_latch_poll(void *opaque, u32 expected_swap_id)
{
    (void)opaque;
    return dcp_iomfb_owner_poll_latch(dcp, expected_swap_id);
}

static void display_guest_owner_wait(void *opaque)
{
    (void)opaque;
    udelay(100);
}

static bool display_guest_present(void *opaque, u64 iova, u32 width, u32 height, u32 stride)
{
    (void)opaque;
    struct display_guest_rect destination;
    if (!iboot)
        return false;
    if (!display_guest_fit(width, height, cur_boot_args.video.width,
                           cur_boot_args.video.height, &destination))
        return false;

    dcp_layer_t layer = {
        .planes = {{
            .addr = iova,
            .stride = stride,
            .addr_format = ADDR_PLANAR,
        }},
        .plane_cnt = 1,
        .width = width,
        .height = height,
        .surface_fmt = FMT_BGRA,
        .colorspace = 2,
        .eotf = EOTF_GAMMA_SDR,
        .transform = XFRM_NONE,
    };
    dcp_rect_t source = {.w = width, .h = height};
    dcp_rect_t target = {
        .w = destination.width,
        .h = destination.height,
        .x = destination.x,
        .y = destination.y,
    };

    int swap_id = dcp_ib_swap_begin(iboot);
    if (swap_id < 0)
        return false;
#ifdef DCP_IOMFB_LATCH_OBSERVER
    dcp_iomfb_observer_arm(dcp, (u32)swap_id);
#endif
    int layer_ret = dcp_ib_swap_set_layer(iboot, 0, &layer, &source, &target);
    int end_ret = dcp_ib_swap_end(iboot);
    if (layer_ret < 0 || end_ret < 0)
        return false;

#ifdef DCP_IOMFB_LATCH_OBSERVER
    int latch = 0;
    for (unsigned int attempt = 0; attempt < 5000 && latch == 0; ++attempt) {
        latch = dcp_iomfb_observer_poll_latch(dcp, (u32)swap_id);
        if (latch == 0)
            udelay(100);
    }
    printf("display: passive IOMFB D589 trace swap_id=%d verdict=%s\n", swap_id,
           latch > 0 ? "MATCHED" : latch == 0 ? "TIMEOUT" : "UNAVAILABLE");
#else
    /* DCP consumes the surface asynchronously. Keep the old mapping alive until
     * the swap has crossed at least one display interval. */
    mdelay(150);
#endif
    printf("display: guest surface scaled %ux%u -> %ux%u at %u,%u (swap_id=%d)\n",
           width, height, destination.width, destination.height, destination.x,
           destination.y, swap_id);
    return true;
}

static void display_guest_unmap(void *opaque, u64 iova, u64 size)
{
    (void)opaque;
    dart_unmap(dcp->dart_disp, iova, size);
    dart_unmap(dcp->dart_dcp, iova, size);
}

static bool display_scanout_build_layer(u64 iova, u32 width, u32 height,
                                        u32 stride, bool rgba, dcp_layer_t *layer,
                                        dcp_rect_t *source, dcp_rect_t *target)
{
    struct display_guest_rect destination;

    if (!layer || !source || !target || !iova || !width || !height ||
        (u64)stride < (u64)width * 4 ||
        !display_guest_fit(width, height, cur_boot_args.video.width,
                           cur_boot_args.video.height, &destination))
        return false;
    memset(layer, 0, sizeof(*layer));
    layer->planes[0].addr = iova;
    layer->planes[0].stride = stride;
    layer->planes[0].addr_format = ADDR_PLANAR;
    layer->plane_cnt = 1;
    layer->width = width;
    layer->height = height;
    layer->surface_fmt = rgba ? FMT_RGBA : FMT_BGRA;
    layer->colorspace = 2;
    layer->eotf = EOTF_GAMMA_SDR;
    layer->transform = XFRM_NONE;
    *source = (dcp_rect_t){.w = width, .h = height};
    *target = (dcp_rect_t){
        .w = destination.width,
        .h = destination.height,
        .x = destination.x,
        .y = destination.y,
    };
    return true;
}

bool display_scanout_ready(void)
{
    return (iboot || dcp_iomfb_owner_active(dcp)) && dcp && dcp->dart_dcp &&
           dcp->dart_disp && dcp->iovad_dcp &&
           !display_is_external;
}

bool display_scanout_latch_source_proven(void)
{
#ifdef DCP_IOMFB_FULL_OWNER
    return display_dcp_frontend_has_latch_source(
        DISPLAY_DCP_FRONTEND_IOMFB, dcp_iomfb_owner_active(dcp));
#else
    return false;
#endif
}

bool display_scanout_reserve_iova(u64 size, u64 alignment, u64 *iova)
{
    u64 allocated;

    if (!iova || !size || !alignment || (alignment & (alignment - 1)) ||
        alignment > SZ_16K || !display_scanout_ready())
        return false;
    allocated = iova_alloc(dcp->iovad_dcp, size);
    if (!allocated || (allocated & (alignment - 1))) {
        if (allocated)
            iova_free(dcp->iovad_dcp, allocated, size);
        return false;
    }
    *iova = allocated;
    return true;
}

void display_scanout_free_iova(u64 iova, u64 size)
{
    if (display_scanout_ready() && iova && size)
        iova_free(dcp->iovad_dcp, iova, size);
}

bool display_scanout_map(unsigned dart_index, u64 iova, u64 pa, u64 size)
{
    dart_dev_t *dart;

    if (!display_scanout_ready() || dart_index > 1 || !iova || !pa || !size)
        return false;
    dart = dart_index == 0 ? dcp->dart_disp : dcp->dart_dcp;
    return dart_map(dart, iova, (void *)(uintptr_t)pa, size) == 0;
}

void display_scanout_unmap(unsigned dart_index, u64 iova, u64 size)
{
    dart_dev_t *dart;

    if (!display_scanout_ready() || dart_index > 1 || !iova || !size)
        return;
    dart = dart_index == 0 ? dcp->dart_disp : dcp->dart_dcp;
    dart_unmap(dart, iova, size);
}

bool display_scanout_present_begin(u64 surface_iova, u32 width, u32 height,
                                   u32 stride, bool rgba, u64 *cookie)
{
    dcp_layer_t layer;
    dcp_rect_t source;
    dcp_rect_t target;

    if (!cookie || scanout_swap_active || !display_scanout_ready() ||
        !display_scanout_build_layer(surface_iova, width, height, stride, rgba,
                                     &layer, &source, &target))
        return false;
    if (dcp_iomfb_owner_active(dcp)) {
        int owner_swap_id = dcp_iomfb_owner_present(
            dcp, surface_iova, width, height, stride,
            rgba ? DCP_IOMFB_FORMAT_RGBA : DCP_IOMFB_FORMAT_BGRA);
        if (owner_swap_id <= 0)
            return false;
        scanout_owner_swap_id = (u32)owner_swap_id;
        scanout_owner_surface_iova = surface_iova;
        scanout_swap_active = true;
        scanout_quiescing = false;
        if (++scanout_cookie == 0)
            ++scanout_cookie;
        *cookie = scanout_cookie;
        return true;
    }
    dcp_ib_swap_async_init(&scanout_swap);
    if (dcp_ib_swap_async_begin(&scanout_swap, iboot, &layer, &source, &target))
        return false;
    scanout_swap_active = true;
    scanout_quiescing = false;
    if (++scanout_cookie == 0)
        ++scanout_cookie;
    *cookie = scanout_cookie;
    return true;
}

int display_scanout_present_poll(u64 cookie, u32 *swap_id)
{
    enum dcp_ib_swap_async_result result;

    if (!swap_id || !scanout_swap_active || scanout_quiescing ||
        cookie != scanout_cookie)
        return -1;
    if (dcp_iomfb_owner_active(dcp)) {
        *swap_id = scanout_owner_swap_id;
        scanout_swap_active = false;
        return *swap_id ? 1 : -1;
    }
    result = dcp_ib_swap_async_poll(&scanout_swap);
    if (result == DCP_IB_SWAP_ASYNC_PENDING)
        return 0;
    scanout_swap_active = false;
    if (result != DCP_IB_SWAP_ASYNC_APPLIED || scanout_swap.swap_id <= 0)
        return -1;
    *swap_id = (u32)scanout_swap.swap_id;
    return 1;
}

int display_scanout_latch_poll(u32 expected_swap_id)
{
    if (!dcp_iomfb_owner_active(dcp) || !expected_swap_id ||
        expected_swap_id != scanout_owner_swap_id)
        return -1;
    return dcp_iomfb_owner_poll_latch(dcp, expected_swap_id);
}

bool display_scanout_observe_latched(u32 *swap_id, u64 *surface_iova)
{
    if (!swap_id || !surface_iova || !dcp || !dcp_iomfb_owner_active(dcp))
        return false;
    *swap_id = dcp->iomfb_latched_swap_id;
    *surface_iova = *swap_id == scanout_owner_swap_id ?
                        scanout_owner_surface_iova : 0;
    return *swap_id != 0;
}

bool display_scanout_quiesce_begin(u64 *cookie)
{
    dcp_layer_t layer;
    dcp_rect_t source;
    dcp_rect_t target;

    if (!cookie || scanout_swap_active || !display_scanout_ready() || !fb_dva ||
        !display_scanout_build_layer(fb_dva, cur_boot_args.video.width,
                                     cur_boot_args.video.height,
                                     cur_boot_args.video.stride, false,
                                     &layer, &source, &target))
        return false;
    if (dcp_iomfb_owner_active(dcp)) {
        int owner_swap_id = dcp_iomfb_owner_present(
            dcp, fb_dva, cur_boot_args.video.width, cur_boot_args.video.height,
            cur_boot_args.video.stride, DCP_IOMFB_FORMAT_BGRA);
        if (owner_swap_id <= 0)
            return false;
        scanout_owner_swap_id = (u32)owner_swap_id;
        scanout_owner_surface_iova = fb_dva;
        scanout_swap_active = true;
        scanout_quiescing = true;
        if (++scanout_cookie == 0)
            ++scanout_cookie;
        *cookie = scanout_cookie;
        return true;
    }
    dcp_ib_swap_async_init(&scanout_swap);
    if (dcp_ib_swap_async_begin(&scanout_swap, iboot, &layer, &source, &target))
        return false;
    scanout_swap_active = true;
    scanout_quiescing = true;
    scanout_quiesce_after = 0;
    if (++scanout_cookie == 0)
        ++scanout_cookie;
    *cookie = scanout_cookie;
    return true;
}

int display_scanout_quiesce_poll(u64 cookie)
{
    enum dcp_ib_swap_async_result result;

    if (!scanout_quiescing || cookie != scanout_cookie)
        return -1;
    if (dcp_iomfb_owner_active(dcp)) {
        int latch = dcp_iomfb_owner_poll_latch(dcp, scanout_owner_swap_id);
        if (latch <= 0)
            return latch;
        scanout_swap_active = false;
        scanout_quiescing = false;
        return 1;
    }
    if (!scanout_quiesce_after) {
        result = dcp_ib_swap_async_poll(&scanout_swap);
        if (result == DCP_IB_SWAP_ASYNC_PENDING)
            return 0;
        if (result != DCP_IB_SWAP_ASYNC_APPLIED) {
            scanout_swap_active = false;
            scanout_quiescing = false;
            return -1;
        }
        scanout_quiesce_after = timeout_calculate(150000);
        return 0;
    }
    if ((s64)(get_ticks() - scanout_quiesce_after) < 0)
        return 0;
    scanout_swap_active = false;
    scanout_quiescing = false;
    return 1;
}

int display_prepare_guest_surface(u64 base, u64 size, u32 width, u32 height, u32 stride,
                                  u32 depth)
{
    if (display_start_dcp() < 0)
        return 0;
    if (display_is_external) {
        printf("display: guest surface handoff only supports the internal panel\n");
        return 0;
    }
    if (dcp_iomfb_owner_active(dcp)) {
        const struct display_guest_owner_ops owner_ops = {
            .map = display_guest_map,
            .present_begin = display_guest_owner_present_begin,
            .latch_poll = display_guest_owner_latch_poll,
            .wait = display_guest_owner_wait,
        };
        u64 new_dva = 0;
        enum display_guest_owner_result result;

        if (width != cur_boot_args.video.width ||
            height != cur_boot_args.video.height ||
            stride != cur_boot_args.video.stride) {
            printf("display: IOMFB owner requires exact POST geometry\n");
            return 0;
        }
        result = display_guest_prepare_owner(
            base, size, width, height, stride, depth, &owner_ops, NULL,
            5000, &new_dva);
        if (result != DISPLAY_GUEST_OWNER_LATCHED) {
            if (new_dva)
                printf("display: IOMFB guest surface not proven latched; retaining DVA=%#lx until reset\n",
                       new_dva);
            else
                printf("display: rejected IOMFB guest surface before publication\n");
            return 0;
        }
        if (guest_fb_dva)
            display_guest_unmap(NULL, guest_fb_dva, guest_fb_size);
        guest_fb_dva = new_dva;
        guest_fb_size = size;
        printf("display: IOMFB guest surface latched PA=%#lx DVA=%#lx size=%#lx %ux%u stride=%u\n",
               base, new_dva, size, width, height, stride);
        return 1;
    }
    if (!iboot) {
        printf("display: no admitted DCP frontend for guest present\n");
        return 0;
    }

    const struct display_guest_ops ops = {
        .map = display_guest_map,
        .present = display_guest_present,
        .unmap = display_guest_unmap,
    };
    u64 new_dva = 0;
    if (!display_guest_prepare(base, size, width, height, stride, depth, &ops, NULL,
                               &new_dva)) {
        printf("display: rejected guest surface PA=%#lx size=%#lx %ux%u stride=%u depth=%u\n",
               base, size, width, height, stride, depth);
        return 0;
    }

    if (guest_fb_dva)
        display_guest_unmap(NULL, guest_fb_dva, guest_fb_size);
    guest_fb_dva = new_dva;
    guest_fb_size = size;

    printf("display: guest surface active PA=%#lx DVA=%#lx size=%#lx %ux%u stride=%u\n",
           base, new_dva, size, width, height, stride);
    return 1;
}

int display_configure(const char *config)
{
    dcp_timing_mode_t want;
    struct display_options opts = {0};

#ifdef NO_DISPLAY
    printf("display: skip configuration (NO_DISPLAY)\n");
    return 0;
#endif

    display_parse_mode(config, &want, &opts);

    u64 start_time = get_ticks();

    int ret = display_start_dcp();
    if (ret < 0)
        return ret;
#ifdef DCP_IOMFB_FULL_OWNER
    if (!iboot) {
#ifdef DCP_IOMFB_SWAP_OBSERVER
        if (!display_iomfb_swap_observer_run())
            return -1;
        printf("display: one-swap observer complete; downstream payload disabled\n");
        return 0;
#else
        printf("display: IOMFB bootstrap trace active; legacy iBoot modeset is disabled\n");
        return 0;
#endif
    }
#endif

    // connect dptx if necessary
    if (display_is_dptx) {
        ret = dcp_connect_dptx(dcp);
        if (ret < 0)
            return ret;
    }

    if (!display_is_external) {
        // Sequoia bug workaround: Force power cycle
        if (display_needs_power_cycle) {
            if ((ret = dcp_ib_set_power(iboot, false)) < 0)
                printf("display: failed to set power off (continuing anyway)\n");
            mdelay(100);
        }
        // Sonoma bug workaround: Power on internal panel early
        if ((ret = dcp_ib_set_power(iboot, true)) < 0)
            printf("display: failed to set power on (continuing anyway)\n");
    }

    // Detect if display is connected
    int timing_cnt, color_cnt;
    int hpd = 0, retries = 0;

    /* After boot DCP does not immediately report a connected display. Retry getting display
     * information for 2 seconds.
     */
    while (retries++ < (DISPLAY_STATUS_RETRIES(display_is_dptx))) {
        dcp_work(dcp);
        hpd = dcp_ib_get_hpd(iboot, &timing_cnt, &color_cnt);
        if (hpd < 0)
            ret = hpd;
        else if (hpd && timing_cnt && color_cnt)
            break;
        if (retries < DISPLAY_STATUS_RETRIES(display_is_dptx))
            mdelay(DISPLAY_STATUS_DELAY);
    }
    printf("display: waited %d ms for display status\n", (retries - 1) * DISPLAY_STATUS_DELAY);
    if (ret < 0) {
        printf("display: failed to get display status\n");
        return 0;
    }

    printf("display: connected:%d timing_cnt:%d color_cnt:%d\n", hpd, timing_cnt, color_cnt);

    if (!hpd || !timing_cnt || !color_cnt)
        return 0;

    // Power on
    if ((ret = dcp_ib_set_power(iboot, true)) < 0) {
        printf("display: failed to set power\n");
        return ret;
    }

    // Sonoma bug workaround
    mdelay(100);

    // Find best modes
    dcp_timing_mode_t *tmodes, tbest;
    if ((ret = dcp_ib_get_timing_modes(iboot, &tmodes)) < 0) {
        printf("display: failed to get timing modes\n");
        return -1;
    }
    assert(ret == timing_cnt);
    display_choose_timing_mode(tmodes, timing_cnt, &tbest, &want);

    dcp_color_mode_t *cmodes, cbest;
    if ((ret = dcp_ib_get_color_modes(iboot, &cmodes)) < 0) {
        printf("display: failed to get color modes\n");
        return -1;
    }
    assert(ret == color_cnt);
    display_choose_color_mode(cmodes, color_cnt, &cbest);

    // Set mode
    if ((ret = dcp_ib_set_mode(iboot, &tbest, &cbest)) < 0) {
        printf("display: failed to set mode. trying again...\n");
        mdelay(500);
        if ((ret = dcp_ib_set_mode(iboot, &tbest, &cbest)) < 0) {
            printf("display: failed to set mode twice.\n");
            return ret;
        }
    }

    u64 fb_pa = cur_boot_args.video.base;
    u64 tmp_dva = 0;

    size_t size =
        ALIGN_UP(tbest.width * tbest.height * ((cbest.bpp + 7) / 8) + 24 * SZ_16K, SZ_16K);

    if (fb_size < size) {
        printf("display: current framebuffer is too small for new mode\n");

        /* rtkit uses 0x10000000 as DVA offset, FB starts in the first page */
        if ((s64)size > 7 * SZ_32M) {
            printf("display: not enough reserved L2 DVA space for fb size 0x%zx\n", size);
            return -1;
        }

        fb_pa = top_of_memory_alloc(size);
        memset((void *)fb_pa, 0, size);

        tmp_dva = iova_alloc(dcp->iovad_dcp, size);

        tmp_dva = display_map_fb(tmp_dva, fb_pa, size);
        if (DART_IS_ERR(tmp_dva)) {
            printf("display: failed to map new fb\n");
            return -1;
        }

        // Swap!
        u32 stride = tbest.width * 4;
        ret = display_swap(tmp_dva, stride, tbest.width, tbest.height);
        if (ret < 0)
            return ret;

        /* wait for swap durations + 1ms */
        u32 delay = (((1000 << 16) + tbest.fps - 1) / tbest.fps) + 1;
        mdelay(delay);
        dart_unmap(dcp->dart_disp, fb_dva, fb_size);
        dart_unmap(dcp->dart_dcp, fb_dva, fb_size);

        fb_dva = display_map_fb(fb_dva, fb_pa, size);
        if (DART_IS_ERR(fb_dva)) {
            printf("display: failed to map new fb\n");
            fb_dva = 0;
            return -1;
        }

        fb_size = size;
        mmu_map_framebuffer(fb_pa, fb_size);

        /* update ADT with the physical address of the new framebuffer */
        u64 fb_reg[2] = {fb_pa, size};
        int node = adt_path_offset(adt, "vram");
        if (node >= 0) {
            // TODO: adt_set_reg(adt, node, "vram", fb_pa, size);?
            ret = adt_setprop(adt, node, "reg", &fb_reg, sizeof(fb_reg));
            if (ret < 0)
                printf("display: failed to update '/vram'\n");
        }
        node = adt_path_offset(adt, "/chosen/carveout-memory-map");
        if (node >= 0) {
            // TODO: adt_set_reg(adt, node, "vram", fb_pa, size);?
            ret = adt_setprop(adt, node, "region-id-14", &fb_reg, sizeof(fb_reg));
            if (ret < 0)
                printf("display: failed to update '/chosen/carveout-memory-map/region-id-14'\n");
        }
    }

    // Swap!
    u32 stride = tbest.width * 4;
    ret = display_swap(fb_dva, stride, tbest.width, tbest.height);
    if (ret < 0)
        return ret;

    printf("display: swapped! (swap_id=%d)\n", ret);

    // Wait until the swap completes before powering down DCP
    // 50ms is too low, 100 works, 150 for good measure
    mdelay(150);

    bool reinit = false;
    if (fb_pa != cur_boot_args.video.base || cur_boot_args.video.stride != stride ||
        cur_boot_args.video.width != tbest.width || cur_boot_args.video.height != tbest.height ||
        cur_boot_args.video.depth != 30) {
        cur_boot_args.video.base = fb_pa;
        cur_boot_args.video.stride = stride;
        cur_boot_args.video.width = tbest.width;
        cur_boot_args.video.height = tbest.height;
        cur_boot_args.video.depth = 30 | (opts.retina ? FB_DEPTH_FLAG_RETINA : 0);
        reinit = true;
    }

    if (!display_is_external && !(cur_boot_args.video.depth & FB_DEPTH_FLAG_RETINA)) {
        cur_boot_args.video.depth |= FB_DEPTH_FLAG_RETINA;
        reinit = true;
    }

    if (reinit)
        fb_reinit();

    /* Update for python / subsequent stages */
    memcpy((void *)boot_args_addr, &cur_boot_args, sizeof(cur_boot_args));

    if (tmp_dva) {
        // unmap / free temporary dva
        dart_unmap(dcp->dart_disp, tmp_dva, size);
        dart_unmap(dcp->dart_dcp, tmp_dva, size);
        iova_free(dcp->iovad_dcp, tmp_dva, size);
    }

    u64 msecs = ticks_to_msecs(get_ticks() - start_time);
    printf("display: Modeset took %ld ms\n", msecs);

    return 1;
}

int display_init(void)
{
    const char *disp_path;
    if (adt_is_compatible(adt, 0, "J180dAP") || adt_is_compatible(adt, 0, "J475dAP"))
        disp_path = "/arm-io/dispext4";
    else
        disp_path = "/arm-io/disp0";

    bool has_notch = false;
    UNUSED(has_notch);

    int product = adt_path_offset(adt, "/product");
    if (product < 0) {
        printf("/product node not found!\n");
    } else {
        u32 val = 0;
        ADT_GETPROP(adt, product, "partially-occluded-display", &val);
        has_notch = !!val;
    }

    int node = adt_path_offset(adt, disp_path);
    if (node < 0) {
        printf("%s node not found!\n", disp_path);
        return -1;
    }

    display_is_external = adt_getprop(adt, node, "external", NULL);
    if (display_is_external)
        printf("display: Display is external\n");
    else
        printf("display: Display is internal\n");

    if ((cur_boot_args.video.width == 640 && cur_boot_args.video.height == 1136) &&
        chip_id != S5L8960X) {
        printf("display: Dummy framebuffer found, initializing display\n");
        return display_configure(NULL);
    } else if (display_is_external && is_mac) {
        printf("display: External display found, reconfiguring\n");
        return display_configure(NULL);
    } else if ((!(cur_boot_args.video.depth & FB_DEPTH_FLAG_RETINA)) && is_mac) {
        printf("display: Internal display with non-retina flag, assuming Sonoma bug and "
               "reconfiguring\n");
        fb_clear_direct(); // Old m1n1 stage1 ends up with an ugly logo situation, clear it.
        return display_configure(NULL);
#ifndef CHAINLOADING
    } else if (!has_notch && firmware_sfw_in_range(V15_0B1, FW_MAX) &&
               os_firmware.version < V15_0B1) {
        printf("display: Internal display on t8103 or t8112 with Sequoia SFW, power cycling\n");
        display_needs_power_cycle = true;
        return display_configure(NULL);
#endif
    } else {
        printf("display: Display is already initialized (%ldx%ld)\n", cur_boot_args.video.width,
               cur_boot_args.video.height);
        return 0;
    }
}

void display_shutdown(dcp_shutdown_mode mode)
{
    /* We have no DCP, so just exit */
    if (!has_dcp)
        return;

    if (iboot) {
        int ret;

        if (dcp_ib_shutdown(iboot) < 0) {
            printf("display: iBoot endpoint shutdown failed; retaining owner until reset\n");
            return;
        }
        iboot = NULL;
        switch (mode) {
            case DCP_QUIESCED:
                printf("display: Quiescing DCP (unconditional)\n");
                ret = dcp_shutdown(dcp, false);
                break;
            case DCP_SLEEP_IF_EXTERNAL:
                if (!display_is_external)
                    printf("display: Quiescing DCP (internal)\n");
                else
                    printf("display: Sleeping DCP (external)\n");
                ret = dcp_shutdown(dcp, display_is_external);
                break;
            case DCP_SLEEP:
                printf("display: Sleeping DCP (unconditional)\n");
                ret = dcp_shutdown(dcp, true);
                break;
            default:
                ret = -1;
                break;
        }
        if (ret == 0)
            dcp = NULL;
        else
            printf("display: DCP shutdown failed; retaining owner until reset\n");
    } else if (dcp) {
        printf("display: Quiescing DCP IOMFB owner\n");
        if (dcp_shutdown(dcp, false) == 0)
            dcp = NULL;
        else
            printf("display: DCP shutdown failed; retaining owner until reset\n");
    }
}

bool display_shutdown_complete(void)
{
    return !dcp && !iboot;
}

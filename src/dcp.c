/* SPDX-License-Identifier: MIT */

#include "../config.h"
#include "../build/build_cfg.h"

#include "adt.h"
#include "afk.h"
#include "dcp.h"
#include "dcp_iomfb_clock.h"
#include "firmware.h"
#include "malloc.h"
#include "pmgr.h"
#include "rtkit.h"
#include "smc.h"
#include "string.h"
#include "utils.h"

#include "dcp/dptx_phy.h"

struct adt_function_smc_gpio {
    u32 phandle;
    char four_cc[4];
    u32 gpio;
    u32 unk;
};

static char dcp_pmgr_dev[16] = "DISP0_CPU0";
static u32 dcp_die;

static bool dcp_iomfb_send(void *opaque, dcp_iomfb_u8 endpoint,
                           dcp_iomfb_u64 message)
{
    dcp_dev_t *dcp = opaque;
    const struct rtkit_message msg = {.ep = endpoint, .msg = message};

    return dcp && rtkit_send(dcp->rtkit, &msg);
}

static int dcp_iomfb_owner_pump(void *opaque)
{
    dcp_dev_t *dcp = opaque;
    return dcp ? afk_epic_work(dcp->afk, -1) : -1;
}

static bool dcp_iomfb_owner_call(void *opaque, const char tag[4],
                                 const void *input, u32 input_size,
                                 void *output, u32 output_size)
{
    dcp_dev_t *dcp = opaque;
    return dcp && dcp_iomfb_rpc_call(&dcp->iomfb_rpc, tag, input,
                                     input_size, output, output_size);
}

static bool dcp_iomfb_resource_allocate(void *opaque, u64 size, u64 alignment,
                                        u64 *pa, void **cpu)
{
    (void)opaque;
    *cpu = memalign(alignment, size);
    if (!*cpu)
        return false;
    *pa = (u64)(uintptr_t)*cpu;
    return true;
}

static void dcp_iomfb_resource_release(void *opaque, void *cpu, u64 size)
{
    (void)opaque;
    (void)size;
    free(cpu);
}

static bool dcp_iomfb_resource_map_dcp(void *opaque, u64 pa, u64 size,
                                       u64 *dva)
{
    dcp_dev_t *dcp = opaque;
    u64 allocated;
    if (!dcp || !dcp->iovad_dcp || !dcp->dart_dcp)
        return false;
    allocated = iova_alloc_aligned(dcp->iovad_dcp, size, SZ_16K);
    if (!allocated)
        return false;
    if (dart_map(dcp->dart_dcp, allocated, (void *)(uintptr_t)pa, size)) {
        iova_free(dcp->iovad_dcp, allocated, size);
        return false;
    }
    *dva = allocated;
    return true;
}

static void dcp_iomfb_resource_unmap_dcp(void *opaque, u64 dva, u64 size)
{
    dcp_dev_t *dcp = opaque;
    if (!dcp || !dcp->iovad_dcp || !dcp->dart_dcp)
        return;
    dart_unmap(dcp->dart_dcp, dva, size);
    iova_free(dcp->iovad_dcp, dva, size);
}

static bool dcp_iomfb_resource_map_piodma(void *opaque, u64 pa, u64 dva,
                                          u64 size)
{
    dcp_dev_t *dcp = opaque;
    return dcp && dcp->dart_piodma &&
           dart_map(dcp->dart_piodma, dva, (void *)(uintptr_t)pa, size) == 0;
}

static void dcp_iomfb_resource_unmap_piodma(void *opaque, u64 dva, u64 size)
{
    dcp_dev_t *dcp = opaque;
    if (dcp && dcp->dart_piodma)
        dart_unmap(dcp->dart_piodma, dva, size);
}

static bool dcp_iomfb_resource_get_reg(void *opaque, unsigned int index,
                                       u64 *pa, u64 *size)
{
    int path[8];
    (void)opaque;
    if (adt_path_offset_trace(adt, "/arm-io/disp0", path) < 0)
        return false;
    return adt_get_reg(adt, path, "reg", index, pa, size) == 0;
}

static u64 dcp_iomfb_resource_get_clock(void *opaque)
{
    (void)opaque;
    return 533333328;
}

static const struct dcp_iomfb_resource_ops dcp_iomfb_resource_ops = {
    .allocate = dcp_iomfb_resource_allocate,
    .release = dcp_iomfb_resource_release,
    .map_dcp = dcp_iomfb_resource_map_dcp,
    .unmap_dcp = dcp_iomfb_resource_unmap_dcp,
    .map_piodma = dcp_iomfb_resource_map_piodma,
    .unmap_piodma = dcp_iomfb_resource_unmap_piodma,
    .get_reg = dcp_iomfb_resource_get_reg,
    .get_clock = dcp_iomfb_resource_get_clock,
};

static void *dcp_iomfb_property_allocate(void *opaque, size_t size)
{
    (void)opaque;
    return calloc(1, size);
}

static void dcp_iomfb_property_release(void *opaque, void *pointer)
{
    (void)opaque;
    free(pointer);
}

static const struct dcp_iomfb_property_ops dcp_iomfb_property_ops = {
    .allocate = dcp_iomfb_property_allocate,
    .release = dcp_iomfb_property_release,
};

static int dcp_iomfb_owner_platform(void *opaque, unsigned int callback_id,
                                    const void *input, u32 input_size,
                                    void *output, u32 output_size)
{
    dcp_dev_t *dcp = opaque;
    u32 completed = 0;
    enum dcp_iomfb_latch_result latch;

    if (!dcp)
        return -1;
    if (callback_id >= 126 && callback_id <= 128)
        return dcp_iomfb_properties_callback(&dcp->iomfb_properties,
                                              callback_id, input, input_size,
                                              output, output_size);
    if (callback_id == 563)
        return dcp_iomfb_properties_scalar_callback(
            &dcp->iomfb_properties, callback_id, input, input_size, output,
            output_size);
    if (callback_id == 209) {
        int chosen = adt_path_offset(adt, "/chosen");
        struct dcp_iomfb_clock_anchor anchor = {0};
        u64 utc_ms;
        if (chosen < 0 ||
            ADT_GETPROP(adt, chosen, "m1n1-iomfb-utc-ms",
                        &anchor.utc_ms) < 0 ||
            ADT_GETPROP(adt, chosen, "m1n1-iomfb-utc-cntpct",
                        &anchor.counter) < 0 ||
            ADT_GETPROP(adt, chosen, "m1n1-iomfb-utc-cntfrq",
                        &anchor.frequency) < 0 ||
            input_size != 0 || output_size != sizeof(utc_ms) ||
            !dcp_iomfb_clock_now(&anchor, mrs(CNTPCT_EL0),
                                 mrs(CNTFRQ_EL0), &utc_ms)) {
            printf("dcp-iomfb: D209 has no authoritative UTC counter anchor\n");
            return -1;
        }
        memcpy(output, &utc_ms, sizeof(utc_ms));
        return 0;
    }
    if (callback_id != DCP_IOMFB_CALLBACK_SWAP_COMPLETE)
        return dcp_iomfb_resources_callback(&dcp->iomfb_resources,
                                             callback_id, input, input_size,
                                             output, output_size);
    latch = dcp_iomfb_parse_swap_complete_payload(
        DCP_IOMFB_PROTOCOL_V13_5, input, input_size,
        dcp->iomfb_expected_swap_id, &completed);
    if (latch == DCP_IOMFB_LATCH_INVALID)
        return -1;
    if (latch == DCP_IOMFB_LATCH_MATCHED) {
        dcp->iomfb_latched_swap_id = completed;
        dcp->iomfb_expected_swap_id = 0;
        printf("dcp-iomfb: exact D589 latch swap_id=%u\n", completed);
    } else {
        printf("dcp-iomfb: stale D589 swap_id=%u expected=%u\n", completed,
               dcp->iomfb_expected_swap_id);
    }
    return 0;
}

static int dcp_iomfb_owner_callback(void *opaque, const char tag[4],
                                    const void *input, u32 input_size,
                                    void *output, u32 output_size)
{
    dcp_dev_t *dcp = opaque;
    int result = dcp ? dcp_iomfb_bootstrap_callback(&dcp->iomfb_bootstrap, tag,
                                                     input, input_size, output,
                                                     output_size) : -1;
    if (result < 0)
        printf("dcp-iomfb: callback %.4s (%u/%u) failed closed\n", tag,
               input_size, output_size);
    return result;
}

enum dcp_iomfb_timing_status {
    DCP_IOMFB_TIMING_ERROR = -2,
    DCP_IOMFB_TIMING_MISMATCH = -1,
    DCP_IOMFB_TIMING_ABSENT = 0,
    DCP_IOMFB_TIMING_MATCH = 1,
};

static enum dcp_iomfb_timing_status
dcp_iomfb_owner_confirm_timing(dcp_dev_t *dcp, u32 expected_timing_mode_id)
{
    u64 observed = 0;
    enum dcp_iomfb_scalar_match property_status = DCP_IOMFB_SCALAR_ABSENT;

    if (!dcp || !dcp->iomfb_owner_registered)
        return DCP_IOMFB_TIMING_ERROR;
    for (unsigned int attempt = 0; attempt < 5000; attempt++) {
        property_status = dcp_iomfb_properties_match_u64(
            &dcp->iomfb_properties, "DPTimingModeId",
            expected_timing_mode_id, &observed);
        if (property_status == DCP_IOMFB_SCALAR_MATCH) {
            printf("dcp-iomfb: applied timing confirmed id=%u\n",
                   expected_timing_mode_id);
            return DCP_IOMFB_TIMING_MATCH;
        }
        if (afk_epic_work(dcp->afk, -1) < 0)
            return DCP_IOMFB_TIMING_ERROR;
        udelay(100);
    }
    /* The final pump may have delivered D563. Re-sample after it so an
     * explicit mismatch can never be mistaken for an absent property. */
    property_status = dcp_iomfb_properties_match_u64(
        &dcp->iomfb_properties, "DPTimingModeId", expected_timing_mode_id,
        &observed);
    if (property_status == DCP_IOMFB_SCALAR_MATCH) {
        printf("dcp-iomfb: applied timing confirmed id=%u\n",
               expected_timing_mode_id);
        return DCP_IOMFB_TIMING_MATCH;
    }
    if (property_status == DCP_IOMFB_SCALAR_MISMATCH) {
        printf("dcp-iomfb: applied timing mismatch expected=%u observed=%lu\n",
               expected_timing_mode_id, observed);
        return DCP_IOMFB_TIMING_MISMATCH;
    } else {
        printf("dcp-iomfb: applied timing property timed out expected=%u\n",
               expected_timing_mode_id);
        return DCP_IOMFB_TIMING_ABSENT;
    }
}

static int dcp_iomfb_owner_receive(void *opaque, afk_raw_u8 endpoint,
                                   afk_raw_u64 message)
{
    dcp_dev_t *dcp = opaque;
    enum dcp_iomfb_rpc_rx_result result;
    unsigned int type = (unsigned int)(message & 0xf);

    if (!dcp || endpoint != DCP_IOMFB_RPC_ENDPOINT)
        return -1;
    if (type == DCP_IOMFB_MESSAGE_TYPE_INITIALIZED) {
        if (dcp->iomfb_initialized)
            return -1;
        dcp->iomfb_initialized = true;
        printf("dcp-iomfb: endpoint initialized\n");
        return 0;
    }
    if (type != DCP_IOMFB_MESSAGE_TYPE_MSG || !dcp->iomfb_initialized)
        return -1;
    result = dcp_iomfb_rpc_receive(&dcp->iomfb_rpc, message);
    if (result == DCP_IOMFB_RPC_RX_INVALID) {
        printf("dcp-iomfb: invalid RPC; owner failed closed\n");
        return -1;
    }
    return 0;
}

#define DCP_IOMFB_START_OBSERVE_MAX_POLLS 100000u
#define DCP_IOMFB_START_OBSERVE_USEC 500000u

#ifdef DCP_IOMFB_SET_SHMEM_OBSERVER
static bool dcp_iomfb_observe_set_shmem_fail_closed(dcp_dev_t *dcp)
{
    u64 deadline;

    if (!rtkit_alloc_buffer_aligned(dcp->rtkit, &dcp->iomfb_shmem,
                                    DCP_IOMFB_RPC_SHMEM_SIZE, 0x10000)) {
        printf("dcp-iomfb: SET_SHMEM observer allocation failed\n");
        return false;
    }
    memset(dcp->iomfb_shmem.bfr, 0, dcp->iomfb_shmem.sz);
    if (!dcp_iomfb_send(dcp, DCP_IOMFB_RPC_ENDPOINT,
                        dcp_iomfb_set_shmem_message(dcp->iomfb_shmem.dva))) {
        printf("dcp-iomfb: SET_SHMEM observer send failed\n");
        return false;
    }
    printf("dcp-iomfb: SET_SHMEM sent dva=0x%lx size=0x%lx\n",
           dcp->iomfb_shmem.dva, dcp->iomfb_shmem.sz);

    deadline = timeout_calculate(DCP_IOMFB_START_OBSERVE_USEC);
    for (unsigned int attempt = 0;
         attempt < DCP_IOMFB_START_OBSERVE_MAX_POLLS &&
         !timeout_expired(deadline); attempt++) {
        struct rtkit_message msg = {.ep = 0xff, .msg = 0};
        int ret = rtkit_recv_one_quiet(dcp->rtkit, &msg);

        if (ret < 0) {
            printf("dcp-iomfb: SET_SHMEM observation saw RTKit failure\n");
            return false;
        }
        if (ret > 0) {
            if (msg.ep == DCP_IOMFB_RPC_ENDPOINT &&
                (msg.msg & 0xf) == DCP_IOMFB_MESSAGE_TYPE_INITIALIZED)
                printf("dcp-iomfb: SET_SHMEM admitted; endpoint initialized\n");
            else
                printf("dcp-iomfb: SET_SHMEM observation saw ep=0x%02x msg=0x%lx\n",
                       msg.ep, msg.msg);
            return false;
        }
        if (msg.ep != 0xff) {
            printf("dcp-iomfb: SET_SHMEM observation saw system ep=0x%02x msg=0x%lx\n",
                   msg.ep, msg.msg);
            return false;
        }
        udelay(1);
    }
    printf("dcp-iomfb: SET_SHMEM observation expired without endpoint response\n");
    return false;
}
#endif

#if defined(DCP_IOMFB_START_OBSERVER) || \
    defined(DCP_IOMFB_EARLY_PIODMA_OBSERVER)
static bool dcp_iomfb_observe_start_fail_closed(dcp_dev_t *dcp)
{
    u64 deadline = timeout_calculate(DCP_IOMFB_START_OBSERVE_USEC);

    for (unsigned int attempt = 0;
         attempt < DCP_IOMFB_START_OBSERVE_MAX_POLLS &&
         !timeout_expired(deadline); attempt++) {
        struct rtkit_message msg = {.ep = 0xff, .msg = 0};
        int ret = rtkit_recv_one_quiet(dcp->rtkit, &msg);

        if (ret < 0) {
            printf("dcp-iomfb: START-only observation saw RTKit failure\n");
            return false;
        }
        if (ret > 0) {
            if (msg.ep == DCP_IOMFB_RPC_ENDPOINT)
                printf("dcp-iomfb: START-only observation accepted ep=0x%02x msg=0x%lx\n",
                       msg.ep, msg.msg);
            else
                printf("dcp-iomfb: START-only observation saw unrelated ep=0x%02x msg=0x%lx\n",
                       msg.ep, msg.msg);
            return false;
        }
        if (msg.ep != 0xff) {
            printf("dcp-iomfb: START-only observation saw system ep=0x%02x msg=0x%lx\n",
                   msg.ep, msg.msg);
            return false;
        }
        udelay(1);
    }
    printf("dcp-iomfb: START-only observation expired without endpoint response\n");
    return false;
}
#endif

bool dcp_iomfb_owner_start(dcp_dev_t *dcp)
{
    struct dcp_iomfb_mode_choice mode;
    const void *color_blob;
    const void *timing_blob;
    size_t color_size;
    size_t timing_size;
    int dart_path[8];
    enum dcp_iomfb_timing_status timing_status;

    if (!dcp || dcp->iomfb_owner_endpoint_started ||
        dcp->iomfb_owner_registered || dcp->iomfb_observer_registered)
        return false;
    if (!dcp_iomfb_owner_supported()) {
        printf("dcp-iomfb: full owner is restricted to J313AP firmware 13.5\n");
        return false;
    }
    if (adt_path_offset_trace(adt, "/arm-io/dart-disp0", dart_path) < 0)
        return false;
    if (!rtkit_start_ep(dcp->rtkit, DCP_IOMFB_RPC_ENDPOINT))
        return false;
    dcp->iomfb_owner_endpoint_started = true;
#if defined(DCP_IOMFB_START_OBSERVER) || \
    defined(DCP_IOMFB_EARLY_PIODMA_OBSERVER)
    /* Receipt-only discriminator: after START, do not perform any further
     * PIODMA, shared-memory, AFK or RPC action. Every result fails closed. */
    return dcp_iomfb_observe_start_fail_closed(dcp);
#endif

#ifdef DCP_IOMFB_SET_SHMEM_OBSERVER
    return dcp_iomfb_observe_set_shmem_fail_closed(dcp);
#endif

    if (!rtkit_alloc_buffer_aligned(dcp->rtkit, &dcp->iomfb_shmem,
                                    DCP_IOMFB_RPC_SHMEM_SIZE, 0x10000))
        goto fail_endpoint;
    memset(dcp->iomfb_shmem.bfr, 0, dcp->iomfb_shmem.sz);
    dcp_iomfb_resources_init(&dcp->iomfb_resources, &dcp_iomfb_resource_ops,
                             dcp);
    dcp_iomfb_properties_init(&dcp->iomfb_properties,
                              &dcp_iomfb_property_ops, dcp);
    dcp_iomfb_rpc_init(&dcp->iomfb_rpc, dcp->iomfb_shmem.bfr,
                       dcp->iomfb_shmem.sz, dcp_iomfb_send,
                       dcp_iomfb_owner_pump, dcp_iomfb_owner_callback, dcp);
    dcp_iomfb_bootstrap_init(&dcp->iomfb_bootstrap, dcp_iomfb_owner_call,
                             dcp_iomfb_owner_platform, dcp);
    if (!afk_epic_register_raw_handler(dcp->afk, DCP_IOMFB_RPC_ENDPOINT,
                                       dcp_iomfb_owner_receive, dcp))
        goto fail_endpoint;
    dcp->iomfb_owner_registered = true;
    if (!dcp_iomfb_send(dcp, DCP_IOMFB_RPC_ENDPOINT,
                        dcp_iomfb_set_shmem_message(dcp->iomfb_shmem.dva)))
        goto fail_endpoint;

    for (unsigned int attempt = 0; attempt < 5000; attempt++) {
        if (dcp->iomfb_initialized)
            break;
        if (afk_epic_work(dcp->afk, -1) < 0)
            goto fail_endpoint;
        udelay(100);
    }
    if (!dcp->iomfb_initialized) {
        printf("dcp-iomfb: initialization timed out\n");
        goto fail_endpoint;
    }
#ifdef DCP_IOMFB_A401_OBSERVER
    {
        u32 result = 0;

        if (!dcp_iomfb_owner_call(dcp, "A401", NULL, 0, &result,
                                  sizeof(result))) {
            printf("dcp-iomfb: A401-only observation failed\n");
            goto fail_endpoint;
        }
        printf("dcp-iomfb: A401 admitted result=%u; downstream calls disabled\n",
               result);
        return false;
    }
#endif
#ifdef DCP_IOMFB_A426_OBSERVER
    if (!dcp_iomfb_bootstrap_start_through_color_remap(
            &dcp->iomfb_bootstrap)) {
        printf("dcp-iomfb: A426-only observation failed\n");
        goto fail_endpoint;
    }
    printf("dcp-iomfb: A426 admitted; downstream calls disabled\n");
    return false;
#endif
#ifdef DCP_IOMFB_A449_OBSERVER
    if (!dcp_iomfb_bootstrap_start_through_video_power_savings(
            &dcp->iomfb_bootstrap)) {
        printf("dcp-iomfb: A449-only observation failed\n");
        goto fail_endpoint;
    }
    printf("dcp-iomfb: A449 admitted; downstream calls disabled\n");
    return false;
#endif
#ifdef DCP_IOMFB_A456_OBSERVER
    if (!dcp_iomfb_bootstrap_start_through_first_client_open(
            &dcp->iomfb_bootstrap)) {
        printf("dcp-iomfb: A456-only observation failed\n");
        goto fail_endpoint;
    }
    printf("dcp-iomfb: A456 admitted; downstream calls disabled\n");
    return false;
#endif
#ifdef DCP_IOMFB_A411_OBSERVER
    if (!dcp_iomfb_bootstrap_start(&dcp->iomfb_bootstrap)) {
        printf("dcp-iomfb: A411-only observation failed\n");
        goto fail_endpoint;
    }
    printf("dcp-iomfb: A411 admitted main_display=%u; downstream calls disabled\n",
           dcp_iomfb_bootstrap_main_display(&dcp->iomfb_bootstrap));
    return false;
#endif
#ifdef DCP_IOMFB_A472_OBSERVER
    if (!dcp_iomfb_bootstrap_start(&dcp->iomfb_bootstrap) ||
        !dcp_iomfb_bootstrap_power_on_firmware(&dcp->iomfb_bootstrap)) {
        printf("dcp-iomfb: A472-only observation failed\n");
        goto fail_endpoint;
    }
    printf("dcp-iomfb: A472 admitted; downstream calls disabled\n");
    return false;
#endif
#ifdef DCP_IOMFB_A410_OBSERVER
    if (!dcp_iomfb_bootstrap_start(&dcp->iomfb_bootstrap) ||
        !dcp_iomfb_bootstrap_power_on_firmware(&dcp->iomfb_bootstrap) ||
        !dcp_iomfb_bootstrap_select_display(&dcp->iomfb_bootstrap)) {
        printf("dcp-iomfb: A410-only observation failed\n");
        goto fail_endpoint;
    }
    printf("dcp-iomfb: A410 admitted; downstream calls disabled\n");
    return false;
#endif
    if (!dcp_iomfb_bootstrap_start(&dcp->iomfb_bootstrap)) {
        printf("dcp-iomfb: A401 bootstrap failed closed\n");
        goto fail_endpoint;
    }
    if (!dcp_iomfb_properties_find(&dcp->iomfb_properties, "ColorElements",
                                    &color_blob, &color_size) ||
        !dcp_iomfb_properties_find(&dcp->iomfb_properties, "TimingElements",
                                    &timing_blob, &timing_size) ||
        !dcp_iomfb_select_modes(color_blob, color_size, timing_blob,
                                timing_size, &mode)) {
        printf("dcp-iomfb: no validated fixed-panel mode; owner failed closed\n");
        goto fail_endpoint;
    }
    if (!dcp_iomfb_bootstrap_power_on(&dcp->iomfb_bootstrap) ||
        !dcp_iomfb_bootstrap_prepare_modeset(&dcp->iomfb_bootstrap) ||
        !dcp_iomfb_bootstrap_modeset(&dcp->iomfb_bootstrap,
                                     mode.color_mode_id,
                                     mode.timing_mode_id)) {
        printf("dcp-iomfb: panel power/modeset failed closed\n");
        goto fail_endpoint;
    }
    timing_status = dcp_iomfb_owner_confirm_timing(dcp,
                                                    mode.timing_mode_id);
    if (timing_status == DCP_IOMFB_TIMING_ERROR) {
        printf("dcp-iomfb: timing confirmation transport failed closed\n");
        goto fail_endpoint;
    }
    if (timing_status != DCP_IOMFB_TIMING_MATCH) {
        printf("dcp-iomfb: reissuing A412 for unapplied timing id=%u\n",
               mode.timing_mode_id);
        if (!dcp_iomfb_bootstrap_remodeset(&dcp->iomfb_bootstrap,
                                           mode.color_mode_id,
                                           mode.timing_mode_id)) {
            printf("dcp-iomfb: bounded A412 reissue failed closed\n");
            goto fail_endpoint;
        }
        timing_status = dcp_iomfb_owner_confirm_timing(
            dcp, mode.timing_mode_id);
        if (timing_status != DCP_IOMFB_TIMING_MATCH) {
            if (timing_status == DCP_IOMFB_TIMING_ABSENT) {
                printf("dcp-iomfb: D563 absent after accepted A412 pair; "
                       "deferring applied proof to exact D589 on first present\n");
            } else
            {
                printf("dcp-iomfb: bounded A412 reissue failed closed status=%d\n",
                       timing_status);
                goto fail_endpoint;
            }
        }
    }
#ifdef DCP_IOMFB_A412_OBSERVER
    printf("dcp-iomfb: A412 applied; downstream calls disabled color=%u timing=%u\n",
           mode.color_mode_id, mode.timing_mode_id);
    return false;
#endif
    printf("dcp-iomfb: single owner MODESET main_display=%u color=%u timing=%u\n",
           dcp_iomfb_bootstrap_main_display(&dcp->iomfb_bootstrap),
           mode.color_mode_id, mode.timing_mode_id);
    return true;

fail_endpoint:
    /* A live failed endpoint cannot be safely restarted in-place. Keep the
     * allocation owned until the enclosing DCP/SoC reset. */
    return false;
}

bool dcp_iomfb_owner_supported(void)
{
    int path[8];
    int node;

    if (!adt_is_compatible(adt, 0, "J313AP") ||
        os_firmware.version != V13_5)
        return false;
    node = adt_path_offset_trace(adt, "/arm-io/disp0", path);
    return node >= 0 && !adt_getprop(adt, node, "external", NULL);
}

bool dcp_iomfb_owner_active(const dcp_dev_t *dcp)
{
    return dcp && dcp->iomfb_owner_registered &&
           dcp_iomfb_bootstrap_state(&dcp->iomfb_bootstrap) ==
               DCP_IOMFB_BOOT_MODESET;
}

void dcp_iomfb_owner_arm(dcp_dev_t *dcp, u32 swap_id)
{
    if (dcp && dcp->iomfb_owner_registered && swap_id)
        dcp->iomfb_expected_swap_id = swap_id;
}

int dcp_iomfb_owner_poll_latch(dcp_dev_t *dcp, u32 expected_swap_id)
{
    if (!dcp || !dcp->iomfb_owner_registered || !expected_swap_id)
        return -1;
    if (dcp->iomfb_latched_swap_id == expected_swap_id)
        return 1;
    if (dcp_iomfb_bootstrap_state(&dcp->iomfb_bootstrap) !=
        DCP_IOMFB_BOOT_MODESET)
        return -1;
    if (afk_epic_work(dcp->afk, -1) < 0)
        return -1;
    return dcp->iomfb_latched_swap_id == expected_swap_id;
}

int dcp_iomfb_owner_present(dcp_dev_t *dcp, u64 surface_iova, u32 width,
                            u32 height, u32 stride, u32 dcp_format)
{
    u8 start_input[DCP_IOMFB_V13_5_SWAP_START_SIZE];
    u8 start_output[DCP_IOMFB_V13_5_SWAP_START_SIZE] = {0};
    u8 submit_output[DCP_IOMFB_V13_5_SWAP_SUBMIT_OUTPUT_SIZE] = {0};
    u32 swap_id = 0;

    if (!dcp_iomfb_owner_active(dcp) || dcp->iomfb_expected_swap_id ||
        !dcp_iomfb_present_build_start_v13_5(start_input,
                                              sizeof(start_input)) ||
        !dcp_iomfb_present_build_v13_5(&dcp->iomfb_present_request,
                                        surface_iova, width, height, stride,
                                        dcp_format,
                                        !dcp->iomfb_surfaces_cleared))
        return -1;
    if (!dcp_iomfb_owner_call(dcp, "A407", start_input, sizeof(start_input),
                               start_output, sizeof(start_output)) ||
        !dcp_iomfb_present_parse_start_v13_5(start_output,
                                              sizeof(start_output), &swap_id) ||
        !dcp_iomfb_present_set_swap_id_v13_5(&dcp->iomfb_present_request,
                                              swap_id))
        return -1;

    /* D589 may arrive while the synchronous A408 ACK is being pumped. */
    dcp_iomfb_owner_arm(dcp, swap_id);
    if (!dcp_iomfb_owner_call(dcp, "A408", &dcp->iomfb_present_request,
                               sizeof(dcp->iomfb_present_request),
                               submit_output, sizeof(submit_output)) ||
        !dcp_iomfb_present_parse_submit_v13_5(submit_output,
                                               sizeof(submit_output))) {
        dcp->iomfb_expected_swap_id = 0;
        return -1;
    }
    printf("dcp-iomfb: A408 APPLIED swap_id=%u; awaiting exact D589 latch\n",
           swap_id);
    dcp->iomfb_surfaces_cleared = true;
    return (int)swap_id;
}

static int dcp_iomfb_receive(void *opaque, afk_raw_u8 endpoint,
                             afk_raw_u64 message)
{
    dcp_dev_t *dcp = opaque;

    if (!dcp || endpoint != DCP_IOMFB_ENDPOINT)
        return 0;
    dcp->iomfb_last_rx = dcp_iomfb_transport_receive(&dcp->iomfb_transport,
                                                      message);
    if (dcp->iomfb_last_rx == DCP_IOMFB_RX_INITIALIZED)
        printf("dcp-iomfb: observer endpoint initialized; A401 remains disabled\n");
    else if (dcp->iomfb_last_rx == DCP_IOMFB_RX_LATCHED)
        printf("dcp-iomfb: exact D589 latch swap_id=%u\n",
               dcp_iomfb_transport_latched_swap(&dcp->iomfb_transport));
    else if (dcp->iomfb_last_rx == DCP_IOMFB_RX_STALE)
        printf("dcp-iomfb: stale D589 callback acknowledged\n");
    else if (dcp->iomfb_last_rx == DCP_IOMFB_RX_INVALID)
        printf("dcp-iomfb: invalid callback message ignored\n");
    return 0;
}

bool dcp_iomfb_observer_start(dcp_dev_t *dcp)
{
    if (!dcp)
        return false;
    if (dcp->iomfb_owner_registered)
        return false;
    if (dcp->iomfb_observer_registered)
        return dcp_iomfb_transport_state(&dcp->iomfb_transport) == DCP_IOMFB_READY;

    if (!rtkit_alloc_buffer_aligned(dcp->rtkit, &dcp->iomfb_shmem,
                                    DCP_IOMFB_SHMEM_SIZE, 0x10000)) {
        printf("dcp-iomfb: failed to allocate aligned shared memory\n");
        return false;
    }
    memset(dcp->iomfb_shmem.bfr, 0, dcp->iomfb_shmem.sz);
    dcp_iomfb_transport_init(&dcp->iomfb_transport,
                             DCP_IOMFB_PROTOCOL_V13_5,
                             dcp->iomfb_shmem.bfr, dcp->iomfb_shmem.sz,
                             dcp->iomfb_shmem.dva, dcp_iomfb_send, dcp);
    if (!afk_epic_register_raw_handler(dcp->afk, DCP_IOMFB_ENDPOINT,
                                       dcp_iomfb_receive, dcp))
        goto fail_buffer;
    dcp->iomfb_observer_registered = true;

    if (!rtkit_start_ep(dcp->rtkit, DCP_IOMFB_ENDPOINT) ||
        !dcp_iomfb_transport_send_shmem(&dcp->iomfb_transport)) {
        printf("dcp-iomfb: failed to start observer endpoint\n");
        return false;
    }

    for (unsigned int attempt = 0; attempt < 5000; ++attempt) {
        if (dcp_iomfb_transport_state(&dcp->iomfb_transport) == DCP_IOMFB_READY)
            return true;
        if (afk_epic_work(dcp->afk, -1) < 0)
            break;
        udelay(100);
    }
    printf("dcp-iomfb: observer initialization timed out\n");
    return false;

fail_buffer:
    rtkit_free_buffer(dcp->rtkit, &dcp->iomfb_shmem);
    memset(&dcp->iomfb_shmem, 0, sizeof(dcp->iomfb_shmem));
    return false;
}

void dcp_iomfb_observer_arm(dcp_dev_t *dcp, u32 swap_id)
{
    if (dcp && dcp->iomfb_observer_registered)
        dcp_iomfb_transport_arm_swap(&dcp->iomfb_transport, swap_id);
}

int dcp_iomfb_observer_poll_latch(dcp_dev_t *dcp, u32 expected_swap_id)
{
    if (!dcp || !dcp->iomfb_observer_registered || !expected_swap_id)
        return -1;
    if (dcp_iomfb_transport_latched_swap(&dcp->iomfb_transport) == expected_swap_id)
        return 1;
    if (dcp_iomfb_transport_state(&dcp->iomfb_transport) != DCP_IOMFB_READY)
        return -1;
    if (afk_epic_work(dcp->afk, -1) < 0)
        return -1;
    return dcp_iomfb_transport_latched_swap(&dcp->iomfb_transport) == expected_swap_id;
}

static int dcp_hdmi_dptx_init(dcp_dev_t *dcp, const display_config_t *cfg)
{
    int node = adt_path_offset(adt, cfg->dp2hdmi_gpio);
    if (node < 0) {
        printf("dcp: failed to find dp2hdmi-gpio node '%s'\n", cfg->dp2hdmi_gpio);
        return -1;
    }
    struct adt_function_smc_gpio dp2hdmi_pwr, hdmi_pwr;

    int err =
        adt_getprop_copy(adt, node, "function-dp2hdmi_pwr_en", &dp2hdmi_pwr, sizeof(dp2hdmi_pwr));
    if (err < 0)
        printf("dcp: failed to get dp2hdmi_pwr_en gpio\n");
    else
        dcp->dp2hdmi_pwr_gpio = dp2hdmi_pwr.gpio;
    err = adt_getprop_copy(adt, node, "function-hdmi_pwr_en", &hdmi_pwr, sizeof(hdmi_pwr));
    if (err < 0)
        printf("dcp: failed to get hdmi_pwr_en gpio\n");
    else
        dcp->hdmi_pwr_gpio = hdmi_pwr.gpio;

    if (dcp->dp2hdmi_pwr_gpio && dcp->hdmi_pwr_gpio) {
        smc_dev_t *smc = smc_init();
        if (smc) {
            smc_write_u32(smc, dcp->dp2hdmi_pwr_gpio, 0x800001);
            smc_write_u32(smc, dcp->hdmi_pwr_gpio, 0x800001);
            smc_shutdown(smc);
        }
    }

    dcp->die = cfg->die;

    dcp->phy = dptx_phy_init(cfg->dptx_phy, cfg->dcp_index);
    if (!dcp->phy) {
        printf("dcp: failed to init (lp)dptx-phy '%s'\n", cfg->dptx_phy);
        return -1;
    }

    dcp->dpav_ep = dcp_dpav_init(dcp);
    if (!dcp->dpav_ep) {
        printf("dcp: failed to initialize dpav endpoint\n");
        return -1;
    }

    dcp->dptx_ep = dcp_dptx_init(dcp, cfg->num_dptxports);
    if (!dcp->dptx_ep) {
        printf("dcp: failed to initialize dptx-port endpoint\n");
        dcp_dpav_shutdown(dcp->dpav_ep);
        return -1;
    }

#ifdef RTKIT_SYSLOG
    // start system endpoint when extended logging is requested
    dcp->system_ep = dcp_system_init(dcp);
    if (!dcp_system_is_ready(dcp->system_ep)) {
        printf("dcp: failed to initialize system endpoint\n");
        dcp_dptx_shutdown(dcp->dptx_ep);
        dcp_dpav_shutdown(dcp->dpav_ep);
        return -1;
    }

    dcp_system_set_property_u64(dcp->system_ep, "gAFKConfigLogMask", 0xffff);
#endif

    return 0;
}

int dcp_connect_dptx(dcp_dev_t *dcp)
{
    if (dcp->dptx_ep && dcp->phy) {
        return dcp_dptx_connect(dcp->dptx_ep, dcp->phy, dcp->die, 0);
    }

    return 0;
}

int dcp_work(dcp_dev_t *dcp)
{
    return afk_epic_work(dcp->afk, -1);
}

static int dcp_create_firmware_mappings(const display_config_t *cfg, dcp_dev_t *dcp)
{
    int created = 0;
    int dcp_node = adt_path_offset(adt, cfg->dcp);
    int node = adt_first_child_offset(adt, dcp_node);
    if (node < 0) {
        printf("dcp: iop-dcp*-nub not found!\n");
        return -1;
    }

    u64 asc_dram_mask;
    if (ADT_GETPROP(adt, node, "asc-dram-mask", &asc_dram_mask) < 0)
        asc_dram_mask = 0;

    const struct adt_segment_ranges *seg;
    u32 segments_len;

    seg = adt_getprop(adt, node, "segment-ranges", &segments_len);
    unsigned int count = segments_len / sizeof(*seg);

    for (unsigned int i = 0; i < count; i++) {
        u64 iova = seg[i].remap & ~asc_dram_mask;
        if (dart_translate_silent(dcp->dart_dcp, iova))
            continue;

        size_t len = ALIGN_UP(seg[i].size, SZ_16K);
        u32 flags = i == 0 ? 0b0100 : 0; // TEXT gets this bit set?
        printf("dcp: Mapping segment #%u %lx -> %lx [%lx]\n", i, iova, seg[i].phys, len);
        if (dart_map_flags(dcp->dart_dcp, iova, (void *)seg[i].phys, len, flags)) {
            printf("dcp: Failed to map segment\n");
            return -1;
        }
        created++;
    }

    return created;
}

dcp_dev_t *dcp_init(const display_config_t *cfg)
{
    u32 sid;

    if (cfg && cfg->dptx_phy[0]) {
        if (os_firmware.version != V13_5) {
            printf("dcp: dtpx-port is only supported with V13_5 OS firmware.\n");
            return NULL;
        }

        strncpy(dcp_pmgr_dev, cfg->pmgr_dev, sizeof(dcp_pmgr_dev));
        dcp_die = cfg->die;
        pmgr_adt_power_enable(cfg->dcp);
        pmgr_adt_power_enable(cfg->dptx_phy);
        mdelay(25);
    }

    int dart_node = adt_path_offset(adt, cfg->dcp_dart);
    int node = adt_first_child_offset(adt, dart_node);
    if (node < 0) {
        printf("dcp: mapper-dcp* not found!\n");
        return NULL;
    }
    if (ADT_GETPROP(adt, node, "reg", &sid) < 0) {
        printf("dcp: failed to read dart stream ID!\n");
        return NULL;
    }

    dcp_dev_t *dcp = calloc(1, sizeof(dcp_dev_t));
    if (!dcp)
        return NULL;

    dcp->dart_dcp = dart_init_adt(cfg->dcp_dart, 0, sid, true);
    if (!dcp->dart_dcp) {
        printf("dcp: failed to initialize DCP DART\n");
        goto out_free;
    }
    u64 vm_base = dart_vm_base(dcp->dart_dcp);
    dart_setup_pt_region(dcp->dart_dcp, cfg->dcp_dart, sid, vm_base);

    dcp->dart_disp = dart_init_adt(cfg->disp_dart, 0, 0, true);
    if (!dcp->dart_disp) {
        printf("dcp: failed to initialize DISP DART\n");
        goto out_dart_dcp;
    }
    // set disp0's page tables at dart-dcp's vm-base
    dart_setup_pt_region(dcp->dart_disp, cfg->disp_dart, 0, vm_base);

#ifdef DCP_IOMFB_FULL_OWNER
    /* Linux creates/configures the PIODMA IOMMU child during probe, before
     * dcp_start() starts any RTKit application endpoint. EXP246 hardware-
     * proved this ordering on J313, so it is a production full-owner
     * invariant rather than an observer-only discriminator. */
    dcp->dart_piodma = dart_init_adt("/arm-io/dart-disp0", 0, 4, true);
    if (!dcp->dart_piodma) {
        printf("dcp-iomfb: failed to initialize early PIODMA DART\n");
        goto out_dart_disp;
    }
    if (dart_setup_pt_region(dcp->dart_piodma, "/arm-io/dart-disp0", 4,
                             vm_base)) {
        printf("dcp-iomfb: failed to configure early PIODMA page tables\n");
        goto out_dart_piodma;
    }
    printf("dcp-iomfb: early PIODMA SID4 ready before RTKit boot\n");
#endif

    /* EXP953: the guest scanout pool spans the whole 952 MiB VidMm local
     * range, so the shared DCP/DISP IOVA window is 1.25 GiB.  dart_map()
     * refuses any page that is already mapped, so a collision with a
     * firmware mapping fails registration instead of aliasing. */
    dcp->iovad_dcp = iovad_init(vm_base + 0x10000000, vm_base + 0x60000000);

    int ret = dcp_create_firmware_mappings(cfg, dcp);
    if (ret < 0) {
        printf("dcp: failed to create firmware mappings\n");
        goto out_iovad;
    }
    if (ret > 0) {
        pmgr_reset(dcp_die, dcp_pmgr_dev);
    }

    dcp->asc = asc_init(cfg->dcp);
    if (!dcp->asc) {
        printf("dcp: failed to initialize ASC\n");
        goto out_iovad;
    }

    dcp->rtkit = rtkit_init("dcp", dcp->asc, dcp->dart_dcp, dcp->iovad_dcp, NULL, false);
    if (!dcp->rtkit) {
        printf("dcp: failed to initialize RTKit\n");
        goto out_iovad;
    }

    if (!rtkit_boot(dcp->rtkit)) {
        printf("dcp: failed to boot RTKit\n");
        goto out_iovad;
    }

    dcp->afk = afk_epic_init(dcp->rtkit);
    if (!dcp->afk) {
        printf("dcp: failed to initialize AFK\n");
        goto out_rtkit;
    }

    if (cfg && cfg->dptx_phy[0]) {
        int ret = dcp_hdmi_dptx_init(dcp, cfg);
        if (ret < 0)
            goto out_afk;
    }

    return dcp;

out_afk:
    afk_epic_shutdown(dcp->afk);
out_rtkit:
    rtkit_quiesce(dcp->rtkit);
    rtkit_free(dcp->rtkit);
out_iovad:
    iovad_shutdown(dcp->iovad_dcp, dcp->dart_dcp);
#ifdef DCP_IOMFB_FULL_OWNER
out_dart_piodma:
    dart_shutdown(dcp->dart_piodma);
out_dart_disp:
#endif
    dart_shutdown(dcp->dart_disp);
out_dart_dcp:
    dart_shutdown(dcp->dart_dcp);
out_free:
    free(dcp);
    return NULL;
}

int dcp_shutdown(dcp_dev_t *dcp, bool sleep)
{
    /* dcp/dcp0 on desktop M2 and M2 Pro/Max devices do not wake from sleep */
    bool iomfb_owner = dcp->iomfb_owner_endpoint_started;
    bool owner_was_registered = dcp->iomfb_owner_registered;

    if (iomfb_owner) {
        /* First close every normal EPIC producer while firmware can still
         * acknowledge it.  Then prove global RTKit quiesce.  Failure retains
         * every owner buffer/mapping until the enclosing SoC reset.  Poison
         * the public owner state before the first fallible step so a partially
         * closed endpoint can never be reused as an active scanout backend. */
        dcp->iomfb_owner_registered = false;
        if (dcp->system_ep && dcp_system_shutdown(dcp->system_ep) < 0)
            return -1;
        dcp->system_ep = NULL;
        if (dcp->dptx_ep && dcp_dptx_shutdown(dcp->dptx_ep) < 0)
            return -1;
        dcp->dptx_ep = NULL;
        if (dcp->dpav_ep && dcp_dpav_shutdown(dcp->dpav_ep) < 0)
            return -1;
        dcp->dpav_ep = NULL;
        if (!rtkit_quiesce(dcp->rtkit)) {
            printf("dcp-iomfb: RTKit quiesce failed; retaining all owner resources\n");
            return -1;
        }
        free(dcp->phy);
        dcp->phy = NULL;
        if (owner_was_registered)
            afk_epic_unregister_raw_handler(dcp->afk,
                                            DCP_IOMFB_RPC_ENDPOINT,
                                            dcp_iomfb_owner_receive, dcp);
        dcp_iomfb_properties_destroy(&dcp->iomfb_properties);
        dcp_iomfb_resources_destroy(&dcp->iomfb_resources);
        rtkit_free_buffer(dcp->rtkit, &dcp->iomfb_shmem);
        memset(&dcp->iomfb_shmem, 0, sizeof(dcp->iomfb_shmem));
        if (dcp->dart_piodma)
            dart_shutdown(dcp->dart_piodma);
        dcp->dart_piodma = NULL;
        dcp->iomfb_owner_registered = false;
        dcp->iomfb_owner_endpoint_started = false;
    } else {
        if (dcp->system_ep && dcp_system_shutdown(dcp->system_ep) < 0) {
            printf("dcp-system: shutdown failed; retaining DCP until reset\n");
            return -1;
        }
        dcp->system_ep = NULL;
        dcp_dptx_shutdown(dcp->dptx_ep);
        dcp_dpav_shutdown(dcp->dpav_ep);
        free(dcp->phy);
    }
    afk_epic_shutdown(dcp->afk);
    if (!iomfb_owner && sleep) {
        rtkit_sleep(dcp->rtkit);
        pmgr_reset(dcp_die, dcp_pmgr_dev);
    } else if (!iomfb_owner) {
        rtkit_quiesce(dcp->rtkit);
    }
    rtkit_free(dcp->rtkit);
    if (!iomfb_owner && dcp->dart_piodma)
        dart_shutdown(dcp->dart_piodma);
    dart_shutdown(dcp->dart_disp);
    iovad_shutdown(dcp->iovad_dcp, dcp->dart_dcp);
    dart_shutdown(dcp->dart_dcp);
    free(dcp);

    return 0;
}

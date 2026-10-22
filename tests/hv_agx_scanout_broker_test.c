#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "../src/hv_agx_scanout_broker.h"

static void write32(struct hv_agx_scanout_broker *broker, uint64_t reg, uint32_t value)
{
    uint64_t wire = value;
    assert(hv_agx_scanout_broker_mmio(broker, reg, &wire, true, 2));
}

static void write64(struct hv_agx_scanout_broker *broker, uint64_t reg, uint64_t value)
{
    assert(hv_agx_scanout_broker_mmio(broker, reg, &value, true, 3));
}

static uint32_t read32(struct hv_agx_scanout_broker *broker, uint64_t reg)
{
    uint64_t value = 0;
    assert(hv_agx_scanout_broker_mmio(broker, reg, &value, false, 2));
    return (uint32_t)value;
}

static uint64_t read64(struct hv_agx_scanout_broker *broker, uint64_t reg)
{
    uint64_t value = 0;
    assert(hv_agx_scanout_broker_mmio(broker, reg, &value, false, 3));
    return value;
}

static void stage_register(struct hv_agx_scanout_broker *broker, uint64_t sequence)
{
    write64(broker, HV_AGX_SCANOUT_REG_POOL_IPA, UINT64_C(0x10000000));
    write64(broker, HV_AGX_SCANOUT_REG_POOL_SIZE, HV_AGX_SCANOUT_J313_POOL_SIZE);
    write64(broker, HV_AGX_SCANOUT_REG_REQUEST_SEQUENCE, sequence);
    write32(broker, HV_AGX_SCANOUT_REG_COMMAND, HV_AGX_SCANOUT_CMD_REGISTER_POOL);
}

static void stage_present_format(struct hv_agx_scanout_broker *broker, uint64_t sequence,
                                 uint64_t offset, uint32_t format);
static void stage_present(struct hv_agx_scanout_broker *broker, uint64_t sequence,
                          uint64_t offset)
{
    stage_present_format(broker, sequence, offset, HV_AGX_SCANOUT_FORMAT_BGRA8888);
}

static void stage_present_format(struct hv_agx_scanout_broker *broker, uint64_t sequence,
                                 uint64_t offset, uint32_t format)
{
    write64(broker, HV_AGX_SCANOUT_REG_SURFACE_OFFSET, offset);
    write64(broker, HV_AGX_SCANOUT_REG_SURFACE_SIZE, HV_AGX_SCANOUT_J313_SURFACE_SIZE);
    write32(broker, HV_AGX_SCANOUT_REG_WIDTH, HV_AGX_SCANOUT_J313_WIDTH);
    write32(broker, HV_AGX_SCANOUT_REG_HEIGHT, HV_AGX_SCANOUT_J313_HEIGHT);
    write32(broker, HV_AGX_SCANOUT_REG_STRIDE, HV_AGX_SCANOUT_J313_STRIDE);
    write32(broker, HV_AGX_SCANOUT_REG_FORMAT, format);
    write64(broker, HV_AGX_SCANOUT_REG_REQUEST_SEQUENCE, sequence);
    write32(broker, HV_AGX_SCANOUT_REG_COMMAND, HV_AGX_SCANOUT_CMD_PRESENT);
}

static void test_identity_and_exact_mmio(void)
{
    struct hv_agx_scanout_broker broker;
    uint64_t value = 0;

    memset(&broker, 0xa5, sizeof(broker));
    hv_agx_scanout_broker_init(&broker);
    assert(read32(&broker, HV_AGX_SCANOUT_REG_MAGIC) == HV_AGX_SCANOUT_MAGIC);
    assert(read32(&broker, HV_AGX_SCANOUT_REG_ABI_VERSION) ==
           HV_AGX_SCANOUT_ABI_VERSION);
    assert(read32(&broker, HV_AGX_SCANOUT_REG_CAPABILITIES) ==
           (HV_AGX_SCANOUT_CAP_FIXED_J313 | HV_AGX_SCANOUT_CAP_BGRA8888 |
            HV_AGX_SCANOUT_CAP_REGISTERED_POOL | HV_AGX_SCANOUT_CAP_APPLIED_RECEIPT |
            HV_AGX_SCANOUT_CAP_RGBA8888));
    assert(read32(&broker, HV_AGX_SCANOUT_REG_STATE) == HV_AGX_SCANOUT_UNREGISTERED);
    assert(read64(&broker, HV_AGX_SCANOUT_REG_RECEIPT_SEQUENCE) == 0);
    assert(read64(&broker, HV_AGX_SCANOUT_REG_APPLIED_SEQUENCE) == 0);
    assert(read64(&broker, HV_AGX_SCANOUT_REG_LATCHED_SEQUENCE) == 0);
    assert(!hv_agx_scanout_broker_mmio(&broker, HV_AGX_SCANOUT_REG_MAGIC, &value, true, 2));
    assert(!hv_agx_scanout_broker_mmio(&broker, HV_AGX_SCANOUT_REG_MAGIC, &value, false, 3));
    assert(!hv_agx_scanout_broker_mmio(&broker, 0x7c, &value, false, 2));
}

static void test_register_snapshot_and_completion(void)
{
    struct hv_agx_scanout_broker broker;
    struct hv_agx_scanout_request request;

    hv_agx_scanout_broker_init(&broker);
    stage_register(&broker, 1);
    assert(read32(&broker, HV_AGX_SCANOUT_REG_STATE) == HV_AGX_SCANOUT_PENDING);
    assert(read64(&broker, HV_AGX_SCANOUT_REG_RECEIPT_SEQUENCE) == 0);
    write64(&broker, HV_AGX_SCANOUT_REG_POOL_IPA, UINT64_C(0x20000000));
    assert(hv_agx_scanout_broker_take_pending(&broker, &request));
    assert(request.Command == HV_AGX_SCANOUT_CMD_REGISTER_POOL);
    assert(request.Sequence == 1);
    assert(request.PoolIpa == UINT64_C(0x10000000));
    assert(request.PoolSize == HV_AGX_SCANOUT_J313_POOL_SIZE);
    assert(!hv_agx_scanout_broker_take_pending(&broker, &request));
    assert(hv_agx_scanout_broker_complete_register(
        &broker, 1, HV_AGX_SCANOUT_RESULT_OK, UINT64_C(0x880000000),
        UINT64_C(0xf00000000)));
    assert(read32(&broker, HV_AGX_SCANOUT_REG_STATE) == HV_AGX_SCANOUT_READY);
    assert(read64(&broker, HV_AGX_SCANOUT_REG_RECEIPT_SEQUENCE) == 1);
    assert(read64(&broker, HV_AGX_SCANOUT_REG_APPLIED_SEQUENCE) == 1);
    assert(read64(&broker, HV_AGX_SCANOUT_REG_POOL_PA) == UINT64_C(0x880000000));
}

static void test_sequence_and_busy_rejection(void)
{
    struct hv_agx_scanout_broker broker;
    struct hv_agx_scanout_request request;

    hv_agx_scanout_broker_init(&broker);
    stage_register(&broker, 1);
    stage_register(&broker, 2);
    assert(read32(&broker, HV_AGX_SCANOUT_REG_RESULT) == HV_AGX_SCANOUT_RESULT_BUSY);
    assert(read64(&broker, HV_AGX_SCANOUT_REG_REJECTED_REQUESTS) == 1);
    assert(read64(&broker, HV_AGX_SCANOUT_REG_RECEIPT_SEQUENCE) == 0);
    assert(hv_agx_scanout_broker_take_pending(&broker, &request));
    assert(request.Sequence == 1);
    assert(hv_agx_scanout_broker_complete_register(
        &broker, 1, HV_AGX_SCANOUT_RESULT_OK, UINT64_C(0x880000000),
        UINT64_C(0xf00000000)));
    stage_present(&broker, 2, 1);
    assert(read64(&broker, HV_AGX_SCANOUT_REG_RECEIPT_SEQUENCE) == 2);
    stage_register(&broker, 1);
    assert(read32(&broker, HV_AGX_SCANOUT_REG_RESULT) ==
           HV_AGX_SCANOUT_RESULT_STALE_SEQUENCE);
    write64(&broker, HV_AGX_SCANOUT_REG_REQUEST_SEQUENCE, 0);
    write32(&broker, HV_AGX_SCANOUT_REG_COMMAND, HV_AGX_SCANOUT_CMD_QUERY);
    assert(read32(&broker, HV_AGX_SCANOUT_REG_RESULT) ==
           HV_AGX_SCANOUT_RESULT_STALE_SEQUENCE);
}

static void test_surface_validation_and_latch_separation(void)
{
    struct hv_agx_scanout_broker broker;
    struct hv_agx_scanout_request request;
    uint64_t highest = HV_AGX_SCANOUT_J313_POOL_SIZE - HV_AGX_SCANOUT_J313_SURFACE_SIZE;

    hv_agx_scanout_broker_init(&broker);
    stage_register(&broker, 1);
    assert(hv_agx_scanout_broker_take_pending(&broker, &request));
    assert(hv_agx_scanout_broker_complete_register(
        &broker, 1, HV_AGX_SCANOUT_RESULT_OK, UINT64_C(0x880000000),
        UINT64_C(0xf00000000)));

    stage_present(&broker, 2, highest & ~UINT64_C(0xffff));
    assert(hv_agx_scanout_broker_take_pending(&broker, &request));
    assert(request.Command == HV_AGX_SCANOUT_CMD_PRESENT);
    assert(request.SurfaceOffset == (highest & ~UINT64_C(0xffff)));
    assert(hv_agx_scanout_broker_complete_present(&broker, 2,
                                                    HV_AGX_SCANOUT_RESULT_OK, 17));
    assert(read32(&broker, HV_AGX_SCANOUT_REG_STATE) == HV_AGX_SCANOUT_ACTIVE);
    assert(read64(&broker, HV_AGX_SCANOUT_REG_APPLIED_SEQUENCE) == 2);
    assert(read64(&broker, HV_AGX_SCANOUT_REG_LATCHED_SEQUENCE) == 0);
    assert(read64(&broker, HV_AGX_SCANOUT_REG_ACTIVE_OFFSET) ==
           (highest & ~UINT64_C(0xffff)));
    assert(read32(&broker, HV_AGX_SCANOUT_REG_SWAP_ID) == 17);
    assert(!hv_agx_scanout_broker_mark_latched(&broker, 2));
    assert(read64(&broker, HV_AGX_SCANOUT_REG_LATCHED_SEQUENCE) == 0);
    assert(read32(&broker, HV_AGX_SCANOUT_REG_IRQ_STATUS) == 0);

    stage_present(&broker, 3, 0);
    assert(read32(&broker, HV_AGX_SCANOUT_REG_RESULT) == HV_AGX_SCANOUT_RESULT_BUSY);
    assert(read64(&broker, HV_AGX_SCANOUT_REG_RECEIPT_SEQUENCE) == 2);
    assert(read64(&broker, HV_AGX_SCANOUT_REG_APPLIED_SEQUENCE) == 2);
    assert(read32(&broker, HV_AGX_SCANOUT_REG_SWAP_ID) == 17);

    stage_present(&broker, 4, 1);
    assert(read32(&broker, HV_AGX_SCANOUT_REG_RESULT) ==
           HV_AGX_SCANOUT_RESULT_INVALID_SURFACE);
    assert(read64(&broker, HV_AGX_SCANOUT_REG_RECEIPT_SEQUENCE) == 4);
    assert(read64(&broker, HV_AGX_SCANOUT_REG_ACTIVE_OFFSET) ==
           (highest & ~UINT64_C(0xffff)));
}

static void test_v2_requires_exact_latch_before_repeated_present(void)
{
    struct hv_agx_scanout_broker broker;
    struct hv_agx_scanout_request request;

    hv_agx_scanout_broker_init_v2(&broker, true);
    assert(read32(&broker, HV_AGX_SCANOUT_REG_ABI_VERSION) == 2);
    assert((read32(&broker, HV_AGX_SCANOUT_REG_CAPABILITIES) &
            (HV_AGX_SCANOUT_CAP_REPEATED_PRESENT |
             HV_AGX_SCANOUT_CAP_LATCHED_RECEIPT |
             HV_AGX_SCANOUT_CAP_LATCHED_IRQ)) ==
           (HV_AGX_SCANOUT_CAP_REPEATED_PRESENT |
            HV_AGX_SCANOUT_CAP_LATCHED_RECEIPT |
            HV_AGX_SCANOUT_CAP_LATCHED_IRQ));

    stage_register(&broker, 1);
    assert(hv_agx_scanout_broker_take_pending(&broker, &request));
    assert(hv_agx_scanout_broker_complete_register(
        &broker, 1, HV_AGX_SCANOUT_RESULT_OK, UINT64_C(0x880000000),
        UINT64_C(0xf00000000)));
    stage_present(&broker, 2, 0);
    assert(hv_agx_scanout_broker_take_pending(&broker, &request));
    assert(hv_agx_scanout_broker_complete_present(
        &broker, 2, HV_AGX_SCANOUT_RESULT_OK, 31));
    assert(read64(&broker, HV_AGX_SCANOUT_REG_APPLIED_SEQUENCE) == 2);
    assert(read64(&broker, HV_AGX_SCANOUT_REG_LATCHED_SEQUENCE) == 0);

    stage_present(&broker, 3, HV_AGX_SCANOUT_ALIGNMENT);
    assert(read32(&broker, HV_AGX_SCANOUT_REG_RESULT) ==
           HV_AGX_SCANOUT_RESULT_BUSY);
    assert(!hv_agx_scanout_broker_mark_latched(&broker, 1));
    assert(hv_agx_scanout_broker_mark_latched(&broker, 2));
    assert(read64(&broker, HV_AGX_SCANOUT_REG_LATCHED_SEQUENCE) == 2);
    assert((read32(&broker, HV_AGX_SCANOUT_REG_IRQ_STATUS) &
            HV_AGX_SCANOUT_IRQ_LATCHED) != 0);

    write32(&broker, HV_AGX_SCANOUT_REG_IRQ_STATUS,
            HV_AGX_SCANOUT_IRQ_LATCHED);
    stage_present(&broker, 4, HV_AGX_SCANOUT_ALIGNMENT);
    assert(hv_agx_scanout_broker_take_pending(&broker, &request));
    assert(hv_agx_scanout_broker_complete_present(
        &broker, 4, HV_AGX_SCANOUT_RESULT_OK, 32));
    assert(hv_agx_scanout_broker_mark_latched(&broker, 4));
    assert(read64(&broker, HV_AGX_SCANOUT_REG_APPLIED_SEQUENCE) == 4);
    assert(read64(&broker, HV_AGX_SCANOUT_REG_LATCHED_SEQUENCE) == 4);
}

static void test_v2_without_latch_source_stays_fail_closed(void)
{
    struct hv_agx_scanout_broker broker;

    hv_agx_scanout_broker_init_v2(&broker, false);
    assert(read32(&broker, HV_AGX_SCANOUT_REG_ABI_VERSION) == 2);
    assert((read32(&broker, HV_AGX_SCANOUT_REG_CAPABILITIES) &
            (HV_AGX_SCANOUT_CAP_REPEATED_PRESENT |
             HV_AGX_SCANOUT_CAP_LATCHED_RECEIPT |
             HV_AGX_SCANOUT_CAP_LATCHED_IRQ)) == 0);
    assert(!hv_agx_scanout_broker_mark_latched(&broker, 1));
}

static void test_v2_irq_injects_only_once_per_enabled_pending_edge(void)
{
    struct hv_agx_scanout_broker broker;
    struct hv_agx_scanout_request request;

    hv_agx_scanout_broker_init_v2(&broker, true);
    stage_register(&broker, 1);
    assert(hv_agx_scanout_broker_take_pending(&broker, &request));
    assert(hv_agx_scanout_broker_complete_register(
        &broker, 1, HV_AGX_SCANOUT_RESULT_OK, UINT64_C(0x880000000),
        UINT64_C(0xf00000000)));
    write32(&broker, HV_AGX_SCANOUT_REG_IRQ_ENABLE,
            HV_AGX_SCANOUT_IRQ_LATCHED | HV_AGX_SCANOUT_IRQ_ERROR);
    stage_present(&broker, 2, 0);
    assert(hv_agx_scanout_broker_take_pending(&broker, &request));
    assert(hv_agx_scanout_broker_complete_present(
        &broker, 2, HV_AGX_SCANOUT_RESULT_OK, 31));
    assert(hv_agx_scanout_broker_mark_latched(&broker, 2));
    assert(hv_agx_scanout_broker_take_irq_edge(&broker));
    assert(!hv_agx_scanout_broker_take_irq_edge(&broker));
    write32(&broker, HV_AGX_SCANOUT_REG_IRQ_STATUS,
            HV_AGX_SCANOUT_IRQ_LATCHED);
    assert(!hv_agx_scanout_broker_take_irq_edge(&broker));

    stage_present(&broker, 3, HV_AGX_SCANOUT_ALIGNMENT);
    assert(hv_agx_scanout_broker_take_pending(&broker, &request));
    assert(hv_agx_scanout_broker_complete_present(
        &broker, 3, HV_AGX_SCANOUT_RESULT_OK, 32));
    assert(hv_agx_scanout_broker_mark_latched(&broker, 3));
    assert(hv_agx_scanout_broker_take_irq_edge(&broker));
}

static void test_failed_present_preserves_active_and_release_fails_closed(void)
{
    struct hv_agx_scanout_broker broker;
    struct hv_agx_scanout_request request;

    hv_agx_scanout_broker_init(&broker);
    stage_register(&broker, 1);
    assert(hv_agx_scanout_broker_take_pending(&broker, &request));
    assert(hv_agx_scanout_broker_complete_register(
        &broker, 1, HV_AGX_SCANOUT_RESULT_OK, UINT64_C(0x880000000),
        UINT64_C(0xf00000000)));
    stage_present(&broker, 2, 0);
    assert(hv_agx_scanout_broker_take_pending(&broker, &request));
    assert(hv_agx_scanout_broker_complete_present(
        &broker, 2, HV_AGX_SCANOUT_RESULT_PRESENT_FAILED, 0));
    assert(read32(&broker, HV_AGX_SCANOUT_REG_STATE) == HV_AGX_SCANOUT_READY);
    assert(read64(&broker, HV_AGX_SCANOUT_REG_ACTIVE_OFFSET) == 0);
    write64(&broker, HV_AGX_SCANOUT_REG_REQUEST_SEQUENCE, 3);
    write32(&broker, HV_AGX_SCANOUT_REG_COMMAND, HV_AGX_SCANOUT_CMD_RELEASE);
    assert(hv_agx_scanout_broker_take_pending(&broker, &request));
    assert(hv_agx_scanout_broker_complete_release(
        &broker, 3, HV_AGX_SCANOUT_RESULT_NOT_QUIESCED, false));
    assert(read32(&broker, HV_AGX_SCANOUT_REG_STATE) == HV_AGX_SCANOUT_READY);
    assert(read64(&broker, HV_AGX_SCANOUT_REG_POOL_PA) == UINT64_C(0x880000000));
}

static void test_register_rejects_invalid_mapping(void)
{
    struct hv_agx_scanout_broker broker;
    struct hv_agx_scanout_request request;

    hv_agx_scanout_broker_init(&broker);
    stage_register(&broker, 1);
    assert(hv_agx_scanout_broker_take_pending(&broker, &request));
    assert(hv_agx_scanout_broker_complete_register(
        &broker, 1, HV_AGX_SCANOUT_RESULT_OK, UINT64_C(0x880000000), 1));
    assert(read32(&broker, HV_AGX_SCANOUT_REG_RESULT) ==
           HV_AGX_SCANOUT_RESULT_MAP_FAILED);
    assert(read32(&broker, HV_AGX_SCANOUT_REG_STATE) == HV_AGX_SCANOUT_UNREGISTERED);
    assert(read64(&broker, HV_AGX_SCANOUT_REG_APPLIED_SEQUENCE) == 0);
    assert(read64(&broker, HV_AGX_SCANOUT_REG_POOL_PA) == 0);
}

/* M2-f2c: the hypervisor sets the pool size from the receipt-sized reserve;
 * only that size registers. */
static void test_pool_size_follows_reserve(void)
{
    struct hv_agx_scanout_broker broker;
    struct hv_agx_scanout_request request;

    hv_agx_scanout_broker_init(&broker);
    assert(broker.pool_size == HV_AGX_SCANOUT_J313_POOL_SIZE);
    assert(!hv_agx_scanout_broker_set_pool_size(&broker,
                                                HV_AGX_SCANOUT_J313_POOL_SIZE + 0x10000));
    assert(!hv_agx_scanout_broker_set_pool_size(&broker, UINT64_C(0x13804000)));
    assert(!hv_agx_scanout_broker_set_pool_size(&broker, UINT64_C(0x800000)));
    assert(hv_agx_scanout_broker_set_pool_size(&broker, UINT64_C(0x13800000)));
    /* A full-size registration no longer matches. */
    stage_register(&broker, 1);
    assert(read32(&broker, HV_AGX_SCANOUT_REG_RESULT) == HV_AGX_SCANOUT_RESULT_INVALID_POOL);

    hv_agx_scanout_broker_init(&broker);
    assert(hv_agx_scanout_broker_set_pool_size(&broker, UINT64_C(0x13800000)));
    write64(&broker, HV_AGX_SCANOUT_REG_POOL_IPA, UINT64_C(0x10000000));
    write64(&broker, HV_AGX_SCANOUT_REG_POOL_SIZE, UINT64_C(0x13800000));
    write64(&broker, HV_AGX_SCANOUT_REG_REQUEST_SEQUENCE, 1);
    write32(&broker, HV_AGX_SCANOUT_REG_COMMAND, HV_AGX_SCANOUT_CMD_REGISTER_POOL);
    assert(hv_agx_scanout_broker_take_pending(&broker, &request));
    assert(request.PoolSize == UINT64_C(0x13800000));
    /* Once a registration is pending the size is fixed. */
    assert(!hv_agx_scanout_broker_set_pool_size(&broker, HV_AGX_SCANOUT_J313_POOL_SIZE));
}

static void test_active_release_failure_preserves_ownership(void)
{
    struct hv_agx_scanout_broker broker;
    struct hv_agx_scanout_request request;

    hv_agx_scanout_broker_init(&broker);
    stage_register(&broker, 1);
    assert(hv_agx_scanout_broker_take_pending(&broker, &request));
    assert(hv_agx_scanout_broker_complete_register(
        &broker, 1, HV_AGX_SCANOUT_RESULT_OK, UINT64_C(0x880000000),
        UINT64_C(0xf00000000)));
    stage_present(&broker, 2, 0);
    assert(hv_agx_scanout_broker_take_pending(&broker, &request));
    assert(hv_agx_scanout_broker_complete_present(
        &broker, 2, HV_AGX_SCANOUT_RESULT_OK, 19));
    write64(&broker, HV_AGX_SCANOUT_REG_REQUEST_SEQUENCE, 3);
    write32(&broker, HV_AGX_SCANOUT_REG_COMMAND, HV_AGX_SCANOUT_CMD_RELEASE);
    assert(hv_agx_scanout_broker_take_pending(&broker, &request));
    assert(hv_agx_scanout_broker_complete_release(
        &broker, 3, HV_AGX_SCANOUT_RESULT_NOT_QUIESCED, false));
    assert(read32(&broker, HV_AGX_SCANOUT_REG_STATE) == HV_AGX_SCANOUT_ACTIVE);
    assert(read64(&broker, HV_AGX_SCANOUT_REG_POOL_PA) == UINT64_C(0x880000000));
    assert(broker.pool_iova == UINT64_C(0xf00000000));
    assert(read64(&broker, HV_AGX_SCANOUT_REG_ACTIVE_OFFSET) == 0);
    assert(read32(&broker, HV_AGX_SCANOUT_REG_SWAP_ID) == 19);
    assert(read64(&broker, HV_AGX_SCANOUT_REG_APPLIED_SEQUENCE) == 2);
    assert(read64(&broker, HV_AGX_SCANOUT_REG_LATCHED_SEQUENCE) == 0);
}

static void test_release_without_quiesce_proof_fails_closed(void)
{
    struct hv_agx_scanout_broker broker;
    struct hv_agx_scanout_request request;

    hv_agx_scanout_broker_init(&broker);
    stage_register(&broker, 1);
    assert(hv_agx_scanout_broker_take_pending(&broker, &request));
    assert(hv_agx_scanout_broker_complete_register(
        &broker, 1, HV_AGX_SCANOUT_RESULT_OK, UINT64_C(0x880000000),
        UINT64_C(0xf00000000)));
    stage_present(&broker, 2, 0);
    assert(hv_agx_scanout_broker_take_pending(&broker, &request));
    assert(hv_agx_scanout_broker_complete_present(
        &broker, 2, HV_AGX_SCANOUT_RESULT_OK, 23));
    write64(&broker, HV_AGX_SCANOUT_REG_REQUEST_SEQUENCE, 3);
    write32(&broker, HV_AGX_SCANOUT_REG_COMMAND, HV_AGX_SCANOUT_CMD_RELEASE);
    assert(hv_agx_scanout_broker_take_pending(&broker, &request));
    assert(hv_agx_scanout_broker_complete_release(
        &broker, 3, HV_AGX_SCANOUT_RESULT_OK, false));
    assert(read32(&broker, HV_AGX_SCANOUT_REG_RESULT) ==
           HV_AGX_SCANOUT_RESULT_NOT_QUIESCED);
    assert(read32(&broker, HV_AGX_SCANOUT_REG_STATE) == HV_AGX_SCANOUT_ACTIVE);
    assert(read64(&broker, HV_AGX_SCANOUT_REG_POOL_PA) == UINT64_C(0x880000000));
    assert(read32(&broker, HV_AGX_SCANOUT_REG_SWAP_ID) == 23);
    assert(read64(&broker, HV_AGX_SCANOUT_REG_APPLIED_SEQUENCE) == 2);
}

/* D3D-FS1: an RGBA8888 present is accepted and carries its format to the
 * consumer; any other format is an invalid surface. */
static void test_rgba_present_format_reaches_the_request(void)
{
    struct hv_agx_scanout_broker broker;
    struct hv_agx_scanout_request request;

    hv_agx_scanout_broker_init(&broker);
    stage_register(&broker, 1);
    assert(hv_agx_scanout_broker_take_pending(&broker, &request));
    assert(hv_agx_scanout_broker_complete_register(
        &broker, 1, HV_AGX_SCANOUT_RESULT_OK, UINT64_C(0x880000000),
        UINT64_C(0xf00000000)));
    stage_present_format(&broker, 2, 0, HV_AGX_SCANOUT_FORMAT_RGBA8888);
    assert(hv_agx_scanout_broker_take_pending(&broker, &request));
    assert(request.Format == HV_AGX_SCANOUT_FORMAT_RGBA8888);
    assert(hv_agx_scanout_broker_complete_present(&broker, 2,
                                                    HV_AGX_SCANOUT_RESULT_OK, 5));
    stage_present_format(&broker, 3, 0, 3u);
    assert(read32(&broker, HV_AGX_SCANOUT_REG_RESULT) ==
           HV_AGX_SCANOUT_RESULT_INVALID_SURFACE);
}

int main(void)
{
    test_rgba_present_format_reaches_the_request();
    test_identity_and_exact_mmio();
    test_register_snapshot_and_completion();
    test_sequence_and_busy_rejection();
    test_surface_validation_and_latch_separation();
    test_v2_requires_exact_latch_before_repeated_present();
    test_v2_without_latch_source_stays_fail_closed();
    test_v2_irq_injects_only_once_per_enabled_pending_edge();
    test_failed_present_preserves_active_and_release_fails_closed();
    test_register_rejects_invalid_mapping();
    test_pool_size_follows_reserve();
    test_active_release_failure_preserves_ownership();
    test_release_without_quiesce_proof_fails_closed();
    return 0;
}

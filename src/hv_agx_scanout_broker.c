/* SPDX-License-Identifier: MIT */

#include "hv_agx_scanout_broker.h"

#include <string.h>

#define SCANOUT_V1_CAPS                                                         \
    (HV_AGX_SCANOUT_CAP_FIXED_J313 | HV_AGX_SCANOUT_CAP_BGRA8888 |             \
     HV_AGX_SCANOUT_CAP_REGISTERED_POOL | HV_AGX_SCANOUT_CAP_APPLIED_RECEIPT | \
     HV_AGX_SCANOUT_CAP_RGBA8888)

static bool add_overflows(uint64_t left, uint64_t right)
{
    return left > UINT64_MAX - right;
}

static bool aligned(uint64_t value)
{
    return value != 0 && (value & (HV_AGX_SCANOUT_ALIGNMENT - 1)) == 0;
}

static void reject(struct hv_agx_scanout_broker *broker,
                   enum hv_agx_scanout_result result)
{
    broker->result = result;
    broker->rejected_requests++;
}

static void reject_terminal(struct hv_agx_scanout_broker *broker,
                            enum hv_agx_scanout_result result, uint64_t sequence)
{
    broker->highest_sequence = sequence;
    broker->receipt_sequence = sequence;
    reject(broker, result);
}

static bool sequence_fresh(const struct hv_agx_scanout_broker *broker, uint64_t sequence)
{
    return sequence != 0 && sequence > broker->highest_sequence;
}

static bool surface_valid(const struct hv_agx_scanout_broker *broker)
{
    uint64_t offset = broker->staging_surface_offset;
    uint64_t size = broker->staging_surface_size;

    if (broker->registered_pool_size != broker->pool_size ||
        (offset & (HV_AGX_SCANOUT_ALIGNMENT - 1)) != 0 ||
        size != HV_AGX_SCANOUT_J313_SURFACE_SIZE ||
        broker->staging_width != HV_AGX_SCANOUT_J313_WIDTH ||
        broker->staging_height != HV_AGX_SCANOUT_J313_HEIGHT ||
        broker->staging_stride != HV_AGX_SCANOUT_J313_STRIDE ||
        (broker->staging_format != HV_AGX_SCANOUT_FORMAT_BGRA8888 &&
         (broker->staging_format != HV_AGX_SCANOUT_FORMAT_RGBA8888 ||
          !(broker->capabilities & HV_AGX_SCANOUT_CAP_RGBA8888))) ||
        add_overflows(offset, size) ||
        offset + size > broker->registered_pool_size)
        return false;
    return true;
}

static bool submit(struct hv_agx_scanout_broker *broker, uint32_t command)
{
    uint64_t sequence = broker->staging_sequence;

    if (!sequence_fresh(broker, sequence)) {
        reject(broker, HV_AGX_SCANOUT_RESULT_STALE_SEQUENCE);
        return true;
    }
    if (broker->state == HV_AGX_SCANOUT_PENDING ||
        broker->state == HV_AGX_SCANOUT_QUIESCING) {
        reject(broker, HV_AGX_SCANOUT_RESULT_BUSY);
        return true;
    }

    if (command == HV_AGX_SCANOUT_CMD_QUERY) {
        broker->highest_sequence = sequence;
        broker->receipt_sequence = sequence;
        broker->result = HV_AGX_SCANOUT_RESULT_OK;
        broker->accepted_requests++;
        return true;
    }
    if (command == HV_AGX_SCANOUT_CMD_REGISTER_POOL) {
        if (broker->state != HV_AGX_SCANOUT_UNREGISTERED ||
            !aligned(broker->staging_pool_ipa) ||
            broker->staging_pool_size != broker->pool_size ||
            add_overflows(broker->staging_pool_ipa, broker->staging_pool_size)) {
            reject_terminal(broker, HV_AGX_SCANOUT_RESULT_INVALID_POOL, sequence);
            return true;
        }
    } else if (command == HV_AGX_SCANOUT_CMD_PRESENT) {
        if ((broker->state != HV_AGX_SCANOUT_READY &&
             broker->state != HV_AGX_SCANOUT_ACTIVE) ||
            !surface_valid(broker)) {
            reject_terminal(broker, HV_AGX_SCANOUT_RESULT_INVALID_SURFACE, sequence);
            return true;
        }
        if (broker->state == HV_AGX_SCANOUT_ACTIVE &&
            broker->latched_sequence != broker->applied_sequence) {
            reject(broker, HV_AGX_SCANOUT_RESULT_BUSY);
            return true;
        }
    } else if (command == HV_AGX_SCANOUT_CMD_RELEASE) {
        if (broker->state != HV_AGX_SCANOUT_READY &&
            broker->state != HV_AGX_SCANOUT_ACTIVE) {
            reject(broker, HV_AGX_SCANOUT_RESULT_BUSY);
            return true;
        }
    } else {
        reject_terminal(broker, HV_AGX_SCANOUT_RESULT_INVALID_COMMAND, sequence);
        return true;
    }

    memset(&broker->pending, 0, sizeof(broker->pending));
    broker->pending.Command = command;
    broker->pending.Sequence = sequence;
    broker->pending.PoolIpa = broker->staging_pool_ipa;
    broker->pending.PoolSize = broker->staging_pool_size;
    broker->pending.SurfaceOffset = broker->staging_surface_offset;
    broker->pending.SurfaceSize = broker->staging_surface_size;
    broker->pending.Width = broker->staging_width;
    broker->pending.Height = broker->staging_height;
    broker->pending.Stride = broker->staging_stride;
    broker->pending.Format = broker->staging_format;
    broker->state_before_pending = broker->state;
    broker->state = command == HV_AGX_SCANOUT_CMD_RELEASE ? HV_AGX_SCANOUT_QUIESCING
                                                          : HV_AGX_SCANOUT_PENDING;
    broker->highest_sequence = sequence;
    broker->pending_taken = false;
    broker->result = HV_AGX_SCANOUT_RESULT_OK;
    broker->accepted_requests++;
    return true;
}

void hv_agx_scanout_broker_init(struct hv_agx_scanout_broker *broker)
{
    if (!broker)
        return;
    memset(broker, 0, sizeof(*broker));
    broker->abi_version = HV_AGX_SCANOUT_ABI_VERSION_V1;
    broker->capabilities = SCANOUT_V1_CAPS;
    broker->state = HV_AGX_SCANOUT_UNREGISTERED;
    broker->state_before_pending = HV_AGX_SCANOUT_UNREGISTERED;
    broker->result = HV_AGX_SCANOUT_RESULT_OK;
    broker->pool_size = HV_AGX_SCANOUT_J313_POOL_SIZE;
}

bool hv_agx_scanout_broker_set_pool_size(struct hv_agx_scanout_broker *broker,
                                         uint64_t bytes)
{
    if (!broker || broker->state != HV_AGX_SCANOUT_UNREGISTERED ||
        !aligned(bytes) || bytes < HV_AGX_SCANOUT_J313_SURFACE_SIZE ||
        bytes > HV_AGX_SCANOUT_J313_POOL_SIZE)
        return false;
    broker->pool_size = bytes;
    return true;
}

void hv_agx_scanout_broker_init_v2(struct hv_agx_scanout_broker *broker,
                                   bool latch_source_proven)
{
    hv_agx_scanout_broker_init(broker);
    if (!broker)
        return;
    broker->abi_version = HV_AGX_SCANOUT_ABI_VERSION_V2;
    if (latch_source_proven)
        broker->capabilities |= HV_AGX_SCANOUT_V2_LATCH_CAPABILITIES;
}

static bool read_u32(const struct hv_agx_scanout_broker *broker, uint64_t offset,
                     uint64_t *value)
{
    switch (offset) {
    case HV_AGX_SCANOUT_REG_MAGIC:
        *value = HV_AGX_SCANOUT_MAGIC;
        break;
    case HV_AGX_SCANOUT_REG_ABI_VERSION:
        *value = broker->abi_version;
        break;
    case HV_AGX_SCANOUT_REG_CAPABILITIES:
        *value = broker->capabilities;
        break;
    case HV_AGX_SCANOUT_REG_STATE:
        *value = broker->state;
        break;
    case HV_AGX_SCANOUT_REG_RESULT:
        *value = broker->result;
        break;
    case HV_AGX_SCANOUT_REG_IRQ_STATUS:
        *value = broker->irq_status;
        break;
    case HV_AGX_SCANOUT_REG_IRQ_ENABLE:
        *value = broker->irq_enable;
        break;
    case HV_AGX_SCANOUT_REG_SWAP_ID:
        *value = broker->swap_id;
        break;
    case HV_AGX_SCANOUT_REG_WIDTH:
        *value = broker->staging_width;
        break;
    case HV_AGX_SCANOUT_REG_HEIGHT:
        *value = broker->staging_height;
        break;
    case HV_AGX_SCANOUT_REG_STRIDE:
        *value = broker->staging_stride;
        break;
    case HV_AGX_SCANOUT_REG_FORMAT:
        *value = broker->staging_format;
        break;
    default:
        return false;
    }
    return true;
}

static bool read_u64(const struct hv_agx_scanout_broker *broker, uint64_t offset,
                     uint64_t *value)
{
    switch (offset) {
    case HV_AGX_SCANOUT_REG_RECEIPT_SEQUENCE:
        *value = broker->receipt_sequence;
        break;
    case HV_AGX_SCANOUT_REG_APPLIED_SEQUENCE:
        *value = broker->applied_sequence;
        break;
    case HV_AGX_SCANOUT_REG_LATCHED_SEQUENCE:
        *value = broker->latched_sequence;
        break;
    case HV_AGX_SCANOUT_REG_ACTIVE_OFFSET:
        *value = broker->active_offset;
        break;
    case HV_AGX_SCANOUT_REG_POOL_PA:
        *value = broker->pool_pa;
        break;
    case HV_AGX_SCANOUT_REG_ACCEPTED_REQUESTS:
        *value = broker->accepted_requests;
        break;
    case HV_AGX_SCANOUT_REG_REJECTED_REQUESTS:
        *value = broker->rejected_requests;
        break;
    case HV_AGX_SCANOUT_REG_POOL_IPA:
        *value = broker->staging_pool_ipa;
        break;
    case HV_AGX_SCANOUT_REG_POOL_SIZE:
        *value = broker->staging_pool_size;
        break;
    case HV_AGX_SCANOUT_REG_SURFACE_OFFSET:
        *value = broker->staging_surface_offset;
        break;
    case HV_AGX_SCANOUT_REG_SURFACE_SIZE:
        *value = broker->staging_surface_size;
        break;
    case HV_AGX_SCANOUT_REG_REQUEST_SEQUENCE:
        *value = broker->staging_sequence;
        break;
    default:
        return false;
    }
    return true;
}

bool hv_agx_scanout_broker_mmio(struct hv_agx_scanout_broker *broker, uint64_t offset,
                                uint64_t *value, bool write, unsigned width)
{
    if (!broker || !value || offset >= HV_AGX_SCANOUT_MMIO_SIZE)
        return false;
    if (width == 2) {
        if (offset & 3)
            return false;
        if (!write)
            return read_u32(broker, offset, value);
        switch (offset) {
        case HV_AGX_SCANOUT_REG_IRQ_STATUS:
            broker->irq_status &= ~((uint32_t)*value & HV_AGX_SCANOUT_IRQ_MASK);
            if ((broker->irq_status & broker->irq_enable) == 0)
                broker->irq_edge_taken = false;
            return true;
        case HV_AGX_SCANOUT_REG_IRQ_ENABLE:
            broker->irq_enable = (uint32_t)*value & HV_AGX_SCANOUT_IRQ_MASK;
            if ((broker->irq_status & broker->irq_enable) == 0)
                broker->irq_edge_taken = false;
            return true;
        case HV_AGX_SCANOUT_REG_WIDTH:
            broker->staging_width = (uint32_t)*value;
            return true;
        case HV_AGX_SCANOUT_REG_HEIGHT:
            broker->staging_height = (uint32_t)*value;
            return true;
        case HV_AGX_SCANOUT_REG_STRIDE:
            broker->staging_stride = (uint32_t)*value;
            return true;
        case HV_AGX_SCANOUT_REG_FORMAT:
            broker->staging_format = (uint32_t)*value;
            return true;
        case HV_AGX_SCANOUT_REG_COMMAND:
            return submit(broker, (uint32_t)*value);
        default:
            return false;
        }
    }
    if (width == 3) {
        if (offset & 7)
            return false;
        if (!write)
            return read_u64(broker, offset, value);
        switch (offset) {
        case HV_AGX_SCANOUT_REG_POOL_IPA:
            broker->staging_pool_ipa = *value;
            return true;
        case HV_AGX_SCANOUT_REG_POOL_SIZE:
            broker->staging_pool_size = *value;
            return true;
        case HV_AGX_SCANOUT_REG_SURFACE_OFFSET:
            broker->staging_surface_offset = *value;
            return true;
        case HV_AGX_SCANOUT_REG_SURFACE_SIZE:
            broker->staging_surface_size = *value;
            return true;
        case HV_AGX_SCANOUT_REG_REQUEST_SEQUENCE:
            broker->staging_sequence = *value;
            return true;
        default:
            return false;
        }
    }
    return false;
}

bool hv_agx_scanout_broker_take_pending(struct hv_agx_scanout_broker *broker,
                                        struct hv_agx_scanout_request *request)
{
    if (!broker || !request ||
        (broker->state != HV_AGX_SCANOUT_PENDING &&
         broker->state != HV_AGX_SCANOUT_QUIESCING) ||
        broker->pending_taken)
        return false;
    *request = broker->pending;
    broker->pending_taken = true;
    return true;
}

static bool completion_matches(const struct hv_agx_scanout_broker *broker,
                               uint64_t sequence, uint32_t command)
{
    return broker && broker->pending_taken && broker->pending.Sequence == sequence &&
           broker->pending.Command == command;
}

bool hv_agx_scanout_broker_complete_register(struct hv_agx_scanout_broker *broker,
                                             uint64_t sequence,
                                             enum hv_agx_scanout_result result,
                                             uint64_t pool_pa, uint64_t pool_iova)
{
    if (!completion_matches(broker, sequence, HV_AGX_SCANOUT_CMD_REGISTER_POOL))
        return false;
    if (result == HV_AGX_SCANOUT_RESULT_OK && aligned(pool_pa) && pool_iova != 0 &&
        (pool_iova & (UINT64_C(0x4000) - 1)) == 0 &&
        !add_overflows(pool_pa, broker->pending.PoolSize) &&
        !add_overflows(pool_iova, broker->pending.PoolSize)) {
        broker->registered_pool_ipa = broker->pending.PoolIpa;
        broker->registered_pool_size = broker->pending.PoolSize;
        broker->pool_pa = pool_pa;
        broker->pool_iova = pool_iova;
        broker->state = HV_AGX_SCANOUT_READY;
        broker->applied_sequence = sequence;
    } else {
        if (result == HV_AGX_SCANOUT_RESULT_OK)
            result = HV_AGX_SCANOUT_RESULT_MAP_FAILED;
        broker->state = HV_AGX_SCANOUT_UNREGISTERED;
        broker->registered_pool_ipa = 0;
        broker->registered_pool_size = 0;
        broker->pool_pa = 0;
        broker->pool_iova = 0;
    }
    broker->result = result;
    broker->receipt_sequence = sequence;
    broker->pending_taken = false;
    return true;
}

bool hv_agx_scanout_broker_complete_present(struct hv_agx_scanout_broker *broker,
                                            uint64_t sequence,
                                            enum hv_agx_scanout_result result,
                                            uint32_t swap_id)
{
    if (!completion_matches(broker, sequence, HV_AGX_SCANOUT_CMD_PRESENT))
        return false;
    if (result == HV_AGX_SCANOUT_RESULT_OK && swap_id != 0) {
        broker->active_offset = broker->pending.SurfaceOffset;
        broker->swap_id = swap_id;
        broker->applied_sequence = sequence;
        broker->state = HV_AGX_SCANOUT_ACTIVE;
    } else {
        if (result == HV_AGX_SCANOUT_RESULT_OK)
            result = HV_AGX_SCANOUT_RESULT_PRESENT_FAILED;
        broker->state = broker->state_before_pending;
        broker->irq_status |= HV_AGX_SCANOUT_IRQ_ERROR;
    }
    broker->result = result;
    broker->receipt_sequence = sequence;
    broker->pending_taken = false;
    return true;
}

bool hv_agx_scanout_broker_mark_latched(struct hv_agx_scanout_broker *broker,
                                        uint64_t sequence)
{
    if (!broker || broker->abi_version != HV_AGX_SCANOUT_ABI_VERSION_V2 ||
        (broker->capabilities & HV_AGX_SCANOUT_V2_LATCH_CAPABILITIES) !=
            HV_AGX_SCANOUT_V2_LATCH_CAPABILITIES ||
        broker->state != HV_AGX_SCANOUT_ACTIVE || sequence == 0 ||
        sequence != broker->applied_sequence ||
        sequence <= broker->latched_sequence || broker->swap_id == 0)
        return false;
    broker->latched_sequence = sequence;
    broker->irq_status |= HV_AGX_SCANOUT_IRQ_LATCHED;
    return true;
}

bool hv_agx_scanout_broker_fail_latch(struct hv_agx_scanout_broker *broker,
                                      uint64_t sequence)
{
    if (!broker || broker->abi_version != HV_AGX_SCANOUT_ABI_VERSION_V2 ||
        broker->state != HV_AGX_SCANOUT_ACTIVE || sequence == 0 ||
        sequence != broker->applied_sequence ||
        sequence <= broker->latched_sequence)
        return false;
    broker->result = HV_AGX_SCANOUT_RESULT_PRESENT_FAILED;
    broker->irq_status |= HV_AGX_SCANOUT_IRQ_ERROR;
    return true;
}

bool hv_agx_scanout_broker_take_irq_edge(struct hv_agx_scanout_broker *broker)
{
    if (!broker || broker->irq_edge_taken ||
        (broker->irq_status & broker->irq_enable) == 0)
        return false;
    broker->irq_edge_taken = true;
    return true;
}

bool hv_agx_scanout_broker_complete_release(struct hv_agx_scanout_broker *broker,
                                            uint64_t sequence,
                                            enum hv_agx_scanout_result result,
                                            bool quiesce_proven)
{
    if (!completion_matches(broker, sequence, HV_AGX_SCANOUT_CMD_RELEASE))
        return false;
    /* Only the bounded service may prove that scanout has quiesced. */
    if (result == HV_AGX_SCANOUT_RESULT_OK && !quiesce_proven)
        result = HV_AGX_SCANOUT_RESULT_NOT_QUIESCED;
    if (result == HV_AGX_SCANOUT_RESULT_OK) {
        broker->state = HV_AGX_SCANOUT_UNREGISTERED;
        broker->active_offset = 0;
        broker->pool_pa = 0;
        broker->pool_iova = 0;
        broker->registered_pool_ipa = 0;
        broker->registered_pool_size = 0;
        broker->applied_sequence = sequence;
    } else {
        broker->state = broker->state_before_pending;
        broker->irq_status |= HV_AGX_SCANOUT_IRQ_ERROR;
    }
    broker->result = result;
    broker->receipt_sequence = sequence;
    broker->pending_taken = false;
    return true;
}

/* SPDX-License-Identifier: MIT */

#ifndef HV_AGX_SCANOUT_BROKER_H
#define HV_AGX_SCANOUT_BROKER_H

#include <stdbool.h>
#include <stdint.h>

#define HV_AGX_SCANOUT_MMIO_OFFSET UINT64_C(0x400)
#define HV_AGX_SCANOUT_MMIO_SIZE UINT64_C(0xa0)
#define HV_AGX_SCANOUT_MAGIC UINT32_C(0x53584741) /* "AGXS" */
#define HV_AGX_SCANOUT_ABI_VERSION_V1 1u
#define HV_AGX_SCANOUT_ABI_VERSION_V2 2u
#define HV_AGX_SCANOUT_ABI_VERSION HV_AGX_SCANOUT_ABI_VERSION_V1

#define HV_AGX_SCANOUT_CAP_FIXED_J313 (1u << 0)
#define HV_AGX_SCANOUT_CAP_BGRA8888 (1u << 1)
/* D3D-FS1: RGBA8888 surfaces scan out as DCP RGBA */
#define HV_AGX_SCANOUT_CAP_RGBA8888 (1u << 7)
#define HV_AGX_SCANOUT_CAP_REGISTERED_POOL (1u << 2)
#define HV_AGX_SCANOUT_CAP_APPLIED_RECEIPT (1u << 3)
#define HV_AGX_SCANOUT_CAP_REPEATED_PRESENT (1u << 4)
#define HV_AGX_SCANOUT_CAP_LATCHED_RECEIPT (1u << 5)
#define HV_AGX_SCANOUT_CAP_LATCHED_IRQ (1u << 6)
#define HV_AGX_SCANOUT_V2_LATCH_CAPABILITIES                                  \
    (HV_AGX_SCANOUT_CAP_REPEATED_PRESENT |                                   \
     HV_AGX_SCANOUT_CAP_LATCHED_RECEIPT | HV_AGX_SCANOUT_CAP_LATCHED_IRQ)

#define HV_AGX_SCANOUT_J313_WIDTH 2560u
#define HV_AGX_SCANOUT_J313_HEIGHT 1600u
#define HV_AGX_SCANOUT_J313_STRIDE 10240u
#define HV_AGX_SCANOUT_J313_SURFACE_SIZE UINT64_C(0xfa0000)
/* M2-f2c: the pool is segment 2 of the receipt-sized local reserve; this is
 * its J313 maximum and the default until the reserve sets the exact size. */
#define HV_AGX_SCANOUT_J313_POOL_SIZE UINT64_C(0x3b800000)
#define HV_AGX_SCANOUT_ALIGNMENT UINT64_C(0x10000)
#define HV_AGX_SCANOUT_FORMAT_BGRA8888 1u
#define HV_AGX_SCANOUT_FORMAT_RGBA8888 2u

#define HV_AGX_SCANOUT_IRQ_LATCHED (1u << 0)
#define HV_AGX_SCANOUT_IRQ_ERROR (1u << 1)
#define HV_AGX_SCANOUT_IRQ_MASK \
    (HV_AGX_SCANOUT_IRQ_LATCHED | HV_AGX_SCANOUT_IRQ_ERROR)

enum hv_agx_scanout_register {
    HV_AGX_SCANOUT_REG_MAGIC = 0x00,
    HV_AGX_SCANOUT_REG_ABI_VERSION = 0x04,
    HV_AGX_SCANOUT_REG_CAPABILITIES = 0x08,
    HV_AGX_SCANOUT_REG_STATE = 0x0c,
    HV_AGX_SCANOUT_REG_RESULT = 0x10,
    HV_AGX_SCANOUT_REG_IRQ_STATUS = 0x14,
    HV_AGX_SCANOUT_REG_IRQ_ENABLE = 0x18,
    HV_AGX_SCANOUT_REG_RECEIPT_SEQUENCE = 0x20,
    HV_AGX_SCANOUT_REG_APPLIED_SEQUENCE = 0x28,
    HV_AGX_SCANOUT_REG_LATCHED_SEQUENCE = 0x30,
    HV_AGX_SCANOUT_REG_ACTIVE_OFFSET = 0x38,
    HV_AGX_SCANOUT_REG_POOL_PA = 0x40,
    HV_AGX_SCANOUT_REG_SWAP_ID = 0x48,
    HV_AGX_SCANOUT_REG_ACCEPTED_REQUESTS = 0x50,
    HV_AGX_SCANOUT_REG_REJECTED_REQUESTS = 0x58,
    HV_AGX_SCANOUT_REG_POOL_IPA = 0x60,
    HV_AGX_SCANOUT_REG_POOL_SIZE = 0x68,
    HV_AGX_SCANOUT_REG_SURFACE_OFFSET = 0x70,
    HV_AGX_SCANOUT_REG_SURFACE_SIZE = 0x78,
    HV_AGX_SCANOUT_REG_WIDTH = 0x80,
    HV_AGX_SCANOUT_REG_HEIGHT = 0x84,
    HV_AGX_SCANOUT_REG_STRIDE = 0x88,
    HV_AGX_SCANOUT_REG_FORMAT = 0x8c,
    HV_AGX_SCANOUT_REG_REQUEST_SEQUENCE = 0x90,
    HV_AGX_SCANOUT_REG_COMMAND = 0x98,
};
enum hv_agx_scanout_command {
    HV_AGX_SCANOUT_CMD_QUERY = 0,
    HV_AGX_SCANOUT_CMD_REGISTER_POOL = 1,
    HV_AGX_SCANOUT_CMD_PRESENT = 2,
    HV_AGX_SCANOUT_CMD_RELEASE = 3,
};

enum hv_agx_scanout_state {
    HV_AGX_SCANOUT_UNREGISTERED = 0,
    HV_AGX_SCANOUT_READY,
    HV_AGX_SCANOUT_PENDING,
    HV_AGX_SCANOUT_ACTIVE,
    HV_AGX_SCANOUT_QUIESCING,
    HV_AGX_SCANOUT_FAILED,
};

enum hv_agx_scanout_result {
    HV_AGX_SCANOUT_RESULT_OK = 0,
    HV_AGX_SCANOUT_RESULT_INVALID_COMMAND,
    HV_AGX_SCANOUT_RESULT_STALE_SEQUENCE,
    HV_AGX_SCANOUT_RESULT_BUSY,
    HV_AGX_SCANOUT_RESULT_INVALID_POOL,
    HV_AGX_SCANOUT_RESULT_UNMAPPED,
    HV_AGX_SCANOUT_RESULT_NOT_RAM,
    HV_AGX_SCANOUT_RESULT_NONCONTIGUOUS,
    HV_AGX_SCANOUT_RESULT_INVALID_SURFACE,
    HV_AGX_SCANOUT_RESULT_MAP_FAILED,
    HV_AGX_SCANOUT_RESULT_PRESENT_FAILED,
    HV_AGX_SCANOUT_RESULT_NOT_QUIESCED,
};

struct hv_agx_scanout_request {
    uint32_t Command;
    uint32_t Width;
    uint32_t Height;
    uint32_t Stride;
    uint32_t Format;
    uint64_t Sequence;
    uint64_t PoolIpa;
    uint64_t PoolSize;
    uint64_t SurfaceOffset;
    uint64_t SurfaceSize;
};

struct hv_agx_scanout_broker {
    uint32_t abi_version;
    uint32_t capabilities;
    enum hv_agx_scanout_state state;
    enum hv_agx_scanout_state state_before_pending;
    enum hv_agx_scanout_result result;
    uint32_t irq_status;
    uint32_t irq_enable;
    uint32_t swap_id;
    uint64_t receipt_sequence;
    uint64_t applied_sequence;
    uint64_t latched_sequence;
    uint64_t accepted_requests;
    uint64_t rejected_requests;
    uint64_t highest_sequence;
    uint64_t active_offset;
    uint64_t pool_pa;
    uint64_t pool_iova;
    uint64_t registered_pool_ipa;
    uint64_t registered_pool_size;
    /* The only pool size REGISTER_POOL accepts. */
    uint64_t pool_size;
    uint64_t staging_pool_ipa;
    uint64_t staging_pool_size;
    uint64_t staging_surface_offset;
    uint64_t staging_surface_size;
    uint32_t staging_width;
    uint32_t staging_height;
    uint32_t staging_stride;
    uint32_t staging_format;
    uint64_t staging_sequence;
    struct hv_agx_scanout_request pending;
    bool pending_taken;
    bool irq_edge_taken;
};

void hv_agx_scanout_broker_init(struct hv_agx_scanout_broker *broker);
/* Sets the pool size a guest must register (segment 2 of the local reserve):
 * aligned, at least one surface, at most the J313 maximum, only while no pool
 * is registered. */
bool hv_agx_scanout_broker_set_pool_size(struct hv_agx_scanout_broker *broker,
                                         uint64_t bytes);
void hv_agx_scanout_broker_init_v2(struct hv_agx_scanout_broker *broker,
                                   bool latch_source_proven);
bool hv_agx_scanout_broker_mmio(struct hv_agx_scanout_broker *broker, uint64_t offset,
                                uint64_t *value, bool write, unsigned width);
bool hv_agx_scanout_broker_take_pending(struct hv_agx_scanout_broker *broker,
                                        struct hv_agx_scanout_request *request);
bool hv_agx_scanout_broker_complete_register(struct hv_agx_scanout_broker *broker,
                                             uint64_t sequence,
                                             enum hv_agx_scanout_result result,
                                             uint64_t pool_pa, uint64_t pool_iova);
bool hv_agx_scanout_broker_complete_present(struct hv_agx_scanout_broker *broker,
                                            uint64_t sequence,
                                            enum hv_agx_scanout_result result,
                                            uint32_t swap_id);
bool hv_agx_scanout_broker_mark_latched(struct hv_agx_scanout_broker *broker,
                                        uint64_t sequence);
bool hv_agx_scanout_broker_fail_latch(struct hv_agx_scanout_broker *broker,
                                      uint64_t sequence);
bool hv_agx_scanout_broker_take_irq_edge(
    struct hv_agx_scanout_broker *broker);
bool hv_agx_scanout_broker_complete_release(struct hv_agx_scanout_broker *broker,
                                            uint64_t sequence,
                                            enum hv_agx_scanout_result result,
                                            bool quiesce_proven);

#endif

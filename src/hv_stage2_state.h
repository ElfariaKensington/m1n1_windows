/* SPDX-License-Identifier: MIT */

#ifndef HV_STAGE2_STATE_H
#define HV_STAGE2_STATE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define HV_STAGE2_STATE_MAX_MAPPINGS 64

enum hv_stage2_mapping_kind {
    HV_STAGE2_MAPPING_UNMAP,
    HV_STAGE2_MAPPING_HARDWARE,
    HV_STAGE2_MAPPING_SOFTWARE,
    HV_STAGE2_MAPPING_HOOK,
};

struct hv_stage2_mapping {
    uint64_t ipa;
    uint64_t pa;
    uint64_t size;
    uint64_t increment;
    uint32_t kind;
};

void hv_stage2_state_reset(void);
bool hv_stage2_state_record(uint64_t ipa, uint64_t pa, uint64_t size, uint64_t increment,
                            uint32_t kind);
bool hv_stage2_state_snapshot(struct hv_stage2_mapping *out, size_t capacity, size_t *count);
bool hv_stage2_state_matches(const struct hv_stage2_mapping *expected, size_t count);

#endif

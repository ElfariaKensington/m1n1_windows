/* SPDX-License-Identifier: MIT */

#include "hv_stage2_state.h"

static struct hv_stage2_mapping observed[HV_STAGE2_STATE_MAX_MAPPINGS];
static size_t observed_count;
static bool observed_overflow;

void hv_stage2_state_reset(void)
{
    observed_count = 0;
    observed_overflow = false;
}

bool hv_stage2_state_record(uint64_t ipa, uint64_t pa, uint64_t size, uint64_t increment,
                            uint32_t kind)
{
    if (!size || kind > HV_STAGE2_MAPPING_HOOK || observed_overflow ||
        observed_count >= HV_STAGE2_STATE_MAX_MAPPINGS) {
        observed_overflow = true;
        return false;
    }

    observed[observed_count++] = (struct hv_stage2_mapping){
        .ipa = ipa,
        .pa = pa,
        .size = size,
        .increment = increment,
        .kind = kind,
    };
    return true;
}

bool hv_stage2_state_snapshot(struct hv_stage2_mapping *out, size_t capacity, size_t *count)
{
    if (!out || !count || observed_overflow || observed_count > capacity)
        return false;

    for (size_t i = 0; i < observed_count; i++)
        out[i] = observed[i];
    *count = observed_count;
    return true;
}

bool hv_stage2_state_matches(const struct hv_stage2_mapping *expected, size_t count)
{
    if (!expected || observed_overflow || count != observed_count)
        return false;

    for (size_t index = 0; index < count; index++) {
        const struct hv_stage2_mapping *actual = &observed[index];
        const struct hv_stage2_mapping *allowed = &expected[index];
        if (actual->ipa != allowed->ipa || actual->pa != allowed->pa ||
            actual->size != allowed->size || actual->increment != allowed->increment ||
            actual->kind != allowed->kind)
            return false;
    }
    return true;
}

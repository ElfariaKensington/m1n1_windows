#include <assert.h>
#include <stdio.h>

#include "../src/hv_stage2_state.h"

int main(void)
{
    struct hv_stage2_mapping mappings[HV_STAGE2_STATE_MAX_MAPPINGS];
    size_t count = 0;

    hv_stage2_state_reset();
    assert(
        hv_stage2_state_record(0x100000, 0x8a0100000ULL, 0x200000, 1, HV_STAGE2_MAPPING_HARDWARE));
    assert(hv_stage2_state_record(0x690000000ULL, 0, 0x100000, 0, HV_STAGE2_MAPPING_HOOK));
    assert(hv_stage2_state_snapshot(mappings, HV_STAGE2_STATE_MAX_MAPPINGS, &count));
    assert(count == 2);
    assert(mappings[0].ipa == 0x100000);
    assert(mappings[0].pa == 0x8a0100000ULL);
    assert(mappings[0].size == 0x200000);
    assert(mappings[0].increment == 1);
    assert(mappings[1].kind == HV_STAGE2_MAPPING_HOOK);
    assert(hv_stage2_state_matches(mappings, count));
    assert(!hv_stage2_state_matches(mappings, count - 1));
    mappings[0].pa += 0x4000;
    assert(!hv_stage2_state_matches(mappings, count));
    mappings[0].pa -= 0x4000;
    assert(hv_stage2_state_record(0x393cc0000ULL, 0x393cc0000ULL, 0x40000, 1,
                                  HV_STAGE2_MAPPING_HARDWARE));
    assert(!hv_stage2_state_matches(mappings, count));

    assert(!hv_stage2_state_snapshot(mappings, 1, &count));
    assert(!hv_stage2_state_record(0, 0, 0, 0, HV_STAGE2_MAPPING_UNMAP));

    hv_stage2_state_reset();
    for (size_t i = 0; i < HV_STAGE2_STATE_MAX_MAPPINGS; i++)
        assert(
            hv_stage2_state_record(i * 0x4000, i * 0x4000, 0x4000, 1, HV_STAGE2_MAPPING_HARDWARE));
    assert(!hv_stage2_state_record(0, 0, 0x4000, 0, HV_STAGE2_MAPPING_UNMAP));
    assert(!hv_stage2_state_snapshot(mappings, HV_STAGE2_STATE_MAX_MAPPINGS, &count));

    puts("hv_stage2_state_test: ok");
    return 0;
}

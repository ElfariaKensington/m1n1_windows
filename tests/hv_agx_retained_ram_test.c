/* SPDX-License-Identifier: MIT */
/* WHAT REAL BUG OR INVARIANT WILL THIS TEST CATCH?
 * A lower published guest RAM base must be usable by the broker without
 * admitting native heap pages, overflowed RAM or a mismatched RAM region. */
#include "../src/hv_agx_retained_backing.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    struct hv_contract_snapshot s = {0};
    const uint64_t floor = 0x810000000, top = 0x9df708000, entry = 0x8510b4000;
    s.boot.ram_base = 0x818000000;
    s.boot.ram_size = top - s.boot.ram_base;
    s.boot.guest_entry = entry;
    s.region_count = 2;
    s.regions[0] = (struct hv_contract_region){HV_CONTRACT_REGION_GUEST_RAM, 0,
                                              s.boot.ram_base, s.boot.ram_size};
    s.regions[1] = (struct hv_contract_region){HV_CONTRACT_REGION_FRAMEBUFFER, 0,
                                              0x85f000000, 0xfa0000};
    assert(hv_agx_retained_ram_valid(&s, floor, top, entry));
    assert(hv_agx_retained_backing_allowed(&s, 0x818000000, 0x4000));
    assert(!hv_agx_retained_backing_allowed(&s, floor, 0x4000));
    assert(!hv_agx_retained_backing_allowed(&s, 0x85f000000, 0x4000));
    assert(!hv_agx_retained_ram_valid(&s, s.boot.ram_base + 1, top, entry));
    assert(!hv_agx_retained_ram_valid(&s, floor, top - 1, entry));
    assert(!hv_agx_retained_ram_valid(&s, floor, top, entry + 0x4000));
    s.regions[0].size -= 0x4000;
    assert(!hv_agx_retained_ram_valid(&s, floor, top, entry));
    s.regions[0].size += 0x4000;
    s.boot.ram_size = UINT64_MAX;
    assert(!hv_agx_retained_ram_valid(&s, floor, top, entry));
    s.boot.ram_size = 0;
    assert(!hv_agx_retained_ram_valid(&s, floor, top, entry));
    s.boot.ram_base = 0x850000000;
    s.boot.ram_size = top - s.boot.ram_base;
    s.regions[0].base = s.boot.ram_base;
    s.regions[0].size = s.boot.ram_size;
    assert(hv_agx_retained_ram_valid(&s, floor, top, entry));
    s.region_count = HV_CONTRACT_MAX_REGIONS + 1;
    assert(!hv_agx_retained_ram_valid(&s, floor, top, entry));
    assert(!hv_agx_retained_ram_valid(NULL, floor, top, entry));
    puts("hv_agx_retained_ram_test: PASS");
}

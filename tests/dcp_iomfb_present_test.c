/* SPDX-License-Identifier: MIT */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "../src/dcp_iomfb_present.h"
static uint32_t u32at(const uint8_t *p) { uint32_t v; memcpy(&v, p, 4); return v; }
static uint64_t u64at(const uint8_t *p) { uint64_t v; memcpy(&v, p, 8); return v; }
static unsigned count_u32(const uint8_t *p, size_t n, uint32_t value)
{
    unsigned found = 0;
    for (size_t i = 0; i + 4 <= n; ++i)
        found += u32at(p + i) == value;
    return found;
}
int main(void)
{
    struct dcp_iomfb_present_request req;
    uint8_t start_input[0x18];
    uint8_t start[0x18] = {0}, submit[0xc] = {0};
    uint32_t swap_id = 0;
    assert(!dcp_iomfb_present_build_v13_5(&req, 0x100000, 2560, 1600, 10176, DCP_IOMFB_FORMAT_BGRA, true));
    assert(!dcp_iomfb_present_build_v13_5(&req, 0x100000, UINT32_MAX, 2, UINT32_MAX, DCP_IOMFB_FORMAT_BGRA, true));
    assert(dcp_iomfb_present_build_v13_5(&req, 0x12340000, 2560, 1600, 10240, DCP_IOMFB_FORMAT_BGRA, true));
    assert(u64at(req.bytes + 0x40) == 0x861202);
    assert(u64at(req.bytes + 0x48) == 4);
    assert(u32at(req.bytes + 0x54) == 3);
    assert(u32at(req.bytes + 0xa4) == 1);
    assert(u32at(req.bytes + 0x104) == 0x80000007u);
    assert(u32at(req.bytes + 0x108) == 0x80000007u);
    assert(u32at(req.bytes + 0x10c) == 0);
    assert(u64at(req.bytes + 0x2e7) == 1);
    assert(u32at(req.bytes + 0x2ef) == 0x58f058d0);
    assert(req.bytes[0x2f3] == 0x40);
    assert(u32at(req.bytes + 0x64 + 8) == 2560);
    assert(u32at(req.bytes + 0x64 + 12) == 1600);
    assert(req.bytes[0x468 + 0] == 0);
    assert(u32at(req.bytes + 0x468 + 3) == 0);
    assert(u32at(req.bytes + 0x468 + 7) == 0);
    assert(req.bytes[0x468 + 0x13] == 13);
    assert(req.bytes[0x468 + 0x14] == 1);
    assert(u32at(req.bytes + 0x468 + 0x15) == 10240);
    assert(req.bytes[0x468 + 0x19] == 4);
    assert(u32at(req.bytes + 0x468 + 0x35) == 3);
    assert(u64at(req.bytes + 0x468 + 0x51) == 1);
    assert(u64at(req.bytes + 0x468 + 0x149) == 1);
    assert(u64at(req.bytes + 0xd18) == 0x12340000);
    assert(req.bytes[0x1876] == 0 && req.bytes[0x1877] == 0);
    assert(req.bytes[0x1878] == 1 && req.bytes[0x187b] == 1);
    assert(req.bytes[0x1880] == 0 && req.bytes[0x1881] == 1);
    assert(req.bytes[0x1882] == 1 && req.bytes[0x1883] == 0);
    assert(dcp_iomfb_present_build_start_v13_5(start_input,
                                                sizeof(start_input)));
    assert(u32at(start_input) == 0);
    assert(u64at(start_input + 4) == 0xfffffe1667ba4a00ull);
    assert(u32at(start_input + 12) == 0);
    assert(start_input[16] == 0 && start_input[17] == 1);
    for (unsigned int i = 18; i < sizeof(start_input); ++i)
        assert(start_input[i] == 0);
    assert(dcp_iomfb_present_build_v13_5(&req, 0x12340000, 2560, 1600, 10240, DCP_IOMFB_FORMAT_BGRA, false));
    assert(u32at(req.bytes + 0x104) == 1 && u32at(req.bytes + 0x108) == 1);
    assert(u32at(req.bytes + 0x10c) == 0);
    /* D3D-FS1: the surface fourcc follows the requested format (Asahi
     * iomfb_plane.h DCP_FORMAT_BGRA 'ARGB', DCP_FORMAT_RGBA 'ABGR'). */
    assert(count_u32(req.bytes, sizeof(req.bytes), 0x42475241u) == 1);
    assert(count_u32(req.bytes, sizeof(req.bytes), 0x52474241u) == 0);
    assert(dcp_iomfb_present_build_v13_5(&req, 0x12340000, 2560, 1600, 10240, DCP_IOMFB_FORMAT_RGBA, false));
    assert(count_u32(req.bytes, sizeof(req.bytes), 0x52474241u) == 1);
    assert(count_u32(req.bytes, sizeof(req.bytes), 0x42475241u) == 0);
    assert(!dcp_iomfb_present_build_v13_5(&req, 0x12340000, 2560, 1600, 10240, 0x30313272u, false));
    assert(dcp_iomfb_present_build_v13_5(&req, 0x12340000, 2560, 1600, 10240, DCP_IOMFB_FORMAT_BGRA, false));
    assert(dcp_iomfb_present_set_swap_id_v13_5(&req, 0x42));
    assert(u32at(req.bytes + 0x50) == 0x42);
    memcpy(start, &(uint32_t){0x42}, 4);
    assert(dcp_iomfb_present_parse_start_v13_5(start, sizeof(start), &swap_id));
    assert(swap_id == 0x42);
    memcpy(start + 0x14, &(uint32_t){1}, 4);
    assert(!dcp_iomfb_present_parse_start_v13_5(start, sizeof(start), &swap_id));
    assert(dcp_iomfb_present_parse_submit_v13_5(submit, sizeof(submit)));
    memcpy(submit + 5, &(uint32_t){1}, 4);
    assert(!dcp_iomfb_present_parse_submit_v13_5(submit, sizeof(submit)));
    puts("dcp_iomfb_present_test: ok");
    return 0;
}

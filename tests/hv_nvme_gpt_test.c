#include <assert.h>
#include <string.h>

#include "../src/hv_nvme_gpt.h"
#include "../src/tinf/tinf.h"

#define BLOCK HV_NVME_GPT_BLOCK_SIZE
#define DISK_BLOCKS 20000000ULL
#define SLICE_START 4096ULL
#define SLICE_END 18000000ULL

static const u8 windows_type[] = {
    0xb9, 0xc4, 0x09, 0x05, 0x59, 0xac, 0xcd, 0x4c,
    0x93, 0xf3, 0x8d, 0xec, 0xb1, 0x7f, 0x35, 0xe3,
};
static u8 headers[2][BLOCK];
static u8 tables[2][4 * BLOCK];
static u8 scratch[HV_NVME_GPT_SCRATCH_SIZE];

static void set32(u8 *p, u32 value)
{
    for (unsigned i = 0; i < 4; i++)
        p[i] = value >> (8 * i);
}

static void set64(u8 *p, u64 value)
{
    set32(p, (u32)value);
    set32(p + 4, (u32)(value >> 32));
}

static void build_header(unsigned copy)
{
    u8 *h = headers[copy];
    memset(h, 0, BLOCK);
    memcpy(h, "EFI PART", 8);
    set32(h + 8, 0x10000);
    set32(h + 12, 92);
    set64(h + 24, copy ? DISK_BLOCKS - 1 : 1);
    set64(h + 32, copy ? 1 : DISK_BLOCKS - 1);
    set64(h + 40, 6);
    set64(h + 48, DISK_BLOCKS - 6);
    h[56] = 1;
    set64(h + 72, copy ? DISK_BLOCKS - 5 : 2);
    set32(h + 80, 128);
    set32(h + 84, 128);
    set32(h + 88, tinf_crc32(tables[copy], sizeof(tables[copy])));
    set32(h + 16, tinf_crc32(h, 92));
}

static bool read_disk(u64 lba, void *block)
{
    if (lba == 1 || lba == DISK_BLOCKS - 1)
        memcpy(block, headers[lba != 1], BLOCK);
    else if (lba >= 2 && lba < 6)
        memcpy(block, tables[0] + (lba - 2) * BLOCK, BLOCK);
    else if (lba >= DISK_BLOCKS - 5 && lba < DISK_BLOCKS - 1)
        memcpy(block, tables[1] + (lba - (DISK_BLOCKS - 5)) * BLOCK, BLOCK);
    else
        return false;
    return true;
}

static void mirror_gpt(void)
{
    memcpy(tables[1], tables[0], sizeof(tables[0]));
    build_header(0);
    build_header(1);
}

int main(void)
{
    struct hv_nvme_disk_slice slice;
    u64 physical;
    u8 *entry = tables[0];
    memcpy(entry, windows_type, sizeof(windows_type));
    entry[16] = 2;
    set64(entry + 32, SLICE_START);
    set64(entry + 40, SLICE_END);
    mirror_gpt();

    assert(hv_nvme_gpt_find_windows(DISK_BLOCKS, read_disk, scratch, &slice));
    assert(hv_nvme_slice_translate(&slice, 0, 1, &physical) && physical == SLICE_START);
    assert(hv_nvme_slice_translate(&slice, slice.blocks - 1, 1, &physical) &&
           physical == SLICE_END);
    assert(!hv_nvme_slice_translate(&slice, slice.blocks - 1, 2, &physical));
    assert(!hv_nvme_slice_translate(&slice, UINT64_MAX, 1, &physical));
    assert(!hv_nvme_slice_translate(&slice, 0, 0, &physical));

    headers[1][16] ^= 1;
    assert(!hv_nvme_gpt_find_windows(DISK_BLOCKS, read_disk, scratch, &slice));
    mirror_gpt();
    set64(tables[1] + 40, SLICE_END + 1);
    build_header(1);
    assert(!hv_nvme_gpt_find_windows(DISK_BLOCKS, read_disk, scratch, &slice));

    u8 *overlap = tables[0] + 128;
    overlap[0] = 1;
    overlap[16] = 3;
    set64(overlap + 32, 6);
    set64(overlap + 40, SLICE_START);
    mirror_gpt();
    assert(!hv_nvme_gpt_find_windows(DISK_BLOCKS, read_disk, scratch, &slice));
    memset(overlap, 0, 128);
    set64(entry + 40, SLICE_START + 100);
    mirror_gpt();
    assert(!hv_nvme_gpt_find_windows(DISK_BLOCKS, read_disk, scratch, &slice));
    entry[0] ^= 1;
    mirror_gpt();
    assert(!hv_nvme_gpt_find_windows(DISK_BLOCKS, read_disk, scratch, &slice));
    return 0;
}

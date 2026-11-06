/* SPDX-License-Identifier: MIT */

#include "hv_nvme_gpt.h"
#ifdef HV_NVME_GPT_HOST_TEST
#include <string.h>
#else
#include "string.h"
#endif
#include "tinf/tinf.h"

/* 0509C4B9-AC59-4CCD-93F3-8DECB17F35E3, in GPT byte order. Only a partition
 * deliberately provisioned with this type may become a nested Windows disk. */
static const u8 windows_type[16] = {
    0xb9, 0xc4, 0x09, 0x05, 0x59, 0xac, 0xcd, 0x4c,
    0x93, 0xf3, 0x8d, 0xec, 0xb1, 0x7f, 0x35, 0xe3,
};
static const u8 empty_guid[16];

struct gpt_header {
    u8 signature[8];
    u32 revision;
    u32 size;
    u32 crc;
    u32 reserved;
    u64 current;
    u64 alternate;
    u64 first;
    u64 last;
    u8 disk_guid[16];
    u64 table;
    u32 entries;
    u32 entry_size;
    u32 table_crc;
} __attribute__((packed));

struct gpt_entry {
    u8 type[16];
    u8 guid[16];
    u64 first;
    u64 last;
    u64 attributes;
    u8 name[72];
} __attribute__((packed));

static bool read_table(u64 disk_blocks, bool backup, hv_nvme_gpt_read_fn read,
                       u8 *block, u8 *entries, struct gpt_header *header)
{
    u64 current = backup ? disk_blocks - 1 : 1;
    u64 alternate = backup ? 1 : disk_blocks - 1;
    if (!read(current, block))
        return false;
    memcpy(header, block, sizeof(*header));
    if (memcmp(header->signature, "EFI PART", 8) || header->revision != 0x10000 ||
        header->size < sizeof(*header) || header->size > HV_NVME_GPT_BLOCK_SIZE ||
        header->reserved || header->current != current || header->alternate != alternate ||
        header->first <= 2 || header->first > header->last || header->last >= disk_blocks - 2 ||
        !memcmp(header->disk_guid, empty_guid, sizeof(empty_guid)) ||
        header->entry_size != sizeof(struct gpt_entry) ||
        header->entries < 128 || header->entries > HV_NVME_GPT_TABLE_SIZE / sizeof(struct gpt_entry))
        return false;
    memset(block + __builtin_offsetof(struct gpt_header, crc), 0, sizeof(header->crc));
    if (tinf_crc32(block, header->size) != header->crc)
        return false;

    u32 size = header->entries * header->entry_size;
    u32 count = (size + HV_NVME_GPT_BLOCK_SIZE - 1) / HV_NVME_GPT_BLOCK_SIZE;
    u64 low = backup ? header->last + 1 : 2;
    u64 high = backup ? current : header->first;
    if (header->table < low || header->table >= high || count > high - header->table)
        return false;
    for (u32 i = 0; i < count; i++)
        if (!read(header->table + i, entries + i * HV_NVME_GPT_BLOCK_SIZE))
            return false;
    return tinf_crc32(entries, size) == header->table_crc;
}

bool hv_nvme_gpt_find_windows_minimum(u64 disk_blocks, hv_nvme_gpt_read_fn read,
                                      void *scratch, struct hv_nvme_disk_slice *slice,
                                      u64 minimum_blocks)
{
    if (!slice)
        return false;
    *slice = (struct hv_nvme_disk_slice){0};
    if (!scratch || !read || disk_blocks < 16)
        return false;
    u8 *block = scratch;
    u8 *primary = block + HV_NVME_GPT_BLOCK_SIZE;
    u8 *backup = primary + HV_NVME_GPT_TABLE_SIZE;
    struct gpt_header first, last;
    if (!read_table(disk_blocks, false, read, block, primary, &first) ||
        !read_table(disk_blocks, true, read, block, backup, &last) ||
        first.first != last.first || first.last != last.last ||
        first.entries != last.entries || first.table_crc != last.table_crc ||
        memcmp(first.disk_guid, last.disk_guid, sizeof(first.disk_guid)) ||
        memcmp(primary, backup, first.entries * first.entry_size))
        return false;

    const struct gpt_entry *entries = (const void *)primary;
    struct hv_nvme_disk_slice candidate = {0};
    for (u32 i = 0; i < first.entries; i++) {
        const struct gpt_entry *entry = &entries[i];
        if (!memcmp(entry->type, empty_guid, sizeof(empty_guid)))
            continue;
        if (!memcmp(entry->guid, empty_guid, sizeof(empty_guid)) ||
            entry->first < first.first || entry->last > first.last || entry->first > entry->last)
            return false;
        for (u32 j = 0; j < i; j++) {
            const struct gpt_entry *other = &entries[j];
            if (memcmp(other->type, empty_guid, sizeof(empty_guid)) &&
                (entry->first <= other->last && other->first <= entry->last))
                return false;
        }
        if (!memcmp(entry->type, windows_type, sizeof(windows_type))) {
            u64 blocks = entry->last - entry->first + 1;
            if (candidate.blocks || !minimum_blocks || blocks < minimum_blocks || entry->attributes)
                return false;
            candidate = (struct hv_nvme_disk_slice){entry->first, blocks};
        }
    }
    if (!candidate.blocks)
        return false;
    *slice = candidate;
    return true;
}

bool hv_nvme_gpt_find_windows(u64 disk_blocks, hv_nvme_gpt_read_fn read,
                             void *scratch, struct hv_nvme_disk_slice *slice)
{
    return hv_nvme_gpt_find_windows_minimum(
        disk_blocks, read, scratch, slice, (64ULL << 30) / HV_NVME_GPT_BLOCK_SIZE);
}

bool hv_nvme_slice_translate(const struct hv_nvme_disk_slice *slice, u64 lba,
                             u32 blocks, u64 *physical)
{
    if (!physical || !slice || !blocks || lba >= slice->blocks || blocks > slice->blocks - lba ||
        slice->first > UINT64_MAX - (lba + blocks - 1))
        return false;
    *physical = slice->first + lba;
    return true;
}

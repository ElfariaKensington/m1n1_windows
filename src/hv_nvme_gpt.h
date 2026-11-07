/* SPDX-License-Identifier: MIT */
#ifndef HV_NVME_GPT_H
#define HV_NVME_GPT_H

#ifdef HV_NVME_GPT_HOST_TEST
#include <stdbool.h>
#include <stdint.h>
typedef uint8_t u8;
typedef uint32_t u32;
typedef uint64_t u64;
#else
#include "types.h"
#endif

#define HV_NVME_GPT_BLOCK_SIZE 4096u
#define HV_NVME_GPT_TABLE_SIZE (128u * 1024)
#define HV_NVME_GPT_SCRATCH_SIZE (HV_NVME_GPT_BLOCK_SIZE + 2 * HV_NVME_GPT_TABLE_SIZE)
/* A separate role from the pre-existing diagnostics container. */
#define HV_NVME_J316S_INSTALL_GUID "CBAE986F-5E92-48D2-AEF2-326CB78B01EB"

struct hv_nvme_disk_slice {
    u64 first;
    u64 blocks;
};

typedef bool (*hv_nvme_gpt_read_fn)(u64 lba, void *block);

bool hv_nvme_gpt_find_windows(u64 disk_blocks, hv_nvme_gpt_read_fn read,
                             void *scratch, struct hv_nvme_disk_slice *slice);
bool hv_nvme_gpt_find_j316s_install(u64 disk_blocks, hv_nvme_gpt_read_fn read,
                                    void *scratch, struct hv_nvme_disk_slice *slice);
bool hv_nvme_gpt_find_windows_minimum(u64 disk_blocks, hv_nvme_gpt_read_fn read,
                                      void *scratch, struct hv_nvme_disk_slice *slice,
                                      u64 minimum_blocks);
bool hv_nvme_slice_translate(const struct hv_nvme_disk_slice *slice, u64 lba,
                             u32 blocks, u64 *physical);

#endif

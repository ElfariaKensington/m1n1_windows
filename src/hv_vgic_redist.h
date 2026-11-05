/* SPDX-License-Identifier: MIT */

#ifndef HV_VGIC_REDIST_H
#define HV_VGIC_REDIST_H

#ifdef HV_VGIC_REDIST_HOST_TEST
#include <stdbool.h>
#include <stdint.h>
typedef uint32_t u32;
typedef uint64_t u64;
#else
#include "types.h"
#endif

#define HV_VGIC_REDIST_STRIDE 0x20000ULL
#define HV_VGIC_T6000_CPU_COUNT 10

struct hv_vgic_redist_addr {
    u32 cpu;
    u64 reg;
};

bool hv_vgic_redist_decode(u64 addr, u64 base, u32 num_cpus,
                           struct hv_vgic_redist_addr *out);
bool hv_vgic_t6000_topology_matches(const u64 *mpidrs, u32 num_cpus);

#endif

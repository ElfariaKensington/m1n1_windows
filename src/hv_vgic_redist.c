/* SPDX-License-Identifier: MIT */

#include "hv_vgic_redist.h"

bool hv_vgic_t6000_topology_matches(const u64 *mpidrs, u32 num_cpus)
{
    /* Guest UIDs follow m1n1 CPU indices: two Icestorm cores, then two
     * four-core Firestorm clusters. MPIDR Aff2 marks Firestorm. */
    static const u64 expected[HV_VGIC_T6000_CPU_COUNT] = {
        0x00000, 0x00001,
        0x10100, 0x10101, 0x10102, 0x10103,
        0x10200, 0x10201, 0x10202, 0x10203,
    };

    if (!mpidrs || num_cpus != HV_VGIC_T6000_CPU_COUNT)
        return false;

    for (u32 cpu = 0; cpu < num_cpus; cpu++)
        if (mpidrs[cpu] != expected[cpu])
            return false;

    return true;
}

bool hv_vgic_redist_decode(u64 addr, u64 base, u32 num_cpus,
                           struct hv_vgic_redist_addr *out)
{
    if (!out || !num_cpus || addr < base)
        return false;

    u64 offset = addr - base;
    u64 cpu = offset / HV_VGIC_REDIST_STRIDE;
    if (cpu >= num_cpus)
        return false;

    out->cpu = cpu;
    out->reg = offset % HV_VGIC_REDIST_STRIDE;
    return true;
}

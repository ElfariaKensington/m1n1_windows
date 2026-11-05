/* SPDX-License-Identifier: MIT */

#ifndef HV_IRQ_ROUTES_H
#define HV_IRQ_ROUTES_H

#ifdef HV_IRQ_ROUTES_HOST_TEST
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
typedef uint32_t u32;
#else
#include "types.h"
#endif

#include "hv_agx_g2.generated.h"

struct hv_irq_route {
    u32 hw_irq;
    u32 vintid;
    bool level;
};

#define HV_IRQ_DYNAMIC_ROUTE_CAPACITY (HV_AGX_G2_INTERRUPT_ROUTE_COUNT + 1u)
#define HV_IRQ_SYNTHETIC_NVME_VINTID 64u

const struct hv_irq_route *hv_irq_route_from_hw(u32 hw_irq);
const struct hv_irq_route *hv_irq_route_from_vintid(u32 vintid);
size_t hv_irq_route_count(void);
const struct hv_irq_route *hv_irq_route_at(size_t index);
bool hv_irq_route_resolve_incoming(u32 hw_irq, u32 reserved_vintid, u32 *vintid);
bool hv_irq_route_level_eoi_target(u32 vintid, bool enabled, u32 *hw_irq);
bool hv_irq_route_register(u32 hw_irq, u32 vintid, bool level);
bool hv_irq_routes_register_agx_g2(void);
void hv_irq_routes_reset(bool enable_j313_routes);

#endif

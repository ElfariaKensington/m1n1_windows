/* SPDX-License-Identifier: MIT */

#include "hv_irq_routes.h"

#define STATIC_ROUTE_COUNT (sizeof(routes) / sizeof(routes[0]))

/*
 * Physical AIC numbers and guest GIC INTIDs are separate namespaces. Keep the
 * handoff explicit even when a route currently uses the same number in both.
 * usb-drd1 is the Type-C port not occupied by the m1n1 proxy on J313.
 */
static const struct hv_irq_route routes[] = {
    {.hw_irq = 857, .vintid = 857, .level = true},
};

static const struct hv_agx_g2_interrupt_route agx_g2_routes[] =
    HV_AGX_G2_INTERRUPT_ROUTE_VALUES;

static struct hv_irq_route dynamic_routes[HV_IRQ_DYNAMIC_ROUTE_CAPACITY];
static size_t dynamic_route_count;
static size_t static_route_count;

_Static_assert(sizeof(agx_g2_routes) / sizeof(agx_g2_routes[0]) ==
                   HV_AGX_G2_INTERRUPT_ROUTE_COUNT,
               "generated AGX G2 route count");

size_t hv_irq_route_count(void)
{
    return static_route_count + dynamic_route_count;
}

const struct hv_irq_route *hv_irq_route_at(size_t index)
{
    if (index < static_route_count)
        return &routes[index];
    index -= static_route_count;
    if (index >= dynamic_route_count)
        return NULL;
    return &dynamic_routes[index];
}

const struct hv_irq_route *hv_irq_route_from_hw(u32 hw_irq)
{
    for (size_t i = 0; i < hv_irq_route_count(); i++) {
        const struct hv_irq_route *route = hv_irq_route_at(i);
        if (route->hw_irq == hw_irq)
            return route;
    }

    return NULL;
}

const struct hv_irq_route *hv_irq_route_from_vintid(u32 vintid)
{
    for (size_t i = 0; i < hv_irq_route_count(); i++) {
        const struct hv_irq_route *route = hv_irq_route_at(i);
        if (route->vintid == vintid)
            return route;
    }

    return NULL;
}

bool hv_irq_route_resolve_incoming(u32 hw_irq, u32 reserved_vintid, u32 *vintid)
{
    const struct hv_irq_route *route = hv_irq_route_from_hw(hw_irq);
    u32 candidate = route ? route->vintid : hw_irq;

    if (!vintid || candidate == reserved_vintid)
        return false;

    *vintid = candidate;
    return true;
}

bool hv_irq_route_level_eoi_target(u32 vintid, bool enabled, u32 *hw_irq)
{
    const struct hv_irq_route *route = hv_irq_route_from_vintid(vintid);

    if (!route || !route->level || !enabled || !hw_irq)
        return false;

    *hw_irq = route->hw_irq;
    return true;
}

bool hv_irq_route_register(u32 hw_irq, u32 vintid, bool level)
{
    if (hw_irq < 32 || hw_irq > 1019 || vintid < 32 || vintid > 1019 ||
        vintid == HV_IRQ_SYNTHETIC_NVME_VINTID ||
        dynamic_route_count >= HV_IRQ_DYNAMIC_ROUTE_CAPACITY ||
        hv_irq_route_from_hw(hw_irq) || hv_irq_route_from_vintid(vintid))
        return false;

    dynamic_routes[dynamic_route_count++] =
        (struct hv_irq_route){.hw_irq = hw_irq, .vintid = vintid, .level = level};
    return true;
}

bool hv_irq_routes_register_agx_g2(void)
{
    for (size_t i = 0; i < HV_AGX_G2_INTERRUPT_ROUTE_COUNT; i++) {
        if (!hv_irq_route_register(agx_g2_routes[i].physical_intid,
                                   agx_g2_routes[i].guest_intid,
                                   (bool)HV_AGX_G2_INTERRUPT_LEVEL))
            return false;
    }

    return true;
}

void hv_irq_routes_reset(bool enable_j313_routes)
{
    static_route_count = enable_j313_routes ? STATIC_ROUTE_COUNT : 0;
    dynamic_route_count = 0;
}

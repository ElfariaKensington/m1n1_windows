#include <assert.h>
#include <stdio.h>

#include "../src/hv_apple_input.generated.h"
#include "../src/hv_irq_routes.h"

static void test_agx_g2_routes_and_apple_input_capacity(void)
{
    static const struct hv_irq_route expected[] = {
        {.hw_irq = 563, .vintid = 880, .level = true},
        {.hw_irq = 564, .vintid = 881, .level = true},
        {.hw_irq = 565, .vintid = 882, .level = true},
        {.hw_irq = 566, .vintid = 883, .level = true},
        {.hw_irq = 579, .vintid = 884, .level = true},
        {.hw_irq = 576, .vintid = 885, .level = true},
        {.hw_irq = 575, .vintid = 886, .level = true},
        {.hw_irq = 578, .vintid = 887, .level = true},
        {.hw_irq = 577, .vintid = 888, .level = true},
    };

    hv_irq_routes_reset(true);
    assert(hv_irq_routes_register_agx_g2());
    assert(hv_irq_route_count() == 1 + sizeof(expected) / sizeof(expected[0]));

    for (size_t i = 0; i < sizeof(expected) / sizeof(expected[0]); i++) {
        const struct hv_irq_route *route = hv_irq_route_at(i + 1);
        assert(route != NULL);
        assert(route->hw_irq == expected[i].hw_irq);
        assert(route->vintid == expected[i].vintid);
        assert(route->level == expected[i].level);
        assert(hv_irq_route_from_hw(expected[i].hw_irq) == route);
        assert(hv_irq_route_from_vintid(expected[i].vintid) == route);

        for (size_t j = i + 1; j < sizeof(expected) / sizeof(expected[0]); j++) {
            assert(expected[i].hw_irq != expected[j].hw_irq);
            assert(expected[i].vintid != expected[j].vintid);
        }
    }

    assert(hv_irq_route_register((u32)HV_APPLE_INPUT_PHYSICAL_PARENT_IRQ,
                                 (u32)HV_APPLE_INPUT_GUEST_VINTID, true));
    assert(hv_irq_route_count() == 11);
    assert(hv_irq_route_from_hw((u32)HV_APPLE_INPUT_PHYSICAL_PARENT_IRQ)->vintid ==
           (u32)HV_APPLE_INPUT_GUEST_VINTID);
    assert(hv_irq_route_from_vintid((u32)HV_APPLE_INPUT_GUEST_VINTID)->hw_irq ==
           (u32)HV_APPLE_INPUT_PHYSICAL_PARENT_IRQ);
    assert(!hv_irq_route_register(700, 900, true));
}

int main(void)
{
    const struct hv_irq_route *route;
    u32 hw_irq = 0;
    u32 vintid = 0;

    hv_irq_routes_reset(false);
    assert(hv_irq_route_count() == 0);
    assert(hv_irq_route_from_hw(857) == NULL);
    assert(hv_irq_route_from_vintid(857) == NULL);
    assert(hv_irq_route_register(333, 865, true));
    assert(hv_irq_route_at(0)->hw_irq == 333);

    hv_irq_routes_reset(true);

    route = hv_irq_route_from_hw(857);
    assert(route != NULL);
    assert(route->hw_irq == 857);
    assert(route->vintid == 857);
    assert(route->level);

    assert(hv_irq_route_from_vintid(857) == route);
    assert(hv_irq_route_from_hw(64) == NULL);
    assert(hv_irq_route_from_vintid(64) == NULL);

    /* Guest EOIs may only unmask a physical line through an explicit route. */
    assert(!hv_irq_route_level_eoi_target(64, true, &hw_irq));
    assert(!hv_irq_route_level_eoi_target(857, false, &hw_irq));
    assert(hv_irq_route_level_eoi_target(857, true, &hw_irq));
    assert(hw_irq == 857);

    /* A raw AIC IRQ must not alias a synthetic guest-only interrupt. */
    assert(!hv_irq_route_resolve_incoming(64, 64, &vintid));
    assert(hv_irq_route_resolve_incoming(123, 64, &vintid));
    assert(vintid == 123);
    assert(hv_irq_route_resolve_incoming(857, 64, &vintid));
    assert(vintid == 857);

    /* Launch-contract capture enumerates the installed route table itself. */
    assert(hv_irq_route_count() == 1);
    route = hv_irq_route_at(0);
    assert(route != NULL);
    assert(route->hw_irq == 857);
    assert(route->vintid == 857);
    assert(route->level);
    assert(hv_irq_route_at(hv_irq_route_count()) == NULL);

    /* Dynamic physical routes are bounded and cannot shadow either namespace. */
    assert(hv_irq_route_register(333, 865, true));
    assert(hv_irq_route_from_hw(333)->vintid == 865);
    assert(hv_irq_route_from_vintid(865)->hw_irq == 333);
    assert(!hv_irq_route_register(334, 865, true));
    assert(!hv_irq_route_register(333, 866, true));
    assert(!hv_irq_route_register(857, 900, true));
    assert(!hv_irq_route_register(334, 31, true));
    assert(!hv_irq_route_register(334, 1020, true));
    assert(!hv_irq_route_register(334, HV_IRQ_SYNTHETIC_NVME_VINTID, true));

    for (u32 i = 1; i < HV_IRQ_DYNAMIC_ROUTE_CAPACITY; i++)
        assert(hv_irq_route_register(333 + i, 865 + i, true));
    assert(!hv_irq_route_register(700, 900, true));
    assert(hv_irq_route_count() == 1 + HV_IRQ_DYNAMIC_ROUTE_CAPACITY);

    hv_irq_routes_reset(true);
    assert(hv_irq_route_count() == 1);
    assert(hv_irq_route_from_hw(333) == NULL);

    test_agx_g2_routes_and_apple_input_capacity();

    hv_irq_routes_reset(false);
    assert(hv_irq_route_register(1207, 857, true));
    assert(hv_irq_route_resolve_incoming(1207, 64, &vintid) && vintid == 857);
    assert(hv_irq_route_level_eoi_target(857, true, &hw_irq) && hw_irq == 1207);
    assert(!hv_irq_route_register(0x10000, 858, true));
    assert(!hv_irq_route_register(1208, 1208, true));

    puts("hv_irq_routes_test: ok");
    return 0;
}

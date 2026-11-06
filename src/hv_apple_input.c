/* SPDX-License-Identifier: MIT */

#include "hv_apple_input.h"

#ifdef HV_APPLE_INPUT_HOST_TEST
#include <string.h>
#endif

#ifndef HV_APPLE_INPUT_HOST_TEST
#include "adt.h"
#include "hv.h"
#include "hv_irq_routes.h"
#include "string.h"
#include "utils.h"
#endif

static bool fail(struct hv_apple_input_preflight *result,
                 enum hv_apple_input_preflight_failure failure)
{
    if (result) {
        result->ready = false;
        result->failure = failure;
        result->parent_irq = 0;
        result->guest_vintid = 0;
    }
    return false;
}

#define REQUIRE_FIELD(member, expected, failure_code) \
    do {                                                \
        if (observed->member != (expected))             \
            return fail(result, (failure_code));        \
    } while (0)

bool hv_apple_input_validate(const struct hv_apple_input_observed *observed,
                             struct hv_apple_input_preflight *result)
{
    if (!observed || !result)
        return fail(result, HV_APPLE_INPUT_PREFLIGHT_NULL);

    result->ready = false;
    result->failure = HV_APPLE_INPUT_PREFLIGHT_OK;
    result->parent_irq = 0;
    result->guest_vintid = 0;

    REQUIRE_FIELD(spi_compatible, true, HV_APPLE_INPUT_PREFLIGHT_SPI_COMPATIBLE);
    REQUIRE_FIELD(hid_compatible, true, HV_APPLE_INPUT_PREFLIGHT_HID_COMPATIBLE);
    REQUIRE_FIELD(spi_base, HV_APPLE_INPUT_SPI_BASE, HV_APPLE_INPUT_PREFLIGHT_SPI_BASE);
    REQUIRE_FIELD(spi_size, HV_APPLE_INPUT_SPI_SIZE, HV_APPLE_INPUT_PREFLIGHT_SPI_SIZE);
    REQUIRE_FIELD(spi_source_hz, HV_APPLE_INPUT_SPI_SOURCE_HZ,
                  HV_APPLE_INPUT_PREFLIGHT_SPI_SOURCE_HZ);
    REQUIRE_FIELD(spi_bus_hz, HV_APPLE_INPUT_SPI_BUS_HZ,
                  HV_APPLE_INPUT_PREFLIGHT_SPI_BUS_HZ);
    REQUIRE_FIELD(ap_gpio_base, HV_APPLE_INPUT_AP_GPIO_BASE,
                  HV_APPLE_INPUT_PREFLIGHT_AP_GPIO_BASE);
    REQUIRE_FIELD(ap_gpio_size, HV_APPLE_INPUT_AP_GPIO_SIZE,
                  HV_APPLE_INPUT_PREFLIGHT_AP_GPIO_SIZE);
    REQUIRE_FIELD(ap_gpio_pin, HV_APPLE_INPUT_AP_GPIO_PIN,
                  HV_APPLE_INPUT_PREFLIGHT_AP_GPIO_PIN);
    REQUIRE_FIELD(nub_gpio_base, HV_APPLE_INPUT_NUB_GPIO_BASE,
                  HV_APPLE_INPUT_PREFLIGHT_NUB_GPIO_BASE);
    REQUIRE_FIELD(nub_gpio_size, HV_APPLE_INPUT_NUB_GPIO_SIZE,
                  HV_APPLE_INPUT_PREFLIGHT_NUB_GPIO_SIZE);
    REQUIRE_FIELD(nub_gpio_pin, HV_APPLE_INPUT_NUB_GPIO_PIN,
                  HV_APPLE_INPUT_PREFLIGHT_NUB_GPIO_PIN);
    if (observed->parent_irq != HV_APPLE_INPUT_PHYSICAL_PARENT_IRQ)
        return fail(result, HV_APPLE_INPUT_PREFLIGHT_PARENT_IRQ);

    result->ready = true;
    result->parent_irq = observed->parent_irq;
    result->guest_vintid = (u32)HV_APPLE_INPUT_GUEST_VINTID;
    return true;
}

static bool prepare_fail(struct hv_apple_input_prepare_result *result,
                         enum hv_apple_input_prepare_failure failure,
                         enum hv_apple_input_preflight_failure preflight_failure)
{
    if (result) {
        result->ready = false;
        result->failure = failure;
        result->preflight_failure = preflight_failure;
    }
    return false;
}

bool hv_apple_input_prepare(const struct hv_apple_input_observed *observed,
                            const struct hv_apple_input_prepare_backend *backend,
                            struct hv_apple_input_prepare_result *result)
{
    struct hv_apple_input_preflight preflight;

    if (!backend || !result || !backend->route_available || !backend->map_identity ||
        !backend->register_level_route)
        return prepare_fail(result, HV_APPLE_INPUT_PREPARE_NULL,
                            HV_APPLE_INPUT_PREFLIGHT_NULL);

    *result = (struct hv_apple_input_prepare_result){0};
    if (!hv_apple_input_validate(observed, &preflight))
        return prepare_fail(result, HV_APPLE_INPUT_PREPARE_PREFLIGHT, preflight.failure);

    /* Route conflicts are reversible to detect; mappings are not.  Therefore
     * no stage-2 state changes before this gate has passed. */
    if (!backend->route_available(backend->context, preflight.parent_irq,
                                  preflight.guest_vintid))
        return prepare_fail(result, HV_APPLE_INPUT_PREPARE_ROUTE_CONFLICT,
                            HV_APPLE_INPUT_PREFLIGHT_OK);

    if (!backend->map_identity(backend->context, observed->spi_base, observed->spi_size))
        return prepare_fail(result, HV_APPLE_INPUT_PREPARE_MAP_SPI,
                            HV_APPLE_INPUT_PREFLIGHT_OK);
    if (!backend->map_identity(backend->context, observed->ap_gpio_base,
                               observed->ap_gpio_size))
        return prepare_fail(result, HV_APPLE_INPUT_PREPARE_MAP_AP_GPIO,
                            HV_APPLE_INPUT_PREFLIGHT_OK);
    if (!backend->map_identity(backend->context, observed->nub_gpio_base,
                               observed->nub_gpio_size))
        return prepare_fail(result, HV_APPLE_INPUT_PREPARE_MAP_NUB_GPIO,
                            HV_APPLE_INPUT_PREFLIGHT_OK);
    if (!backend->register_level_route(backend->context, preflight.parent_irq,
                                       preflight.guest_vintid, true))
        return prepare_fail(result, HV_APPLE_INPUT_PREPARE_REGISTER_ROUTE,
                            HV_APPLE_INPUT_PREFLIGHT_OK);

    result->ready = true;
    result->failure = HV_APPLE_INPUT_PREPARE_OK;
    result->preflight_failure = HV_APPLE_INPUT_PREFLIGHT_OK;
    return true;
}

struct apple_input_adt_gpio_function {
    u32 phandle;
    char name[4];
    u32 pin;
};

bool hv_apple_input_decode_gpio_function(const void *value, u32 length,
                                         u32 *controller_phandle, u32 *pin)
{
    struct apple_input_adt_gpio_function function;

    if (!value || !controller_phandle || !pin || length < sizeof(function))
        return false;
    memcpy(&function, value, sizeof(function));
    /* ADT fourcc values are stored as a little-endian u32.  The textual
     * fourcc GPIO is therefore laid out as OIPG in memory on AArch64. */
    if (memcmp(function.name, "OIPG", sizeof(function.name)))
        return false;
    *controller_phandle = function.phandle;
    *pin = function.pin;
    return true;
}

bool hv_apple_input_select_parent_irq(const u32 *parents, size_t parent_count,
                                      u32 group, u32 *parent_irq)
{
    if (!parents || !parent_irq || group >= parent_count)
        return false;
    *parent_irq = parents[group];
    return true;
}

#ifndef HV_APPLE_INPUT_HOST_TEST

static bool read_reg(const char *path, u64 *base, u64 *size)
{
    int trace[8];
    int node = adt_path_offset_trace(adt, path, trace);
    return node >= 0 && adt_get_reg(adt, trace, "reg", 0, base, size) == 0;
}

static bool read_phandle(int node, u32 *phandle)
{
    return node >= 0 && ADT_GETPROP(adt, node, "AAPL,phandle", phandle) == sizeof(*phandle);
}

static int find_arm_io_child_by_phandle(u32 phandle)
{
    int arm_io = adt_path_offset(adt, "/arm-io");
    int child = arm_io;

    if (arm_io < 0)
        return -1;
    ADT_FOREACH_CHILD(adt, child) {
        u32 candidate;
        if (read_phandle(child, &candidate) && candidate == phandle)
            return child;
    }
    return -1;
}

static bool read_arm_io_child_reg(int node, u64 *base, u64 *size)
{
    int trace[8] = {0};
    int arm_io = adt_path_offset_trace(adt, "/arm-io", trace);
    size_t depth = 0;

    if (arm_io < 0 || node < 0)
        return false;
    while (depth < ARRAY_SIZE(trace) && trace[depth])
        depth++;
    if (depth + 1 >= ARRAY_SIZE(trace))
        return false;
    trace[depth] = node;
    return adt_get_reg(adt, trace, "reg", 0, base, size) == 0;
}

static bool read_gpio_function(int node, const char *name, u32 *controller_phandle, u32 *pin)
{
    u32 length = 0;
    const void *value = adt_getprop(adt, node, name, &length);
    const u8 *bytes = value;

    if (value) {
        printf("HV: Apple input %s length=%u raw=", name, length);
        for (u32 i = 0; i < length && i < 24; i++)
            printf("%02x", bytes[i]);
        printf("\n");
    }
    return hv_apple_input_decode_gpio_function(value, length, controller_phandle, pin);
}

bool hv_apple_input_observe_adt(struct hv_apple_input_observed *observed)
{
    static const u32 j313_parents[] = HV_APPLE_INPUT_PARENT_IRQ_VALUES;
    static const u32 j316s_parents[] = {623, 624, 625, 626, 627, 628, 629};
    bool j316s = adt_is_compatible(adt, 0, "J316sAP");
    const u32 *expected_parents = j316s ? j316s_parents : j313_parents;
    size_t parent_count = j316s ? ARRAY_SIZE(j316s_parents) : ARRAY_SIZE(j313_parents);
    struct hv_apple_input_observed value = {
        /* These clocks are reviewed J313 controller settings.  The live ADT
         * proves identity and resources; it does not publish both rates. */
        .spi_source_hz = j316s ? 24000000 : HV_APPLE_INPUT_SPI_SOURCE_HZ,
        .spi_bus_hz = HV_APPLE_INPUT_SPI_BUS_HZ,
    };
    int spi = adt_path_offset(adt, "/arm-io/spi3");
    int hid = adt_path_offset(adt, "/arm-io/spi3/ipd");
    int ap_gpio, nub_gpio;
    u32 ap_phandle, hid_parent, ap_pin, length = 0;
    const u32 *hid_interrupts;
    const u32 *parent_interrupts;

    if (!observed || spi < 0 || hid < 0) {
        printf("HV: Apple input ADT fail stage=nodes spi=%d hid=%d\n", spi, hid);
        return false;
    }
    value.spi_compatible = adt_is_compatible(adt, spi, "spi-1,spimc");
    value.hid_compatible = adt_is_compatible(adt, hid, "hid-transport,spi");
    if (!read_reg("/arm-io/spi3", &value.spi_base, &value.spi_size)) {
        printf("HV: Apple input ADT fail stage=spi-reg\n");
        return false;
    }
    if (!read_gpio_function(hid, "function-spi_en", &ap_phandle, &ap_pin)) {
        printf("HV: Apple input ADT fail stage=spi-en\n");
        return false;
    }
    if (ADT_GETPROP(adt, hid, "interrupt-parent", &hid_parent) != sizeof(hid_parent)) {
        printf("HV: Apple input ADT fail stage=interrupt-parent\n");
        return false;
    }

    /* Resolve both GPIO controllers from the live bindings.  Node names are
     * firmware details (J313 uses /arm-io/gpio, not /arm-io/gpio0). */
    ap_gpio = find_arm_io_child_by_phandle(ap_phandle);
    nub_gpio = find_arm_io_child_by_phandle(hid_parent);
    if (ap_gpio < 0 || nub_gpio < 0) {
        printf("HV: Apple input ADT fail stage=gpio-node ap=%d nub=%d aph=0x%x nph=0x%x\n",
               ap_gpio, nub_gpio, ap_phandle, hid_parent);
        return false;
    }
    if (!read_arm_io_child_reg(ap_gpio, &value.ap_gpio_base, &value.ap_gpio_size) ||
        !read_arm_io_child_reg(nub_gpio, &value.nub_gpio_base, &value.nub_gpio_size)) {
        printf("HV: Apple input ADT fail stage=gpio-reg ap=%d nub=%d\n", ap_gpio, nub_gpio);
        return false;
    }
    value.ap_gpio_pin = ap_pin;

    hid_interrupts = adt_getprop(adt, hid, "interrupts", &length);
    if (!hid_interrupts || length < 2 * sizeof(u32)) {
        printf("HV: Apple input ADT fail stage=hid-interrupts length=%u\n", length);
        return false;
    }
    value.nub_gpio_pin = hid_interrupts[0];

    parent_interrupts = adt_getprop(adt, nub_gpio, "interrupts", &length);
    if (!parent_interrupts || length != parent_count * sizeof(*expected_parents)) {
        printf("HV: Apple input ADT fail stage=parent-interrupts length=%u expected=%zu\n",
               length, parent_count * sizeof(*expected_parents));
        return false;
    }
    for (size_t i = 0; i < parent_count; i++) {
        if (parent_interrupts[i] != expected_parents[i]) {
            printf("HV: Apple input ADT fail stage=parent-value index=%zu actual=%u expected=%u\n",
                   i, parent_interrupts[i], expected_parents[i]);
            return false;
        }
    }
    if (!hv_apple_input_select_parent_irq(parent_interrupts, parent_count,
                                          HV_APPLE_INPUT_IRQ_STARTUP_GROUP,
                                          &value.parent_irq))
        return false;
    printf("HV: Apple input ADT observed ap_phandle=0x%x ap_pin=%u parent_phandle=0x%x irq=%u\n",
           ap_phandle, ap_pin, hid_parent, value.parent_irq);
    *observed = value;
    return true;
}

static bool runtime_route_available(void *context, u32 parent_irq, u32 guest_vintid)
{
    UNUSED(context);
    return !hv_irq_route_from_hw(parent_irq) && !hv_irq_route_from_vintid(guest_vintid);
}

static bool runtime_map_identity(void *context, u64 base, u64 size)
{
    UNUSED(context);
    return hv_map_hw(base, base, size) == 0;
}

static bool runtime_register_route(void *context, u32 parent_irq, u32 guest_vintid,
                                   bool level)
{
    UNUSED(context);
    return hv_irq_route_register(parent_irq, guest_vintid, level);
}

bool hv_apple_input_prepare_runtime(struct hv_apple_input_prepare_result *result)
{
    struct hv_apple_input_observed observed;
    const struct hv_apple_input_prepare_backend backend = {
        .route_available = runtime_route_available,
        .map_identity = runtime_map_identity,
        .register_level_route = runtime_register_route,
    };

    if (!result)
        return false;
    if (!hv_apple_input_observe_adt(&observed))
        return prepare_fail(result, HV_APPLE_INPUT_PREPARE_PREFLIGHT,
                            HV_APPLE_INPUT_PREFLIGHT_NULL);
    return hv_apple_input_prepare(&observed, &backend, result);
}

#endif

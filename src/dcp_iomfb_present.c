/* SPDX-License-Identifier: MIT */
#include "dcp_iomfb_present.h"
#ifdef DCP_IOMFB_PRESENT_HOST_TEST
#include <string.h>
#else
#include "string.h"
#endif
#define SURFACES 4u
#define SECONDARY_SURFACES 5u
#define XFER_SDR 13u
#define COLORSPACE_SRGB 1u
#define SET_BACKGROUND (1u << 31)
#define J313_IOMFB_USER_CLIENT 0xfffffe1667ba4a00ull
struct iomfb_rect { uint32_t x, y, w, h; } __attribute__((packed));
struct iomfb_component_types { uint8_t count, types[7]; } __attribute__((packed));
struct iomfb_plane {
    uint32_t width, height, base, offset, stride, size;
    uint16_t tile_size;
    uint8_t tile_w, tile_h;
    uint32_t unknown[13];
} __attribute__((packed));
struct iomfb_surface_base {
    uint8_t is_tiled, is_tearing_allowed, is_premultiplied;
    uint32_t plane_count, plane_count_2, format, ycbcr_matrix;
    uint8_t transfer_function, colorspace;
    uint32_t stride;
    uint16_t pixel_size;
    uint8_t pel_width, pel_height;
    uint32_t offset, width, height, buffer_size;
    uint64_t protection_options;
    uint32_t surface_id;
    struct iomfb_component_types component_types[3];
    uint64_t has_components;
    struct iomfb_plane planes[3];
    uint64_t has_planes;
    uint32_t compression_info[3][13];
    uint64_t has_compression_info;
    uint32_t unknown_numerator, unknown_denominator;
} __attribute__((packed));
struct iomfb_surface_v13_5 {
    struct iomfb_surface_base base;
    uint8_t padding[47];
} __attribute__((packed));
struct iomfb_swap_v13_5 {
    uint64_t timestamps_and_unknown[8], flags_1, flags_2;
    uint32_t swap_id, surface_ids[SURFACES];
    struct iomfb_rect source[SURFACES];
    uint32_t surface_flags[SURFACES], surface_unknown[SURFACES];
    struct iomfb_rect destination[SURFACES];
    uint32_t swap_enabled, swap_completed, background_color;
    uint8_t unknown_110[0x1b8];
    uint32_t unknown_2c8;
    uint8_t unknown_2cc[0x14];
    uint32_t unknown_2e0;
    uint8_t unknown_2e4[3];
    uint64_t backlight_unknown;
    uint32_t backlight_value;
    uint8_t backlight_power, unknown_2f4[0x2d], unknown_321[0x147];
} __attribute__((packed));
struct iomfb_submit_v13_5 {
    struct iomfb_swap_v13_5 swap;
    struct iomfb_surface_v13_5 surfaces[SURFACES];
    uint64_t surface_iova[SURFACES], unknown_iova[SURFACES];
    struct iomfb_surface_v13_5 secondary[SECONDARY_SURFACES];
    uint64_t secondary_iova[SECONDARY_SURFACES];
    uint8_t unknown_bool;
    uint64_t unknown_double, unknown_u64;
    uint8_t unknown_bool_2;
    uint32_t clear, unknown_u32_pointer;
    uint8_t swap_null, surface_null[SURFACES];
    uint8_t secondary_null[SECONDARY_SURFACES];
    uint8_t unknown_out_bool_null, unknown_u32_pointer_null;
    uint8_t unknown_u32_out_null, padding;
} __attribute__((packed));
struct iomfb_swap_start_v13_5 {
    uint32_t swap_id;
    uint64_t client_address;
    uint32_t client_unknown;
    uint8_t client_flag_1, client_flag_2;
    uint8_t swap_id_null, client_null;
    uint8_t padding[4];
} __attribute__((packed));
_Static_assert(sizeof(struct iomfb_plane) == 0x50, "plane ABI");
_Static_assert(sizeof(struct iomfb_surface_base) == 0x1fd, "surface ABI");
_Static_assert(sizeof(struct iomfb_surface_v13_5) == 0x22c, "v13.5 surface ABI");
_Static_assert(sizeof(struct iomfb_swap_v13_5) == 0x468, "v13.5 swap ABI");
_Static_assert(sizeof(struct iomfb_submit_v13_5) == 0x1884, "v13.5 submit ABI");
_Static_assert(sizeof(struct iomfb_swap_start_v13_5) ==
               DCP_IOMFB_V13_5_SWAP_START_SIZE, "v13.5 swap-start ABI");
bool dcp_iomfb_present_build_start_v13_5(void *input, size_t input_size)
{
    struct iomfb_swap_start_v13_5 *wire = input;

    if (!wire || input_size != sizeof(*wire))
        return false;
    memset(wire, 0, sizeof(*wire));
    wire->client_address = J313_IOMFB_USER_CLIENT;
    wire->client_flag_2 = 1;
    return true;
}
bool dcp_iomfb_present_build_v13_5(struct dcp_iomfb_present_request *request,
                                   uint64_t iova, uint32_t width,
                                   uint32_t height, uint32_t stride,
                                   uint32_t dcp_format,
                                   bool clear_boot_surfaces)
{
    struct iomfb_submit_v13_5 *wire;
    struct iomfb_surface_base *surface;
    uint64_t total = (uint64_t)height * stride;
    if (!request || !iova || !width || !height ||
        (uint64_t)stride < (uint64_t)width * 4 || (stride & 63) ||
        !total || total > UINT32_MAX ||
        (dcp_format != DCP_IOMFB_FORMAT_BGRA && dcp_format != DCP_IOMFB_FORMAT_RGBA))
        return false;
    memset(request, 0, sizeof(*request));
    wire = (struct iomfb_submit_v13_5 *)request->bytes;
    wire->swap.flags_1 = 0x861202;
    wire->swap.flags_2 = 0x04;
    wire->swap.surface_ids[0] = 3;
    wire->swap.source[0] = (struct iomfb_rect){0, 0, width, height};
    wire->swap.surface_flags[0] = 1;
    wire->swap.destination[0] = (struct iomfb_rect){0, 0, width, height};
    wire->swap.swap_enabled = clear_boot_surfaces ?
        (SET_BACKGROUND | 0x7u) : 1u;
    wire->swap.swap_completed = wire->swap.swap_enabled;
    wire->swap.backlight_unknown = 1;
    wire->swap.backlight_value = 0x58f058d0;
    wire->swap.backlight_power = 0x40;
    for (unsigned i = 1; i < SURFACES; i++) wire->surface_null[i] = 1;
    for (unsigned i = 0; i < SECONDARY_SURFACES; i++) wire->secondary_null[i] = 1;
    wire->unknown_u32_pointer_null = wire->unknown_u32_out_null = 1;
    surface = &wire->surfaces[0].base;
    surface->format = dcp_format;
    surface->transfer_function = XFER_SDR;
    surface->colorspace = COLORSPACE_SRGB;
    surface->stride = stride;
    surface->pixel_size = 4;
    surface->pel_width = surface->pel_height = 1;
    surface->width = width;
    surface->height = height;
    surface->buffer_size = (uint32_t)total;
    surface->surface_id = 3;
    surface->has_components = surface->has_planes = 1;
    wire->surface_iova[0] = iova;
    return true;
}
bool dcp_iomfb_present_set_swap_id_v13_5(struct dcp_iomfb_present_request *request,
                                          uint32_t swap_id)
{
    if (!request || !swap_id) return false;
    ((struct iomfb_submit_v13_5 *)request->bytes)->swap.swap_id = swap_id;
    return true;
}
bool dcp_iomfb_present_parse_start_v13_5(const void *response, size_t size,
                                         uint32_t *swap_id)
{
    uint32_t value, result;
    if (!response || !swap_id || size != DCP_IOMFB_V13_5_SWAP_START_SIZE) return false;
    memcpy(&value, response, 4);
    memcpy(&result, (const uint8_t *)response + 0x14, 4);
    if (!value || result) return false;
    *swap_id = value;
    return true;
}
bool dcp_iomfb_present_parse_submit_v13_5(const void *response, size_t size)
{
    uint32_t result;
    if (!response || size != DCP_IOMFB_V13_5_SWAP_SUBMIT_OUTPUT_SIZE) return false;
    memcpy(&result, (const uint8_t *)response + 5, 4);
    return result == 0;
}

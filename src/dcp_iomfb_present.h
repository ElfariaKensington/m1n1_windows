/* SPDX-License-Identifier: MIT */
#ifndef DCP_IOMFB_PRESENT_H
#define DCP_IOMFB_PRESENT_H
#ifdef DCP_IOMFB_PRESENT_HOST_TEST
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#else
#include "types.h"
#endif
#define DCP_IOMFB_V13_5_SWAP_START_SIZE 0x18u
#define DCP_IOMFB_V13_5_SWAP_SUBMIT_INPUT_SIZE 0x1884u
#define DCP_IOMFB_V13_5_SWAP_SUBMIT_OUTPUT_SIZE 0x0cu
struct dcp_iomfb_present_request {
    uint8_t bytes[DCP_IOMFB_V13_5_SWAP_SUBMIT_INPUT_SIZE];
};
bool dcp_iomfb_present_build_start_v13_5(void *input, size_t input_size);
/* D3D-FS1: DCP surface fourccs (Asahi iomfb_plane.h DCP_FORMAT_BGRA 'ARGB',
 * DCP_FORMAT_RGBA 'ABGR'). */
#define DCP_IOMFB_FORMAT_BGRA 0x42475241u
#define DCP_IOMFB_FORMAT_RGBA 0x52474241u
bool dcp_iomfb_present_build_v13_5(struct dcp_iomfb_present_request *request,
                                   uint64_t surface_iova, uint32_t width,
                                   uint32_t height, uint32_t stride,
                                   uint32_t dcp_format,
                                   bool clear_boot_surfaces);
bool dcp_iomfb_present_set_swap_id_v13_5(
    struct dcp_iomfb_present_request *request, uint32_t swap_id);
bool dcp_iomfb_present_parse_start_v13_5(
    const void *response, size_t response_size, uint32_t *swap_id);
bool dcp_iomfb_present_parse_submit_v13_5(
    const void *response, size_t response_size);
#endif

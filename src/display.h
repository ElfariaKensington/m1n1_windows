/* SPDX-License-Identifier: MIT */

#ifndef DISPLAY_H
#define DISPLAY_H

#include "dcp.h"
#include "types.h"

typedef enum _dcp_shutdown_mode {
    DCP_QUIESCED = 0,
    DCP_SLEEP_IF_EXTERNAL = 1,
    DCP_SLEEP = 2,
} dcp_shutdown_mode;

extern bool display_is_external;

int display_init(void);
int display_start_dcp(void);
int display_prepare_guest_surface(u64 base, u64 size, u32 width, u32 height, u32 stride,
                                  u32 depth);
int display_configure(const char *config);
void display_shutdown(dcp_shutdown_mode mode);
bool display_shutdown_complete(void);
const display_config_t *display_get_config(void);

/* Non-blocking fixed-panel backend used by the EL2 scanout broker. */
bool display_scanout_ready(void);
bool display_scanout_latch_source_proven(void);
bool display_scanout_reserve_iova(u64 size, u64 alignment, u64 *iova);
void display_scanout_free_iova(u64 iova, u64 size);
bool display_scanout_map(unsigned dart_index, u64 iova, u64 pa, u64 size);
void display_scanout_unmap(unsigned dart_index, u64 iova, u64 size);
/* D3D-FS1: rgba selects the RGBA8888 surface order (else BGRA8888) */
bool display_scanout_present_begin(u64 surface_iova, u32 width, u32 height,
                                   u32 stride, bool rgba, u64 *cookie);
int display_scanout_present_poll(u64 cookie, u32 *swap_id);
int display_scanout_latch_poll(u32 expected_swap_id);
bool display_scanout_observe_latched(u32 *swap_id, u64 *surface_iova);
bool display_scanout_quiesce_begin(u64 *cookie);
int display_scanout_quiesce_poll(u64 cookie);

#endif

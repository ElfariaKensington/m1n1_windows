/* SPDX-License-Identifier: MIT */

#ifndef DCP_H
#define DCP_H

#include "afk.h"
#include "asc.h"
#include "dart.h"
#include "rtkit.h"
#include "dcp_iomfb_transport.h"
#include "dcp_iomfb_rpc.h"
#include "dcp_iomfb_bootstrap.h"
#include "dcp_iomfb_mode_select.h"
#include "dcp_iomfb_properties.h"
#include "dcp_iomfb_present.h"
#include "dcp_iomfb_resources.h"

#include "dcp/dpav_ep.h"
#include "dcp/dptx_port_ep.h"
#include "dcp/system_ep.h"

typedef struct {
    const char dcp[24];
    const char dcp_dart[24];
    const char disp_dart[24];
    const char dptx_phy[24];
    const char dp2hdmi_gpio[24];
    const char pmgr_dev[24];
    const char dcp_alias[8];
    u32 dcp_index;
    u8 num_dptxports;
    u8 die;
} display_config_t;

typedef struct dcp_dev {
    dart_dev_t *dart_dcp;
    dart_dev_t *dart_disp;
    dart_dev_t *dart_piodma;
    iova_domain_t *iovad_dcp;
    asc_dev_t *asc;
    rtkit_dev_t *rtkit;
    afk_epic_t *afk;
    dcp_system_if_t *system_ep;
    dcp_dpav_if_t *dpav_ep;
    dcp_dptx_if_t *dptx_ep;
    dptx_phy_t *phy;
    struct rtkit_buffer iomfb_shmem;
    struct dcp_iomfb_transport iomfb_transport;
    enum dcp_iomfb_rx_result iomfb_last_rx;
    bool iomfb_observer_registered;
    bool iomfb_owner_endpoint_started;
    bool iomfb_owner_registered;
    bool iomfb_initialized;
    struct dcp_iomfb_rpc iomfb_rpc;
    struct dcp_iomfb_bootstrap iomfb_bootstrap;
    struct dcp_iomfb_properties iomfb_properties;
    struct dcp_iomfb_present_request iomfb_present_request;
    struct dcp_iomfb_resources iomfb_resources;
    u32 iomfb_expected_swap_id;
    u32 iomfb_latched_swap_id;
    bool iomfb_surfaces_cleared;
    u32 die;
    u32 dp2hdmi_pwr_gpio;
    u32 hdmi_pwr_gpio;
} dcp_dev_t;

int dcp_connect_dptx(dcp_dev_t *dcp);
int dcp_work(dcp_dev_t *dcp);
bool dcp_iomfb_observer_start(dcp_dev_t *dcp);
void dcp_iomfb_observer_arm(dcp_dev_t *dcp, u32 swap_id);
int dcp_iomfb_observer_poll_latch(dcp_dev_t *dcp, u32 expected_swap_id);
bool dcp_iomfb_owner_start(dcp_dev_t *dcp);
bool dcp_iomfb_owner_supported(void);
bool dcp_iomfb_owner_active(const dcp_dev_t *dcp);
void dcp_iomfb_owner_arm(dcp_dev_t *dcp, u32 swap_id);
int dcp_iomfb_owner_poll_latch(dcp_dev_t *dcp, u32 expected_swap_id);
int dcp_iomfb_owner_present(dcp_dev_t *dcp, u64 surface_iova, u32 width,
                            u32 height, u32 stride, u32 dcp_format);

dcp_dev_t *dcp_init(const display_config_t *config);

int dcp_shutdown(dcp_dev_t *dcp, bool sleep);

#endif

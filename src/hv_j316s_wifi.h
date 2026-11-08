/* SPDX-License-Identifier: MIT */
#ifndef HV_J316S_WIFI_H
#define HV_J316S_WIFI_H
#include "types.h"
bool hv_j316s_wifi_init(bool enable);
bool hv_j316s_wifi_mapped(void);
u64 hv_j316s_wifi_window_pa(void);
void hv_j316s_wifi_poll(void);
#endif

/* SPDX-License-Identifier: MIT */

#ifndef PCIE_H
#define PCIE_H

int pcie_init(void);
int pcie_shutdown(void);
/* T6000 WLAN only; leaves the SD reader and other ports untouched. */
int pcie_init_wlan(void);

#endif

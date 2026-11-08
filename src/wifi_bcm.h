/* SPDX-License-Identifier: MIT */
#ifndef WIFI_BCM_H
#define WIFI_BCM_H
#include "types.h"

struct bcm_dma { void *cpu; u64 iova; size_t size; };
struct bcm_platform {
    u32 (*read32)(u32 address);
    void (*write32)(u32 address, u32 value);
    void (*read_ram)(u32 address, void *out, size_t size);
    void (*write_ram)(u32 address, const void *data, size_t size);
    bool (*dma_alloc)(size_t size, struct bcm_dma *out);
    void (*dma_sync)(struct bcm_dma *buffer);
    void (*doorbell)(u32 value);
    u64 (*time_us)(void);
    void (*delay_us)(u32 delay);
    /* Optional: BAR0+0x2000+off alias used by brcmfmac's *_pcie32 accessors. */
    u32 (*read_pcie32)(u32 offset);
    void (*write_pcie32)(u32 offset, u32 value);
    /* Optional: endpoint PCI config-space write (BRCMF_PCIE_REG_SBMBX doorbell). */
    void (*write_cfg32)(u32 offset, u32 value);
};
struct bcm_blob { const u8 *data; size_t size; };
struct bcm_fw_bundle {
    struct bcm_blob firmware, nvram, clm, txcap, calibration, signature;
};
/* init may wait only before guest entry; poll/join/send/receive never wait. */
int bcm_wifi_init(const struct bcm_platform *platform,
                  const struct bcm_fw_bundle *bundle, const u8 mac[6]);
int bcm_wifi_join(const u8 *ssid, u32 ssid_len, const u8 *psk, u32 psk_len);
/* Synchronous (pre-guest only): scan for the SSID, match its security, join, retry. */
int bcm_wifi_connect(const u8 *ssid, u32 ssid_len, const u8 *psk, u32 psk_len,
                     int (*out)(const char *, ...));
void bcm_wifi_poll(u32 budget);
/* send: 0 accepted; -11 backpressure (caller retains frame); other <0 error. */
int bcm_wifi_send(const void *frame, u32 size);
/* receive: >0 frame length, 0 empty, -90 insufficient capacity, other <0 error. */
int bcm_wifi_receive(void *frame, u32 capacity);
bool bcm_wifi_link(void);
u32 bcm_wifi_phase(void);
int bcm_wifi_error(void);
/* Print ring/mailbox state through the supplied printf (failure diagnostics). */
void bcm_wifi_dump(int (*out)(const char *, ...));
#endif

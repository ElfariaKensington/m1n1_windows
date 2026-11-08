/* SPDX-License-Identifier: ISC
 * Copyright (c) 2013-2014 Broadcom Corporation
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */
#ifndef WIFI_BCM_PROTO_H
#define WIFI_BCM_PROTO_H
#include "wifi_bcm.h"

/* Independent bare-metal adaptation of the ISC brcmfmac protocol, inspected at
 * current AsahiLinux/linux asahi HEAD, 2026-11-07:
 * 6ba0637d1ed9a2e3639dd3fc9eca6cd0483a6434
 * https://github.com/AsahiLinux/linux/tree/6ba0637d1ed9a2e3639dd3fc9eca6cd0483a6434/drivers/net/wireless/broadcom/brcm80211/brcmfmac
 * chip.c: DMP descriptors, AI reset, CR4/CA7 RAM sizing, BCM4387 rambase.
 * pcie.c: shared/ring layout, rTLV seed, SMAR, host capabilities, doorbells.
 * msgbuf.c/h: wire messages, packet ownership, completion padding.
 * firmware.c: NVRAM normalization and length/complement token.
 * common.c: CLM/txcap/cal download; cfg80211.c/fw il_types.h/fweh.h:
 * WPA2 offload, PMK, association and supplicant events.
 * BCMA/chipcommon numeric definitions checked against include/linux/bcma and
 * brcm80211/include/chipcommon.h. No pcie-apple implementation is used.
 *
 * Integration contract (public API deliberately unchanged):
 * - Single caller, externally serialized, including init/join/send/receive.
 * - read32/write32 use absolute backplane addresses; RAM callbacks use absolute
 *   chip addresses, BCM4387 base 0x740000, plus special reset-vector address 0.
 * - dma_alloc must return disjoint, cache-line-isolated allocations. dma_sync
 *   cleans+invalidates with DSB; firmware-owned buffers are never CPU-written.
 * - init owns core reset/firmware; platform owns PCIe power/BAR/DART/busmaster.
 * - All allocations/SEP randomness/waits occur in init before guest entry.
 * - No DMA free/reset API: init is one-shot, including after an attempted boot.
 * - No signature bootloader protocol is guessed: nonempty signatures fail.
 * - Runtime calls return -11 for backpressure; send returns 0 on acceptance,
 *   receive returns Ethernet length, 0 if empty, -90 if capacity is too small.
 * - join takes 64 hex digits or an 8..63-byte ASCII passphrase; 32-character
 *   passphrases remain passphrases. WPA2-PSK/CCMP infrastructure, interface 0,
 *   one best-effort station flowring; no AP/TDLS/monitor/roaming recovery.
 * - phase: 0 idle, 1 enum, 2 download, 3 shared, 4 config, 5 ready, 6 join
 *   configuration, 7 associating/key exchange, 8 authorized; bit31 = fatal.
 *   error is a negative errno or a negative firmware status. It is never
 *   cleared by a fabricated completion or an unverified link-up indication.
 */
#define BCM_ENUM_BASE 0x18000000u
#define BCM_RAM_BASE 0x740000u
#define BCM_IOCTRL 0x408u
#define BCM_RESETCTRL 0x800u
#define BCM_CPUHALT 0x20u
#define BCM_CORE_D11 0x812u
#define BCM_CORE_PCIE2 0x83cu
#define BCM_CORE_CR4 0x83eu
#define BCM_CORE_CA7 0x847u
#define BCM_CORE_SYSMEM 0x849u
#define BCM_SHARED_DAR 0x80000000u
#define BCM_SHARED_HOSTREADY 0x10000000u
#define BCM_SHARED_MAILBOX 0x02000000u
#define BCM_FATAL 0x80000000u

enum {
    BCM_FLOW_CREATE = 3, BCM_FLOW_CREATED = 4,
    BCM_FLOW_DELETE = 5, BCM_FLOW_DELETED = 6,
    BCM_IOCTL = 9, BCM_IOCTL_ACK = 10, BCM_IOCTL_POST = 11,
    BCM_IOCTL_DONE = 12, BCM_EVENT_POST = 13, BCM_EVENT = 14,
    BCM_TX = 15, BCM_TX_DONE = 16, BCM_RX_POST = 17, BCM_RX = 18,
    BCM_H2D_MB = 0x23, BCM_D2H_MB = 0x24,
};
enum {
    BCM_C_UP = 2, BCM_C_DOWN = 3, BCM_C_INFRA = 20, BCM_C_AUTH = 22,
    BCM_C_SSID = 26, BCM_C_DISASSOC = 52, BCM_C_PM = 86,
    BCM_C_GET_VAR = 262, BCM_C_SET_VAR = 263, BCM_C_PMK = 268,
};
enum {
    BCM_E_SSID = 0, BCM_E_DEAUTH = 5, BCM_E_DEAUTH_IND = 6,
    BCM_E_DISASSOC_IND = 12, BCM_E_LINK = 16, BCM_E_PSK = 46,
    BCM_E_ESCAN = 69,
};
#define BCM_CTL_SIZE 8192u
#define BCM_PKT_SIZE 2048u
#define BCM_NRX 256u
#define BCM_NCTL 16u /* eight IOCTL response and eight event buffers */
#define BCM_NTX 64u
#define BCM_NBUF (BCM_NRX + BCM_NCTL)
#define BCM_FLOW_DEPTH 512u
#define BCM_TIMEOUT 2000000u

/* Byte-oriented access avoids alignment assumptions about packed wire data. */
static inline u16 bcm_get16(const void *p)
{
    const u8 *b = p;
    return b[0] | ((u16)b[1] << 8);
}
static inline u32 bcm_get32(const void *p)
{
    const u8 *b = p;
    return bcm_get16(b) | ((u32)bcm_get16(b + 2) << 16);
}
static inline u16 bcm_be16(const void *p)
{
    const u8 *b = p;
    return ((u16)b[0] << 8) | b[1];
}
static inline u32 bcm_be32(const void *p)
{
    const u8 *b = p;
    return ((u32)bcm_be16(b) << 16) | bcm_be16(b + 2);
}
static inline void bcm_put16(void *p, u16 n)
{
    u8 *b = p;
    b[0] = n;
    b[1] = n >> 8;
}
static inline void bcm_put32(void *p, u32 n)
{
    u8 *b = p;
    bcm_put16(b, n);
    bcm_put16(b + 2, n >> 16);
}
static inline void bcm_put64(void *p, u64 n)
{
    u8 *b = p;
    bcm_put32(b, n);
    bcm_put32(b + 4, n >> 32);
}
#endif

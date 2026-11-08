/* SPDX-License-Identifier: MIT
 * Physical PCIe/DART owner and bounded Ethernet bridge for J316s BCM4387.
 * Protocol references: Asahi pcie-apple.c, brcmfmac/{pcie,chip}.c.
 * No physical WLAN register or DMA address is exposed to Windows.
 */
#include "../build/build_cfg.h"
#include "adt.h"
#include "aic.h"
#include "dart.h"
#include "hv.h"
#include "hv_j316s_wifi.h"
#include "malloc.h"
#include "memory.h"
#include "pcie.h"
#include "string.h"
#include "utils.h"
#include "wifi_bcm.h"
#include "../../drivers/j316-wifi/bridge.h"
#ifdef M1N1_J316S_WIFI
#include "wifi_firmware.h"
#if __has_include("wifi_credentials.h")
#include "wifi_credentials.h" /* local-only: j316_wifi_ssid[], j316_wifi_psk[] */
#define J316_WIFI_AUTOJOIN 1
#endif
#endif

#define WLAN_ECAM 0x590000000ULL
#define WLAN_PORT 0x581000000ULL
#define WLAN_DART 0x581008000ULL
#define WLAN_RAM_BASE 0x740000u /* chip.c:brcmf_chip_tcm_rambase(4387) */
#define DMA_IOVA 0x10000000ULL
#define DMA_SIZE (16u * 1024 * 1024)
static u8 *window, *dma_pool;
static u64 endpoint, bar0, bar2, bar2_size;
static u32 dma_used, io_error;
static bool mapped, ready;
static u64 next_poll;

static u32 load_reg(u32 offset)
{
    return __atomic_load_n((u32 *)(window + offset), __ATOMIC_ACQUIRE);
}
static void store_reg(u32 offset, u32 value)
{
    __atomic_store_n((u32 *)(window + offset), value, __ATOMIC_RELEASE);
}
static bool bridge_mmio(struct exc_info *ctx, u64 address, u64 *value, bool write, int width)
{
    (void)ctx;
    u64 off = address - JW_BASE;
    if (!window || width < 0 || width > 3 || off >= JW_SIZE ||
        (1u << width) > JW_SIZE - off)
        return false;
    size_t bytes = 1u << width;
    if (write) {
        bool packet = off >= JW_TX_BASE && off + bytes <= JW_RX_BASE;
        bool config = off >= JW_CONFIG_SSID_LEN && off + bytes <= JW_CONFIG_PSK + 64;
        bool reg = bytes == 4 && !(off & 3) &&
                   (off == JW_REG_TX_PROD || off == JW_REG_RX_CONS || off == JW_REG_COMMAND);
        if (reg)
            store_reg(off, (u32)*value);
        else if (packet || config)
            memcpy(window + off, value, bytes);
    } else {
        *value = 0;
        if (bytes == 4 && !(off & 3) && off < 0x100)
            *value = load_reg(off);
        else
            memcpy(value, window + off, bytes);
    }
    return true;
}

static void cfg_write(u32 offset, u32 value)
{
    write32(endpoint + offset, value);
}
static u32 bp_read(u32 address)
{
    write32(endpoint + 0x80, address & ~0xfffu);
    (void)read32(endpoint + 0x80);
    return read32(bar0 + (address & 0xfff));
}
static void bp_write(u32 address, u32 value)
{
    write32(endpoint + 0x80, address & ~0xfffu);
    (void)read32(endpoint + 0x80);
    write32(bar0 + (address & 0xfff), value);
    sysop("dsb sy");
}
static bool ram_valid(u32 address, size_t size)
{
    /* brcmfmac pcie.c: BAR2 ("tcm") is indexed by the chip's absolute address
     * (tcm + rambase + off). Chip RAM begins at WLAN_RAM_BASE; address 0 is the
     * ARM reset-vector alias written by brcmf_chip_ca7_set_active. */
    if (address == 0 && size == 4)
        return true;
    return address >= WLAN_RAM_BASE && (u64)address + size <= bar2_size;
}
static void ram_read(u32 address, void *out, size_t size)
{
    if (!ram_valid(address, size)) {
        memset(out, 0, size);
        io_error = 0xe102;
        return;
    }
    u8 *dst = out;
    for (size_t i = 0; i < size; i++)
        dst[i] = read8(bar2 + address + i);
}
static void ram_write(u32 address, const void *data, size_t size)
{
    if (!ram_valid(address, size)) {
        io_error = 0xe103;
        return;
    }
    const u8 *src = data;
    for (size_t i = 0; i < size; i++)
        write8(bar2 + address + i, src[i]);
    sysop("dsb sy");
}
static bool dma_alloc(size_t size, struct bcm_dma *out)
{
    u32 offset = (dma_used + 63u) & ~63u;
    if (!size || size > DMA_SIZE || offset > DMA_SIZE - size)
        return false;
    out->cpu = dma_pool + offset;
    out->iova = DMA_IOVA + offset;
    out->size = size;
    dma_used = offset + size;
    memset(out->cpu, 0, size);
    dc_civac_range(out->cpu, size);
    return true;
}
static void dma_sync(struct bcm_dma *buffer)
{
    dc_civac_range(buffer->cpu, buffer->size);
    sysop("dsb sy");
}
static u64 time_us(void)
{
    u64 ticks = get_ticks(), hz = mrs(CNTFRQ_EL0);
    return (ticks / hz) * 1000000ULL + (ticks % hz) * 1000000ULL / hz;
}
static void no_doorbell(u32 value) { (void)value; }
/* brcmfmac pcie.c: read/write_pcie32 use BAR0 + 0x2000 + offset. */
static u32 pcie_alias_read(u32 offset) { return read32(bar0 + 0x2000 + offset); }
static void pcie_alias_write(u32 offset, u32 value)
{
    write32(bar0 + 0x2000 + offset, value);
    sysop("dsb sy");
}

static u64 size_bar(u32 offset, u32 *flags)
{
    u32 low = read32(endpoint + offset);
    bool wide = (low & 6) == 4;
    u32 high = wide ? read32(endpoint + offset + 4) : 0;
    if (low & 1)
        return 0;
    write32(endpoint + offset, ~0u);
    if (wide)
        write32(endpoint + offset + 4, ~0u);
    u64 mask = read32(endpoint + offset) & ~15u;
    if (wide)
        mask |= (u64)read32(endpoint + offset + 4) << 32;
    else
        mask |= 0xffffffff00000000ULL;
    write32(endpoint + offset, low);
    if (wide)
        write32(endpoint + offset + 4, high);
    *flags = low & 15;
    return mask ? (~mask + 1) : 0;
}
static bool hardware_init(void)
{
    int path[8];
    int node = adt_path_offset_trace(adt, "/arm-io/apcie", path);
    u64 ecam, ecam_size;
    u32 length;
    if (node < 0 || !adt_is_compatible(adt, 0, "J316sAP") ||
        adt_get_reg(adt, path, "reg", 0, &ecam, &ecam_size) ||
        ecam != WLAN_ECAM || ecam_size < 0x200000)
        return false;
    const u8 *ranges = adt_getprop(adt, node, "ranges", &length);
    bool memory_window = false;
    for (u32 off = 0; ranges && off + 28 <= length; off += 28) {
        u32 flags; u64 pci, cpu, size;
        memcpy(&flags, ranges + off, 4);
        memcpy(&pci, ranges + off + 4, 8);
        memcpy(&cpu, ranges + off + 12, 8);
        memcpy(&size, ranges + off + 20, 8);
        if ((flags & 0x03000000) == 0x02000000 && pci == 0xc0000000ULL &&
            cpu == 0x5c0000000ULL && size >= 0x4000000)
            memory_window = true;
    }
    if (!memory_window || pcie_init_wlan())
        return false;
    aic_set_mask(1270, true);
    aic_set_mask(1271, true);
    write32(ecam + 0x18, 0x00010100); /* root0 -> secondary1/subordinate1 */
    endpoint = ecam + 0x100000;
    if (read32(endpoint) != 0x443314e4)
        return false;
    write16(endpoint + 4, 1u << 10); /* bus mastering remains OFF */
    if ((read32(endpoint + 0x1000) & 0xffff) == 0x14e4)
        write16(endpoint + 0x1004, 1u << 10); /* BT must not DMA through Wi-Fi SID */
    u32 f0, f2;
    u64 s0 = size_bar(0x10, &f0), s2 = size_bar(0x18, &f2);
    if (s0 < 0x8000 || s0 > 0x100000 || s2 < 0x200000 || s2 > 0x2000000 ||
        (s0 & (s0 - 1)) || (s2 & (s2 - 1)))
        return false;
    u32 pci0 = 0xc0000000;
    u32 pci2 = (pci0 + s0 + s2 - 1) & ~(s2 - 1);
    if ((u64)pci2 + s2 > 0xc4000000ULL)
        return false;
    write32(endpoint + 0x10, pci0 | f0);
    if ((f0 & 6) == 4) write32(endpoint + 0x14, 0);
    write32(endpoint + 0x18, pci2 | f2);
    if ((f2 & 6) == 4) write32(endpoint + 0x1c, 0);
    write32(ecam + 0x20, ((pci2 + s2 - 1) & 0xfff00000u) | (pci0 >> 16));
    write32(ecam + 0x24, 0x0000fff0); /* disable prefetch window */
    write16(ecam + 4, 6);
    bar0 = 0x500000000ULL + pci0;
    bar2 = 0x500000000ULL + pci2;
    bar2_size = s2;
    /* Standard PCI MSI/MSI-X capabilities: use bounded polling, never ITS. */
    u8 cap = read8(endpoint + 0x34) & ~3u;
    for (u32 n = 0; cap >= 0x40 && n < 48; n++) {
        u8 id = read8(endpoint + cap);
        u16 control = read16(endpoint + cap + 2);
        if (id == 5) write16(endpoint + cap + 2, control & ~1u);
        if (id == 0x11) write16(endpoint + cap + 2, (control | 0x4000) & ~0x8000u);
        u8 next = read8(endpoint + cap + 1) & ~3u;
        if (next == cap) break;
        cap = next;
    }
    for (u32 i = 0; i < 64; i++) write32(WLAN_PORT + 0x828 + i * 4, 0);
    write32(WLAN_PORT + 0x828, BIT(31) | (1u << 16) | 0x100);
    if (read32(WLAN_PORT + 0x828) != (BIT(31) | (1u << 16) | 0x100))
        return false;
    dma_pool = memalign(SZ_16K, DMA_SIZE);
    if (!dma_pool || !dart_t6000_map_exclusive(WLAN_DART, 1, DMA_IOVA,
                                              (u64)dma_pool, DMA_SIZE))
        return false;
    dma_used = 0;
    write16(endpoint + 4, (1u << 10) | 6);
    return true;
}

#ifdef J316_WIFI_AUTOJOIN
static u16 ip_csum(const u8 *p, u32 n)
{
    u32 sum = 0;
    for (u32 i = 0; i + 1 < n; i += 2) sum += ((u32)p[i] << 8) | p[i + 1];
    while (sum >> 16) sum = (sum & 0xffff) + (sum >> 16);
    return (u16)~sum;
}
/* End-to-end data-path check before guest entry: broadcast DHCPDISCOVER through
 * the TX flow ring, wait for the router's DHCPOFFER on the RX path. The offer is
 * never accepted, so Windows performs its own DHCP afterwards. */
static bool dhcp_probe(void)
{
    static u8 f[342], rx[1600];
    const u32 xid = 0x4a333136;
    memset(f, 0, sizeof(f));
    memset(f, 0xff, 6);
    memcpy(f + 6, j316_wifi_mac, 6);
    f[12] = 0x08; f[13] = 0x00;
    u8 *ip = f + 14, *udp = ip + 20, *bp = udp + 8;
    ip[0] = 0x45; ip[2] = (328 >> 8); ip[3] = 328 & 0xff; ip[8] = 64; ip[9] = 17;
    memset(ip + 16, 0xff, 4);
    u16 c = ip_csum(ip, 20); ip[10] = c >> 8; ip[11] = c & 0xff;
    udp[1] = 68; udp[3] = 67; udp[4] = 308 >> 8; udp[5] = 308 & 0xff;
    bp[0] = 1; bp[1] = 1; bp[2] = 6;
    bp[4] = (u8)(xid >> 24); bp[5] = (u8)(xid >> 16); bp[6] = (u8)(xid >> 8); bp[7] = (u8)xid;
    bp[10] = 0x80; /* broadcast reply */
    memcpy(bp + 28, j316_wifi_mac, 6);
    u8 *o = bp + 236;
    static const u8 opts[] = {99, 130, 83, 99, 53, 1, 1, 55, 3, 1, 3, 6, 255};
    memcpy(o, opts, sizeof(opts));
    u64 end = time_us() + 10u * 1000000u, next_tx = 0;
    u32 frames = 0;
    int last = 0;
    while (time_us() < end) {
        bcm_wifi_poll(64);
        if ((bcm_wifi_phase() & 0x80000000u) || !bcm_wifi_link()) {
            printf("J316 Wi-Fi: DATA TEST FAILED - link lost phase=%x error=%d\n", bcm_wifi_phase(),
                   bcm_wifi_error());
            return false;
        }
        if (time_us() >= next_tx) {
            last = bcm_wifi_send(f, sizeof(f));
            if (last == 0) next_tx = time_us() + 2000000;
            else if (last != -11) next_tx = time_us() + 500000;
        }
        int n;
        while ((n = bcm_wifi_receive(rx, sizeof(rx))) > 0) {
            frames++;
            const u8 *ri = rx + 14, *ru = ri + 20, *rb = ru + 8;
            if (n < 14 + 20 + 8 + 240 || rx[12] != 0x08 || rx[13] != 0 || ri[9] != 17 ||
                (ri[0] & 0x0f) != 5 || ru[0] != 0 || ru[1] != 67 || ru[3] != 68 || rb[0] != 2 ||
                rb[4] != (u8)(xid >> 24) || rb[5] != (u8)(xid >> 16) || rb[6] != (u8)(xid >> 8) ||
                rb[7] != (u8)xid)
                continue;
            printf("J316 Wi-Fi: DATA PATH OK - router %u.%u.%u.%u offered %u.%u.%u.%u\n", ri[12],
                   ri[13], ri[14], ri[15], rb[16], rb[17], rb[18], rb[19]);
            return true;
        }
        udelay(1000);
    }
    printf("J316 Wi-Fi: DATA TEST FAILED - no DHCP offer in 10 s (send=%d rx_frames=%u error=%d)\n",
           last, frames, bcm_wifi_error());
    return false;
}
#endif
bool hv_j316s_wifi_mapped(void) { return mapped; }
bool hv_j316s_wifi_init(bool enable)
{
    window = calloc(1, JW_SIZE);
    if (!window || hv_map_hook(JW_BASE, bridge_mmio, JW_SIZE))
        return false;
    mapped = true;
    store_reg(JW_REG_MAGIC, JW_MAGIC);
    store_reg(JW_REG_VERSION, JW_VERSION);
    if (!enable)
        return true;
#ifdef M1N1_J316S_WIFI
    store_reg(JW_REG_PHASE, 1);
    if (!hardware_init()) {
        store_reg(JW_REG_ERROR, 0xe101);
        if (endpoint) write16(endpoint + 4, 1u << 10);
        printf("J316 Wi-Fi: PCIe/DART initialization failed; Windows boot continues\n");
        udelay(90u * 1000000u); /* keep the failure readable before Mu takes the screen */
        return true;
    }
    static const struct bcm_platform ops = {bp_read, bp_write, ram_read, ram_write,
                                    dma_alloc, dma_sync, no_doorbell, time_us, udelay,
                                    pcie_alias_read, pcie_alias_write, cfg_write};
    int result = bcm_wifi_init(&ops, &j316_wifi_firmware, j316_wifi_mac);
    ready = result == 0 && !io_error;
    if (!ready) write16(endpoint + 4, 1u << 10);
    u32 mac_low; u16 mac_high;
    memcpy(&mac_low, j316_wifi_mac, 4); memcpy(&mac_high, j316_wifi_mac + 4, 2);
    store_reg(JW_REG_MAC_LOW, mac_low); store_reg(JW_REG_MAC_HIGH, mac_high);
    store_reg(JW_REG_ERROR, io_error ? io_error : (u32)result);
    store_reg(JW_REG_PHASE, bcm_wifi_phase());
    store_reg(JW_REG_STATUS, ready ? JW_STATUS_READY : 0);
    printf("J316 Wi-Fi: firmware result=%d phase=%u; Windows boot continues\n",
           result, bcm_wifi_phase());
    if (!ready) {
        printf("J316 Wi-Fi: FAILED result=%d io=%x phase=%x; holding 90 s\n", result, io_error,
               bcm_wifi_phase());
        bcm_wifi_dump(debug_printf);
        udelay(90u * 1000000u);
    }
#ifdef J316_WIFI_AUTOJOIN
    if (ready) {
        /* Join before guest entry: bounded, and the outcome is shown on screen so no
         * Windows-side tooling is needed. The guest then simply sees link up. */
        int join = bcm_wifi_connect(j316_wifi_ssid, sizeof(j316_wifi_ssid) - 1,
                                    j316_wifi_psk, sizeof(j316_wifi_psk) - 1, debug_printf);
        store_reg(JW_REG_PHASE, bcm_wifi_phase());
        store_reg(JW_REG_ERROR, io_error ? io_error : bcm_wifi_error());
        store_reg(JW_REG_STATUS, JW_STATUS_READY | (bcm_wifi_link() ? JW_STATUS_LINK : 0));
        printf("J316 Wi-Fi: join=%d link=%d phase=%u error=%d io=%x\n", join,
               bcm_wifi_link(), bcm_wifi_phase() & 0x7fffffffu, bcm_wifi_error(), io_error);
        if (!bcm_wifi_link())
            udelay(90u * 1000000u); /* keep the failure readable on the boot screen */
        else if (!dhcp_probe())
            udelay(30u * 1000000u);
        store_reg(JW_REG_PHASE, bcm_wifi_phase());
        store_reg(JW_REG_ERROR, bcm_wifi_error());
        store_reg(JW_REG_STATUS, JW_STATUS_READY | (bcm_wifi_link() ? JW_STATUS_LINK : 0));
    }
#endif
#else
    store_reg(JW_REG_ERROR, 0xe100);
#endif
    return true;
}

void hv_j316s_wifi_poll(void)
{
    if (!mapped || !ready || time_us() < next_poll)
        return;
    next_poll = time_us() + 2000;
    bcm_wifi_poll(16);
    store_reg(JW_REG_PHASE, bcm_wifi_phase());
    store_reg(JW_REG_ERROR, bcm_wifi_error());
    store_reg(JW_REG_STATUS, JW_STATUS_READY | (bcm_wifi_link() ? JW_STATUS_LINK : 0));
    u32 cmd = __atomic_exchange_n((u32 *)(window + JW_REG_COMMAND), 0, __ATOMIC_ACQ_REL);
    if (cmd == JW_COMMAND_JOIN) {
        u32 ssid_len = load_reg(JW_CONFIG_SSID_LEN), psk_len = load_reg(JW_CONFIG_PSK_LEN);
        int error = -1;
        if (ssid_len && ssid_len <= 32 && psk_len >= 8 && psk_len <= 63)
            error = bcm_wifi_join(window + JW_CONFIG_SSID, ssid_len,
                                   window + JW_CONFIG_PSK, psk_len);
        memset(window + JW_CONFIG_PSK, 0, 64);
        store_reg(JW_REG_COMMAND_STATUS, error);
    }
    u32 tx = load_reg(JW_REG_TX_CONS), produced = load_reg(JW_REG_TX_PROD);
    if (produced - tx > JW_RING_COUNT) {
        store_reg(JW_REG_ERROR, 0xe201);
        store_reg(JW_REG_TX_CONS, produced);
        return;
    }
    for (u32 n = 0; tx != produced && n < 8; n++) {
        u8 *slot = window + JW_TX_BASE + (tx % JW_RING_COUNT) * JW_SLOT_SIZE;
        u32 size; memcpy(&size, slot, 4);
        if (size >= 14 && size <= JW_FRAME_MAX) {
            int result = bcm_wifi_send(slot + 4, size);
            if (result == -11)
                break; /* flow ring being created or full: retry this frame */
            /* Link down (-100) or error: drop, so the guest queue never jams. */
            if (result < 0)
                store_reg(JW_REG_ERROR, result);
        }
        store_reg(JW_REG_TX_CONS, ++tx);
    }
    u32 rx = load_reg(JW_REG_RX_PROD), consumed = load_reg(JW_REG_RX_CONS);
    if (rx - consumed > JW_RING_COUNT) {
        store_reg(JW_REG_ERROR, 0xe202);
        return;
    }
    for (u32 n = 0; rx - consumed < JW_RING_COUNT && n < 8; n++) {
        u8 *slot = window + JW_RX_BASE + (rx % JW_RING_COUNT) * JW_SLOT_SIZE;
        int size = bcm_wifi_receive(slot + 4, JW_FRAME_MAX);
        if (size <= 0) break;
        if (size < 14 || (u32)size > JW_FRAME_MAX) continue;
        memcpy(slot, &size, 4);
        store_reg(JW_REG_RX_PROD, ++rx);
    }
}

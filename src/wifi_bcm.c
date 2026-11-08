/* SPDX-License-Identifier: ISC
 * Copyright (c) 2013-2014 Broadcom Corporation
 * ISC permission/warranty notice and exact source references:
 * see wifi_bcm_proto.h (distributed with this implementation).
 */
#include <string.h>
#include "sep.h"
#include "wifi_bcm_proto.h"

struct bcm_core { u32 base, wrap; u16 id; u8 rev; };
/* Security/association parameters, replayed for every (re)connect. */
struct bcm_cfg {
    bool valid, secure, pinned;
    u8 ie[24], pmk[32], bssid[6];
    u32 ie_len, wpa_auth, wsec, mfp;
    u16 chanspec;
};
struct bcm_sec { bool present; u8 group; u16 pair, akm, caps; }; /* bit n = suite type n */
struct bcm_ap {
    bool found, privacy;
    u8 bssid[6], ctl_ch;
    u16 chanspec;
    s16 rssi;
    struct bcm_sec rsn, wpa;
};
struct bcm_ring {
    struct bcm_dma dma;
    u32 wi, ri;       /* chip-RAM index addresses (TCM-index mode) */
    u8 *wp, *rp;      /* host-memory index slots (DMA-index mode) */
    u16 depth, item, w, r;
};
struct bcm_buffer {
    struct bcm_dma dma;
    u16 len, offset;
    u8 kind, state; /* 0 available, 1 device-owned, 2 receive queue */
};
static struct {
    struct bcm_platform p;
    struct bcm_core cores[64];
    u32 ncores, ram_size, shared, flags, rxoffset, phase;
    struct bcm_core *arm, *pcie, *mem;
    struct bcm_ring rings[5], flow;
    struct bcm_buffer buf[BCM_NBUF], tx[BCM_NTX];
    struct bcm_dma ctl, scratch, ringupd;
    struct bcm_dma idx[4]; /* h2d_w, h2d_r, d2h_w, d2h_r; one writer each, own cache lines */
    u8 idx_sz;             /* 0: chip-RAM indices, 2/4: host-memory indices */
    u16 rxq[BCM_NRX], qread, qwrite, qcount;
    u16 rxlimit, post_cursor, trans, ctl_len;
    u32 ctl_cmd, mb_in, mb_out;
    u64 ctl_deadline, join_deadline, flow_deadline;
    int error, ctl_status;
    u8 mac[6], bssid[6], ssid[32], key[64];
    u32 ssid_len, key_len, join_step, flow_state;
    u8 next_ring, poll_turn;
    bool attempted, running, ctl_pending, ctl_done, associated, keyed, joining, raw_key;
    /* Boot-time connect: escan result for the target SSID and the failing event. */
    bool scanning;
    u32 scan_status, ev_type, ev_status, ev_reason, ev_seen;
    struct bcm_ap ap;
    struct bcm_cfg cfg;
    bool reconnect;
    u32 fails;
    u64 reconnect_at;
} w;

static int fail(int error)
{
    w.error = error;
    w.phase |= BCM_FATAL;
    w.associated = w.keyed = false;
    return error;
}
static bool ram_valid(u32 a, u32 n)
{
    return a >= BCM_RAM_BASE && n <= w.ram_size && a - BCM_RAM_BASE <= w.ram_size - n;
}
static u32 ram32(u32 a)
{
    u8 v[4];
    w.p.read_ram(a, v, sizeof(v));
    return bcm_get32(v);
}
static u16 ram16(u32 a)
{
    u8 v[2];
    w.p.read_ram(a, v, sizeof(v));
    return bcm_get16(v);
}
static void wr32(u32 a, u32 v)
{
    u8 b[4];
    bcm_put32(b, v);
    w.p.write_ram(a, b, sizeof(b));
}
static void wr16(u32 a, u16 v)
{
    u8 b[2];
    bcm_put16(b, v);
    w.p.write_ram(a, b, sizeof(b));
}
static bool dma_alloc(u32 n, struct bcm_dma *d)
{
    if (!w.p.dma_alloc(n, d) || !d->cpu || d->size < n ||
        d->iova + n < d->iova || (d->iova & 7))
        return false;
    memset(d->cpu, 0, n);
    w.p.dma_sync(d);
    return true;
}
static void pcie_wr(u32 off, u32 value)
{
    /* Linux uses the BAR0+0x2000 alias; also write the core-window route, so a
     * wrong assumption about either path cannot lose the doorbell. */
    if (w.p.write_pcie32) w.p.write_pcie32(off, value);
    w.p.write32(w.pcie->base + off, value);
}
static void bell(bool ready)
{
    /* pcie.c: DAR selection is negotiated, not inferred from a fixed revision. */
    u32 off = (w.flags & BCM_SHARED_DAR) ? 0xa20 : 0x140;
    pcie_wr(off + (ready ? 4 : 0), 1);
}

static u8 *reserve(struct bcm_ring *r);
static void commit(struct bcm_ring *r);
/* pcie.c brcmf_pcie_send_mb_data(): control-ring message unless the firmware
 * asked for the TCM mailbox (BRCMF_PCIE_SHARED_USE_MAILBOX). */
static void send_mb(u32 data)
{
    if (!(w.flags & BCM_SHARED_MAILBOX)) {
        u8 *p = reserve(&w.rings[0]);
        if (!p) return; /* the firmware re-requests; a full ring is transient */
        p[0] = BCM_H2D_MB;
        p[1] = 0xff;
        bcm_put32(p + 8, data);
        commit(&w.rings[0]);
        return;
    }
    if (!w.mb_in || ram32(w.mb_in)) return; /* previous transaction still pending */
    wr32(w.mb_in, data);
    if (w.p.write_cfg32) {
        w.p.write_cfg32(0x98, 1); /* BRCMF_PCIE_REG_SBMBX */
        if (w.pcie->rev <= 13) w.p.write_cfg32(0x98, 1);
    }
}
static void d2h_mb(u32 bits)
{
    if (bits & 0x10000000) { fail(-5); return; } /* BRCMF_D2H_DEV_FWHALT */
    /* BRCMF_D2H_DEV_DS_ENTER_REQ: host-cap DS_NO_OOB_DW was offered, so the
     * device wakes itself on the next doorbell; Linux simply ACKs. */
    if (bits & 2) send_mb(2); /* BRCMF_H2D_HOST_DS_ACK */
}

/* All reset polling below is confined to preguest init. */
static int core_disable(struct bcm_core *c, u32 pre, u32 reset)
{
    if (!c || !c->wrap)
        return -95;
    if (!(w.p.read32(c->wrap + BCM_RESETCTRL) & 1)) {
        w.p.write32(c->wrap + BCM_IOCTRL, pre | 3);
        (void)w.p.read32(c->wrap + BCM_IOCTRL);
        w.p.write32(c->wrap + BCM_RESETCTRL, 1);
        for (u32 i = 0; i < 30; i++) {
            w.p.delay_us(10);
            if (w.p.read32(c->wrap + BCM_RESETCTRL) == 1)
                goto reset_asserted;
        }
        return -110;
    }
reset_asserted:
    w.p.write32(c->wrap + BCM_IOCTRL, reset | 3);
    (void)w.p.read32(c->wrap + BCM_IOCTRL);
    return 0;
}
static int core_reset(struct bcm_core *c, u32 pre, u32 reset, u32 post)
{
    int err = core_disable(c, pre, reset);
    if (err)
        return err;
    for (u32 i = 0; i < 50; i++) {
        w.p.write32(c->wrap + BCM_RESETCTRL, 0);
        w.p.delay_us(50);
        if (!(w.p.read32(c->wrap + BCM_RESETCTRL) & 1)) {
            w.p.write32(c->wrap + BCM_IOCTRL, post | 1);
            (void)w.p.read32(c->wrap + BCM_IOCTRL);
            return 0;
        }
    }
    return -110;
}
static u32 desc_type(u32 v)
{
    u32 t = v & 15;
    return (t & ~8u) == 5 ? 5 : t;
}
static int enumerate(void)
{
    u32 id = w.p.read32(BCM_ENUM_BASE), erom = w.p.read32(BCM_ENUM_BASE + 0xfc);
    if ((id & 0xffff) != 0x4387 || ((id >> 16) & 15) != 7 || (id >> 28) != 1)
        return -95;
    if (!erom || erom == 0xffffffff || (erom & 3) || erom > 0xffffefff)
        return -71;
    u32 pos = erom, end = erom + 4096;
    bool eot = false;
    while (pos < end) {
        u32 a = w.p.read32(pos);
        pos += 4;
        if (desc_type(a) == 15) { eot = true; break; }
        if (!(a & 1) || desc_type(a) != 1)
            continue;
        if (pos == end)
            return -71;
        u32 b = w.p.read32(pos);
        pos += 4;
        if (desc_type(b) != 1)
            return -71;
        u32 base = 0, wrap = 0, wraptype = 2;
        bool first = true;
        while (pos < end) {
            u32 d = w.p.read32(pos), type = desc_type(d);
            if (type == 1 || type == 15)
                break;
            pos += 4;
            if (first) { wraptype = type == 3 ? 3 : 2; first = false; }
            if (type != 5)
                continue;
            /* Reject >32-bit register regions instead of truncating them. */
            bool high = false;
            if (d & 8) {
                if (pos == end) return -71;
                high = w.p.read32(pos) != 0;
                pos += 4;
            }
            u32 sz = (d >> 4) & 3;
            if (sz == 3) {
                if (pos == end) return -71;
                u32 sd = w.p.read32(pos);
                pos += 4;
                if (sd & 8) {
                    if (pos == end) return -71;
                    pos += 4;
                }
            }
            if (high || sz > 1)
                continue;
            u32 st = (d >> 6) & 3;
            if (!base && st == 0) base = d & 0xfffff000;
            if (!wrap && st == wraptype) wrap = d & 0xfffff000;
        }
        u16 cid = (a >> 8) & 0xfff;
        if (!base || !wrap || !(((b >> 14) & 31) + ((b >> 19) & 31)))
            continue;
        if (w.ncores == 64)
            return -95;
        struct bcm_core *c = &w.cores[w.ncores++];
        *c = (struct bcm_core){.base = base, .wrap = wrap, .id = cid, .rev = b >> 24};
        if (cid == BCM_CORE_CR4 || cid == BCM_CORE_CA7) {
            if (w.arm) return -95;
            w.arm = c;
        }
        if (cid == BCM_CORE_PCIE2) {
            if (w.pcie) return -95;
            w.pcie = c;
        }
        if (cid == BCM_CORE_SYSMEM) {
            if (w.mem) return -95;
            w.mem = c;
        }
    }
    if (!eot || !w.arm || !w.pcie || (w.arm->id == BCM_CORE_CA7 && !w.mem))
        return -95;
    return 0;
}
static int passive_and_ram(void)
{
    int err = core_reset(w.arm, w.p.read32(w.arm->wrap + BCM_IOCTRL) & BCM_CPUHALT,
                         BCM_CPUHALT, BCM_CPUHALT);
    if (err) return err;
    bool d11 = false;
    for (u32 i = 0; i < w.ncores; i++) {
        if (w.cores[i].id != BCM_CORE_D11) continue;
        d11 = true;
        /* BCM4387 requires holding ALL D11 cores in reset. */
        if ((err = core_disable(&w.cores[i], 12, 4))) return err;
    }
    if (!d11) return -95;
    if (w.arm->id == BCM_CORE_CR4) {
        u32 cap = w.p.read32(w.arm->base + 4);
        if (cap == 0xffffffff) return -5;
        u32 nb = (cap & 15) + ((cap >> 4) & 15);
        for (u32 i = 0; i < nb; i++) {
            w.p.write32(w.arm->base + 0x40, i);
            u32 v = w.p.read32(w.arm->base + 0x44);
            if (v == 0xffffffff) return -5;
            w.ram_size += ((v & 127) + 1) * ((v & 0x200) ? 1024 : 8192);
        }
    } else {
        if (!(w.p.read32(w.mem->wrap + BCM_IOCTRL) & 1) ||
            (w.p.read32(w.mem->wrap + BCM_RESETCTRL) & 1)) {
            if ((err = core_reset(w.mem, 0, 0, 0))) return err;
        }
        u32 info = w.p.read32(w.mem->base);
        if (info == 0xffffffff) return -5;
        u32 rom = w.mem->rev >= 12 ? info >> 24 : (info >> 5) & 31;
        u32 nb = w.mem->rev >= 12 ? (info >> 16) & 255 : info & 31;
        if (nb + rom > 256) return -95;
        for (u32 i = 0; i < nb; i++) {
            w.p.write32(w.mem->base + 0x10, i + rom);
            u32 v = w.p.read32(w.mem->base + 0x40);
            if (v == 0xffffffff) return -5;
            w.ram_size += ((v & 127) + 1) * 8192;
        }
    }
    return w.ram_size && w.ram_size <= 0x400000 ? 0 : -95;
}

static u8 nvram[64000];
static int make_nvram(const struct bcm_blob *blob, u32 *length)
{
    if (!blob->data || !blob->size || blob->size > sizeof(nvram) - 128)
        return -22;
    /* Accept an already normalized/tokenized single-board bundle as well. */
    size_t size = blob->size;
    if (size >= 8 && !(size & 3)) {
        u32 token = bcm_get32(blob->data + size - 4), n = (size - 4) / 4;
        if (token == ((n & 0xffff) | ((~n & 0xffff) << 16)))
            size -= 4;
    }
    u32 out = 0;
    bool boardrev = false;
    for (size_t i = 0; i < size;) {
        while (i < size && (blob->data[i] == ' ' || blob->data[i] == '\t' ||
               blob->data[i] == '\r' || blob->data[i] == '\n' || !blob->data[i])) i++;
        size_t start = i, stop, eq = size;
        while (i < size && blob->data[i] && blob->data[i] != '\n' &&
               blob->data[i] != '\r' && blob->data[i] != '#') {
            if (eq == size && blob->data[i] == '=') eq = i;
            if (blob->data[i] < 32 || blob->data[i] >= 127) return -22;
            i++;
        }
        stop = i;
        if (i < size && blob->data[i] == '#')
            while (i < size && blob->data[i] && blob->data[i] != '\n') i++;
        if (eq == size || eq == start) continue;
        /* No PCI domain/bus arguments in public API: reject multiboard files. */
        if ((eq - start >= 7 && !memcmp(blob->data + start, "devpath", 7)) ||
            (eq - start >= 5 && !memcmp(blob->data + start, "pcie/", 5))) return -95;
        /* Apple's file has keys such as "slice/1/olpc_thresh2g =32": trim the
         * padding before '=' (it is not part of the key name). */
        size_t kend = eq;
        while (kend > start && (blob->data[kend - 1] == ' ' || blob->data[kend - 1] == '\t'))
            kend--;
        if (kend == start) continue;
        if (kend - start == 4 && !memcmp(blob->data + start, "RAW1", 4)) continue;
        if (kend - start == 7 && !memcmp(blob->data + start, "macaddr", 7)) continue;
        for (size_t k = start; k < kend; k++)
            if (blob->data[k] == ' ') return -22;
        if (kend - start == 8 && !memcmp(blob->data + start, "boardrev", 8)) boardrev = true;
        u32 klen = kend - start, vlen = stop - eq;
        if (out + klen + vlen + 128 >= sizeof(nvram)) return -90;
        memcpy(nvram + out, blob->data + start, klen);
        out += klen;
        memcpy(nvram + out, blob->data + eq, vlen);
        out += vlen;
        nvram[out++] = 0;
    }
    if (!out) return -22;
    if (!boardrev) {
        static const char def[] = "boardrev=0xff";
        memcpy(nvram + out, def, sizeof(def));
        out += sizeof(def);
    }
    static const char hex[] = "0123456789abcdef";
    memcpy(nvram + out, "macaddr=", 8);
    out += 8;
    for (u32 i = 0; i < 6; i++) {
        nvram[out++] = hex[w.mac[i] >> 4];
        nvram[out++] = hex[w.mac[i] & 15];
        nvram[out++] = i == 5 ? 0 : ':';
    }
    u32 padded = (out + 4) & ~3u;
    memset(nvram + out, 0, padded - out);
    u32 words = padded / 4;
    bcm_put32(nvram + padded, words | ((~words & 0xffff) << 16));
    *length = padded + 4;
    return 0;
}
static int boot(const struct bcm_fw_bundle *b, const u8 seed[256], u32 nvlen)
{
    const struct bcm_blob *fw = &b->firmware;
    if (fw->size >= 0x74 && bcm_get32(fw->data + 0x6c) == 0x534d4152) {
        u32 n = bcm_get32(fw->data + 0x70);
        if (!n || (n & 3) || n > w.ram_size) return -95;
        w.ram_size = n;
    }
    u32 top = BCM_RAM_BASE + w.ram_size;
    /* Seed rTLV and bootloader heap gap follow current pcie.c allocation rule. */
    if (fw->size + nvlen + 264 + 0x11004 > w.ram_size) return -90;
    w.p.write_ram(BCM_RAM_BASE, fw->data, fw->size);
    u32 addr = top - nvlen;
    w.p.write_ram(addr, nvram, nvlen);
    u8 footer[8];
    bcm_put32(footer, 256);
    bcm_put32(footer + 4, 0xfeedc0de);
    w.p.write_ram(addr - 8, footer, 8);
    addr -= 264;
    w.p.write_ram(addr, seed, 256);
    u8 zero[256] = {0};
    for (u32 p = BCM_RAM_BASE + fw->size; p < addr;) {
        u32 n = addr - p;
        if (n > sizeof(zero)) n = sizeof(zero);
        w.p.write_ram(p, zero, n);
        p += n;
    }
    /* Mask PCIe mailbox interrupts: poll owns progress; no IRQ API is required.
     * Platform must have masked host PCI/MSI before granting ownership. */
    u32 status = w.pcie->rev >= 64 ? 0xc30 : 0x48;
    pcie_wr(status + 4, 0);
    pcie_wr(status, 0xffffffff);
    u32 written = ram32(top - 4);
    /* This is TCM address zero, NOT rambase; platform must special-case it. */
    wr32(0, bcm_get32(fw->data));
    int err = core_reset(w.arm, BCM_CPUHALT, 0, 0);
    if (err) return err;
    for (u32 i = 0; i < 100; i++) {
        u32 ptr = ram32(top - 4);
        if (ptr != written) {
            if ((ptr & 3) || !ram_valid(ptr, 120)) return -71;
            w.shared = ptr;
            return 0;
        }
        w.p.delay_us(50000);
    }
    return -110;
}

static u16 idx_load(const u8 *p)
{
    struct bcm_dma t = {(void *)(uintptr_t)p, 0, w.idx_sz};
    w.p.dma_sync(&t); /* device-written slot: invalidate before reading */
    return w.idx_sz == 2 ? bcm_get16(p) : (u16)bcm_get32(p);
}
static void idx_store(u8 *p, u16 v)
{
    if (w.idx_sz == 2) bcm_put16(p, v); else bcm_put32(p, v);
    struct bcm_dma t = {p, 0, w.idx_sz};
    w.p.dma_sync(&t);
}
static u16 ring_wi(const struct bcm_ring *r) { return w.idx_sz ? idx_load(r->wp) : ram16(r->wi); }
static u16 ring_ri(const struct bcm_ring *r) { return w.idx_sz ? idx_load(r->rp) : ram16(r->ri); }
static void ring_set_wi(struct bcm_ring *r, u16 v)
{
    if (w.idx_sz) idx_store(r->wp, v); else wr16(r->wi, v);
}
static void ring_set_ri(struct bcm_ring *r, u16 v)
{
    if (w.idx_sz) idx_store(r->rp, v); else wr16(r->ri, v);
}
static int ring_alloc(struct bcm_ring *r, u16 depth, u16 item, u32 wi, u32 ri)
{
    if (!w.idx_sz && (!ram_valid(wi, 4) || !ram_valid(ri, 4) || (wi & 3) || (ri & 3))) return -71;
    if (!dma_alloc((u32)depth * item, &r->dma)) return -12;
    r->depth = depth;
    r->item = item;
    r->wi = wi;
    r->ri = ri;
    r->w = r->r = 0;
    return 0;
}
/* DMA-index mode: bind a ring to its slot in the h2d (arrays 0/1) or d2h (2/3) arrays. */
static void ring_bind_idx(struct bcm_ring *r, u32 first_array, u32 slot)
{
    r->wp = (u8 *)w.idx[first_array].cpu + slot * w.idx_sz;
    r->rp = (u8 *)w.idx[first_array + 1].cpu + slot * w.idx_sz;
}
static int setup_shared(void)
{
    w.flags = ram32(w.shared);
    u32 ver = w.flags & 255;
    /* Current source supports v5..7; require v6/7 explicit host caps. */
    if (ver != 6 && ver != 7) return -95;
    u32 info = ram32(w.shared + 48);
    if ((info & 3) || !ram_valid(info, 60)) return -71;
    u8 ri[60];
    w.p.read_ram(info, ri, sizeof(ri));
    u32 ringmem = bcm_get32(ri), hw = bcm_get32(ri + 4), hr = bcm_get32(ri + 8);
    u32 dw = bcm_get32(ri + 12), dr = bcm_get32(ri + 16);
    u16 nf = bcm_get16(ri + 52), ns = bcm_get16(ri + 54), nc = bcm_get16(ri + 56);
    if (!nf || nf > 512 || ns < 3 || ns > 514 || ns < (u32)nf + 2 || nc < 3 || nc > 512)
        return -95;
    if ((ringmem & 3) || !ram_valid(ringmem, 5 * 16) ||
        !ram_valid(hw, ns * 4) || !ram_valid(hr, ns * 4) ||
        !ram_valid(dw, nc * 4) || !ram_valid(dr, nc * 4)) return -71;
    w.rxoffset = ram32(w.shared + 36);
    /* This prefix applies to every D2H completion AND event/RX payloads. */
    if (w.rxoffset > (ver == 7 ? 8u : 0u)) return -95;
    w.rxlimit = ram16(w.shared + 34);
    if (!w.rxlimit) w.rxlimit = 255;
    if (w.rxlimit > BCM_NRX) w.rxlimit = BCM_NRX;
    w.mb_in = ram32(w.shared + 40);
    w.mb_out = ram32(w.shared + 44);
    if ((w.flags & BCM_SHARED_MAILBOX) &&
        (!ram_valid(w.mb_in, 4) || !ram_valid(w.mb_out, 4) || (w.mb_in & 3) || (w.mb_out & 3)))
        return -71;
    /* pcie.c: host-memory ("DMA") indices are used whenever the firmware offers them
     * (BRCMF_PCIE_SHARED_DMA_INDEX); chip-RAM indices only when that is absent. */
    u8 hostaddr[32] = {0};
    w.idx_sz = 0;
    if (w.flags & 0x10000) {
        w.idx_sz = (w.flags & 0x100000) ? 2 : 4;
        const u32 counts[4] = {ns, ns, nc, nc};
        for (u32 i = 0; i < 4; i++) {
            /* Separate 128-byte-aligned arrays: one writer per cache line. */
            u32 bytes = ((counts[i] * w.idx_sz + 127u) & ~127u) + 128u;
            if (!dma_alloc(bytes, &w.idx[i])) return -12;
            u64 mis = w.idx[i].iova & 127u;
            if (mis) {
                u64 adj = 128u - mis;
                w.idx[i].cpu = (u8 *)w.idx[i].cpu + adj;
                w.idx[i].iova += adj;
                w.idx[i].size -= adj;
            }
            bcm_put64(hostaddr + i * 8, w.idx[i].iova);
        }
    }
    w.p.write_ram(info + 20, hostaddr, sizeof(hostaddr));
    static const u16 depths[5] = {64, 1024, 64, 1024, 1024}; /* brcmfmac *_MAX_ITEM */
    u16 items[5] = {40, 32, 24, ver == 7 ? 24 : 16, ver == 7 ? 40 : 32};
    for (u32 i = 0; i < 5; i++) {
        int err = ring_alloc(&w.rings[i], depths[i], items[i],
                             i < 2 ? hw + i * 4 : dw + (i - 2) * 4,
                             i < 2 ? hr + i * 4 : dr + (i - 2) * 4);
        if (err) return err;
        if (w.idx_sz) ring_bind_idx(&w.rings[i], i < 2 ? 0 : 2, i < 2 ? i : i - 2);
        ring_set_wi(&w.rings[i], 0);
        ring_set_ri(&w.rings[i], 0);
        u8 desc[12];
        bcm_put16(desc, depths[i]);
        bcm_put16(desc + 2, items[i]);
        bcm_put64(desc + 4, w.rings[i].dma.iova);
        w.p.write_ram(ringmem + i * 16 + 4, desc, sizeof(desc));
    }
    int err = ring_alloc(&w.flow, BCM_FLOW_DEPTH, 48, hw + 8, hr + 8);
    if (err) return err;
    if (w.idx_sz) ring_bind_idx(&w.flow, 0, 2);
    ring_set_wi(&w.flow, 0);
    ring_set_ri(&w.flow, 0);
    if (!dma_alloc(8, &w.scratch) || !dma_alloc(1024, &w.ringupd) ||
        !dma_alloc(BCM_CTL_SIZE, &w.ctl)) return -12;
    wr32(w.shared + 52, 8);
    u8 addr[8];
    bcm_put64(addr, w.scratch.iova);
    w.p.write_ram(w.shared + 56, addr, 8);
    wr32(w.shared + 64, 1024);
    bcm_put64(addr, w.ringupd.iova);
    w.p.write_ram(w.shared + 68, addr, 8);
    u32 cap = ver | 0x1000; /* DS_NO_OOB_DW */
    if (w.flags & BCM_SHARED_HOSTREADY) cap |= 0x400;
    if (w.flags & BCM_SHARED_DAR) cap |= 0x10000;
    wr32(w.shared + 84, cap);
    wr32(w.shared + 112, 0);
    for (u32 i = 0; i < BCM_NBUF; i++) {
        if (i < BCM_NRX && i >= w.rxlimit) continue;
        w.buf[i].kind = i < BCM_NRX ? BCM_RX_POST :
                        i < BCM_NRX + 8 ? BCM_IOCTL_POST : BCM_EVENT_POST;
        if (!dma_alloc(i < BCM_NRX ? BCM_PKT_SIZE : BCM_CTL_SIZE, &w.buf[i].dma)) return -12;
    }
    for (u32 i = 0; i < BCM_NTX; i++)
        if (!dma_alloc(BCM_PKT_SIZE, &w.tx[i].dma)) return -12;
    return 0;
}
static u8 *reserve(struct bcm_ring *r)
{
    u16 rd = ring_ri(r);
    if (rd >= r->depth) { fail(-71); return NULL; }
    if ((r->w + 1) % r->depth == rd) return NULL;
    u8 *p = (u8 *)r->dma.cpu + r->w * r->item;
    memset(p, 0, r->item);
    return p;
}
static void commit(struct bcm_ring *r)
{
    w.p.dma_sync(&r->dma);
    r->w = (r->w + 1) % r->depth;
    ring_set_wi(r, r->w);
    bell(false);
}
static bool post(u32 id)
{
    struct bcm_buffer *b = &w.buf[id];
    struct bcm_ring *r = &w.rings[b->kind == BCM_RX_POST ? 1 : 0];
    u8 *p = reserve(r);
    if (!p) return false;
    w.p.dma_sync(&b->dma);
    p[0] = b->kind;
    bcm_put32(p + 4, id);
    if (b->kind == BCM_RX_POST) {
        bcm_put16(p + 10, BCM_PKT_SIZE);
        bcm_put64(p + 24, b->dma.iova);
    } else {
        bcm_put16(p + 8, BCM_CTL_SIZE);
        bcm_put64(p + 16, b->dma.iova);
    }
    b->state = 1;
    commit(r);
    return true;
}
static void refill(u32 *budget)
{
    for (u32 scanned = 0; scanned < BCM_NBUF && *budget; scanned++) {
        u32 id = w.post_cursor++;
        if (w.post_cursor == BCM_NBUF) w.post_cursor = 0;
        struct bcm_buffer *b = &w.buf[id];
        if (!b->kind || b->state) continue;
        if (post(id)) (*budget)--;
        if (w.phase & BCM_FATAL) break;
    }
}
static int ioctl_start(u32 cmd, const void *data, u32 length)
{
    if (w.ctl_pending || w.ctl_done) return -16;
    if (length > BCM_CTL_SIZE) return -90;
    u8 *p = reserve(&w.rings[0]);
    if (!p) return -11;
    memset(w.ctl.cpu, 0, BCM_CTL_SIZE);
    if (data && length) memcpy(w.ctl.cpu, data, length);
    w.p.dma_sync(&w.ctl);
    p[0] = BCM_IOCTL;
    bcm_put32(p + 4, 0xfffe);
    bcm_put32(p + 8, cmd);
    bcm_put16(p + 12, ++w.trans);
    bcm_put16(p + 14, length);
    bcm_put16(p + 16, length);
    bcm_put64(p + 24, w.ctl.iova);
    w.ctl_cmd = cmd;
    w.ctl_pending = true;
    w.ctl_deadline = w.p.time_us() + BCM_TIMEOUT;
    commit(&w.rings[0]);
    return 0;
}
static int iovar_start(const char *name, const void *data, u32 n, bool get)
{
    u8 buf[1536];
    u32 keylen = strlen(name) + 1;
    if (keylen + n > sizeof(buf)) return -90;
    memset(buf, 0, keylen + n);
    memcpy(buf, name, keylen);
    if (!get && n) memcpy(buf + keylen, data, n);
    return ioctl_start(get ? BCM_C_GET_VAR : BCM_C_SET_VAR, buf, keylen + n);
}
static int integer_start(u32 cmd, const char *name, u32 value)
{
    u8 v[4];
    bcm_put32(v, value);
    return name ? iovar_start(name, v, sizeof(v), false) : ioctl_start(cmd, v, sizeof(v));
}
static int wait_ioctl(void)
{
    for (u32 i = 0; i < 2100 && !(w.phase & BCM_FATAL); i++) {
        bcm_wifi_poll(64);
        /* Firmware publishes its shared pointer before it services doorbells; a
         * single early doorbell can be lost, so repeat it while a request is pending. */
        if (w.ctl_pending && (i % 20) == 19) bell(false);
        if (w.ctl_done) {
            w.ctl_done = false;
            return w.ctl_status;
        }
        w.p.delay_us(1000);
    }
    return fail(-110);
}
static int sync_var(const char *name, const void *data, u32 n, bool get)
{
    int err = iovar_start(name, data, n, get);
    return err ? err : wait_ioctl();
}
static int sync_int(u32 cmd, const char *name, u32 value)
{
    int err = integer_start(cmd, name, value);
    return err ? err : wait_ioctl();
}
static int download_blob(const struct bcm_blob *b, const char *name)
{
    if (!b->size) return 0;
    if (!b->data || b->size > 0xffffffffu) return -22;
    u8 chunk[1412];
    for (u32 off = 0; off < b->size;) {
        u32 n = b->size - off;
        if (n > 1400) n = 1400;
        u16 flags = 0x1000 | (off == 0 ? 2 : 0) | (off + n == b->size ? 4 : 0);
        bcm_put16(chunk, flags);
        bcm_put16(chunk + 2, 2);
        bcm_put32(chunk + 4, n);
        bcm_put32(chunk + 8, 0);
        memcpy(chunk + 12, b->data + off, n);
        int err = sync_var(name, chunk, n + 12, false);
        if (err) return err;
        off += n;
    }
    return 0;
}
static void join_failed(int error)
{
    w.error = error;
    w.associated = w.keyed = w.joining = false;
    w.phase = 5;
    memset(w.key, 0, sizeof(w.key));
    w.key_len = 0;
    /* Background reconnect with the stored configuration (Linux: supplicant retry). */
    if (w.reconnect) {
        w.fails++;
        w.reconnect_at = w.p.time_us() + (w.fails < 3 ? 2000000u : 10000000u);
    }
}
static void join_failed_ev(int error, u32 type, u32 status, u32 reason)
{
    w.ev_type = type;
    w.ev_status = status;
    w.ev_reason = reason;
    join_failed(error);
}

/* Security elements of a scanned BSS (IEEE 802.11-2020 9.4.2.24; WPA1 vendor IE). */
static void parse_suites(const u8 *d, u32 n, const u8 oui[3], bool rsn, struct bcm_sec *s)
{
    s->present = true;
    s->group = rsn ? 4 : 2; /* defaults when the element is truncated */
    s->pair = 1u << s->group;
    s->akm = 1u << 2;
    s->caps = 0;
    if (n < 4) return;
    if (!memcmp(d, oui, 3) && d[3] < 16) s->group = d[3];
    d += 4; n -= 4;
    if (n < 2) return;
    u32 c = bcm_get16(d);
    d += 2; n -= 2;
    if (c * 4 > n) return;
    s->pair = 0;
    for (u32 i = 0; i < c; i++, d += 4, n -= 4)
        if (!memcmp(d, oui, 3) && d[3] < 16) s->pair |= 1u << d[3];
    if (n < 2) return;
    c = bcm_get16(d);
    d += 2; n -= 2;
    if (c * 4 > n) return;
    s->akm = 0;
    for (u32 i = 0; i < c; i++, d += 4, n -= 4)
        if (!memcmp(d, oui, 3) && d[3] < 16) s->akm |= 1u << d[3];
    if (rsn && n >= 2) s->caps = bcm_get16(d);
}
static void parse_ies(const u8 *ie, u32 n, struct bcm_ap *a)
{
    static const u8 rsn_oui[3] = {0x00, 0x0f, 0xac}, wpa_oui[3] = {0x00, 0x50, 0xf2};
    memset(&a->rsn, 0, sizeof(a->rsn));
    memset(&a->wpa, 0, sizeof(a->wpa));
    for (u32 i = 0; i + 2 <= n;) {
        u32 id = ie[i], len = ie[i + 1];
        const u8 *d = ie + i + 2;
        if (len > n - i - 2) break;
        if (id == 48 && len >= 2 && bcm_get16(d) == 1)
            parse_suites(d + 2, len - 2, rsn_oui, true, &a->rsn);
        if (id == 221 && len >= 6 && !memcmp(d, wpa_oui, 3) && d[3] == 1 && bcm_get16(d + 4) == 1)
            parse_suites(d + 6, len - 6, wpa_oui, false, &a->wpa);
        i += 2 + len;
    }
}
/* brcmf_escan_result_le + brcmf_bss_info_le (fwil_types.h, natural alignment). */
static void escan_event(u32 status, const u8 *d, u32 n)
{
    if (status != 8) { /* BRCMF_E_STATUS_PARTIAL carries results; anything else ends it */
        w.scan_status = status;
        w.scanning = false;
        return;
    }
    if (n < 12 + 128) return;
    const u8 *bi = d + 12;
    u32 len = bcm_get32(bi + 4);
    if (len < 128 || len > n - 12) return;
    if (bi[18] != w.ssid_len || memcmp(bi + 19, w.ssid, w.ssid_len)) return;
    u32 ie_off = bcm_get16(bi + 116), ie_len = bcm_get32(bi + 120);
    if (ie_off > len || ie_len > len - ie_off) return;
    s16 rssi = (s16)bcm_get16(bi + 78);
    if (w.ap.found && rssi <= w.ap.rssi) return;
    struct bcm_ap a = {0};
    a.found = true;
    memcpy(a.bssid, bi + 8, 6);
    a.privacy = (bcm_get16(bi + 16) & 0x10) != 0;
    a.chanspec = bcm_get16(bi + 72);
    a.ctl_ch = bi[88];
    a.rssi = rssi;
    parse_ies(bi + ie_off, ie_len, &a);
    w.ap = a;
}
static void event_frame(const u8 *p, u32 n)
{
    static const u8 oui[3] = {0, 0x10, 0x18};
    if (n < 72 || bcm_be16(p + 12) != 0x886c ||
        memcmp(p + 19, oui, 3) || bcm_be16(p + 22) != 1 ||
        bcm_be16(p + 24) != 2 || p[70] || p[71]) return;
    u32 type = bcm_be32(p + 28), status = bcm_be32(p + 32), reason = bcm_be32(p + 36);
    u32 datalen = bcm_be32(p + 44);
    if (datalen > n - 72) return;
    if (type == BCM_E_ESCAN) {
        if (w.scanning) escan_event(status, p + 72, datalen);
        return;
    }
    if (!w.joining && w.phase != 8) return;
    if (w.joining) w.ev_seen |= 1u << (type < 31 ? type : 31);
    if (type == BCM_E_DEAUTH || type == BCM_E_DEAUTH_IND || type == BCM_E_DISASSOC_IND ||
        (type == BCM_E_LINK && !(bcm_be16(p + 26) & 1))) {
        join_failed_ev(-100, type, status, reason);
        return;
    }
    if (type == BCM_E_SSID) {
        if (status) { if (w.joining) join_failed_ev(-100, type, status, reason); return; }
        w.associated = true;
        w.ev_seen |= 0x40000000u; /* association succeeded */
        memcpy(w.bssid, p + 48, 6);
    }
    if (type == BCM_E_PSK) {
        if (status == 6) w.keyed = true;
        else if (status == 4 || status == 5 || status == 8 || status == 9 ||
                 status == 10 || status == 11) return;
        else if (w.joining) { join_failed_ev(-129, type, status, reason); return; }
        else return; /* connected: rekey progress, Linux ignores */
    }
    if (type == BCM_E_LINK && status == 3) {
        if (w.joining) join_failed_ev(-100, type, status, reason);
        return;
    }
    if (w.phase == 7 && w.associated && w.keyed) {
        w.joining = false;
        w.phase = 8;
        w.error = 0;
        w.fails = 0;
    }
}
static struct bcm_buffer *take(u32 id, u8 kind)
{
    if (id >= BCM_NBUF || w.buf[id].kind != kind || w.buf[id].state != 1) {
        fail(-71);
        return NULL;
    }
    struct bcm_buffer *b = &w.buf[id];
    w.p.dma_sync(&b->dma);
    b->state = 0;
    return b;
}
static void complete(const u8 *p, u32 ring)
{
    u32 id = bcm_get32(p + 4), type = p[0];
    int status = (s16)bcm_get16(p + 8);
    u32 flowid = bcm_get16(p + 10);
    if ((type == BCM_RX && ring != 4) || (type == BCM_TX_DONE && ring != 3) ||
        (type != BCM_RX && type != BCM_TX_DONE && ring != 2)) {
        fail(-71);
        return;
    }
    if (type == BCM_IOCTL_DONE) {
        struct bcm_buffer *b = take(id, BCM_IOCTL_POST);
        if (!b) return;
        if (!w.ctl_pending || bcm_get16(p + 14) != w.trans || bcm_get32(p + 16) != w.ctl_cmd) {
            fail(-71); return;
        }
        u32 n = bcm_get16(p + 12);
        if (n > BCM_CTL_SIZE) { fail(-71); return; }
        w.p.dma_sync(&w.ctl);
        if (n) memcpy(w.ctl.cpu, b->dma.cpu, n);
        w.ctl_len = n;
        w.ctl_status = status > 0 ? -5 : status;
        w.ctl_pending = false;
        w.ctl_done = true;
        return;
    }
    if (type == BCM_IOCTL_ACK) {
        /* msgbuf.c ignores the request ACK; the result arrives in IOCTL_CMPLT. */
        return;
    }
    if (type == BCM_FLOW_CREATED || type == BCM_FLOW_DELETED) {
        if (flowid != 2 || w.flow_state != (type == BCM_FLOW_CREATED ? 1u : 3u)) {
            fail(-71); return;
        }
        if (status) { fail(status < 0 ? status : -5); return; }
        w.flow_state = type == BCM_FLOW_CREATED ? 2 : 0;
        return;
    }
    if (type == BCM_EVENT) {
        struct bcm_buffer *b = take(id, BCM_EVENT_POST);
        if (!b) return;
        u32 n = bcm_get16(p + 12);
        if (n > BCM_CTL_SIZE - w.rxoffset) { fail(-71); return; }
        if (!status) event_frame((u8 *)b->dma.cpu + w.rxoffset, n);
        return;
    }
    if (type == BCM_RX) {
        struct bcm_buffer *b = take(id, BCM_RX_POST);
        if (!b) return;
        u32 n = bcm_get16(p + 14), off = bcm_get16(p + 16);
        if (!off) off = w.rxoffset;
        if (off > BCM_PKT_SIZE || n > BCM_PKT_SIZE - off) { fail(-71); return; }
        if (p[1] || status || (bcm_get16(p + 18) & 7) > 1 || n < 14 || n > 1518) return;
        const u8 *frame = (u8 *)b->dma.cpu + off;
        if (bcm_be16(frame + 12) == 0x886c) { event_frame(frame, n); return; }
        if (!bcm_wifi_link() || w.qcount == BCM_NRX) return;
        b->state = 2;
        b->len = n;
        b->offset = off;
        w.rxq[w.qwrite] = id;
        w.qwrite = (w.qwrite + 1) % BCM_NRX;
        w.qcount++;
        return;
    }
    if (type == BCM_TX_DONE) {
        if (!id || id > BCM_NTX || w.tx[id - 1].state != 1 || flowid != 2) {
            fail(-71); return;
        }
        w.p.dma_sync(&w.tx[id - 1].dma);
        w.tx[id - 1].state = 0;
        if (status || bcm_get16(p + 14)) w.error = -5;
        return;
    }
    if (type == BCM_D2H_MB) {
        d2h_mb(bcm_get32(p + 12));
        return;
    }
    /* GEN_STATUS/RING_STATUS and any other message type: msgbuf.c only logs. */
}
static void drain(struct bcm_ring *r, u32 which, u32 *budget)
{
    u16 wi = ring_wi(r);
    if (wi >= r->depth) { fail(-71); return; }
    if (wi == r->r || !*budget) return;
    w.p.dma_sync(&r->dma);
    while (r->r != wi && *budget && !(w.phase & BCM_FATAL)) {
        const u8 *p = (u8 *)r->dma.cpu + r->r * r->item + w.rxoffset;
        u32 available = r->item - w.rxoffset;
        u32 need;
        switch (p[0]) {
        case BCM_IOCTL_DONE: need = 20; break;
        case BCM_RX: need = 20; break;
        case BCM_EVENT: need = 14; break;
        case BCM_TX_DONE: case BCM_D2H_MB: need = 16; break;
        default: need = 12; break;
        }
        if (available < need) { fail(-95); return; }
        complete(p, which);
        r->r = (r->r + 1) % r->depth;
        (*budget)--;
    }
    ring_set_ri(r, r->r);
}
static void put_join(u8 *j, u32 *size, bool ext)
{
    const struct bcm_cfg *c = &w.cfg;
    bool pin = c->pinned && w.fails < 2;
    if (ext) { /* brcmf_ext_join_params_le */
        memset(j, 0, 72);
        bcm_put32(j, w.ssid_len);
        memcpy(j + 4, w.ssid, w.ssid_len);
        j[36] = 0xff;
        bcm_put32(j + 40, pin ? 320 / 20 : 0xffffffffu);
        bcm_put32(j + 44, pin ? 320 : 0xffffffffu);
        bcm_put32(j + 48, pin ? 400 : 0xffffffffu);
        bcm_put32(j + 52, 0xffffffffu);
        if (pin) memcpy(j + 56, c->bssid, 6); else memset(j + 56, 0xff, 6);
        if (pin) { bcm_put32(j + 64, 1); bcm_put16(j + 68, c->chanspec); }
        *size = pin ? 70 : 68;
    } else { /* brcmf_join_params */
        memset(j, 0, 52);
        bcm_put32(j, w.ssid_len);
        memcpy(j + 4, w.ssid, w.ssid_len);
        if (pin) memcpy(j + 36, c->bssid, 6); else memset(j + 36, 0xff, 6);
        if (pin) { bcm_put32(j + 44, 1); bcm_put16(j + 48, c->chanspec); }
        *size = pin ? 52 : 36;
    }
}
/* Asynchronous connect sequence (brcmf_cfg80211_connect order), driven by poll. */
static int join_command(void)
{
    const struct bcm_cfg *c = &w.cfg;
    u8 j[132];
    u32 n;
    int err;
    switch (w.join_step) {
    case 0: return ioctl_start(BCM_C_DISASSOC, NULL, 0);
    case 1: return iovar_start("wpaie", c->ie_len ? c->ie : NULL, c->ie_len, false);
    case 2: return integer_start(0, "wpa_auth", c->wpa_auth);
    case 3: return integer_start(BCM_C_AUTH, NULL, 0);
    case 4: return integer_start(0, "wsec", c->wsec);
    case 5: return integer_start(0, "mfp", c->mfp);
    case 6: return integer_start(0, "sup_wpa", c->secure ? 1 : 0);
    case 7:
        memset(j, 0, sizeof(j));
        bcm_put16(j, 32);
        memcpy(j + 4, c->pmk, 32);
        err = ioctl_start(BCM_C_PMK, j, 132);
        memset(j, 0, sizeof(j));
        return err;
    case 8:
    case 9:
        put_join(j, &n, w.join_step == 8);
        err = w.join_step == 8 ? iovar_start("join", j, n, false) : ioctl_start(BCM_C_SSID, j, n);
        if (!err) {
            w.joining = true;
            w.join_deadline = w.p.time_us() + 20000000;
        }
        return err;
    default: return -71;
    }
}
static void join_finish(void)
{
    w.phase = w.associated && w.keyed ? 8 : 7;
    if (w.phase == 8) { w.joining = false; w.fails = 0; w.error = 0; }
}
static void join_progress(void)
{
    if (w.phase == 5 && w.ctl_done) w.ctl_done = false;
    if (w.phase != 6 || w.ctl_pending) return;
    if (w.ctl_done) {
        w.ctl_done = false;
        int st = w.ctl_status;
        u32 s = w.join_step;
        if (s == 8 && !st) { join_finish(); return; }
        if (s == 9) { if (st) join_failed(st); else join_finish(); return; }
        bool optional = s == 0 || s == 1 || s == 8 || (s == 5 && !w.cfg.mfp);
        if (st && !optional) { join_failed(st); return; }
        w.join_step++;
        if (w.join_step == 7 && !w.cfg.secure) w.join_step = 8;
    }
    int err = join_command();
    if (err && err != -11) join_failed(err);
}
static void start_join(void)
{
    w.join_step = 0;
    w.phase = 6;
    w.associated = w.joining = false;
    w.keyed = !w.cfg.secure;
    while (w.qcount) {
        w.buf[w.rxq[w.qread]].state = 0;
        w.qread = (w.qread + 1) % BCM_NRX;
        w.qcount--;
    }
    w.qwrite = w.qread;
}
void bcm_wifi_poll(u32 budget)
{
    if (!w.running || (w.phase & BCM_FATAL) || !budget) return;
    if (budget > 64) budget = 64;
    u64 now = w.p.time_us();
    if (w.ctl_pending && now >= w.ctl_deadline) { fail(-110); return; }
    if ((w.flow_state == 1 || w.flow_state == 3) && now >= w.flow_deadline) {
        fail(-110); return;
    }
    /* Even budget=1 alternates completions and reposts. Rotate completion
     * priority so a busy control or TX ring cannot starve Ethernet receive. */
    u32 work = budget == 1 ? (++w.poll_turn & 1) : (budget + 1) / 2;
    u32 share = (work + 2) / 3, consumed = 0;
    for (u32 i = 0; i < 3 && work; i++) {
        u32 ring = 2 + (w.next_ring + i) % 3;
        u32 quota = work < share ? work : share, left = quota;
        drain(&w.rings[ring], ring, &left);
        consumed += quota - left;
        work -= quota - left;
        if (w.phase & BCM_FATAL) return;
    }
    w.next_ring = (w.next_ring + 1) % 3;
    if (w.phase & BCM_FATAL) return;
    if ((w.flags & BCM_SHARED_MAILBOX) && w.mb_out) {
        u32 mb = ram32(w.mb_out);
        if (mb) {
            wr32(w.mb_out, 0);
            d2h_mb(mb);
            if (w.phase & BCM_FATAL) return;
        }
    }
    u32 posts = budget - consumed;
    refill(&posts);
    if (w.phase & BCM_FATAL) return;
    join_progress();
    if (w.joining && now >= w.join_deadline) join_failed(-110);
    if (w.reconnect && w.cfg.valid && w.phase == 5 && !w.ctl_pending && !w.ctl_done &&
        (w.flow_state == 0 || w.flow_state == 2) && now >= w.reconnect_at)
        start_join();
}
int bcm_wifi_init(const struct bcm_platform *p, const struct bcm_fw_bundle *b, const u8 mac[6])
{
    if (w.attempted) return -16;
    if (!p || !b || !mac || !p->read32 || !p->write32 || !p->read_ram || !p->write_ram ||
        !p->dma_alloc || !p->dma_sync || !p->time_us || !p->delay_us ||
        !b->firmware.data || b->firmware.size < 4 || b->firmware.size > 0x400000 ||
        (mac[0] & 1) || !(mac[0] | mac[1] | mac[2] | mac[3] | mac[4] | mac[5])) return -22;
    memset(&w, 0, sizeof(w));
    w.p = *p;
    memcpy(w.mac, mac, 6);
    if (b->signature.size) return fail(-95);
    u32 nvlen = 0;
    int err = make_nvram(&b->nvram, &nvlen);
    if (err) return fail(err);
    u8 seed[256];
    if (sep_get_random(seed, sizeof(seed)) != sizeof(seed)) return fail(-5);
    w.attempted = true;
    w.phase = 1;
    err = enumerate();
    if (!err) err = passive_and_ram();
    if (!err) {
        w.phase = 2;
        err = boot(b, seed, nvlen);
    }
    memset(seed, 0, sizeof(seed));
    memset(nvram, 0, sizeof(nvram));
    if (err) return fail(err);
    w.phase = 3;
    w.p.delay_us(200000); /* let firmware finish init before touching its rings */
    if ((err = setup_shared())) return fail(err);
    w.running = true;
    for (u32 i = BCM_NRX; i < BCM_NBUF; i++)
        if (!post(i)) return fail(-5);
    for (u32 i = 0; i < w.rxlimit; i++)
        if (!post(i)) return fail(-5);
    if (w.flags & BCM_SHARED_HOSTREADY) bell(true);
    w.phase = 4;
    if ((err = sync_var("cur_etheraddr", w.mac, 6, false)) ||
        (err = download_blob(&b->clm, "clmload")) ||
        (err = download_blob(&b->txcap, "txcapload")) ||
        (err = download_blob(&b->calibration, "calload"))) return fail(err);
    u8 events[32];
    if ((err = sync_var("event_msgs", NULL, sizeof(events), true))) return fail(err);
    if (w.ctl_len < sizeof(events)) return fail(-71);
    memcpy(events, w.ctl.cpu, sizeof(events));
    static const u8 needed[] = {BCM_E_SSID, BCM_E_DEAUTH, BCM_E_DEAUTH_IND,
                                BCM_E_DISASSOC_IND, BCM_E_LINK, BCM_E_PSK, BCM_E_ESCAN};
    for (u32 i = 0; i < sizeof(needed); i++) events[needed[i] / 8] |= 1u << (needed[i] % 8);
    if ((err = sync_var("event_msgs", events, sizeof(events), false)) ||
        (err = sync_int(BCM_C_INFRA, NULL, 1)) ||
        (err = sync_int(0, "roam_off", 1)) ||
        (err = sync_int(0, "mpc", 0)) ||
        (err = sync_int(BCM_C_UP, NULL, 0)) ||
        (err = sync_int(BCM_C_PM, NULL, 0))) return fail(err);
    /* Windows owns multicast filtering and ARP/ND; optional on older firmware. */
    if (sync_int(0, "allmulti", 1) == -110 || sync_int(0, "arpoe", 0) == -110 ||
        sync_int(0, "ndoe", 0) == -110 || (w.phase & BCM_FATAL)) return fail(w.error ? w.error : -110);
    w.phase = 5;
    return 0;
}
static int hexval(u8 c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}
static void wpa_pmk(const u8 *pass, u32 plen, const u8 *ssid, u32 slen, u8 pmk[32]);
int bcm_wifi_join(const u8 *ssid, u32 ssid_len, const u8 *psk, u32 psk_len)
{
    if (w.phase & BCM_FATAL) return w.error;
    if (!w.running) return -100;
    if (w.phase != 5 || w.ctl_pending || w.ctl_done || (w.flow_state != 0 && w.flow_state != 2))
        return -16;
    if (!ssid || !ssid_len || ssid_len > 32 || !psk || psk_len < 8 || psk_len > 64) return -22;
    u8 pmk[32];
    if (psk_len == 64) {
        for (u32 i = 0; i < 32; i++) {
            int a = hexval(psk[i * 2]), b = hexval(psk[i * 2 + 1]);
            if (a < 0 || b < 0) return -22;
            pmk[i] = (u8)((a << 4) | b);
        }
    } else {
        for (u32 i = 0; i < psk_len; i++)
            if (psk[i] < 32 || psk[i] > 126) return -22;
        wpa_pmk(psk, psk_len, ssid, ssid_len, pmk);
    }
    bool same = w.cfg.valid && ssid_len == w.ssid_len && !memcmp(ssid, w.ssid, ssid_len);
    if (!same) { /* unknown network: WPA2-PSK CCMP, firmware-side scan */
        static const u8 def[] = {48, 20, 1, 0, 0, 15, 172, 4, 1, 0, 0, 15, 172, 4,
                                 1, 0, 0, 15, 172, 2, 0, 0};
        memset(&w.cfg, 0, sizeof(w.cfg));
        memcpy(w.cfg.ie, def, sizeof(def));
        w.cfg.ie_len = sizeof(def);
        w.cfg.wpa_auth = 0x80;
        w.cfg.wsec = 4;
        w.cfg.secure = w.cfg.valid = true;
    }
    memcpy(w.cfg.pmk, pmk, 32);
    memset(pmk, 0, sizeof(pmk));
    memcpy(w.ssid, ssid, ssid_len);
    w.ssid_len = ssid_len;
    w.error = 0;
    w.fails = 0;
    w.reconnect = true;
    start_join();
    return 0;
}
/* ---- WPA PSK -> PMK: PBKDF2-HMAC-SHA1(passphrase, SSID, 4096, 32), IEEE 802.11i H.4 ---- */
struct sha1 { u32 h[5]; u8 b[64]; u32 used; u64 total; };
static u32 rol32(u32 x, u32 n) { return (x << n) | (x >> (32 - n)); }
static void sha1_block(u32 h[5], const u8 *p)
{
    u32 m[80], a = h[0], b = h[1], c = h[2], d = h[3], e = h[4];
    for (u32 i = 0; i < 16; i++)
        m[i] = ((u32)p[4 * i] << 24) | ((u32)p[4 * i + 1] << 16) | ((u32)p[4 * i + 2] << 8) | p[4 * i + 3];
    for (u32 i = 16; i < 80; i++) m[i] = rol32(m[i - 3] ^ m[i - 8] ^ m[i - 14] ^ m[i - 16], 1);
    for (u32 i = 0; i < 80; i++) {
        u32 f, k;
        if (i < 20) { f = (b & c) | (~b & d); k = 0x5a827999; }
        else if (i < 40) { f = b ^ c ^ d; k = 0x6ed9eba1; }
        else if (i < 60) { f = (b & c) | (b & d) | (c & d); k = 0x8f1bbcdc; }
        else { f = b ^ c ^ d; k = 0xca62c1d6; }
        u32 t = rol32(a, 5) + f + e + k + m[i];
        e = d; d = c; c = rol32(b, 30); b = a; a = t;
    }
    h[0] += a; h[1] += b; h[2] += c; h[3] += d; h[4] += e;
}
static void sha1_init(struct sha1 *s)
{
    static const u32 iv[5] = {0x67452301, 0xefcdab89, 0x98badcfe, 0x10325476, 0xc3d2e1f0};
    memcpy(s->h, iv, sizeof(iv));
    s->used = 0;
    s->total = 0;
}
static void sha1_update(struct sha1 *s, const u8 *p, u32 n)
{
    s->total += n;
    while (n) {
        u32 k = 64 - s->used;
        if (k > n) k = n;
        memcpy(s->b + s->used, p, k);
        s->used += k; p += k; n -= k;
        if (s->used == 64) { sha1_block(s->h, s->b); s->used = 0; }
    }
}
static void sha1_final(struct sha1 *s, u8 out[20])
{
    u64 bits = s->total * 8;
    u8 pad = 0x80, zero = 0, len[8];
    sha1_update(s, &pad, 1);
    while (s->used != 56) sha1_update(s, &zero, 1);
    for (u32 i = 0; i < 8; i++) len[i] = (u8)(bits >> (56 - 8 * i));
    sha1_update(s, len, 8);
    for (u32 i = 0; i < 20; i++) out[i] = (u8)(s->h[i / 4] >> (24 - 8 * (i % 4)));
}
/* key <= 64 bytes (a WPA passphrase is at most 63). */
static void hmac_sha1(const u8 *key, u32 klen, const u8 *msg, u32 mlen, u8 out[20])
{
    u8 pad[64], inner[20];
    struct sha1 s;
    memset(pad, 0x36, sizeof(pad));
    for (u32 i = 0; i < klen; i++) pad[i] ^= key[i];
    sha1_init(&s); sha1_update(&s, pad, 64); sha1_update(&s, msg, mlen); sha1_final(&s, inner);
    for (u32 i = 0; i < 64; i++) pad[i] ^= 0x36 ^ 0x5c;
    sha1_init(&s); sha1_update(&s, pad, 64); sha1_update(&s, inner, 20); sha1_final(&s, out);
    memset(pad, 0, sizeof(pad));
}
static void wpa_pmk(const u8 *pass, u32 plen, const u8 *ssid, u32 slen, u8 pmk[32])
{
    u8 salt[36], u[20], t[20];
    memcpy(salt, ssid, slen);
    for (u32 blk = 1; blk <= 2; blk++) {
        salt[slen] = 0; salt[slen + 1] = 0; salt[slen + 2] = 0; salt[slen + 3] = (u8)blk;
        hmac_sha1(pass, plen, salt, slen + 4, u);
        memcpy(t, u, 20);
        for (u32 i = 1; i < 4096; i++) {
            hmac_sha1(pass, plen, u, 20, u);
            for (u32 j = 0; j < 20; j++) t[j] ^= u[j];
        }
        memcpy(pmk + (blk - 1) * 20, t, blk == 1 ? 20 : 12);
    }
    memset(u, 0, sizeof(u));
    memset(t, 0, sizeof(t));
}

/* ---- Boot-time connect, following brcmf_cfg80211_connect() ---- */
static int run_escan(bool v2)
{
    u8 prm[8 + 72 + 36] = {0};
    u8 *q = prm + 8;
    u32 size;
    bcm_put32(prm, v2 ? 2 : 1);   /* BRCMF_ESCAN_REQ_VERSION(_V2) */
    bcm_put16(prm + 4, 1);        /* WL_ESCAN_ACTION_START */
    bcm_put16(prm + 6, 0x1234);
    if (v2) {
        bcm_put16(q, 2);
        bcm_put16(q + 2, 72 + 36);
        memset(q + 40, 0xff, 6);
        q[46] = 2;                    /* DOT11_BSSTYPE_ANY */
        bcm_put32(q + 48, 0);         /* active */
        for (u32 i = 0; i < 4; i++) bcm_put32(q + 52 + 4 * i, 0xffffffffu);
        bcm_put32(q + 68, 1u << 16);  /* one SSID, all channels */
        bcm_put32(q + 72, w.ssid_len);
        memcpy(q + 76, w.ssid, w.ssid_len);
        size = 8 + 72 + 36;
    } else {
        memset(q + 36, 0xff, 6);
        q[42] = 2;
        q[43] = 0;
        for (u32 i = 0; i < 4; i++) bcm_put32(q + 44 + 4 * i, 0xffffffffu);
        bcm_put32(q + 60, 1u << 16);
        bcm_put32(q + 64, w.ssid_len);
        memcpy(q + 68, w.ssid, w.ssid_len);
        size = 8 + 64 + 36;
    }
    w.scanning = true;
    w.scan_status = 0xffffffffu;
    int err = sync_var("escan", prm, size, false);
    if (err) { w.scanning = false; return err; }
    u64 deadline = w.p.time_us() + 12000000;
    while (w.scanning && w.p.time_us() < deadline && !(w.phase & BCM_FATAL)) {
        bcm_wifi_poll(64);
        w.p.delay_us(1000);
    }
    if (w.phase & BCM_FATAL) return w.error;
    if (w.scanning) { w.scanning = false; return -110; }
    return 0;
}
static int sync_cmd(u32 cmd, const void *data, u32 n)
{
    int err = ioctl_start(cmd, data, n);
    return err ? err : wait_ioctl();
}
static const char *cipher_name(u32 t)
{
    switch (t) { case 1: return "WEP40"; case 2: return "TKIP"; case 4: return "CCMP";
    case 5: return "WEP104"; case 8: return "GCMP"; case 9: return "GCMP256";
    case 10: return "CCMP256"; default: return "?"; }
}
static int connect_once(const u8 pmk[32], int (*out)(const char *, ...))
{
    const struct bcm_ap *a = &w.ap;
    u8 ie[24];
    u32 ie_len = 0, wpa_auth = 0, wsec = 0, mfp = 0;
    bool secure = true;
    if (a->found && a->rsn.present && (a->rsn.akm & ((1u << 2) | (1u << 6)))) {
        u32 pair = (a->rsn.pair & (1u << 4)) ? 4 : (a->rsn.pair & (1u << 2)) ? 2 : 0;
        u32 group = a->rsn.group, akm = (a->rsn.akm & (1u << 2)) ? 2 : 6;
        if (!pair || (group != 2 && group != 4)) return -95;
        u16 caps = 0;
        if (a->rsn.caps & 0x40) { caps = 0xc0; mfp = 2; }      /* AP requires PMF */
        else if (akm == 6 && (a->rsn.caps & 0x80)) { caps = 0x80; mfp = 1; }
        static const u8 hdr[] = {48, 20, 1, 0};
        memcpy(ie, hdr, 4);
        ie[4] = 0x00; ie[5] = 0x0f; ie[6] = 0xac; ie[7] = (u8)group;
        ie[8] = 1; ie[9] = 0;
        ie[10] = 0x00; ie[11] = 0x0f; ie[12] = 0xac; ie[13] = (u8)pair;
        ie[14] = 1; ie[15] = 0;
        ie[16] = 0x00; ie[17] = 0x0f; ie[18] = 0xac; ie[19] = (u8)akm;
        bcm_put16(ie + 20, caps);
        ie_len = 22;
        wpa_auth = akm == 2 ? 0x80 : 0x8000;
        wsec = (pair == 4 ? 4 : 2) | (group == 4 ? 4 : 2);
    } else if (a->found && a->wpa.present && (a->wpa.akm & (1u << 2))) {
        u32 pair = (a->wpa.pair & (1u << 4)) ? 4 : (a->wpa.pair & (1u << 2)) ? 2 : 0;
        u32 group = a->wpa.group;
        if (!pair || (group != 2 && group != 4)) return -95;
        static const u8 hdr[] = {221, 22, 0x00, 0x50, 0xf2, 1, 1, 0};
        memcpy(ie, hdr, 8);
        ie[8] = 0x00; ie[9] = 0x50; ie[10] = 0xf2; ie[11] = (u8)group;
        ie[12] = 1; ie[13] = 0;
        ie[14] = 0x00; ie[15] = 0x50; ie[16] = 0xf2; ie[17] = (u8)pair;
        ie[18] = 1; ie[19] = 0;
        ie[20] = 0x00; ie[21] = 0x50; ie[22] = 0xf2; ie[23] = 2;
        ie_len = 24;
        wpa_auth = 0x4;
        wsec = (pair == 4 ? 4 : 2) | (group == 4 ? 4 : 2);
    } else if (a->found && !a->privacy && !a->rsn.present && !a->wpa.present) {
        secure = false;
    } else if (a->found && (a->rsn.present || a->wpa.present)) {
        return -95; /* WPA3-SAE-only or enterprise: not supported here */
    } else {
        /* SSID not seen: WPA2-PSK CCMP default, let the firmware scan. */
        static const u8 def[] = {48, 20, 1, 0, 0, 15, 172, 4, 1, 0, 0, 15, 172, 4,
                                 1, 0, 0, 15, 172, 2, 0, 0};
        memcpy(ie, def, sizeof(def));
        ie_len = sizeof(def);
        wpa_auth = 0x80;
        wsec = 4;
    }
    out("J316 Wi-Fi: config wpa_auth=%x wsec=%x mfp=%u ie=%u\n", wpa_auth, wsec, mfp, ie_len);
    memset(&w.cfg, 0, sizeof(w.cfg));
    w.cfg.valid = true;
    w.cfg.secure = secure;
    memcpy(w.cfg.ie, ie, ie_len);
    w.cfg.ie_len = ie_len;
    w.cfg.wpa_auth = wpa_auth;
    w.cfg.wsec = wsec;
    w.cfg.mfp = mfp;
    memcpy(w.cfg.pmk, pmk, 32);
    w.cfg.pinned = a->found && a->chanspec;
    if (w.cfg.pinned) { memcpy(w.cfg.bssid, a->bssid, 6); w.cfg.chanspec = a->chanspec; }

    int err;
    (void)sync_cmd(BCM_C_DISASSOC, NULL, 0);
    if (w.phase & BCM_FATAL) return w.error;
    (void)sync_var("wpaie", ie_len ? ie : NULL, ie_len, false);
    if ((err = sync_int(0, "wpa_auth", wpa_auth))) { out("J316 Wi-Fi: wpa_auth err %d\n", err); return err; }
    if ((err = sync_int(BCM_C_AUTH, NULL, 0))) { out("J316 Wi-Fi: auth err %d\n", err); return err; }
    if ((err = sync_int(0, "wsec", wsec))) { out("J316 Wi-Fi: wsec err %d\n", err); return err; }
    err = sync_int(0, "mfp", mfp);
    if (err && mfp) { out("J316 Wi-Fi: mfp err %d\n", err); return err; }
    if (w.phase & BCM_FATAL) return w.error;
    if ((err = sync_int(0, "sup_wpa", secure ? 1 : 0))) { out("J316 Wi-Fi: sup_wpa err %d\n", err); return err; }
    if (secure) {
        u8 pk[132] = {0}; /* brcmf_wsec_pmk_le: raw PMK, flags 0 (brcmf_set_pmk) */
        bcm_put16(pk, 32);
        memcpy(pk + 4, pmk, 32);
        err = sync_cmd(BCM_C_PMK, pk, sizeof(pk));
        memset(pk, 0, sizeof(pk));
        if (err) { out("J316 Wi-Fi: pmk err %d\n", err); return err; }
    }

    w.associated = false;
    w.keyed = !secure;
    w.ev_type = w.ev_status = w.ev_reason = 0xffffffffu;
    w.ev_seen = 0;
    w.joining = true;
    w.join_deadline = w.p.time_us() + 20000000;
    w.phase = 7;
    u8 j[72] = {0}; /* brcmf_ext_join_params_le */
    bool chan = a->found && a->chanspec;
    bcm_put32(j, w.ssid_len);
    memcpy(j + 4, w.ssid, w.ssid_len);
    j[36] = 0xff;
    bcm_put32(j + 40, chan ? 320 / 20 : 0xffffffffu);
    bcm_put32(j + 44, chan ? 320 : 0xffffffffu);
    bcm_put32(j + 48, chan ? 400 : 0xffffffffu);
    bcm_put32(j + 52, 0xffffffffu);
    if (a->found) memcpy(j + 56, a->bssid, 6); else memset(j + 56, 0xff, 6);
    if (chan) { bcm_put32(j + 64, 1); bcm_put16(j + 68, a->chanspec); }
    err = sync_var("join", j, chan ? 70 : 68, false);
    if (err && !(w.phase & BCM_FATAL) && w.joining) {
        out("J316 Wi-Fi: join iovar err %d, using SET_SSID\n", err);
        u8 s[52] = {0}; /* brcmf_join_params */
        bcm_put32(s, w.ssid_len);
        memcpy(s + 4, w.ssid, w.ssid_len);
        if (a->found) memcpy(s + 36, a->bssid, 6); else memset(s + 36, 0xff, 6);
        if (chan) { bcm_put32(s + 44, 1); bcm_put16(s + 48, a->chanspec); }
        err = sync_cmd(BCM_C_SSID, s, chan ? 52 : 36);
    }
    if (w.phase & BCM_FATAL) return w.error;
    if (err) { w.joining = false; w.phase = 5; out("J316 Wi-Fi: join command err %d\n", err); return err; }
    while (w.phase == 7 && !(w.phase & BCM_FATAL)) {
        bcm_wifi_poll(64);
        w.p.delay_us(1000);
    }
    if (bcm_wifi_link()) return 0;
    if (w.phase & BCM_FATAL) return w.error;
    return w.error ? w.error : -100;
}
int bcm_wifi_connect(const u8 *ssid, u32 ssid_len, const u8 *psk, u32 psk_len,
                     int (*out)(const char *, ...))
{
    if (w.phase & BCM_FATAL) return w.error;
    if (!w.running) return -100;
    if (w.phase != 5 || w.ctl_pending || w.ctl_done || w.flow_state) return -16;
    if (!ssid || !ssid_len || ssid_len > 32 || !psk || psk_len < 8 || psk_len > 64) return -22;
    u8 pmk[32];
    if (psk_len == 64) {
        for (u32 i = 0; i < 32; i++) {
            int hi = hexval(psk[i * 2]), lo = hexval(psk[i * 2 + 1]);
            if (hi < 0 || lo < 0) return -22;
            pmk[i] = (u8)((hi << 4) | lo);
        }
    } else {
        for (u32 i = 0; i < psk_len; i++)
            if (psk[i] < 32 || psk[i] > 126) return -22;
        wpa_pmk(psk, psk_len, ssid, ssid_len, pmk);
    }
    memcpy(w.ssid, ssid, ssid_len);
    w.ssid_len = ssid_len;
    w.error = 0;
    /* Phase 7 keeps join_progress() from consuming our synchronous ioctl results. */
    w.phase = 7;
    w.joining = false;
    int err = sync_var("scan_ver", NULL, 4, true);
    if (w.phase & BCM_FATAL) goto out;
    bool v2 = err == 0;
    int result = -100;
    for (u32 attempt = 1; attempt <= 3; attempt++) {
        w.phase = 7;
        w.joining = false;
        if (!w.ap.found || attempt == 3) {
            memset(&w.ap, 0, sizeof(w.ap));
            for (u32 s = 0; s < 2 && !w.ap.found; s++) {
                err = run_escan(v2);
                if (w.phase & BCM_FATAL) goto out;
                if (err) out("J316 Wi-Fi: escan v%d err %d\n", v2 ? 2 : 1, err);
            }
            if (w.ap.found)
                out("J316 Wi-Fi: found %02x:%02x:%02x:%02x:%02x:%02x ch=%u chanspec=%x rssi=%d "
                    "rsn=%d g=%s p=%x akm=%x caps=%x wpa=%d priv=%d\n",
                    w.ap.bssid[0], w.ap.bssid[1], w.ap.bssid[2], w.ap.bssid[3], w.ap.bssid[4],
                    w.ap.bssid[5], w.ap.ctl_ch, w.ap.chanspec, w.ap.rssi, w.ap.rsn.present,
                    cipher_name(w.ap.rsn.group), w.ap.rsn.pair, w.ap.rsn.akm, w.ap.rsn.caps,
                    w.ap.wpa.present, w.ap.privacy);
            else
                out("J316 Wi-Fi: SSID not seen in scan (status %d); trying blind join\n",
                    (int)w.scan_status);
        }
        result = connect_once(pmk, out);
        if (!result) {
            out("J316 Wi-Fi: CONNECTED to %02x:%02x:%02x:%02x:%02x:%02x\n", w.bssid[0], w.bssid[1],
                w.bssid[2], w.bssid[3], w.bssid[4], w.bssid[5]);
            break;
        }
        if (w.phase & BCM_FATAL) break;
        out("J316 Wi-Fi: attempt %u failed err=%d event=%d status=%d reason=%d seen=%x\n", attempt,
            result, (int)w.ev_type, (int)w.ev_status, (int)w.ev_reason, w.ev_seen);
        if (result == -95) {
            out("J316 Wi-Fi: network security not supported (WPA3-only/enterprise?)\n");
            break;
        }
        if (w.ev_type == BCM_E_PSK || ((w.ev_type == BCM_E_DEAUTH_IND || w.ev_type == BCM_E_DISASSOC_IND) &&
                                       (w.ev_seen & 0x40000000u)))
            out("J316 Wi-Fi: router rejected the key exchange - PASSWORD LIKELY WRONG\n");
        w.p.delay_us(500000);
    }
    memset(pmk, 0, sizeof(pmk));
    if (!(w.phase & BCM_FATAL) && !bcm_wifi_link()) { w.phase = 5; w.joining = false; }
    w.fails = bcm_wifi_link() ? 0 : 3;
    w.reconnect_at = w.p.time_us() + 2000000;
    w.reconnect = w.cfg.valid; /* keep retrying in the background after guest entry */
    return result;
out:
    memset(pmk, 0, sizeof(pmk));
    return w.error;
}
static int create_flow(const u8 *frame)
{
    u8 *p = reserve(&w.rings[0]);
    if (!p) return -11;
    p[0] = BCM_FLOW_CREATE;
    memcpy(p + 8, frame, 6);
    memcpy(p + 14, w.mac, 6);
    /* ADDR_INDIRECT uses one infrastructure FIFO for all destinations. */
    bcm_put16(p + 22, 2);
    bcm_put16(p + 28, BCM_FLOW_DEPTH);
    bcm_put16(p + 30, 48);
    bcm_put64(p + 32, w.flow.dma.iova);
    w.flow_state = 1;
    w.flow_deadline = w.p.time_us() + BCM_TIMEOUT;
    commit(&w.rings[0]);
    return 0;
}
int bcm_wifi_send(const void *frame, u32 size)
{
    if (w.phase & BCM_FATAL) return w.error;
    if (!bcm_wifi_link()) return -100;
    if (!frame || size < 14 || size > 1518) return -22;
    if (!w.flow_state) {
        int err = create_flow(frame);
        return err ? err : -11;
    }
    if (w.flow_state != 2) return -11;
    u32 id;
    for (id = 0; id < BCM_NTX; id++) if (!w.tx[id].state) break;
    if (id == BCM_NTX) return -11;
    u8 *p = reserve(&w.flow);
    if (!p) return -11;
    struct bcm_buffer *b = &w.tx[id];
    memcpy(b->dma.cpu, frame, size);
    w.p.dma_sync(&b->dma);
    p[0] = BCM_TX;
    bcm_put32(p + 4, id + 1);
    memcpy(p + 8, frame, 14);
    p[22] = 1; /* FRAME_802_3, best effort priority 0 */
    p[23] = 1;
    bcm_put64(p + 32, b->dma.iova + 14);
    bcm_put16(p + 42, size - 14);
    b->state = 1;
    commit(&w.flow);
    return 0;
}
int bcm_wifi_receive(void *frame, u32 capacity)
{
    if (w.phase & BCM_FATAL) return w.error;
    if (!w.running) return -100;
    if (!w.qcount) return 0;
    if (!bcm_wifi_link()) return -100;
    struct bcm_buffer *b = &w.buf[w.rxq[w.qread]];
    if (!frame || capacity < b->len) return -90;
    memcpy(frame, (u8 *)b->dma.cpu + b->offset, b->len);
    int n = b->len;
    b->state = 0;
    w.qread = (w.qread + 1) % BCM_NRX;
    w.qcount--;
    return n;
}
bool bcm_wifi_link(void)
{
    return w.phase == 8 && w.associated && w.keyed;
}
u32 bcm_wifi_phase(void) { return w.phase; }
int bcm_wifi_error(void) { return w.error; }

void bcm_wifi_dump(int (*out)(const char *, ...))
{
    /* TCM reads only: PCIe core registers can raise external aborts on this chip. */
    if (!w.shared) { out("bcm: no shared area (phase %x)\n", w.phase); return; }
    out("bcm: flags=%x shared=%x ctl_pending=%d trans=%u error=%d\n", w.flags, w.shared,
        w.ctl_pending, w.trans, w.error);
    for (u32 i = 0; i < 5; i++)
        out("bcm: ring%u host_w=%u dev_r=%u dev_w=%u host_r=%u depth=%u item=%u\n", i,
            w.rings[i].w, ring_ri(&w.rings[i]), ring_wi(&w.rings[i]), w.rings[i].r,
            w.rings[i].depth, w.rings[i].item);
    /* Firmware console (brcmf_pcie_bus_console_read layout). */
    u32 con = ram32(w.shared + 20);
    if (!con || !ram_valid(con, 20)) { out("bcm: no console (%x)\n", con); return; }
    u32 buf = ram32(con + 8), size = ram32(con + 12), widx = ram32(con + 16);
    out("bcm: console buf=%x size=%u widx=%u\n", buf, size, widx);
    if (!size || size > 0x10000 || widx >= size || !ram_valid(buf, size)) return;
    u32 n = size < 640 ? size : 640, pos = (widx + size - n) % size;
    char line[100];
    u32 len = 0;
    for (u32 i = 0; i < n; i++) {
        u8 c;
        w.p.read_ram(buf + pos, &c, 1);
        pos = (pos + 1) % size;
        bool nl = c == '\n' || c == '\r';
        if (!nl && c >= 32 && c < 127) line[len++] = (char)c;
        if ((nl && len) || len == sizeof(line) - 1) { line[len] = 0; out("fw> %s\n", line); len = 0; }
    }
    if (len) { line[len] = 0; out("fw> %s\n", line); }
}

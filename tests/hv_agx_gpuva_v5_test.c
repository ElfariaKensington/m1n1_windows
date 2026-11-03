#include <assert.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "../src/hv_agx_gpuva_v5.h"
#include "apple_agx_gpuva_broker_v5.h"

#define P_ROOT UINT64_C(0x10000000)
#define P_L1   UINT64_C(0x10004000)
#define P_L2   UINT64_C(0x10008000)
#define P_DATA UINT64_C(0x20000000)
#define P_ROOT2 UINT64_C(0x1000c000)
#define Q_ROOT UINT64_C(0x11000000)
#define Q_L1   UINT64_C(0x11004000)
#define Q_L2   UINT64_C(0x11008000)
#define Q_DATA UINT64_C(0x30000000)

struct fixture {
    struct hv_agx_gpuva_v5 broker;
    uint64_t pages[9][2048];
    uint64_t ipa[9];
    uint64_t slots[64][2];
    unsigned invalidations[64], fail_once;
    uint64_t blocked_ipa, remapped_ipa, remapped_pa;
    bool prefix;
    bool legacy_slot63;
    bool range_backing;
};
static uint64_t translate(void *opaque, uint64_t ipa)
{
    struct fixture *f = opaque;
    unsigned i;
    if (ipa == f->blocked_ipa) return 0;
    if (ipa == f->remapped_ipa) return f->remapped_pa;
    if (f->range_backing && ipa >= P_DATA &&
        ipa < P_DATA + UINT64_C(4096) * 1024 * 1024 &&
        !(ipa & (HV_AGX_GPUVA_V5_PAGE - 1))) return ipa;
    for (i = 0; i < 9; ++i) if (f->ipa[i] == ipa) return ipa;
    return 0;
}
static uint64_t *map_page(void *opaque, uint64_t ipa, uint64_t pa)
{
    struct fixture *f = opaque;
    unsigned i;
    if (ipa != pa) return NULL;
    for (i = 0; i < 9; ++i) if (f->ipa[i] == ipa) return f->pages[i];
    return NULL;
}
static bool read_slot(void *opaque, unsigned slot, uint64_t *low, uint64_t *high)
{
    struct fixture *f = opaque;
    if (slot >= 64) return false;
    *low = f->slots[slot][0]; *high = f->slots[slot][1]; return true;
}
static bool write_slot(void *opaque, unsigned slot, uint64_t low, uint64_t high)
{
    struct fixture *f = opaque;
    if (!slot || slot >= 64 || high) return false;
    f->slots[slot][0] = low; f->slots[slot][1] = high; return true;
}
static bool sync_tables(void *opaque) { (void)opaque; return true; }
static bool invalidate(void *opaque, unsigned slot)
{
    struct fixture *f = opaque;
    if (!slot || slot >= 64) return false;
    if (f->fail_once) { --f->fail_once; return false; }
    ++f->invalidations[slot]; return true;
}
static bool prefix(void *opaque) { return ((struct fixture *)opaque)->prefix; }
static bool legacy_slot63(void *opaque)
{ return ((struct fixture *)opaque)->legacy_slot63; }
static struct fixture *new_fixture_with(void *storage, unsigned system_backings);
static struct fixture *new_fixture(void) { return new_fixture_with(NULL, 0); }
static struct fixture *new_fixture_with(void *storage, unsigned system_backings)
{
    struct fixture *f = calloc(1, sizeof(*f));
    struct hv_agx_gpuva_v5_ops ops;
    assert(f);
    f->ipa[0]=P_ROOT; f->ipa[1]=P_L1; f->ipa[2]=P_L2;
    f->ipa[3]=P_DATA; f->ipa[4]=Q_ROOT; f->ipa[5]=Q_L1;
    f->ipa[6]=Q_L2; f->ipa[7]=Q_DATA; f->ipa[8]=P_ROOT2;
    f->slots[0][0] = UINT64_C(0x91000001);
    f->slots[0][1] = UINT64_C(0x90000001);
    f->prefix = true;
    ops = (struct hv_agx_gpuva_v5_ops){f,translate,map_page,read_slot,
                                      write_slot,sync_tables,invalidate,prefix,
                                      legacy_slot63,NULL,0};
    ops.storage = storage;
    ops.storage_system_backings = system_backings;
    assert(hv_agx_gpuva_v5_init(&f->broker, 7, &ops) == HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_validate_envelope(&f->broker, 0, 1, 0) ==
           HV_AGX_GPUVA_V5_STALE);
    assert(hv_agx_gpuva_v5_validate_envelope(&f->broker, 7, 1, 2) ==
           HV_AGX_GPUVA_V5_INVALID);
    assert(hv_agx_gpuva_v5_validate_envelope(&f->broker, 7, 1, 0) ==
           HV_AGX_GPUVA_V5_OK);
    return f;
}
static void prepare(struct fixture *f, uint64_t id, uint64_t root,
                    uint64_t l1, uint64_t l2, uint64_t data, bool paging)
{
    uint64_t logical[4] = {data,data+0x1000,data+0x2000,data+0x3000};
    assert(hv_agx_gpuva_v5_create(&f->broker,id,1,root,paging)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_register_table(&f->broker,id,1,l1,1)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_register_table(&f->broker,id,1,l2,2)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_register_backing(&f->broker,id,1,17,data)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_update_parent(&f->broker,id,1,root,0,l1)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_update_parent(&f->broker,id,1,l1,0,l2)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_update_leaf(&f->broker,id,1,l2,8,logical,17,15,15,15)==
           HV_AGX_GPUVA_V5_OK);
}
static void independent_flush(void)
{
    struct fixture *f = new_fixture();
    uint64_t token1 = 0, token2 = 0;
    uint64_t frozen0 = f->slots[0][0], frozen1 = f->slots[0][1];
    uint64_t generation;
    prepare(f, 1, P_ROOT, P_L1, P_L2, P_DATA, false);
    generation = f->broker.processes[0].map_generation;
    assert(hv_agx_gpuva_v5_lease(&f->broker,1,1,1,&token1)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_lease(&f->broker,1,1,2,&token2)==HV_AGX_GPUVA_V5_OK);
    unsigned before1=f->invalidations[1], before2=f->invalidations[2];
    assert(hv_agx_gpuva_v5_flush_tlb(&f->broker,1,1,P_ROOT,0,0)==HV_AGX_GPUVA_V5_OK);
    assert(f->invalidations[1]==before1+1 && f->invalidations[2]==before2+1);
    assert(f->broker.processes[0].map_generation==generation);
    assert(f->slots[0][0]==frozen0 && f->slots[0][1]==frozen1);
    assert(hv_agx_gpuva_v5_flush_tlb(&f->broker,1,1,Q_ROOT,0,0)==HV_AGX_GPUVA_V5_OWNERSHIP);
    assert(hv_agx_gpuva_v5_flush_tlb(&f->broker,1,1,P_ROOT,0x1234,0x4000)==HV_AGX_GPUVA_V5_INVALID);
    assert(hv_agx_gpuva_v5_flush_tlb(&f->broker,1,1,P_ROOT,0x4000,0x4000)==HV_AGX_GPUVA_V5_INVALID);
    assert(hv_agx_gpuva_v5_job_begin(&f->broker,1,token1)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_flush_tlb(&f->broker,1,1,P_ROOT,0,0)==HV_AGX_GPUVA_V5_BUSY);
    assert(hv_agx_gpuva_v5_job_end(&f->broker,1,token1)==HV_AGX_GPUVA_V5_OK);
    f->fail_once=1;
    assert(hv_agx_gpuva_v5_flush_tlb(&f->broker,1,1,P_ROOT,0,0)==HV_AGX_GPUVA_V5_TLB);
    assert(f->broker.tainted);
    free(f);
}
static void guest_backing_validation(void)
{
    struct fixture *f = new_fixture();
    uint64_t contiguous[4] = {P_DATA,P_DATA+0x1000,P_DATA+0x2000,P_DATA+0x3000};
    uint64_t scattered[4] = {P_DATA,P_DATA+0x1000,P_DATA+0x3000,P_DATA+0x4000};
    uint64_t unaligned[4] = {P_DATA+0x1000,P_DATA+0x2000,P_DATA+0x3000,P_DATA+0x4000};
    assert(hv_agx_gpuva_v5_create(&f->broker,1,1,P_ROOT,false)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_register_table(&f->broker,1,1,P_L2,2)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_update_leaf(&f->broker,1,1,P_L2,8,contiguous,17,15,15,15)==HV_AGX_GPUVA_V5_OWNERSHIP);
    assert(hv_agx_gpuva_v5_register_backing(&f->broker,1,1,17,P_DATA+0x4000)==HV_AGX_GPUVA_V5_OWNERSHIP);
    assert(hv_agx_gpuva_v5_register_backing(&f->broker,1,1,17,P_DATA)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_update_leaf(&f->broker,1,1,P_L2,8,unaligned,17,15,15,15)==HV_AGX_GPUVA_V5_INVALID);
    assert(hv_agx_gpuva_v5_update_leaf(&f->broker,1,1,P_L2,8,scattered,17,15,15,15)==HV_AGX_GPUVA_V5_OWNERSHIP);
    f->blocked_ipa=P_DATA;
    assert(hv_agx_gpuva_v5_update_leaf(&f->broker,1,1,P_L2,8,contiguous,17,15,15,15)==HV_AGX_GPUVA_V5_OWNERSHIP);
    f->blocked_ipa=0;
    f->remapped_ipa=P_DATA; f->remapped_pa=Q_DATA;
    assert(hv_agx_gpuva_v5_update_leaf(&f->broker,1,1,P_L2,8,contiguous,17,15,15,15)==HV_AGX_GPUVA_V5_OWNERSHIP);
    assert(f->broker.tables[1].entries[8]==0);
    free(f);
}
static void destroy_order_with_live_tables(void)
{
    struct fixture *f = new_fixture();
    uint64_t contiguous[4] = {P_DATA,P_DATA+0x1000,P_DATA+0x2000,P_DATA+0x3000};
    assert(hv_agx_gpuva_v5_create(&f->broker,1,1,P_ROOT,false)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_register_table(&f->broker,1,1,P_L1,1)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_register_table(&f->broker,1,1,P_L2,2)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_register_table(&f->broker,1,1,P_ROOT2,0)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_register_backing(&f->broker,1,1,17,P_DATA)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_update_parent(&f->broker,1,1,P_ROOT,0,P_L1)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_update_parent(&f->broker,1,1,P_L1,0,P_L2)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_update_leaf(&f->broker,1,1,P_L2,8,contiguous,17,15,15,15)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_destroy(&f->broker,1,1)==HV_AGX_GPUVA_V5_BUSY);
    assert(hv_agx_gpuva_v5_update_leaf(&f->broker,1,1,P_L2,8,0,0,15,0,0)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_revoke_backing(&f->broker,1,1,17,P_DATA)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_update_parent(&f->broker,1,1,P_L1,0,0)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_update_parent(&f->broker,1,1,P_ROOT,0,0)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_revoke_table(&f->broker,1,1,P_L2,2)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_revoke_table(&f->broker,1,1,P_L1,1)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_revoke_table(&f->broker,1,1,P_ROOT2,0)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_destroy(&f->broker,1,1)==HV_AGX_GPUVA_V5_OK);
    free(f);
}
static void shared_graph_grants(void)
{
    struct fixture *f = new_fixture();
    uint64_t logical[4] = {P_DATA,P_DATA+0x1000,P_DATA+0x2000,P_DATA+0x3000};
    assert(hv_agx_gpuva_v5_create(&f->broker, 1, 1, P_ROOT, false) == HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_create(&f->broker, 2, 1, Q_ROOT, true) == HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_register_shared_backing(&f->broker,1,1,17,P_DATA)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_register_backing(&f->broker,2,1,17,P_DATA)==HV_AGX_GPUVA_V5_OWNERSHIP);
    assert(hv_agx_gpuva_v5_register_shared_backing(&f->broker,2,1,18,P_DATA)==HV_AGX_GPUVA_V5_OWNERSHIP);
    assert(hv_agx_gpuva_v5_register_shared_backing(&f->broker,2,1,17,P_DATA)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_register_table(&f->broker,1,1,P_L1,1)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_register_table(&f->broker,1,1,P_L2,2)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_register_table(&f->broker,2,1,Q_L1,1)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_register_table(&f->broker,2,1,Q_L2,2)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_update_parent(&f->broker,1,1,P_ROOT,0,P_L1)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_update_parent(&f->broker,1,1,P_L1,0,P_L2)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_update_parent(&f->broker,2,1,Q_ROOT,0,Q_L1)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_update_parent(&f->broker,2,1,Q_L1,0,Q_L2)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_update_leaf(&f->broker,1,1,P_L2,8,logical,17,15,15,15)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_update_leaf(&f->broker,2,1,Q_L2,8,logical,17,15,15,15)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_revoke_backing(&f->broker,1,1,17,P_DATA)==HV_AGX_GPUVA_V5_BUSY);
    assert(hv_agx_gpuva_v5_update_leaf(&f->broker,1,1,P_L2,8,NULL,0,15,0,0)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_revoke_backing(&f->broker,1,1,17,P_DATA)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_revoke_backing(&f->broker,2,1,17,P_DATA)==HV_AGX_GPUVA_V5_BUSY);
    assert(hv_agx_gpuva_v5_update_leaf(&f->broker,2,1,Q_L2,8,NULL,0,15,0,0)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_revoke_backing(&f->broker,2,1,17,P_DATA)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_update_parent(&f->broker,1,1,P_L1,0,0)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_update_parent(&f->broker,1,1,P_ROOT,0,0)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_update_parent(&f->broker,2,1,Q_L1,0,0)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_update_parent(&f->broker,2,1,Q_ROOT,0,0)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_destroy(&f->broker,1,1)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_destroy(&f->broker,2,1)==HV_AGX_GPUVA_V5_OK);
    free(f);
}
static void shared_reserve_capacity(void)
{
    struct fixture *f = new_fixture();
    unsigned owner, page;
    f->range_backing = true;
    assert(hv_agx_gpuva_v5_create(&f->broker, 1, 1, P_ROOT, false) == HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_create(&f->broker, 2, 1, Q_ROOT, false) == HV_AGX_GPUVA_V5_OK);
    /* One 56-MiB local reserve has 3584 native 16-KiB pages.  The broker
     * must admit simultaneous grants to two processes without weakening
     * per-process ownership or generation checks. */
    for (owner = 1; owner <= 2; ++owner)
        for (page = 0; page < 3584; ++page)
            assert(hv_agx_gpuva_v5_register_shared_backing(
                &f->broker, owner, 1, 17,
                P_DATA + (uint64_t)page * HV_AGX_GPUVA_V5_PAGE) == HV_AGX_GPUVA_V5_OK);
    free(f);
}
static void system_backings_hold_uma_capacity(void)
{
    /* M2-d: 2 GiB of distinct 16-KiB system pages, each found in O(1). The
     * page past the limit is refused without mutation; a sharer joining an
     * existing record takes no capacity; a revoke frees a record. */
    struct fixture *f = new_fixture();
    struct hv_agx_gpuva_v5 *before = malloc(sizeof(*before));
    const uint64_t last = P_DATA +
        (uint64_t)HV_AGX_GPUVA_V5_BACKINGS * HV_AGX_GPUVA_V5_PAGE;
    const struct hv_agx_gpuva_v5_backing *record;
    assert(before && HV_AGX_GPUVA_V5_BACKINGS == 131072);
    f->range_backing = true;
    assert(hv_agx_gpuva_v5_create(&f->broker, 1, 1, P_ROOT, false) == HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_create(&f->broker, 2, 1, Q_ROOT, false) == HV_AGX_GPUVA_V5_OK);
    for (unsigned page = 0; page < HV_AGX_GPUVA_V5_BACKINGS; ++page)
        assert(hv_agx_gpuva_v5_register_shared_backing(&f->broker, 1, 1, 17,
            P_DATA + (uint64_t)page * HV_AGX_GPUVA_V5_PAGE) == HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_live_backings(&f->broker) == HV_AGX_GPUVA_V5_BACKINGS);
    memcpy(before, &f->broker, sizeof(*before));
    assert(hv_agx_gpuva_v5_register_shared_backing(&f->broker, 1, 1, 17, last) ==
           HV_AGX_GPUVA_V5_CAPACITY);
    assert(!memcmp(before, &f->broker, sizeof(*before)));
    assert(hv_agx_gpuva_v5_register_shared_backing(&f->broker, 2, 1, 17, P_DATA) ==
           HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_live_backings(&f->broker) == HV_AGX_GPUVA_V5_BACKINGS + 1u);
    /* Exclusive identity and a second shared generation stay refused. */
    assert(hv_agx_gpuva_v5_register_backing(&f->broker, 2, 1, 17, P_DATA + 0x4000) ==
           HV_AGX_GPUVA_V5_OWNERSHIP);
    assert(hv_agx_gpuva_v5_register_shared_backing(&f->broker, 2, 1, 18, P_DATA + 0x4000) ==
           HV_AGX_GPUVA_V5_OWNERSHIP);
    assert(hv_agx_gpuva_v5_register_shared_backing(&f->broker, 2, 1, 17, last) ==
           HV_AGX_GPUVA_V5_CAPACITY);
    assert(hv_agx_gpuva_v5_revoke_backing(&f->broker, 1, 1, 17, P_DATA + 0x14000) ==
           HV_AGX_GPUVA_V5_OK);
    assert(!hv_agx_gpuva_v5_backing_by_ipa(&f->broker, P_DATA + 0x14000));
    assert(hv_agx_gpuva_v5_register_backing(&f->broker, 2, 1, 19, last) == HV_AGX_GPUVA_V5_OK);
    record = hv_agx_gpuva_v5_backing_by_pa(&f->broker, last);
    assert(record && record->ipa == last && record->generation == 19 && !record->shared &&
           hv_agx_gpuva_v5_backing_owned(record, 1) && !hv_agx_gpuva_v5_backing_owned(record, 0));
    /* Owner 1 (index 0) still holds the shared first page with owner 2. */
    record = hv_agx_gpuva_v5_backing_by_ipa(&f->broker, P_DATA);
    assert(record && hv_agx_gpuva_v5_backing_owned(record, 0) &&
           hv_agx_gpuva_v5_backing_owned(record, 1));
    assert(hv_agx_gpuva_v5_destroy(&f->broker, 2, 1) == HV_AGX_GPUVA_V5_BUSY);
    free(before);
    free(f);
}
static void backing_index_survives_deletion(void)
{
    /* Backward-shift deletion: after revoking every third page and
     * re-registering some, every live page is found and no revoked one. */
    struct fixture *f = new_fixture();
    const unsigned pages = 50000;
    f->range_backing = true;
    assert(hv_agx_gpuva_v5_create(&f->broker, 1, 1, P_ROOT, false) == HV_AGX_GPUVA_V5_OK);
    for (unsigned page = 0; page < pages; ++page)
        assert(hv_agx_gpuva_v5_register_backing(&f->broker, 1, 1, 17,
            P_DATA + (uint64_t)page * HV_AGX_GPUVA_V5_PAGE) == HV_AGX_GPUVA_V5_OK);
    for (unsigned page = 0; page < pages; page += 3)
        assert(hv_agx_gpuva_v5_revoke_backing(&f->broker, 1, 1, 17,
            P_DATA + (uint64_t)page * HV_AGX_GPUVA_V5_PAGE) == HV_AGX_GPUVA_V5_OK);
    for (unsigned page = 0; page < pages; page += 9)
        assert(hv_agx_gpuva_v5_register_backing(&f->broker, 1, 1, 21,
            P_DATA + (uint64_t)page * HV_AGX_GPUVA_V5_PAGE) == HV_AGX_GPUVA_V5_OK);
    for (unsigned page = 0; page < pages; ++page) {
        uint64_t ipa = P_DATA + (uint64_t)page * HV_AGX_GPUVA_V5_PAGE;
        const struct hv_agx_gpuva_v5_backing *by_ipa =
            hv_agx_gpuva_v5_backing_by_ipa(&f->broker, ipa);
        const struct hv_agx_gpuva_v5_backing *by_pa =
            hv_agx_gpuva_v5_backing_by_pa(&f->broker, ipa);
        if (page % 9 == 0) assert(by_ipa && by_ipa == by_pa && by_ipa->generation == 21);
        else if (page % 3 == 0) assert(!by_ipa && !by_pa);
        else assert(by_ipa && by_ipa == by_pa && by_ipa->generation == 17);
    }
    assert(hv_agx_gpuva_v5_live_backings(&f->broker) ==
           pages - (pages + 2) / 3 + (pages + 8) / 9);
    free(f);
}
static void shared_backing_leaf_references(void)
{
    /* A sharer without a leaf may revoke while another owner's leaf maps the
     * page; the mapping owner may not until its leaf is gone. */
    struct fixture *f = new_fixture();
    uint64_t contiguous[4] = {P_DATA,P_DATA+0x1000,P_DATA+0x2000,P_DATA+0x3000};
    assert(hv_agx_gpuva_v5_create(&f->broker,1,1,P_ROOT,false)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_create(&f->broker,2,1,Q_ROOT,false)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_register_table(&f->broker,1,1,P_L1,1)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_register_table(&f->broker,1,1,P_L2,2)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_update_parent(&f->broker,1,1,P_ROOT,0,P_L1)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_update_parent(&f->broker,1,1,P_L1,0,P_L2)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_register_shared_backing(&f->broker,1,1,17,P_DATA)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_register_shared_backing(&f->broker,2,1,17,P_DATA)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_update_leaf(&f->broker,1,1,P_L2,8,contiguous,17,15,15,15)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_backing_by_pa(&f->broker,P_DATA)->leaf_refs==1);
    assert(hv_agx_gpuva_v5_revoke_backing(&f->broker,2,1,17,P_DATA)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_revoke_backing(&f->broker,1,1,17,P_DATA)==HV_AGX_GPUVA_V5_BUSY);
    assert(hv_agx_gpuva_v5_register_table(&f->broker,2,1,P_DATA,2)==HV_AGX_GPUVA_V5_OWNERSHIP);
    assert(hv_agx_gpuva_v5_update_leaf(&f->broker,1,1,P_L2,8,0,0,15,0,0)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_backing_by_pa(&f->broker,P_DATA)->leaf_refs==0);
    assert(hv_agx_gpuva_v5_revoke_backing(&f->broker,1,1,17,P_DATA)==HV_AGX_GPUVA_V5_OK);
    assert(!hv_agx_gpuva_v5_backing_by_pa(&f->broker,P_DATA));
    free(f);
}
static void local_bitmap_ownership_and_eviction(void)
{
    struct fixture *f = new_fixture();
    uint64_t logical[4] = {P_DATA,P_DATA+0x1000,P_DATA+0x2000,P_DATA+0x3000};
    f->range_backing = true;
    assert(hv_agx_gpuva_v5_configure_local(&f->broker,P_DATA,
        UINT64_C(0x3b800000),UINT64_C(0x4000000)) == HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_create(&f->broker,1,1,P_ROOT,false)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_create(&f->broker,2,1,Q_ROOT,false)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_register_shared_backing(&f->broker,1,1,P_DATA,P_DATA)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_register_shared_backing(&f->broker,2,1,P_DATA,P_DATA)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_register_backing(&f->broker,1,1,P_DATA,P_DATA+0x4000)==HV_AGX_GPUVA_V5_OWNERSHIP);
    assert(hv_agx_gpuva_v5_register_shared_backing(&f->broker,1,1,17,P_DATA+0x4000)==HV_AGX_GPUVA_V5_OWNERSHIP);
    assert(hv_agx_gpuva_v5_register_table(&f->broker,1,1,P_DATA,2)==HV_AGX_GPUVA_V5_OWNERSHIP);
    assert(hv_agx_gpuva_v5_register_table(&f->broker,1,1,P_L2,2)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_update_leaf(&f->broker,1,1,P_L2,8,logical,P_DATA,15,15,15)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_revoke_backing(&f->broker,1,1,P_DATA,P_DATA)==HV_AGX_GPUVA_V5_BUSY);
    assert(hv_agx_gpuva_v5_update_leaf(&f->broker,1,1,P_L2,8,NULL,0,15,0,0)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_revoke_backing(&f->broker,1,1,P_DATA,P_DATA)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_update_leaf(&f->broker,1,1,P_L2,8,logical,P_DATA,15,15,15)==HV_AGX_GPUVA_V5_OWNERSHIP);
    assert(hv_agx_gpuva_v5_revoke_backing(&f->broker,2,1,P_DATA,P_DATA)==HV_AGX_GPUVA_V5_OK);
    free(f);
}
/* REVOKE of a local grant scanned every level-2 table of the owner (EXP1128:
 * 59200 revokes cost 1.95 s while Settings closed). The broker now counts
 * leaf references per local page and scans only when the count is nonzero;
 * the count must stay exact through aliases and other owners' leaves. */
static void local_leaf_reference_count(void)
{
    struct fixture *f = new_fixture();
    uint64_t logical[4] = {P_DATA,P_DATA+0x1000,P_DATA+0x2000,P_DATA+0x3000};
    unsigned page = 0;
    f->range_backing = true;
    assert(hv_agx_gpuva_v5_configure_local(&f->broker,P_DATA,
        UINT64_C(0x3b800000),UINT64_C(0x4000000)) == HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_create(&f->broker,1,1,P_ROOT,false)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_create(&f->broker,2,1,Q_ROOT,false)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_register_table(&f->broker,1,1,P_L2,2)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_register_table(&f->broker,2,1,Q_L2,2)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_register_shared_backing(&f->broker,1,1,P_DATA,P_DATA)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_register_shared_backing(&f->broker,2,1,P_DATA,P_DATA)==HV_AGX_GPUVA_V5_OK);
    /* Two leaves of owner 1 and one of owner 2 reference the same page. */
    assert(hv_agx_gpuva_v5_update_leaf(&f->broker,1,1,P_L2,8,logical,P_DATA,15,15,15)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_update_leaf(&f->broker,1,1,P_L2,9,logical,P_DATA,15,15,0)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_update_leaf(&f->broker,2,1,Q_L2,3,logical,P_DATA,15,15,15)==HV_AGX_GPUVA_V5_OK);
    assert(f->broker.local_leaf_refs[page]==3u);
    /* Rewriting a leaf with the same page leaves the count unchanged. */
    assert(hv_agx_gpuva_v5_update_leaf(&f->broker,1,1,P_L2,9,logical,P_DATA,15,15,15)==HV_AGX_GPUVA_V5_OK);
    assert(f->broker.local_leaf_refs[page]==3u);
    assert(hv_agx_gpuva_v5_update_leaf(&f->broker,1,1,P_L2,8,NULL,0,15,0,0)==HV_AGX_GPUVA_V5_OK);
    assert(f->broker.local_leaf_refs[page]==2u);
    /* Owner 1 still has the alias at index 9. */
    assert(hv_agx_gpuva_v5_revoke_backing(&f->broker,1,1,P_DATA,P_DATA)==HV_AGX_GPUVA_V5_BUSY);
    assert(hv_agx_gpuva_v5_update_leaf(&f->broker,1,1,P_L2,9,NULL,0,15,0,0)==HV_AGX_GPUVA_V5_OK);
    /* Only owner 2's leaf remains: owner 1 may revoke, owner 2 may not. */
    assert(f->broker.local_leaf_refs[page]==1u);
    assert(hv_agx_gpuva_v5_revoke_backing(&f->broker,1,1,P_DATA,P_DATA)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_revoke_backing(&f->broker,2,1,P_DATA,P_DATA)==HV_AGX_GPUVA_V5_BUSY);
    /* A refused update (no grant) does not change the count. */
    assert(hv_agx_gpuva_v5_update_leaf(&f->broker,1,1,P_L2,8,logical,P_DATA,15,15,15)==HV_AGX_GPUVA_V5_OWNERSHIP);
    assert(f->broker.local_leaf_refs[page]==1u);
    assert(hv_agx_gpuva_v5_update_leaf(&f->broker,2,1,Q_L2,3,NULL,0,15,0,0)==HV_AGX_GPUVA_V5_OK);
    assert(f->broker.local_leaf_refs[page]==0u);
    assert(hv_agx_gpuva_v5_revoke_backing(&f->broker,2,1,P_DATA,P_DATA)==HV_AGX_GPUVA_V5_OK);
    free(f);
}
static void reject_prepopulated_tables(void)
{
    struct fixture *f = new_fixture();
    f->pages[0][0] = UINT64_C(0x20000001);
    assert(hv_agx_gpuva_v5_create(&f->broker, 1, 1, P_ROOT, false) ==
           HV_AGX_GPUVA_V5_OWNERSHIP);
    f->pages[0][0] = 0;
    assert(hv_agx_gpuva_v5_create(&f->broker, 1, 1, P_ROOT, false) ==
           HV_AGX_GPUVA_V5_OK);
    f->pages[1][0] = UINT64_C(0x20000001);
    assert(hv_agx_gpuva_v5_register_table(&f->broker, 1, 1, P_L1, 1) ==
           HV_AGX_GPUVA_V5_OWNERSHIP);
    free(f);
}
static void cleanup(struct fixture *f, uint64_t id, uint64_t root,
                    uint64_t l1, uint64_t l2, uint64_t data)
{
    assert(hv_agx_gpuva_v5_revoke_backing(&f->broker,id,1,17,data)==
           HV_AGX_GPUVA_V5_BUSY);
    assert(hv_agx_gpuva_v5_update_leaf(&f->broker,id,1,l2,8,NULL,0,15,0,0)==
           HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_revoke_backing(&f->broker,id,1,17,data)==
           HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_revoke_table(&f->broker,id,1,l2,2)==
           HV_AGX_GPUVA_V5_BUSY);
    assert(hv_agx_gpuva_v5_update_parent(&f->broker,id,1,l1,0,0)==
           HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_revoke_table(&f->broker,id,1,l2,2)==
           HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_update_parent(&f->broker,id,1,root,0,0)==
           HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_revoke_table(&f->broker,id,1,l1,1)==
           HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_destroy(&f->broker,id,1)==HV_AGX_GPUVA_V5_OK);
}
static void mailbox_ownership(void)
{
    struct fixture *f = new_fixture();
    uint64_t pa = 1;
    prepare(f, 1, P_ROOT, P_L1, P_L2, P_DATA, true);
    assert(hv_agx_gpuva_v5_validate_envelope(&f->broker, 7,
           AGX_GPUVA_V5_ATTACH_MAILBOX, 0) == HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_validate_envelope(&f->broker, 7,
           AGX_GPUVA_V5_ATTACH_MAILBOX, 1) == HV_AGX_GPUVA_V5_INVALID);
    assert(hv_agx_gpuva_v5_validate_envelope(&f->broker, 7,
           AGX_GPUVA_V5_NEGOTIATE_LIVE + 1, 0) == HV_AGX_GPUVA_V5_INVALID);
    /* A live table or backing page can never become the response page. */
    assert(hv_agx_gpuva_v5_attach_mailbox(&f->broker, P_L2, &pa) ==
           HV_AGX_GPUVA_V5_OWNERSHIP && pa == 1);
    assert(hv_agx_gpuva_v5_attach_mailbox(&f->broker, P_DATA, &pa) ==
           HV_AGX_GPUVA_V5_OWNERSHIP);
    assert(hv_agx_gpuva_v5_attach_mailbox(&f->broker, P_ROOT2 + 0x1000, &pa) ==
           HV_AGX_GPUVA_V5_INVALID);
    f->blocked_ipa = P_ROOT2;
    assert(hv_agx_gpuva_v5_attach_mailbox(&f->broker, P_ROOT2, &pa) ==
           HV_AGX_GPUVA_V5_OWNERSHIP);
    f->blocked_ipa = 0;
    assert(hv_agx_gpuva_v5_attach_mailbox(&f->broker, P_ROOT2, &pa) ==
           HV_AGX_GPUVA_V5_OK && pa == P_ROOT2 &&
           f->broker.mailbox_pa == P_ROOT2);
    /* While attached, the mailbox page can never become a table or backing. */
    assert(hv_agx_gpuva_v5_register_table(&f->broker, 1, 1, P_ROOT2, 1) ==
           HV_AGX_GPUVA_V5_OWNERSHIP);
    assert(hv_agx_gpuva_v5_register_backing(&f->broker, 1, 1, 18, P_ROOT2) ==
           HV_AGX_GPUVA_V5_OWNERSHIP);
    assert(hv_agx_gpuva_v5_register_shared_backing(&f->broker, 1, 1, 18,
           P_ROOT2) == HV_AGX_GPUVA_V5_OWNERSHIP);
    assert(hv_agx_gpuva_v5_create(&f->broker, 3, 1, P_ROOT2, true) ==
           HV_AGX_GPUVA_V5_OWNERSHIP);
    assert(hv_agx_gpuva_v5_attach_mailbox(&f->broker, 0, &pa) ==
           HV_AGX_GPUVA_V5_OK && pa == 0 && f->broker.mailbox_pa == 0);
    assert(hv_agx_gpuva_v5_register_table(&f->broker, 1, 1, P_ROOT2, 1) ==
           HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_attach_mailbox(&f->broker, P_ROOT2, &pa) ==
           HV_AGX_GPUVA_V5_OWNERSHIP);
    assert(hv_agx_gpuva_v5_attach_mailbox(&f->broker, P_ROOT2, NULL) ==
           HV_AGX_GPUVA_V5_INVALID);
    free(f);
}
/* M2-f6: the built-in table (HV_AGX_GPUVA_V5_BACKINGS, 2 GiB) capped segment
 * 3 at 1792 MiB on every Mac. Storage the hypervisor sizes from guest RAM
 * holds more system pages, found by PA and IPA as before; the page past it
 * is refused without mutation, and the configurable limit follows it. */
static void external_storage_scales_system_backings(void)
{
    const unsigned pages = HV_AGX_GPUVA_V5_BACKINGS + 8192u;
    const size_t bytes = hv_agx_gpuva_v5_storage_bytes(pages);
    void *storage = malloc(bytes);
    struct fixture *f;
    const uint64_t last = P_DATA + (uint64_t)pages * HV_AGX_GPUVA_V5_PAGE;
    assert(bytes > hv_agx_gpuva_v5_storage_bytes(HV_AGX_GPUVA_V5_BACKINGS));
    assert(hv_agx_gpuva_v5_storage_bytes(0) == 0);
    assert(storage);
    memset(storage, 0xa5, bytes); /* init clears it */
    f = new_fixture_with(storage, pages);
    assert(hv_agx_gpuva_v5_system_capacity(&f->broker) == pages);
    f->range_backing = true;
    assert(hv_agx_gpuva_v5_create(&f->broker, 1, 1, P_ROOT, false) == HV_AGX_GPUVA_V5_OK);
    for (unsigned page = 0; page < pages; ++page)
        assert(hv_agx_gpuva_v5_register_shared_backing(&f->broker, 1, 1, 17,
            P_DATA + (uint64_t)page * HV_AGX_GPUVA_V5_PAGE) == HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_live_backings(&f->broker) == pages);
    assert(hv_agx_gpuva_v5_backing_by_pa(&f->broker, last - HV_AGX_GPUVA_V5_PAGE));
    assert(hv_agx_gpuva_v5_register_shared_backing(&f->broker, 1, 1, 17, last) ==
           HV_AGX_GPUVA_V5_CAPACITY);
    assert(hv_agx_gpuva_v5_configure_system_backings(&f->broker, pages) ==
           HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_configure_system_backings(&f->broker, pages + 1u) ==
           HV_AGX_GPUVA_V5_INVALID);
    free(f);
    free(storage);
    /* Without storage the built-in table keeps its capacity. */
    f = new_fixture();
    assert(hv_agx_gpuva_v5_system_capacity(&f->broker) == HV_AGX_GPUVA_V5_BACKINGS);
    free(f);
}

int main(void)
{
    independent_flush();
    guest_backing_validation();
    destroy_order_with_live_tables();
    reject_prepopulated_tables();
    shared_graph_grants();
    shared_reserve_capacity();
    system_backings_hold_uma_capacity();
    external_storage_scales_system_backings();
    backing_index_survives_deletion();
    shared_backing_leaf_references();
    local_bitmap_ownership_and_eviction();
    local_leaf_reference_count();
    struct fixture *f = new_fixture();
    uint64_t ptoken, qtoken, logical[4]={P_DATA,P_DATA+0x1000,
                                          P_DATA+0x2000,P_DATA+0x3000};
    uint64_t original, ctx0lo=f->slots[0][0], ctx0hi=f->slots[0][1];
    prepare(f,1,P_ROOT,P_L1,P_L2,P_DATA,true);
    prepare(f,2,Q_ROOT,Q_L1,Q_L2,Q_DATA,false);
    assert(f->pages[2][8] != f->pages[6][8]);
    original=f->pages[2][8];
    assert(hv_agx_gpuva_v5_update_leaf(&f->broker,1,1,P_L2,8,logical,17,2,2,2)==
           HV_AGX_GPUVA_V5_OK);
    assert(f->pages[2][8]==original);
    assert(hv_agx_gpuva_v5_update_leaf(&f->broker,1,1,P_L2,8,NULL,0,2,0,0)==
           HV_AGX_GPUVA_V5_INVALID);
    assert(f->pages[2][8]==original);
    assert(hv_agx_gpuva_v5_update_leaf(&f->broker,1,1,P_L2,9,logical,18,15,15,15)==
           HV_AGX_GPUVA_V5_OWNERSHIP);
    logical[1]=Q_DATA+0x1000;
    assert(hv_agx_gpuva_v5_update_leaf(&f->broker,1,1,P_L2,9,logical,17,15,15,15)==
           HV_AGX_GPUVA_V5_OWNERSHIP);
    assert(f->pages[2][9]==0);
    assert(hv_agx_gpuva_v5_lease(&f->broker,1,1,0,&ptoken)==HV_AGX_GPUVA_V5_INVALID);
    assert(hv_agx_gpuva_v5_lease(&f->broker,1,1,1,&ptoken)==HV_AGX_GPUVA_V5_OK);
    assert(f->slots[1][1]==0);
    assert(hv_agx_gpuva_v5_job_begin(&f->broker,1,ptoken)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_register_table(&f->broker,1,1,P_ROOT2,0)==
           HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_relocate_root(&f->broker,1,1,P_ROOT2)==
           HV_AGX_GPUVA_V5_BUSY);
    assert(hv_agx_gpuva_v5_release(&f->broker,1,ptoken)==HV_AGX_GPUVA_V5_BUSY);
    assert(hv_agx_gpuva_v5_lease(&f->broker,2,1,1,&qtoken)==HV_AGX_GPUVA_V5_BUSY);
    assert(hv_agx_gpuva_v5_job_end(&f->broker,1,ptoken)==HV_AGX_GPUVA_V5_OK);
    original=f->pages[2][8]; f->fail_once=1;
    assert(hv_agx_gpuva_v5_update_leaf(&f->broker,1,1,P_L2,8,NULL,0,15,0,0)==
           HV_AGX_GPUVA_V5_TLB);
    assert(f->pages[2][8]==original && !f->broker.tainted);
    assert(hv_agx_gpuva_v5_release(&f->broker,1,ptoken)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_relocate_root(&f->broker,1,1,P_ROOT2)==
           HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_revoke_table(&f->broker,1,1,P_ROOT2,0)==
           HV_AGX_GPUVA_V5_BUSY);
    assert(hv_agx_gpuva_v5_lease(&f->broker,2,1,1,&qtoken)==HV_AGX_GPUVA_V5_OK);
    assert(qtoken!=ptoken && f->slots[1][1]==0);
    assert(hv_agx_gpuva_v5_job_begin(&f->broker,1,ptoken)==HV_AGX_GPUVA_V5_STALE);
    assert(hv_agx_gpuva_v5_release(&f->broker,1,qtoken)==HV_AGX_GPUVA_V5_OK);
    f->legacy_slot63=true;
    assert(hv_agx_gpuva_v5_lease(&f->broker,1,1,63,&ptoken)==
           HV_AGX_GPUVA_V5_BUSY);
    f->legacy_slot63=false;
    assert(hv_agx_gpuva_v5_lease(&f->broker,1,1,63,&ptoken)==
           HV_AGX_GPUVA_V5_OK);
    assert(f->slots[63][1]==0);
    assert(hv_agx_gpuva_v5_release(&f->broker,63,ptoken)==HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_destroy(&f->broker,1,1)==HV_AGX_GPUVA_V5_BUSY);
    cleanup(f,1,P_ROOT,P_L1,P_L2,P_DATA);
    assert(hv_agx_gpuva_v5_revoke_table(&f->broker,1,1,P_ROOT2,0)==
           HV_AGX_GPUVA_V5_STALE);
    cleanup(f,2,Q_ROOT,Q_L1,Q_L2,Q_DATA);
    assert(f->slots[0][0]==ctx0lo && f->slots[0][1]==ctx0hi && f->prefix);
    free(f);
    f=new_fixture();
    f->slots[0][1] ^= 1;
    assert(hv_agx_gpuva_v5_create(&f->broker,1,1,P_ROOT,true)==
           HV_AGX_GPUVA_V5_TAINTED);
    free(f);
    f=new_fixture();
    prepare(f,1,P_ROOT,P_L1,P_L2,P_DATA,true);
    f->fail_once=2;
    assert(hv_agx_gpuva_v5_update_leaf(&f->broker,1,1,P_L2,8,NULL,0,15,0,0)==
           HV_AGX_GPUVA_V5_OK); /* No active slot, no TLBI dependency. */
    assert(hv_agx_gpuva_v5_lease(&f->broker,1,1,1,&ptoken)==
           HV_AGX_GPUVA_V5_TAINTED); /* Publish and rollback both lack ack. */
    free(f);
    f=new_fixture();
    f->legacy_slot63=true;
    assert(hv_agx_gpuva_v5_create(&f->broker,1,1,P_ROOT,true)==
           HV_AGX_GPUVA_V5_OK);
    uint64_t tokens[64]={0};
    for (unsigned slot=1;slot<=62;++slot)
        assert(hv_agx_gpuva_v5_lease(&f->broker,1,1,slot,&tokens[slot])==
               HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_lease(&f->broker,1,1,63,&tokens[63])==
           HV_AGX_GPUVA_V5_BUSY);
    for (unsigned slot=1;slot<=62;++slot)
        assert(hv_agx_gpuva_v5_release(&f->broker,slot,tokens[slot])==
               HV_AGX_GPUVA_V5_OK);
    assert(hv_agx_gpuva_v5_destroy(&f->broker,1,1)==HV_AGX_GPUVA_V5_OK);
    free(f);
    mailbox_ownership();
    return 0;
}

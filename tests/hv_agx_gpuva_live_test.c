/* WHAT REAL BUG OR INVARIANT WILL THIS TEST CATCH?
 * Negotiation must authenticate actual empty native descriptors through real
 * broker/MMIO code; two live jobs must never permit an in-use map mutation. */
#include <stdio.h>
#define main legacy_broker_selftest
#include "hv_agx_gpuva_v5_test.c"
#undef main
#include "../src/hv_agx_gpuva_v5_mmio.h"

static unsigned sync_failures;
static unsigned invalidate_slot;
static bool selected_invalidate(void *opaque, unsigned slot)
{
    if (slot == invalidate_slot) { invalidate_slot = 0; return false; }
    return invalidate(opaque, slot);
}
static bool checked_sync(void *opaque)
{
    (void)opaque;
    if (sync_failures) { --sync_failures; return false; }
    return true;
}
#include "retained_execute_fixture.h"
static void extension_execute(void *opaque, const AGX_GPUVA_V5_REQUEST *q,
                              AGX_GPUVA_V5_RESPONSE *r)
{
    gpuva_execute(opaque, q, r);
}
static unsigned request(struct fixture *f, struct hv_agx_gpuva_v5_wire *wire,
                        AGX_GPUVA_V5_REQUEST *q)
{
    uint64_t bell = AGX_GPUVA_V5_DOORBELL_WINDOW;
    q->Version = 6; q->Bytes = sizeof(*q); q->Sequence++;
    wire->request = *q;
    assert(hv_agx_gpuva_v5_mmio(wire, AGX_GPUVA_V5_DOORBELL, &bell,
                              true, 2, extension_execute, f));
    assert(wire->response.Receipt == q->Sequence);
    return wire->response.Status;
}
static void negotiate(struct fixture *f, bool enable)
{
    AGX_GPUVA_V5_REQUEST q = {0};
    struct hv_agx_gpuva_v5_wire wire = {0};
    q.Epoch = 7; q.Command = AGX_GPUVA_V5_NEGOTIATE_LIVE;
    q.ProcessId = 1; q.ProcessGeneration = 1;
    q.Token = enable ? AGX_GPUVA_V5_CAP_LIVE_ADDITIVE : 0;
    q.AllocationGeneration = AGX_GPUVA_V5_LIVE_ABI;
    assert(request(f, &wire, &q) == HV_AGX_GPUVA_V5_OK);
    assert(wire.response.Reserved[0] == q.Token &&
           wire.response.Reserved[1] == AGX_GPUVA_V5_LIVE_ABI);
}
static void protocol(void)
{
    struct fixture *f = new_fixture();
    struct hv_agx_gpuva_v5_wire wire = {0};
    AGX_GPUVA_V5_REQUEST q = {0};
    q.Epoch = 7; q.Command = AGX_GPUVA_V5_QUERY_CAPS;
    assert(request(f, &wire, &q) == HV_AGX_GPUVA_V5_OK);
    assert(wire.response.Reserved[0] == AGX_GPUVA_V5_CAP_LIVE_ADDITIVE);
    /* The same query traverses the real attached mailbox transport. */
    unsigned char mailbox[AGX_GPUVA_V5_MAILBOX_BYTES] = {0};
    wire.mailbox = mailbox;
    q.Sequence++;
    memcpy(mailbox, &q, sizeof(q));
    uint64_t mailbox_bell = AGX_GPUVA_V5_DOORBELL_MAILBOX;
    assert(hv_agx_gpuva_v5_mmio(&wire, AGX_GPUVA_V5_DOORBELL, &mailbox_bell,
                              true, 2, extension_execute, f));
    AGX_GPUVA_V5_RESPONSE mailbox_response;
    memcpy(&mailbox_response, mailbox + AGX_GPUVA_V5_MAILBOX_RESPONSE,
           sizeof(mailbox_response));
    assert(mailbox_response.Status == 0 &&
           mailbox_response.Reserved[0] == AGX_GPUVA_V5_CAP_LIVE_ADDITIVE);
    q.TableIpa = P_ROOT;
    assert(request(f, &wire, &q) == HV_AGX_GPUVA_V5_INVALID);
    q.TableIpa = 0; q.Epoch = 6;
    assert(request(f, &wire, &q) == HV_AGX_GPUVA_V5_STALE);
    q.Epoch = 7; q.Command = AGX_GPUVA_V5_REGISTER_SYSMEM_BACKING;
    assert(request(f, &wire, &q) == HV_AGX_GPUVA_V5_INVALID);
    q.Command = AGX_GPUVA_V5_NEGOTIATE_LIVE;
    q.ProcessId = 1; q.ProcessGeneration = 1;
    q.Token = AGX_GPUVA_V5_CAP_LIVE_ADDITIVE;
    q.AllocationGeneration = AGX_GPUVA_V5_LIVE_ABI;
    assert(request(f, &wire, &q) == HV_AGX_GPUVA_V5_STALE);
    prepare(f, 1, P_ROOT, P_L1, P_L2, P_DATA, false);
    q.AllocationGeneration++;
    assert(request(f, &wire, &q) == HV_AGX_GPUVA_V5_INVALID);
    q.AllocationGeneration--; q.Token |= 1;
    assert(request(f, &wire, &q) == HV_AGX_GPUVA_V5_INVALID);
    q.Token = AGX_GPUVA_V5_CAP_LIVE_ADDITIVE; q.Flags = 1;
    assert(request(f, &wire, &q) == HV_AGX_GPUVA_V5_INVALID);
    q.Flags = 0;
    assert(request(f, &wire, &q) == HV_AGX_GPUVA_V5_OK);
    uint64_t bell = 1;
    assert(hv_agx_gpuva_v5_mmio(&wire, AGX_GPUVA_V5_DOORBELL, &bell,
                              true, 2, extension_execute, f));
    assert(wire.response.Status == HV_AGX_GPUVA_V5_INVALID); /* replay */
    uint64_t token;
    assert(hv_agx_gpuva_v5_lease(&f->broker,1,1,1,&token)==0);
    assert(hv_agx_gpuva_v5_job_begin(&f->broker,1,token)==0);
    assert(request(f, &wire, &q) == HV_AGX_GPUVA_V5_BUSY);
    q.Token = 0;
    assert(request(f, &wire, &q) == HV_AGX_GPUVA_V5_BUSY);
    assert(f->broker.processes[0].live_additive);
    free(f);
}
static void live_maps(void)
{
    struct fixture *f = new_fixture();
    uint64_t token1, token2;
    uint64_t logical[4] = {P_DATA,P_DATA+0x1000,P_DATA+0x2000,P_DATA+0x3000};
    prepare(f,1,P_ROOT,P_L1,P_L2,P_DATA,false);
    prepare(f,2,Q_ROOT,Q_L1,Q_L2,Q_DATA,false);
    negotiate(f,true);
    assert(hv_agx_gpuva_v5_lease(&f->broker,1,1,1,&token1)==0);
    assert(hv_agx_gpuva_v5_lease(&f->broker,1,1,2,&token2)==0);
    assert(hv_agx_gpuva_v5_job_begin(&f->broker,1,token1)==0);
    assert(hv_agx_gpuva_v5_job_begin(&f->broker,1,token1)==0); /* depth two */
    assert(hv_agx_gpuva_v5_job_begin(&f->broker,2,token2)==0);
    uint64_t generation = f->broker.processes[0].map_generation;
    f->broker.processes[0].map_generation = UINT64_MAX;
    unsigned before_overflow = f->invalidations[1];
    assert(hv_agx_gpuva_v5_update_leaf(&f->broker,1,1,P_L2,9,logical,17,15,15,15)==HV_AGX_GPUVA_V5_CAPACITY);
    assert(!f->pages[2][9] && f->invalidations[1]==before_overflow);
    f->broker.processes[0].map_generation = generation;
    unsigned inv1 = f->invalidations[1], inv2 = f->invalidations[2];
    AGX_GPUVA_V5_REQUEST map = {0};
    struct hv_agx_gpuva_v5_wire wire = {0};
    map.Epoch=7; map.Command=AGX_GPUVA_V5_UPDATE_LEAF; map.Flags=15;
    map.ProcessId=1; map.ProcessGeneration=1; map.TableIpa=P_L2;
    map.Index=9; map.AllocationGeneration=17; map.ValidMask=15; map.WritableMask=15;
    for (unsigned i=0;i<4;++i) map.LogicalIpa[i]=logical[i];
    assert(request(f,&wire,&map)==HV_AGX_GPUVA_V5_OK);
    assert(wire.response.MapGeneration==generation+1 && !wire.response.Flags);
    assert(f->pages[2][9] == f->pages[2][8]);
    assert(f->invalidations[1]==inv1+1 && f->invalidations[2]==inv2+1);
    assert(f->broker.processes[0].map_generation==generation+1);
    assert(hv_agx_gpuva_v5_flush_tlb(&f->broker,1,1,P_ROOT,0,0)==0);
    assert(hv_agx_gpuva_v5_update_leaf(&f->broker,1,1,P_L2,10,logical,17,1,1,1)==HV_AGX_GPUVA_V5_INVALID);
    assert(hv_agx_gpuva_v5_update_leaf(&f->broker,1,1,P_L2,8,logical,17,1,1,1)==HV_AGX_GPUVA_V5_BUSY);
    assert(hv_agx_gpuva_v5_update_leaf(&f->broker,1,1,P_L2,8,NULL,0,15,0,0)==HV_AGX_GPUVA_V5_BUSY);
    assert(hv_agx_gpuva_v5_update_leaf(&f->broker,1,1,P_L2,8,logical,17,15,15,0)==HV_AGX_GPUVA_V5_BUSY);
    /* A valid owned replacement is destructive too. */
    assert(hv_agx_gpuva_v5_register_backing(&f->broker,1,1,19,P_ROOT2)==0);
    uint64_t replacement[4]={P_ROOT2,P_ROOT2+0x1000,P_ROOT2+0x2000,P_ROOT2+0x3000};
    assert(hv_agx_gpuva_v5_update_leaf(&f->broker,1,1,P_L2,8,replacement,19,15,15,15)==HV_AGX_GPUVA_V5_BUSY);
    assert(hv_agx_gpuva_v5_update_leaf(&f->broker,1,1,P_L2,10,logical,18,15,15,15)==HV_AGX_GPUVA_V5_OWNERSHIP);
    assert(hv_agx_gpuva_v5_update_leaf(&f->broker,1,2,P_L2,10,logical,17,15,15,15)==HV_AGX_GPUVA_V5_STALE);
    assert(hv_agx_gpuva_v5_update_leaf(&f->broker,2,1,P_L2,10,logical,17,15,15,15)==HV_AGX_GPUVA_V5_INVALID);
    assert(hv_agx_gpuva_v5_update_parent(&f->broker,1,1,P_L1,1,Q_L2)==HV_AGX_GPUVA_V5_OWNERSHIP);
    assert(hv_agx_gpuva_v5_update_parent(&f->broker,1,1,P_L1,1,P_L2)==0);
    assert(hv_agx_gpuva_v5_update_parent(&f->broker,1,1,P_L1,0,0)==HV_AGX_GPUVA_V5_BUSY);
    assert(hv_agx_gpuva_v5_revoke_backing(&f->broker,1,1,17,P_DATA)==HV_AGX_GPUVA_V5_BUSY);
    assert(hv_agx_gpuva_v5_revoke_table(&f->broker,1,1,P_L2,2)==HV_AGX_GPUVA_V5_BUSY);
    assert(hv_agx_gpuva_v5_relocate_root(&f->broker,1,1,P_ROOT)==HV_AGX_GPUVA_V5_BUSY);
    assert(hv_agx_gpuva_v5_destroy(&f->broker,1,1)==HV_AGX_GPUVA_V5_BUSY);
    assert(hv_agx_gpuva_v5_job_end(&f->broker,1,token1+99)==HV_AGX_GPUVA_V5_STALE);
    assert(hv_agx_gpuva_v5_job_end(&f->broker,1,token1)==0);
    assert(hv_agx_gpuva_v5_update_leaf(&f->broker,1,1,P_L2,8,NULL,0,15,0,0)==HV_AGX_GPUVA_V5_BUSY);
    assert(hv_agx_gpuva_v5_job_end(&f->broker,1,token1)==0);
    assert(hv_agx_gpuva_v5_update_leaf(&f->broker,1,1,P_L2,8,NULL,0,15,0,0)==HV_AGX_GPUVA_V5_BUSY);
    assert(hv_agx_gpuva_v5_job_end(&f->broker,2,token2)==0);
    assert(hv_agx_gpuva_v5_update_leaf(&f->broker,1,1,P_L2,8,NULL,0,15,0,0)==0);
    negotiate(f,false);
    assert(hv_agx_gpuva_v5_job_begin(&f->broker,1,token1)==0);
    assert(hv_agx_gpuva_v5_update_leaf(&f->broker,1,1,P_L2,10,logical,17,15,15,15)==HV_AGX_GPUVA_V5_BUSY);
    free(f);
}
static void empty_chain(void)
{
    struct fixture *f = new_fixture();
    uint64_t token;
    uint64_t logical[4]={P_DATA,P_DATA+0x1000,P_DATA+0x2000,P_DATA+0x3000};
    prepare(f,1,P_ROOT,P_L1,P_L2,P_DATA,false);
    negotiate(f,true);
    assert(hv_agx_gpuva_v5_lease(&f->broker,1,1,1,&token)==0);
    assert(hv_agx_gpuva_v5_job_begin(&f->broker,1,token)==0);
    assert(hv_agx_gpuva_v5_register_table(&f->broker,1,1,Q_L1,1)==0);
    assert(hv_agx_gpuva_v5_register_table(&f->broker,1,1,Q_L2,2)==0);
    assert(hv_agx_gpuva_v5_update_parent(&f->broker,1,1,P_ROOT,1,Q_L1)==0);
    assert(hv_agx_gpuva_v5_update_parent(&f->broker,1,1,Q_L1,0,Q_L2)==0);
    assert(hv_agx_gpuva_v5_update_leaf(&f->broker,1,1,Q_L2,0,logical,17,15,15,15)==0);
    assert(f->pages[0][1] && f->pages[5][0] && f->pages[6][0]);
    assert(f->broker.slots[1].jobs==1 && !f->broker.tainted);
    free(f);
}
static void failures(void)
{
    for (unsigned mode=0;mode<5;++mode) {
        struct fixture *f=new_fixture();
        uint64_t token;
        uint64_t logical[4]={P_DATA,P_DATA+0x1000,P_DATA+0x2000,P_DATA+0x3000};
        prepare(f,1,P_ROOT,P_L1,P_L2,P_DATA,false);
        negotiate(f,true);
        assert(hv_agx_gpuva_v5_lease(&f->broker,1,1,1,&token)==0);
        assert(hv_agx_gpuva_v5_job_begin(&f->broker,1,token)==0);
        uint64_t generation=f->broker.processes[0].map_generation;
        f->broker.ops.sync_tables=checked_sync;
        sync_failures=mode<2 ? mode+1 : 0;
        f->fail_once=mode>=2 && mode<4 ? mode-1 : 0;
        if (mode==4) {
            uint64_t second;
            assert(hv_agx_gpuva_v5_lease(&f->broker,1,1,2,&second)==0);
            assert(hv_agx_gpuva_v5_job_begin(&f->broker,2,second)==0);
            f->broker.ops.invalidate=selected_invalidate;
            invalidate_slot=2; /* failure after slot1 already acknowledged */
        }
        assert(hv_agx_gpuva_v5_update_leaf(&f->broker,1,1,P_L2,9,logical,17,15,15,15)==HV_AGX_GPUVA_V5_TAINTED);
        assert(!f->pages[2][9] && f->broker.tainted);
        assert(f->broker.processes[0].map_generation==generation);
        assert(hv_agx_gpuva_v5_job_end(&f->broker,1,token)==HV_AGX_GPUVA_V5_TAINTED);
        assert(hv_agx_gpuva_v5_revoke_backing(&f->broker,1,1,17,P_DATA)==HV_AGX_GPUVA_V5_TAINTED);
        /* Software broker re-init after platform teardown loses negotiation;
         * this is not a hardware reset or permission to skip GPU quiescence. */
        struct hv_agx_gpuva_v5_ops ops=f->broker.ops;
        memset(&f->broker,0,sizeof(f->broker));
        assert(hv_agx_gpuva_v5_init(&f->broker,8,&ops)==0);
        assert(!f->broker.processes[0].live_additive);
        assert(hv_agx_gpuva_v5_job_begin(&f->broker,1,token)==HV_AGX_GPUVA_V5_STALE);
        assert(hv_agx_gpuva_v5_validate_envelope(&f->broker,7,AGX_GPUVA_V5_NEGOTIATE_LIVE,0)==HV_AGX_GPUVA_V5_STALE);
        free(f);
    }
}
static void negotiation_lifetime(void)
{
    struct fixture *f=new_fixture();
    assert(hv_agx_gpuva_v5_create(&f->broker,1,1,P_ROOT,false)==0);
    negotiate(f,true);
    assert(hv_agx_gpuva_v5_destroy(&f->broker,1,1)==0);
    assert(hv_agx_gpuva_v5_create(&f->broker,1,2,P_ROOT,false)==0);
    assert(!f->broker.processes[0].live_additive);
    free(f);
}
int main(void)
{
    negotiation_lifetime(); protocol(); live_maps(); empty_chain(); failures();
    puts("Q22b broker: negotiated live leaf/parent, depth two, ownership, rollback quarantine PASS");
    return 0;
}

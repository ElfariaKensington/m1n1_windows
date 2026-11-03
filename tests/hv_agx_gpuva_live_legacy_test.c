/* WHAT REAL BUG OR INVARIANT WILL THIS TEST CATCH?
 * The deployed old wire must reject optional discovery/negotiation without
 * execution, allowing a new KMD to choose its legacy path. */
#include <assert.h>
#include "../src/hv_agx_gpuva_v5_mmio.h"
static unsigned calls;
static void execute(void *unused, const AGX_GPUVA_V5_REQUEST *q,
                    AGX_GPUVA_V5_RESPONSE *r)
{
    (void)unused; (void)q;
    ++calls; r->Status=0;
}
int main(void)
{
    struct hv_agx_gpuva_v5_wire wire={0};
    uint64_t bell=1;
    wire.request.Version=6;
    wire.request.Bytes=sizeof(wire.request);
    wire.request.Sequence=1;
    wire.request.Command=AGX_GPUVA_V5_QUERY_CAPS;
    assert(hv_agx_gpuva_v5_mmio(&wire,AGX_GPUVA_V5_DOORBELL,&bell,true,2,execute,0));
    assert(wire.response.Status==1 && !calls && !wire.response.Reserved[0]);
    wire.request.Sequence++;
    wire.request.Command=AGX_GPUVA_V5_NEGOTIATE_LIVE;
    assert(hv_agx_gpuva_v5_mmio(&wire,AGX_GPUVA_V5_DOORBELL,&bell,true,2,execute,0));
    assert(wire.response.Status==1 && !calls && !wire.response.Reserved[0]);
    wire.request.Sequence++;
    wire.request.Command=AGX_GPUVA_V5_CREATE;
    assert(hv_agx_gpuva_v5_mmio(&wire,AGX_GPUVA_V5_DOORBELL,&bell,true,2,execute,0));
    assert(wire.response.Status==0 && calls==1);
    return 0;
}

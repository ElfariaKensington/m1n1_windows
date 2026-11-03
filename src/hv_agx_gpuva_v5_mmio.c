#include "hv_agx_gpuva_v5_mmio.h"
#include <string.h>
_Static_assert(sizeof(AGX_GPUVA_V5_REQUEST)==128, "v5 request ABI");
_Static_assert(sizeof(AGX_GPUVA_V5_RESPONSE)==64, "v5 response ABI");
bool hv_agx_gpuva_v5_mmio(struct hv_agx_gpuva_v5_wire *s, uint64_t offset,
    uint64_t *value, bool write, unsigned width, hv_agx_gpuva_v5_execute execute,
    void *context)
{
    unsigned bytes;
    if (!s || !value || width > 3) return false;
    bytes = 1u << width;
    if (offset & (bytes - 1)) return false;
    if (write && offset == AGX_GPUVA_V5_DOORBELL) {
        AGX_GPUVA_V5_REQUEST q;
        AGX_GPUVA_V5_RESPONSE r = {0};
        unsigned char *box = s->mailbox;
        if (width != 2) return false;
        if (*value == AGX_GPUVA_V5_DOORBELL_WINDOW) q = s->request;
        else if (*value == AGX_GPUVA_V5_DOORBELL_MAILBOX && box)
            memcpy(&q, box, sizeof(q)); /* single snapshot of guest memory */
        else return false;
        r.Receipt = q.Sequence;
        r.Status = 1;
        if (q.Version == AGX_GPUVA_V5_VERSION && q.Bytes == sizeof(q) &&
            q.Sequence && q.Sequence > s->last_sequence &&
            q.Command >= AGX_GPUVA_V5_CREATE &&
            q.Command <= AGX_GPUVA_V5_NEGOTIATE_LIVE && execute) {
            s->last_sequence = q.Sequence;
            execute(context, &q, &r);
        }
        s->response = r;
        /* The page that carried the request carries its response, even when
         * the request detached or replaced the mailbox. */
        if (*value == AGX_GPUVA_V5_DOORBELL_MAILBOX)
            memcpy(box + AGX_GPUVA_V5_MAILBOX_RESPONSE, &r, sizeof(r));
        return true;
    }
    if (offset <= sizeof(s->request) - bytes) {
        if (write) memcpy((unsigned char *)&s->request + offset, value, bytes);
        else { *value = 0; memcpy(value, (unsigned char *)&s->request + offset, bytes); }
        return true;
    }
    if (!write && offset >= AGX_GPUVA_V5_RESPONSE_OFFSET &&
        offset - AGX_GPUVA_V5_RESPONSE_OFFSET <= sizeof(s->response) - bytes) {
        *value = 0;
        memcpy(value, (unsigned char *)&s->response + offset - AGX_GPUVA_V5_RESPONSE_OFFSET,
               bytes);
        return true;
    }
    return false;
}

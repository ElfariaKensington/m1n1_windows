#include "hv_agx_gpuva_v5.h"
#include "apple_agx_gpuva_broker_v5.h"
#include "../../drivers/apple-agx/shared/include/apple_agx_uat.h"
#include <string.h>
#define GPUVA_PA_MASK UINT64_C(0x000000ffffffc000)

static bool local_index(const struct hv_agx_gpuva_v5 *b, uint64_t ipa,
                        unsigned *index)
{
    if (!b->local_bytes || ipa < b->local_base ||
        ipa - b->local_base >= b->local_bytes) return false;
    *index = (unsigned)((ipa - b->local_base) / HV_AGX_GPUVA_V5_PAGE);
    return true;
}

static bool private_local(const struct hv_agx_gpuva_v5 *b, uint64_t ipa)
{
    uint64_t base = b->local_base + b->local_bytes;
    return b->private_bytes && ipa >= base &&
           ipa - base < b->private_bytes;
}

static bool local_granted(const struct hv_agx_gpuva_v5 *b, unsigned owner,
                          unsigned index)
{
    return (b->local_grants[owner][index / 64u] &
            (UINT64_C(1) << (index % 64u))) != 0;
}

static void local_set_grant(struct hv_agx_gpuva_v5 *b, unsigned owner,
                            unsigned index, bool granted)
{
    uint64_t *word = &b->local_grants[owner][index / 64u];
    uint64_t bit = UINT64_C(1) << (index % 64u);
    if (granted) *word |= bit;
    else *word &= ~bit;
}

/* Count a published leaf descriptor that maps a local-reserve page. */
static void local_leaf_ref(struct hv_agx_gpuva_v5 *b, uint64_t descriptor,
                           bool add)
{
    unsigned page;
    uint16_t *refs;
    if (!descriptor || !local_index(b, descriptor & GPUVA_PA_MASK, &page))
        return;
    refs = &b->local_leaf_refs[page];
    if (*refs == UINT16_MAX) return;
    if (add) ++*refs;
    else if (*refs) --*refs;
}

/* M2-d backing index: records are found by PA and by IPA through two
 * linear-probing tables of record index + 1, kept gap-free by
 * backward-shift deletion, so every lookup is O(1) expected. */
#define BACKING_SLOT_MASK(b) ((1u << (b)->backing_slot_bits) - 1u)

static uint32_t backing_hash(const struct hv_agx_gpuva_v5 *b, uint64_t key)
{
    return (uint32_t)(((key / HV_AGX_GPUVA_V5_PAGE) *
                       UINT64_C(0x9e3779b97f4a7c15)) >>
                      (64u - b->backing_slot_bits));
}

static uint64_t backing_key(const struct hv_agx_gpuva_v5 *b, uint32_t ref,
                            bool by_ipa)
{
    return by_ipa ? b->backings[ref - 1u].ipa : b->backings[ref - 1u].pa;
}

static struct hv_agx_gpuva_v5_backing *backing_find(
    const struct hv_agx_gpuva_v5 *b, uint64_t key, bool by_ipa)
{
    const uint32_t *index = by_ipa ? b->backing_by_ipa : b->backing_by_pa;
    uint32_t slot = backing_hash(b, key), probes;
    for (probes = 0; probes <= BACKING_SLOT_MASK(b); ++probes) {
        uint32_t ref = index[slot];
        if (!ref) return NULL;
        if (backing_key(b, ref, by_ipa) == key)
            return (struct hv_agx_gpuva_v5_backing *)&b->backings[ref - 1u];
        slot = (slot + 1u) & BACKING_SLOT_MASK(b);
    }
    return NULL;
}

static void backing_index_insert(struct hv_agx_gpuva_v5 *b, uint32_t ref,
                                 bool by_ipa)
{
    uint32_t *index = by_ipa ? b->backing_by_ipa : b->backing_by_pa;
    uint32_t slot = backing_hash(b, backing_key(b, ref, by_ipa));
    while (index[slot]) slot = (slot + 1u) & BACKING_SLOT_MASK(b);
    index[slot] = ref;
}

static void backing_index_erase(struct hv_agx_gpuva_v5 *b, uint32_t ref,
                                bool by_ipa)
{
    uint32_t *index = by_ipa ? b->backing_by_ipa : b->backing_by_pa;
    uint32_t slot = backing_hash(b, backing_key(b, ref, by_ipa)), next;
    while (index[slot] != ref) slot = (slot + 1u) & BACKING_SLOT_MASK(b);
    for (;;) {
        index[slot] = 0;
        next = slot;
        for (;;) {
            uint32_t moved, home;
            next = (next + 1u) & BACKING_SLOT_MASK(b);
            moved = index[next];
            if (!moved) return;
            home = backing_hash(b, backing_key(b, moved, by_ipa));
            /* An entry may fill the hole unless its home lies in (slot, next]. */
            if (((next - home) & BACKING_SLOT_MASK(b)) >=
                ((next - slot) & BACKING_SLOT_MASK(b))) break;
        }
        index[slot] = index[next];
        slot = next;
    }
}

static bool backing_owner(const struct hv_agx_gpuva_v5_backing *r,
                          unsigned owner)
{
    return (r->owners[owner / 64u] >> (owner % 64u)) & 1u;
}

static void backing_set_owner(struct hv_agx_gpuva_v5_backing *r,
                              unsigned owner, bool owned)
{
    uint64_t bit = UINT64_C(1) << (owner % 64u);
    if (owned) r->owners[owner / 64u] |= bit;
    else r->owners[owner / 64u] &= ~bit;
}

static bool backing_unowned(const struct hv_agx_gpuva_v5_backing *r)
{
    unsigned i;
    for (i = 0; i < HV_AGX_GPUVA_V5_PROCESSES / 64u; ++i)
        if (r->owners[i]) return false;
    return true;
}

static void backing_drop(struct hv_agx_gpuva_v5 *b,
                         struct hv_agx_gpuva_v5_backing *r)
{
    uint32_t ref = (uint32_t)(r - b->backings) + 1u;
    backing_index_erase(b, ref, false);
    backing_index_erase(b, ref, true);
    if (private_local(b, r->ipa)) --b->private_backings;
    else --b->system_backings;
    memset(r, 0, sizeof(*r));
    b->backing_free[b->backing_free_count++] = ref;
}

/* Count a published leaf descriptor that maps a local-reserve or a
 * registered system/private page. */
static void backing_leaf_ref(struct hv_agx_gpuva_v5 *b, uint64_t descriptor,
                             bool add)
{
    struct hv_agx_gpuva_v5_backing *r;
    unsigned page;
    if (!descriptor || local_index(b, descriptor & GPUVA_PA_MASK, &page)) {
        local_leaf_ref(b, descriptor, add);
        return;
    }
    r = backing_find(b, descriptor & GPUVA_PA_MASK, false);
    if (!r || r->leaf_refs == UINT32_MAX) return;
    if (add) ++r->leaf_refs;
    else if (r->leaf_refs) --r->leaf_refs;
}

static enum hv_agx_gpuva_v5_result check(struct hv_agx_gpuva_v5 *b)
{
    uint64_t low, high;
    unsigned slot;
    if (!b || !b->active || b->tainted) return HV_AGX_GPUVA_V5_TAINTED;
    if (b->slots[63].occupied &&
        b->ops.legacy_slot63_active(b->ops.context)) {
        b->tainted = true;
        return HV_AGX_GPUVA_V5_TAINTED;
    }
    if (!b->ops.prefix_unchanged(b->ops.context) ||
        !b->ops.read_slot(b->ops.context, 0, &low, &high) ||
        low != b->context0_ttbr0 || high != b->context0_ttbr1) {
        b->tainted = true;
        return HV_AGX_GPUVA_V5_TAINTED;
    }
    for (slot = 1; slot < HV_AGX_GPUVA_V5_SLOTS; ++slot)
        if (b->slots[slot].occupied) {
            unsigned owner = b->slots[slot].owner;
            uint64_t expected = b->processes[owner].root_pa | UINT64_C(1) |
                                ((uint64_t)slot << 48);
            if (!b->processes[owner].live ||
                !b->ops.read_slot(b->ops.context, slot, &low, &high) ||
                low != expected || high) {
                b->tainted = true;
                return HV_AGX_GPUVA_V5_TAINTED;
            }
        }
    return HV_AGX_GPUVA_V5_OK;
}

enum hv_agx_gpuva_v5_result hv_agx_gpuva_v5_verify(struct hv_agx_gpuva_v5 *b)
{
    return check(b);
}

enum hv_agx_gpuva_v5_result hv_agx_gpuva_v5_validate_envelope(
    const struct hv_agx_gpuva_v5 *b, uint64_t epoch, unsigned command,
    unsigned flags)
{
    if (!b || !b->active || b->tainted) return HV_AGX_GPUVA_V5_TAINTED;
    if (epoch != b->epoch) return HV_AGX_GPUVA_V5_STALE;
    if (command < AGX_GPUVA_V5_CREATE ||
        command > AGX_GPUVA_V5_NEGOTIATE_LIVE ||
        command == AGX_GPUVA_V5_REGISTER_SYSMEM_BACKING)
        return HV_AGX_GPUVA_V5_INVALID;
    if (command == AGX_GPUVA_V5_CREATE ? flags > 1u :
        command == AGX_GPUVA_V5_UPDATE_LEAF ?
            (!flags || flags > 15u) : flags != 0u)
        return HV_AGX_GPUVA_V5_INVALID;
    return HV_AGX_GPUVA_V5_OK;
}

static int process_index(struct hv_agx_gpuva_v5 *b, uint64_t id, uint64_t generation)
{
    unsigned i;
    for (i = 0; i < HV_AGX_GPUVA_V5_PROCESSES; ++i)
        if (b->processes[i].live && b->processes[i].identity == id &&
            b->processes[i].generation == generation) return (int)i;
    return -1;
}

/* Negotiation is tied to the same process generation as its owned graph.
 * No protocol version change: old clients never enable this extension. */
enum hv_agx_gpuva_v5_result hv_agx_gpuva_v5_extension(
    struct hv_agx_gpuva_v5 *b, const AGX_GPUVA_V5_REQUEST *q,
    AGX_GPUVA_V5_RESPONSE *r)
{
    int owner;
    unsigned slot;
    enum hv_agx_gpuva_v5_result result = check(b);
    if (result) return result;
    if (!q || !r) return HV_AGX_GPUVA_V5_INVALID;
    result = hv_agx_gpuva_v5_validate_envelope(b, q->Epoch, q->Command, q->Flags);
    if (result) return result;
    if (q->TableIpa || q->AuxIpa || q->Slot || q->Index || q->ValidMask ||
        q->WritableMask || q->LogicalIpa[0] || q->LogicalIpa[1] ||
        q->LogicalIpa[2] || q->LogicalIpa[3]) return HV_AGX_GPUVA_V5_INVALID;
    if (q->Command == AGX_GPUVA_V5_QUERY_CAPS) {
        if (q->ProcessId || q->ProcessGeneration || q->Token ||
            q->AllocationGeneration) return HV_AGX_GPUVA_V5_INVALID;
        r->Reserved[0] = AGX_GPUVA_V5_CAP_LIVE_ADDITIVE;
        r->Reserved[1] = AGX_GPUVA_V5_LIVE_ABI;
        return HV_AGX_GPUVA_V5_OK;
    }
    if (q->Command != AGX_GPUVA_V5_NEGOTIATE_LIVE ||
        q->AllocationGeneration != AGX_GPUVA_V5_LIVE_ABI ||
        (q->Token != 0 && q->Token != AGX_GPUVA_V5_CAP_LIVE_ADDITIVE))
        return HV_AGX_GPUVA_V5_INVALID;
    owner = process_index(b, q->ProcessId, q->ProcessGeneration);
    if (owner < 0) return HV_AGX_GPUVA_V5_STALE;
    for (slot = 1; slot < HV_AGX_GPUVA_V5_SLOTS; ++slot)
        if (b->slots[slot].occupied && b->slots[slot].owner == (unsigned)owner &&
            b->slots[slot].jobs) return HV_AGX_GPUVA_V5_BUSY;
    b->processes[owner].live_additive = q->Token != 0;
    r->Reserved[0] = q->Token;
    r->Reserved[1] = AGX_GPUVA_V5_LIVE_ABI;
    return HV_AGX_GPUVA_V5_OK;
}

static int table_index(struct hv_agx_gpuva_v5 *b, unsigned owner, uint64_t ipa,
                       unsigned level)
{
    unsigned i;
    for (i = 0; i < HV_AGX_GPUVA_V5_TABLES; ++i)
        if (b->tables[i].live && b->tables[i].owner == owner &&
            b->tables[i].ipa == ipa && b->tables[i].level == level) return (int)i;
    return -1;
}

static bool referenced(struct hv_agx_gpuva_v5 *b, unsigned owner,
                       uint64_t pa, unsigned level)
{
    unsigned table, index;
    for (table = 0; table < HV_AGX_GPUVA_V5_TABLES; ++table)
        if (b->tables[table].live && b->tables[table].owner == owner &&
            b->tables[table].level == level) {
            unsigned count = level == 0 ? 8u : 2048u;
            for (index = 0; index < count; ++index)
                if ((b->tables[table].entries[index] & GPUVA_PA_MASK) == pa &&
                    b->tables[table].entries[index]) return true;
        }
    return false;
}

static bool table_nonzero(const struct hv_agx_gpuva_v5_table *table)
{
    unsigned i, count = table->level == 0 ? 8u : 2048u;
    for (i = 0; i < count; ++i) if (table->entries[i]) return true;
    return false;
}

static enum hv_agx_gpuva_v5_result table_add(struct hv_agx_gpuva_v5 *b,
                                              unsigned owner, uint64_t ipa,
                                              unsigned level)
{
    unsigned i, local_page, grant_owner;
    uint64_t pa, *entries;
    if (!ipa || (ipa & (HV_AGX_GPUVA_V5_PAGE - 1)) || level > 2)
        return HV_AGX_GPUVA_V5_INVALID;
    pa = b->ops.translate_page(b->ops.context, ipa);
    if (!pa || (pa & (HV_AGX_GPUVA_V5_PAGE - 1)) || pa >= (UINT64_C(1) << 40) ||
        pa == b->mailbox_pa)
        return HV_AGX_GPUVA_V5_OWNERSHIP;
    entries = b->ops.map_page(b->ops.context, ipa, pa);
    if (!entries) return HV_AGX_GPUVA_V5_OWNERSHIP;
    /* Registration must not import an unvalidated page-table graph.  Every
     * descriptor is published later through the owned update operations. */
    for (i = 0; i < HV_AGX_GPUVA_V5_PAGE / sizeof(*entries); ++i)
        if (entries[i]) return HV_AGX_GPUVA_V5_OWNERSHIP;
    for (i = 0; i < HV_AGX_GPUVA_V5_TABLES; ++i)
        if (b->tables[i].live &&
            (b->tables[i].pa == pa || b->tables[i].ipa == ipa))
            return HV_AGX_GPUVA_V5_OWNERSHIP;
    if (backing_find(b, pa, false)) return HV_AGX_GPUVA_V5_OWNERSHIP;
    if (local_index(b, ipa, &local_page))
        for (grant_owner = 0; grant_owner < HV_AGX_GPUVA_V5_PROCESSES;
             ++grant_owner)
            if (local_granted(b, grant_owner, local_page))
                return HV_AGX_GPUVA_V5_OWNERSHIP;
    for (i = 0; i < HV_AGX_GPUVA_V5_TABLES; ++i)
        if (!b->tables[i].live) {
            b->tables[i] = (struct hv_agx_gpuva_v5_table){ipa, pa, entries, owner,
                                                            level, true};
            return HV_AGX_GPUVA_V5_OK;
        }
    return HV_AGX_GPUVA_V5_CAPACITY;
}

static enum hv_agx_gpuva_v5_result publish_entry(struct hv_agx_gpuva_v5 *b,
    unsigned owner, uint64_t *entry, uint64_t descriptor)
{
    uint64_t old = *entry;
    unsigned slot;
    bool live = false;
    for (slot = 1; slot < HV_AGX_GPUVA_V5_SLOTS; ++slot)
        if (b->slots[slot].occupied && b->slots[slot].owner == owner &&
            b->slots[slot].jobs) live = true;
    /* The native descriptor, not a logical 4-KiB sub-entry, proves freshness.
     * Callers already checked the owned child/full leaf and backing grant. */
    if (live && (!b->processes[owner].live_additive || old || !descriptor))
        return HV_AGX_GPUVA_V5_BUSY;
    if (b->processes[owner].map_generation == UINT64_MAX)
        return HV_AGX_GPUVA_V5_CAPACITY;
    *entry = descriptor;
    if (!b->ops.sync_tables(b->ops.context)) goto rollback;
    for (slot = 1; slot < HV_AGX_GPUVA_V5_SLOTS; ++slot)
        if (b->slots[slot].occupied && b->slots[slot].owner == owner &&
            !b->ops.invalidate(b->ops.context, slot)) goto rollback;
    if (check(b) != HV_AGX_GPUVA_V5_OK) return HV_AGX_GPUVA_V5_TAINTED;
    ++b->processes[owner].map_generation;
    return HV_AGX_GPUVA_V5_OK;
rollback:
    /* A live GPU may have observed the temporary entry. Even a successful
     * rollback cannot authorize reuse or paging-fence success. */
    if (live) b->tainted = true;
    *entry = old;
    if (!b->ops.sync_tables(b->ops.context)) b->tainted = true;
    for (slot = 1; slot < HV_AGX_GPUVA_V5_SLOTS; ++slot)
        if (b->slots[slot].occupied && b->slots[slot].owner == owner &&
            !b->ops.invalidate(b->ops.context, slot)) b->tainted = true;
    return b->tainted ? HV_AGX_GPUVA_V5_TAINTED : HV_AGX_GPUVA_V5_TLB;
}

/* M2-f6: the storage layout for n system pages: records, the PA index, the
 * IPA index (2^bits slots each, at least twice the records) and the free
 * list. */
static unsigned backing_slot_bits_for(unsigned records)
{
    unsigned bits = HV_AGX_GPUVA_V5_BACKING_SLOT_BITS;
    while (bits < 31u && (UINT64_C(1) << bits) < 2ull * records) ++bits;
    return bits;
}

size_t hv_agx_gpuva_v5_storage_bytes(unsigned system_backings)
{
    uint64_t records, slots;
    if (!system_backings) return 0;
    records = (uint64_t)system_backings + HV_AGX_GPUVA_V5_PRIVATE_BACKINGS;
    if (records > UINT32_MAX / 4u) return 0;
    slots = UINT64_C(1) << backing_slot_bits_for((unsigned)records);
    return (size_t)(records * sizeof(struct hv_agx_gpuva_v5_backing) +
                    2u * slots * sizeof(uint32_t) + records * sizeof(uint32_t));
}

static void backing_table_attach(struct hv_agx_gpuva_v5 *b, void *storage,
                                 unsigned system_backings)
{
    if (storage && system_backings > HV_AGX_GPUVA_V5_BACKINGS &&
        hv_agx_gpuva_v5_storage_bytes(system_backings)) {
        unsigned records = system_backings + HV_AGX_GPUVA_V5_PRIVATE_BACKINGS;
        unsigned bits = backing_slot_bits_for(records);
        uint8_t *p = storage;
        memset(storage, 0, hv_agx_gpuva_v5_storage_bytes(system_backings));
        b->backings = (struct hv_agx_gpuva_v5_backing *)p;
        p += (size_t)records * sizeof(struct hv_agx_gpuva_v5_backing);
        b->backing_by_pa = (uint32_t *)p;
        p += (size_t)(1u << bits) * sizeof(uint32_t);
        b->backing_by_ipa = (uint32_t *)p;
        p += (size_t)(1u << bits) * sizeof(uint32_t);
        b->backing_free = (uint32_t *)p;
        b->backing_records = records;
        b->backing_slot_bits = bits;
        return;
    }
    b->backings = b->builtin_backings;
    b->backing_by_pa = b->builtin_by_pa;
    b->backing_by_ipa = b->builtin_by_ipa;
    b->backing_free = b->builtin_free;
    b->backing_records = HV_AGX_GPUVA_V5_BACKING_RECORDS;
    b->backing_slot_bits = HV_AGX_GPUVA_V5_BACKING_SLOT_BITS;
}

unsigned hv_agx_gpuva_v5_system_capacity(const struct hv_agx_gpuva_v5 *b)
{
    return b && b->backing_records > HV_AGX_GPUVA_V5_PRIVATE_BACKINGS
               ? b->backing_records - HV_AGX_GPUVA_V5_PRIVATE_BACKINGS : 0u;
}

enum hv_agx_gpuva_v5_result hv_agx_gpuva_v5_init(
    struct hv_agx_gpuva_v5 *b, uint64_t epoch, const struct hv_agx_gpuva_v5_ops *ops)
{
    uint64_t low, high;
    if (!b || b->active || !epoch || !ops || !ops->translate_page || !ops->map_page ||
        !ops->read_slot || !ops->write_slot || !ops->sync_tables ||
        !ops->invalidate || !ops->prefix_unchanged ||
        !ops->legacy_slot63_active ||
        !ops->prefix_unchanged(ops->context) ||
        !ops->read_slot(ops->context, 0, &low, &high) || !high)
        return HV_AGX_GPUVA_V5_INVALID;
    memset(b, 0, sizeof(*b));
    b->ops = *ops;
    b->epoch = epoch;
    b->context0_ttbr0 = low;
    b->context0_ttbr1 = high;
    backing_table_attach(b, ops->storage, ops->storage_system_backings);
    b->system_backing_limit = hv_agx_gpuva_v5_system_capacity(b);
    b->active = true;
    return HV_AGX_GPUVA_V5_OK;
}

enum hv_agx_gpuva_v5_result hv_agx_gpuva_v5_configure_local(
    struct hv_agx_gpuva_v5 *b, uint64_t base, uint64_t bytes,
    uint64_t private_bytes)
{
    unsigned index;
    if (!b || !b->active || b->local_bytes || !base || !bytes ||
        (base & (HV_AGX_GPUVA_V5_PAGE - 1u)) ||
        (bytes & (HV_AGX_GPUVA_V5_PAGE - 1u)) ||
        bytes / HV_AGX_GPUVA_V5_PAGE > HV_AGX_GPUVA_V5_LOCAL_PAGES ||
        base > UINT64_MAX - bytes ||
        private_bytes != UINT64_C(0x4000000) ||
        bytes > UINT64_MAX - private_bytes ||
        base > UINT64_MAX - bytes - private_bytes)
        return HV_AGX_GPUVA_V5_INVALID;
    for (index = 0; index < HV_AGX_GPUVA_V5_PROCESSES; ++index)
        if (b->processes[index].live) return HV_AGX_GPUVA_V5_BUSY;
    b->local_base = base;
    b->local_bytes = bytes;
    b->private_bytes = private_bytes;
    return HV_AGX_GPUVA_V5_OK;
}

enum hv_agx_gpuva_v5_result hv_agx_gpuva_v5_create(
    struct hv_agx_gpuva_v5 *b, uint64_t id, uint64_t generation,
    uint64_t root_ipa, bool paging)
{
    unsigned i;
    enum hv_agx_gpuva_v5_result result = check(b);
    if (result) return result;
    if (!id || !generation) return HV_AGX_GPUVA_V5_INVALID;
    for (i = 0; i < HV_AGX_GPUVA_V5_PROCESSES; ++i)
        if (b->processes[i].live && b->processes[i].identity == id)
            return HV_AGX_GPUVA_V5_STALE;
    for (i = 0; i < HV_AGX_GPUVA_V5_PROCESSES; ++i)
        if (!b->processes[i].live) {
            result = table_add(b, i, root_ipa, 0);
            if (result) return result;
            b->processes[i] = (struct hv_agx_gpuva_v5_process){
                id, generation, b->tables[table_index(b, i, root_ipa, 0)].pa,
                1, 0, true, paging, false};
            return HV_AGX_GPUVA_V5_OK;
        }
    return HV_AGX_GPUVA_V5_CAPACITY;
}

enum hv_agx_gpuva_v5_result hv_agx_gpuva_v5_register_table(
    struct hv_agx_gpuva_v5 *b, uint64_t id, uint64_t generation,
    uint64_t table_ipa, unsigned level)
{
    int owner;
    enum hv_agx_gpuva_v5_result result = check(b);
    if (result) return result;
    owner = process_index(b, id, generation);
    if (owner < 0) return HV_AGX_GPUVA_V5_STALE;
    return table_add(b, (unsigned)owner, table_ipa, level);
}

static enum hv_agx_gpuva_v5_result register_backing(
    struct hv_agx_gpuva_v5 *b, uint64_t id, uint64_t generation,
    uint64_t allocation_generation, uint64_t page_ipa, bool shared)
{
    int owner;
    uint64_t pa;
    unsigned i, local_page;
    uint32_t ref;
    bool private_page;
    struct hv_agx_gpuva_v5_backing *by_pa, *by_ipa, *record;
    enum hv_agx_gpuva_v5_result result = check(b);
    if (result) return result;
    owner = process_index(b, id, generation);
    if (owner < 0) return HV_AGX_GPUVA_V5_STALE;
    if (!allocation_generation || !page_ipa ||
        (page_ipa & (HV_AGX_GPUVA_V5_PAGE - 1))) return HV_AGX_GPUVA_V5_INVALID;
    pa = b->ops.translate_page(b->ops.context, page_ipa);
    if (!pa || (pa & (HV_AGX_GPUVA_V5_PAGE - 1)) || pa >= (UINT64_C(1) << 40) ||
        pa == b->mailbox_pa)
        return HV_AGX_GPUVA_V5_OWNERSHIP;
    for (i = 0; i < HV_AGX_GPUVA_V5_TABLES; ++i)
        if (b->tables[i].live && b->tables[i].pa == pa)
            return HV_AGX_GPUVA_V5_OWNERSHIP;
    if (local_index(b, page_ipa, &local_page)) {
        if (!shared || allocation_generation != b->local_base || pa != page_ipa ||
            local_granted(b, (unsigned)owner, local_page))
            return HV_AGX_GPUVA_V5_OWNERSHIP;
        local_set_grant(b, (unsigned)owner, local_page, true);
        return HV_AGX_GPUVA_V5_OK;
    }
    private_page = private_local(b, page_ipa);
    if (private_page && (shared || pa != page_ipa))
        return HV_AGX_GPUVA_V5_OWNERSHIP;
    by_pa = backing_find(b, pa, false);
    by_ipa = backing_find(b, page_ipa, true);
    if (by_pa || by_ipa) {
        /* Only another owner's exact shared identity may join a record. */
        if (!shared || by_pa != by_ipa || !by_pa->shared ||
            by_pa->generation != allocation_generation ||
            backing_owner(by_pa, (unsigned)owner))
            return HV_AGX_GPUVA_V5_OWNERSHIP;
        backing_set_owner(by_pa, (unsigned)owner, true);
        ++b->owner_backings[owner];
        return HV_AGX_GPUVA_V5_OK;
    }
    if (private_page ? b->private_backings >= HV_AGX_GPUVA_V5_PRIVATE_BACKINGS
                     : b->system_backings >= b->system_backing_limit)
        return HV_AGX_GPUVA_V5_CAPACITY;
    if (b->backing_free_count) ref = b->backing_free[--b->backing_free_count];
    else if (b->backing_high < b->backing_records)
        ref = ++b->backing_high;
    else return HV_AGX_GPUVA_V5_CAPACITY;
    record = &b->backings[ref - 1u];
    *record = (struct hv_agx_gpuva_v5_backing){
        .ipa = page_ipa, .pa = pa, .generation = allocation_generation,
        .live = true, .shared = shared};
    backing_set_owner(record, (unsigned)owner, true);
    backing_index_insert(b, ref, false);
    backing_index_insert(b, ref, true);
    if (private_page) ++b->private_backings;
    else ++b->system_backings;
    ++b->owner_backings[owner];
    return HV_AGX_GPUVA_V5_OK;
}

enum hv_agx_gpuva_v5_result hv_agx_gpuva_v5_register_backing(
    struct hv_agx_gpuva_v5 *b, uint64_t id, uint64_t generation,
    uint64_t allocation_generation, uint64_t page_ipa)
{
    return register_backing(b, id, generation, allocation_generation,
                            page_ipa, false);
}

enum hv_agx_gpuva_v5_result hv_agx_gpuva_v5_register_shared_backing(
    struct hv_agx_gpuva_v5 *b, uint64_t id, uint64_t generation,
    uint64_t allocation_generation, uint64_t page_ipa)
{
    return register_backing(b, id, generation, allocation_generation,
                            page_ipa, true);
}

enum hv_agx_gpuva_v5_result hv_agx_gpuva_v5_revoke_backing(
    struct hv_agx_gpuva_v5 *b, uint64_t id, uint64_t generation,
    uint64_t allocation_generation, uint64_t page_ipa)
{
    int owner;
    unsigned local_page;
    struct hv_agx_gpuva_v5_backing *record;
    enum hv_agx_gpuva_v5_result result = check(b);
    if (result) return result;
    owner = process_index(b, id, generation);
    if (owner < 0) return HV_AGX_GPUVA_V5_STALE;
    if (local_index(b, page_ipa, &local_page)) {
        if (allocation_generation != b->local_base ||
            !local_granted(b, (unsigned)owner, local_page))
            return HV_AGX_GPUVA_V5_STALE;
        /* No leaf of any owner maps the page: skip the owner's table scan. */
        if (b->local_leaf_refs[local_page] &&
            referenced(b, (unsigned)owner, page_ipa, 2))
            return HV_AGX_GPUVA_V5_BUSY;
        local_set_grant(b, (unsigned)owner, local_page, false);
        return HV_AGX_GPUVA_V5_OK;
    }
    record = backing_find(b, page_ipa, true);
    if (!record || !backing_owner(record, (unsigned)owner) ||
        record->generation != allocation_generation)
        return HV_AGX_GPUVA_V5_STALE;
    /* No leaf of any owner maps the page: skip the owner's table scan. */
    if (record->leaf_refs && referenced(b, (unsigned)owner, record->pa, 2))
        return HV_AGX_GPUVA_V5_BUSY;
    backing_set_owner(record, (unsigned)owner, false);
    --b->owner_backings[owner];
    if (backing_unowned(record)) backing_drop(b, record);
    return HV_AGX_GPUVA_V5_OK;
}

enum hv_agx_gpuva_v5_result hv_agx_gpuva_v5_revoke_table(
    struct hv_agx_gpuva_v5 *b, uint64_t id, uint64_t generation,
    uint64_t table_ipa, unsigned level)
{
    int owner, table;
    enum hv_agx_gpuva_v5_result result = check(b);
    if (result) return result;
    owner = process_index(b, id, generation);
    if (owner < 0) return HV_AGX_GPUVA_V5_STALE;
    table = table_index(b, (unsigned)owner, table_ipa, level);
    if (table < 0) return HV_AGX_GPUVA_V5_STALE;
    if (b->tables[table].pa == b->processes[owner].root_pa ||
        table_nonzero(&b->tables[table]) ||
        (level && referenced(b, (unsigned)owner, b->tables[table].pa, level-1)))
        return HV_AGX_GPUVA_V5_BUSY;
    memset(&b->tables[table], 0, sizeof(b->tables[table]));
    return HV_AGX_GPUVA_V5_OK;
}

enum hv_agx_gpuva_v5_result hv_agx_gpuva_v5_update_parent(
    struct hv_agx_gpuva_v5 *b, uint64_t id, uint64_t generation,
    uint64_t table_ipa, unsigned index, uint64_t child_ipa)
{
    int owner, parent, child;
    unsigned level;
    unsigned long long encoded = 0;
    uint64_t descriptor = 0;
    enum hv_agx_gpuva_v5_result result = check(b);
    if (result) return result;
    owner = process_index(b, id, generation);
    if (owner < 0) return HV_AGX_GPUVA_V5_STALE;
    for (level = 0; level < 2; ++level)
        if ((parent = table_index(b, (unsigned)owner, table_ipa, level)) >= 0)
            break;
    if (level == 2 || index >= (level == 0 ? 8u : 2048u))
        return HV_AGX_GPUVA_V5_INVALID;
    if (child_ipa) {
        child = table_index(b, (unsigned)owner, child_ipa, level + 1);
        if (child < 0) return HV_AGX_GPUVA_V5_OWNERSHIP;
        if (AppleAgxUatEncodeTableDescriptor(b->tables[child].pa,
                &encoded) != AppleAgxUatResultOk) return HV_AGX_GPUVA_V5_INVALID;
        descriptor = encoded;
    }
    return publish_entry(b, (unsigned)owner, &b->tables[parent].entries[index],
                         descriptor);
}

enum hv_agx_gpuva_v5_result hv_agx_gpuva_v5_update_leaf(
    struct hv_agx_gpuva_v5 *b, uint64_t id, uint64_t generation,
    uint64_t table_ipa, unsigned index, const uint64_t logical_ipa[4],
    uint64_t allocation_generation, unsigned update_mask, unsigned valid_mask,
    unsigned writable_mask)
{
    int owner, table;
    uint64_t pa = 0, descriptor = 0, before, ipa[4] = {0};
    unsigned long long encoded = 0, ro = 0, rw = 0;
    unsigned i, local_page, final_valid = 0, final_write = 0;
    struct hv_agx_gpuva_v5_backing *record;
    enum hv_agx_gpuva_v5_result result = check(b);
    if (result) return result;
    owner = process_index(b, id, generation);
    if (owner < 0) return HV_AGX_GPUVA_V5_STALE;
    table = table_index(b, (unsigned)owner, table_ipa, 2);
    if (table < 0 || index >= 2048u || !update_mask || update_mask > 15u ||
        (valid_mask && !logical_ipa) ||
        (valid_mask & ~update_mask) || (writable_mask & ~valid_mask))
        return HV_AGX_GPUVA_V5_INVALID;
    before = b->tables[table].entries[index];
    if (before) {
        pa = before & GPUVA_PA_MASK;
        if (AppleAgxUatEncodePageDescriptor(1u, pa,
                AppleAgxUatGpuSharedReadOnly, &ro) != AppleAgxUatResultOk ||
            AppleAgxUatEncodePageDescriptor(1u, pa,
                AppleAgxUatGpuSharedReadWrite, &rw) != AppleAgxUatResultOk ||
            (before != ro && before != rw)) return HV_AGX_GPUVA_V5_OWNERSHIP;
        /* Local-reserve pages are tracked only by the grant bitmap; the
         * backing table never holds one (register_backing returns first). */
        record = local_index(b, pa, &local_page) ? NULL :
                 backing_find(b, pa, false);
        if (record && !backing_owner(record, (unsigned)owner)) record = NULL;
        if (!record) {
            if (!local_index(b, pa, &local_page) ||
                !local_granted(b, (unsigned)owner, local_page))
                return HV_AGX_GPUVA_V5_OWNERSHIP;
            for (unsigned part = 0; part < 4; ++part)
                ipa[part] = pa + part * UINT64_C(0x1000);
        } else {
            for (unsigned part = 0; part < 4; ++part)
                ipa[part] = record->ipa + part * UINT64_C(0x1000);
        }
        final_valid = 15u;
        final_write = before == rw ? 15u : 0u;
    }
    for (i = 0; i < 4; ++i) if (update_mask & (1u << i)) {
        ipa[i] = (valid_mask & (1u << i)) ? logical_ipa[i] : 0;
        final_valid = (final_valid & ~(1u << i)) | (valid_mask & (1u << i));
        final_write = (final_write & ~(1u << i)) | (writable_mask & (1u << i));
    }
    if (final_valid != 0u && final_valid != 15u) return HV_AGX_GPUVA_V5_INVALID;
    if (final_valid == 15u) {
        if ((ipa[0] & (HV_AGX_GPUVA_V5_PAGE - 1)) ||
            (final_write != 0u && final_write != 15u))
            return HV_AGX_GPUVA_V5_INVALID;
        pa = b->ops.translate_page(b->ops.context, ipa[0]);
        if (!pa || (pa & (HV_AGX_GPUVA_V5_PAGE - 1)))
            return HV_AGX_GPUVA_V5_OWNERSHIP;
        for (i = 0; i < 4; ++i)
            if (ipa[i] != ipa[0] + i * UINT64_C(0x1000))
                return HV_AGX_GPUVA_V5_OWNERSHIP;
        record = local_index(b, ipa[0], &local_page) ? NULL :
                 backing_find(b, pa, false);
        if (record && (record->generation != allocation_generation ||
                       !backing_owner(record, (unsigned)owner) ||
                       record->ipa != ipa[0])) record = NULL;
        if (!record &&
            (!local_index(b, ipa[0], &local_page) ||
             allocation_generation != b->local_base || pa != ipa[0] ||
             !local_granted(b, (unsigned)owner, local_page)))
            return HV_AGX_GPUVA_V5_OWNERSHIP;
        if (AppleAgxUatEncodePageDescriptor(1u, pa,
              final_write ? AppleAgxUatGpuSharedReadWrite :
                            AppleAgxUatGpuSharedReadOnly,
              &encoded) != AppleAgxUatResultOk) return HV_AGX_GPUVA_V5_INVALID;
        descriptor = encoded;
    }
    result = publish_entry(b, (unsigned)owner, &b->tables[table].entries[index],
                           descriptor);
    if (result == HV_AGX_GPUVA_V5_OK) {
        backing_leaf_ref(b, before, false);
        backing_leaf_ref(b, descriptor, true);
    }
    return result;
}

enum hv_agx_gpuva_v5_result hv_agx_gpuva_v5_relocate_root(
    struct hv_agx_gpuva_v5 *b, uint64_t id, uint64_t generation, uint64_t root_ipa)
{
    int owner, table;
    unsigned i;
    enum hv_agx_gpuva_v5_result result = check(b);
    if (result) return result;
    owner = process_index(b, id, generation);
    if (owner < 0) return HV_AGX_GPUVA_V5_STALE;
    table = table_index(b, (unsigned)owner, root_ipa, 0);
    if (table < 0) return HV_AGX_GPUVA_V5_OWNERSHIP;
    for (i = 1; i < HV_AGX_GPUVA_V5_SLOTS; ++i)
        if (b->slots[i].occupied && b->slots[i].owner == (unsigned)owner)
            return HV_AGX_GPUVA_V5_BUSY;
    b->processes[owner].root_pa = b->tables[table].pa;
    ++b->processes[owner].root_generation;
    return HV_AGX_GPUVA_V5_OK;
}

enum hv_agx_gpuva_v5_result hv_agx_gpuva_v5_lease(
    struct hv_agx_gpuva_v5 *b, uint64_t id, uint64_t generation,
    unsigned slot, uint64_t *token)
{
    int owner;
    uint64_t prior0, prior1, expected;
    enum hv_agx_gpuva_v5_result result = check(b);
    if (result) return result;
    owner = process_index(b, id, generation);
    if (owner < 0) return HV_AGX_GPUVA_V5_STALE;
    if (!slot || slot >= HV_AGX_GPUVA_V5_SLOTS || !token)
        return HV_AGX_GPUVA_V5_INVALID;
    if (slot == 63u && b->ops.legacy_slot63_active(b->ops.context))
        return HV_AGX_GPUVA_V5_BUSY;
    if (b->slots[slot].occupied) return HV_AGX_GPUVA_V5_BUSY;
    if (!b->ops.read_slot(b->ops.context, slot, &prior0, &prior1) ||
        prior0 || prior1) return HV_AGX_GPUVA_V5_OWNERSHIP;
    expected = b->processes[owner].root_pa | UINT64_C(1) |
               ((uint64_t)slot << 48);
    if (!b->ops.write_slot(b->ops.context, slot, expected, 0) ||
        !b->ops.invalidate(b->ops.context, slot)) goto rollback;
    if (!b->ops.read_slot(b->ops.context, slot, &prior0, &prior1) ||
        prior0 != expected || prior1 || check(b)) goto rollback;
    b->slots[slot] = (struct hv_agx_gpuva_v5_slot){++b->next_token,
                                                    (unsigned)owner, 0, true};
    *token = b->next_token;
    return HV_AGX_GPUVA_V5_OK;
rollback:
    if (!b->ops.write_slot(b->ops.context, slot, 0, 0) ||
        !b->ops.invalidate(b->ops.context, slot)) b->tainted = true;
    return b->tainted ? HV_AGX_GPUVA_V5_TAINTED : HV_AGX_GPUVA_V5_TLB;
}

enum hv_agx_gpuva_v5_result hv_agx_gpuva_v5_job_begin(
    struct hv_agx_gpuva_v5 *b, unsigned slot, uint64_t token)
{
    enum hv_agx_gpuva_v5_result result = check(b);
    if (result) return result;
    if (!slot || slot >= HV_AGX_GPUVA_V5_SLOTS || !token ||
        !b->slots[slot].occupied || b->slots[slot].token != token)
        return HV_AGX_GPUVA_V5_STALE;
    if (b->slots[slot].jobs == UINT32_MAX) return HV_AGX_GPUVA_V5_CAPACITY;
    ++b->slots[slot].jobs;
    return HV_AGX_GPUVA_V5_OK;
}

enum hv_agx_gpuva_v5_result hv_agx_gpuva_v5_job_end(
    struct hv_agx_gpuva_v5 *b, unsigned slot, uint64_t token)
{
    enum hv_agx_gpuva_v5_result result = check(b);
    if (result) return result;
    if (!slot || slot >= HV_AGX_GPUVA_V5_SLOTS || !token ||
        !b->slots[slot].occupied || b->slots[slot].token != token ||
        !b->slots[slot].jobs) return HV_AGX_GPUVA_V5_STALE;
    --b->slots[slot].jobs;
    return HV_AGX_GPUVA_V5_OK;
}

enum hv_agx_gpuva_v5_result hv_agx_gpuva_v5_release(
    struct hv_agx_gpuva_v5 *b, unsigned slot, uint64_t token)
{
    enum hv_agx_gpuva_v5_result result = check(b);
    if (result) return result;
    if (!slot || slot >= HV_AGX_GPUVA_V5_SLOTS || !token ||
        !b->slots[slot].occupied || b->slots[slot].token != token)
        return HV_AGX_GPUVA_V5_STALE;
    if (b->slots[slot].jobs) return HV_AGX_GPUVA_V5_BUSY;
    if (!b->ops.write_slot(b->ops.context, slot, 0, 0) ||
        !b->ops.invalidate(b->ops.context, slot)) {
        b->tainted = true;
        return HV_AGX_GPUVA_V5_TAINTED;
    }
    memset(&b->slots[slot], 0, sizeof(b->slots[slot]));
    return check(b);
}

enum hv_agx_gpuva_v5_result hv_agx_gpuva_v5_flush_tlb(
    struct hv_agx_gpuva_v5 *b, uint64_t id, uint64_t generation,
    uint64_t root_ipa, uint64_t start_va, uint64_t end_va)
{
    int owner, root;
    unsigned slot;
    enum hv_agx_gpuva_v5_result result = check(b);
    if (result) return result;
    owner = process_index(b, id, generation);
    if (owner < 0) return HV_AGX_GPUVA_V5_STALE;
    root = table_index(b, (unsigned)owner, root_ipa, 0);
    if (root < 0 || b->tables[root].pa != b->processes[owner].root_pa)
        return HV_AGX_GPUVA_V5_OWNERSHIP;
    if (((start_va | end_va) & UINT64_C(0xfff)) ||
        ((start_va != 0 || end_va != 0) &&
         (end_va <= start_va || end_va > (UINT64_C(1) << 39))))
        return HV_AGX_GPUVA_V5_INVALID;
    for (slot = 1; slot < HV_AGX_GPUVA_V5_SLOTS; ++slot)
        if (b->slots[slot].occupied && b->slots[slot].owner == (unsigned)owner &&
            b->slots[slot].jobs && !b->processes[owner].live_additive)
            return HV_AGX_GPUVA_V5_BUSY;
    if (!b->ops.sync_tables(b->ops.context)) {
        b->tainted = true;
        return HV_AGX_GPUVA_V5_TLB;
    }
    for (slot = 1; slot < HV_AGX_GPUVA_V5_SLOTS; ++slot)
        if (b->slots[slot].occupied && b->slots[slot].owner == (unsigned)owner &&
            !b->ops.invalidate(b->ops.context, slot)) {
            b->tainted = true;
            return HV_AGX_GPUVA_V5_TLB;
        }
    return check(b);
}

enum hv_agx_gpuva_v5_result hv_agx_gpuva_v5_destroy(
    struct hv_agx_gpuva_v5 *b, uint64_t id, uint64_t generation)
{
    int owner;
    unsigned i;
    enum hv_agx_gpuva_v5_result result = check(b);
    if (result) return result;
    owner = process_index(b, id, generation);
    if (owner < 0) return HV_AGX_GPUVA_V5_STALE;
    for (i = 1; i < HV_AGX_GPUVA_V5_SLOTS; ++i)
        if (b->slots[i].occupied && b->slots[i].owner == (unsigned)owner)
            return HV_AGX_GPUVA_V5_BUSY;
    if (b->owner_backings[owner]) return HV_AGX_GPUVA_V5_BUSY;
    for (i = 0; i < HV_AGX_GPUVA_V5_LOCAL_PAGES / 64u; ++i)
        if (b->local_grants[owner][i]) return HV_AGX_GPUVA_V5_BUSY;
    for (i = 0; i < HV_AGX_GPUVA_V5_TABLES; ++i)
        if (b->tables[i].live && b->tables[i].owner == (unsigned)owner &&
            table_nonzero(&b->tables[i])) return HV_AGX_GPUVA_V5_BUSY;
    for (i = 0; i < HV_AGX_GPUVA_V5_TABLES; ++i)
        if (b->tables[i].live && b->tables[i].owner == (unsigned)owner)
            memset(&b->tables[i], 0, sizeof(b->tables[i]));
    memset(b->local_grants[owner], 0, sizeof(b->local_grants[owner]));
    memset(&b->processes[owner], 0, sizeof(b->processes[owner]));
    return check(b);
}

enum hv_agx_gpuva_v5_result hv_agx_gpuva_v5_attach_mailbox(
    struct hv_agx_gpuva_v5 *b, uint64_t ipa, uint64_t *mailbox_pa)
{
    unsigned i, local_page;
    uint64_t pa;
    enum hv_agx_gpuva_v5_result result = check(b);
    if (result) return result;
    if (!mailbox_pa || (ipa & (HV_AGX_GPUVA_V5_PAGE - 1)))
        return HV_AGX_GPUVA_V5_INVALID;
    if (!ipa) {
        b->mailbox_pa = 0;
        *mailbox_pa = 0;
        return HV_AGX_GPUVA_V5_OK;
    }
    /* The response store targets this page, so it must be ordinary guest RAM
     * that no process can map as a table or GPU backing while attached. */
    if (local_index(b, ipa, &local_page)) return HV_AGX_GPUVA_V5_OWNERSHIP;
    pa = b->ops.translate_page(b->ops.context, ipa);
    if (!pa || (pa & (HV_AGX_GPUVA_V5_PAGE - 1)) || pa >= (UINT64_C(1) << 40))
        return HV_AGX_GPUVA_V5_OWNERSHIP;
    for (i = 0; i < HV_AGX_GPUVA_V5_TABLES; ++i)
        if (b->tables[i].live &&
            (b->tables[i].pa == pa || b->tables[i].ipa == ipa))
            return HV_AGX_GPUVA_V5_OWNERSHIP;
    if (backing_find(b, pa, false) || backing_find(b, ipa, true))
        return HV_AGX_GPUVA_V5_OWNERSHIP;
    b->mailbox_pa = pa;
    *mailbox_pa = pa;
    return HV_AGX_GPUVA_V5_OK;
}

enum hv_agx_gpuva_v5_result hv_agx_gpuva_v5_configure_system_backings(
    struct hv_agx_gpuva_v5 *b, unsigned limit)
{
    if (!b || !b->active || !limit || limit > hv_agx_gpuva_v5_system_capacity(b))
        return HV_AGX_GPUVA_V5_INVALID;
    if (limit < b->system_backings) return HV_AGX_GPUVA_V5_BUSY;
    b->system_backing_limit = limit;
    return HV_AGX_GPUVA_V5_OK;
}

unsigned hv_agx_gpuva_v5_live_backings(const struct hv_agx_gpuva_v5 *b)
{
    unsigned i, count = 0;
    for (i = 0; i < HV_AGX_GPUVA_V5_PROCESSES; ++i) count += b->owner_backings[i];
    return count;
}

const struct hv_agx_gpuva_v5_backing *hv_agx_gpuva_v5_backing_by_pa(
    const struct hv_agx_gpuva_v5 *b, uint64_t pa)
{
    return backing_find(b, pa, false);
}

const struct hv_agx_gpuva_v5_backing *hv_agx_gpuva_v5_backing_by_ipa(
    const struct hv_agx_gpuva_v5 *b, uint64_t ipa)
{
    return backing_find(b, ipa, true);
}

bool hv_agx_gpuva_v5_backing_owned(const struct hv_agx_gpuva_v5_backing *r,
                                   unsigned owner)
{
    return r && owner < HV_AGX_GPUVA_V5_PROCESSES && backing_owner(r, owner);
}

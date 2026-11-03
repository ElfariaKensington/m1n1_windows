#ifndef HV_AGX_GPUVA_V5_H
#define HV_AGX_GPUVA_V5_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#define HV_AGX_GPUVA_V5_VERSION 6u
#define HV_AGX_GPUVA_V5_PAGE UINT64_C(0x4000)
#define HV_AGX_GPUVA_V5_PROCESSES 128u
/* Page-table records shared by every process (CREATE takes one for the root).
 * EXP1323: the FL 11_0 desktop of eleven GPU processes exhausted 512 and
 * every later REGISTER_TABLE/CREATE answered CAPACITY; Asahi's UAT allocates
 * page tables on demand without a global cap. 4096 records (40 bytes each)
 * cover 4096 16-KiB tables. */
#define HV_AGX_GPUVA_V5_TABLES 4096u
/* M2-d: 16-KiB system pages a GPU leaf may map (2 GiB) and the 64-MiB
 * private local pool, one record per page found by PA or IPA in O(1). */
#define HV_AGX_GPUVA_V5_BACKINGS 131072u
#define HV_AGX_GPUVA_V5_PRIVATE_BACKINGS 4096u
#define HV_AGX_GPUVA_V5_BACKING_RECORDS \
    (HV_AGX_GPUVA_V5_BACKINGS + HV_AGX_GPUVA_V5_PRIVATE_BACKINGS)
#define HV_AGX_GPUVA_V5_BACKING_SLOT_BITS 18u
#define HV_AGX_GPUVA_V5_BACKING_SLOTS (1u << HV_AGX_GPUVA_V5_BACKING_SLOT_BITS)
#define HV_AGX_GPUVA_V5_SLOTS 64u
#define HV_AGX_GPUVA_V5_LOCAL_PAGES 65536u

enum hv_agx_gpuva_v5_result {
    HV_AGX_GPUVA_V5_OK = 0,
    HV_AGX_GPUVA_V5_INVALID,
    HV_AGX_GPUVA_V5_STALE,
    HV_AGX_GPUVA_V5_BUSY,
    HV_AGX_GPUVA_V5_OWNERSHIP,
    HV_AGX_GPUVA_V5_TLB,
    HV_AGX_GPUVA_V5_TAINTED,
    HV_AGX_GPUVA_V5_CAPACITY,
};

struct hv_agx_gpuva_v5_ops {
    void *context;
    /* Full aligned 16-KiB guest normal RAM page, already screened from
     * firmware/private regions. Zero means forbidden. */
    uint64_t (*translate_page)(void *, uint64_t ipa);
    uint64_t *(*map_page)(void *, uint64_t ipa, uint64_t pa);
    bool (*read_slot)(void *, unsigned slot, uint64_t *ttbr0, uint64_t *ttbr1);
    bool (*write_slot)(void *, unsigned slot, uint64_t ttbr0, uint64_t ttbr1);
    bool (*sync_tables)(void *);
    /* DSB store visibility + ASID TLBI + DSB/ISB acknowledgement. */
    bool (*invalidate)(void *, unsigned slot);
    bool (*prefix_unchanged)(void *);
    /* v4 uses fixed context63 until its owner is explicitly disabled. */
    bool (*legacy_slot63_active)(void *);
    /* M2-f6: optional backing table for storage_system_backings system pages
     * (hv_agx_gpuva_v5_storage_bytes bytes, 8-byte aligned, cleared by init),
     * sized by the hypervisor from guest RAM. NULL or a count no larger than
     * HV_AGX_GPUVA_V5_BACKINGS keeps the built-in table. */
    void *storage;
    unsigned storage_system_backings;
};

struct hv_agx_gpuva_v5_process {
    uint64_t identity, generation, root_pa, root_generation, map_generation;
    bool live, paging, live_additive;
};
struct hv_agx_gpuva_v5_table {
    uint64_t ipa, pa;
    uint64_t *entries;
    unsigned owner, level;
    bool live;
};
struct hv_agx_gpuva_v5_slot {
    uint64_t token;
    unsigned owner, jobs;
    bool occupied;
};
struct hv_agx_gpuva_v5_backing {
    uint64_t ipa, pa, generation;
    /* Owners that registered the page: one for an exclusive backing, every
     * sharer of the same IPA, PA and allocation generation for a shared one. */
    uint64_t owners[HV_AGX_GPUVA_V5_PROCESSES / 64u];
    /* Published leaf descriptors (all owners) that map pa; UINT32_MAX is
     * sticky (revoke then always scans the owner's tables). */
    uint32_t leaf_refs;
    bool live;
    bool shared;
};
struct hv_agx_gpuva_v5 {
    struct hv_agx_gpuva_v5_ops ops;
    uint64_t epoch, context0_ttbr0, context0_ttbr1, next_token;
    struct hv_agx_gpuva_v5_process processes[HV_AGX_GPUVA_V5_PROCESSES];
    struct hv_agx_gpuva_v5_table tables[HV_AGX_GPUVA_V5_TABLES];
    /* The active backing table: the built-in arrays below or the storage
     * given at init (M2-f6), backing_records records and 2^backing_slot_bits
     * index slots. */
    struct hv_agx_gpuva_v5_backing *backings;
    /* Record index + 1 by PA and by IPA (linear probing, 0 is empty). */
    uint32_t *backing_by_pa, *backing_by_ipa, *backing_free;
    unsigned backing_records, backing_slot_bits;
    unsigned backing_free_count, backing_high;
    struct hv_agx_gpuva_v5_backing builtin_backings[HV_AGX_GPUVA_V5_BACKING_RECORDS];
    uint32_t builtin_by_pa[HV_AGX_GPUVA_V5_BACKING_SLOTS];
    uint32_t builtin_by_ipa[HV_AGX_GPUVA_V5_BACKING_SLOTS];
    uint32_t builtin_free[HV_AGX_GPUVA_V5_BACKING_RECORDS];
    unsigned system_backings, private_backings, system_backing_limit;
    unsigned owner_backings[HV_AGX_GPUVA_V5_PROCESSES];
    uint64_t local_base, local_bytes, private_bytes;
    uint64_t local_grants[HV_AGX_GPUVA_V5_PROCESSES][HV_AGX_GPUVA_V5_LOCAL_PAGES / 64u];
    /* Leaf entries (all owners) that map each local-reserve page; 0xffff is
     * sticky (the revoke check then always scans). */
    uint16_t local_leaf_refs[HV_AGX_GPUVA_V5_LOCAL_PAGES];
    struct hv_agx_gpuva_v5_slot slots[HV_AGX_GPUVA_V5_SLOTS];
    /* Attached request/response page; never a table or a backing. */
    uint64_t mailbox_pa;
    bool active, tainted;
};

enum hv_agx_gpuva_v5_result hv_agx_gpuva_v5_init(
    struct hv_agx_gpuva_v5 *, uint64_t epoch, const struct hv_agx_gpuva_v5_ops *);
enum hv_agx_gpuva_v5_result hv_agx_gpuva_v5_configure_local(
    struct hv_agx_gpuva_v5 *, uint64_t base, uint64_t bytes,
    uint64_t private_bytes);
struct _AGX_GPUVA_V5_REQUEST;
struct _AGX_GPUVA_V5_RESPONSE;
/* Optional v6 capability query and per-owner negotiation. */
enum hv_agx_gpuva_v5_result hv_agx_gpuva_v5_extension(
    struct hv_agx_gpuva_v5 *, const struct _AGX_GPUVA_V5_REQUEST *,
    struct _AGX_GPUVA_V5_RESPONSE *);

enum hv_agx_gpuva_v5_result hv_agx_gpuva_v5_verify(struct hv_agx_gpuva_v5 *);
enum hv_agx_gpuva_v5_result hv_agx_gpuva_v5_validate_envelope(
    const struct hv_agx_gpuva_v5 *, uint64_t epoch, unsigned command,
    unsigned flags);
enum hv_agx_gpuva_v5_result hv_agx_gpuva_v5_create(
    struct hv_agx_gpuva_v5 *, uint64_t id, uint64_t generation, uint64_t root_ipa,
    bool paging);
enum hv_agx_gpuva_v5_result hv_agx_gpuva_v5_register_table(
    struct hv_agx_gpuva_v5 *, uint64_t id, uint64_t generation,
    uint64_t table_ipa, unsigned level);
enum hv_agx_gpuva_v5_result hv_agx_gpuva_v5_register_backing(
    struct hv_agx_gpuva_v5 *, uint64_t id, uint64_t generation,
    uint64_t allocation_generation, uint64_t page_ipa);
enum hv_agx_gpuva_v5_result hv_agx_gpuva_v5_register_shared_backing(
    struct hv_agx_gpuva_v5 *, uint64_t id, uint64_t generation,
    uint64_t allocation_generation, uint64_t page_ipa);
enum hv_agx_gpuva_v5_result hv_agx_gpuva_v5_revoke_backing(
    struct hv_agx_gpuva_v5 *, uint64_t id, uint64_t generation,
    uint64_t allocation_generation, uint64_t page_ipa);
enum hv_agx_gpuva_v5_result hv_agx_gpuva_v5_revoke_table(
    struct hv_agx_gpuva_v5 *, uint64_t id, uint64_t generation,
    uint64_t table_ipa, unsigned level);
enum hv_agx_gpuva_v5_result hv_agx_gpuva_v5_update_parent(
    struct hv_agx_gpuva_v5 *, uint64_t id, uint64_t generation,
    uint64_t table_ipa, unsigned index, uint64_t child_ipa);
enum hv_agx_gpuva_v5_result hv_agx_gpuva_v5_update_leaf(
    struct hv_agx_gpuva_v5 *, uint64_t id, uint64_t generation,
    uint64_t table_ipa, unsigned index, const uint64_t logical_ipa[4],
    uint64_t allocation_generation, unsigned update_mask, unsigned valid_mask,
    unsigned writable_mask);
enum hv_agx_gpuva_v5_result hv_agx_gpuva_v5_relocate_root(
    struct hv_agx_gpuva_v5 *, uint64_t id, uint64_t generation,
    uint64_t root_ipa);
enum hv_agx_gpuva_v5_result hv_agx_gpuva_v5_lease(
    struct hv_agx_gpuva_v5 *, uint64_t id, uint64_t generation,
    unsigned slot, uint64_t *token);
enum hv_agx_gpuva_v5_result hv_agx_gpuva_v5_job_begin(
    struct hv_agx_gpuva_v5 *, unsigned slot, uint64_t token);
enum hv_agx_gpuva_v5_result hv_agx_gpuva_v5_job_end(
    struct hv_agx_gpuva_v5 *, unsigned slot, uint64_t token);
enum hv_agx_gpuva_v5_result hv_agx_gpuva_v5_release(
    struct hv_agx_gpuva_v5 *, unsigned slot, uint64_t token);
enum hv_agx_gpuva_v5_result hv_agx_gpuva_v5_flush_tlb(
    struct hv_agx_gpuva_v5 *, uint64_t id, uint64_t generation,
    uint64_t root_ipa, uint64_t start_va, uint64_t end_va);
enum hv_agx_gpuva_v5_result hv_agx_gpuva_v5_destroy(
    struct hv_agx_gpuva_v5 *, uint64_t id, uint64_t generation);
enum hv_agx_gpuva_v5_result hv_agx_gpuva_v5_attach_mailbox(
    struct hv_agx_gpuva_v5 *, uint64_t mailbox_ipa, uint64_t *mailbox_pa);
/* M2-f6: bytes of storage for a table of system_backings system pages (plus
 * the private pages); 0 for 0. */
size_t hv_agx_gpuva_v5_storage_bytes(unsigned system_backings);
/* System pages the active table holds (the default limit). */
unsigned hv_agx_gpuva_v5_system_capacity(const struct hv_agx_gpuva_v5 *);
/* Set the system-page limit (1..the table's system capacity, not below the
 * pages registered now); the default is the capacity. */
enum hv_agx_gpuva_v5_result hv_agx_gpuva_v5_configure_system_backings(
    struct hv_agx_gpuva_v5 *, unsigned limit);
/* Live (owner, page) registrations, and the record for a PA or an IPA. */
unsigned hv_agx_gpuva_v5_live_backings(const struct hv_agx_gpuva_v5 *);
const struct hv_agx_gpuva_v5_backing *hv_agx_gpuva_v5_backing_by_pa(
    const struct hv_agx_gpuva_v5 *, uint64_t pa);
const struct hv_agx_gpuva_v5_backing *hv_agx_gpuva_v5_backing_by_ipa(
    const struct hv_agx_gpuva_v5 *, uint64_t ipa);
bool hv_agx_gpuva_v5_backing_owned(const struct hv_agx_gpuva_v5_backing *,
                                   unsigned owner);

#endif

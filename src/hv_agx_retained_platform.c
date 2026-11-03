#include "hv.h"
#include "hv_agx_retained_platform.h"
#include "hv_agx_local_reserve.h"
#include "hv_agx_retained_root.h"
#include "hv_agx_retained_mmio.h"
#include "hv_agx_retained_backing.h"
#include "hv_agx_gpuva_v5.h"
#include "hv_agx_gpuva_v5_mmio.h"
#include "hv_launch_j313.h"
#include "hv_autonomous_layout.generated.h"
#include "hv_agx_g2.generated.h"
#include "adt.h"
#include "firmware.h"
#include "heapblock.h"
#include "../../drivers/apple-agx/shared/include/apple_agx_hwdata_profile.h"
#include "utils.h"
#include "xnuboot.h"
#include <malloc.h>
#include <string.h>

static struct hv_agx_retained_root root_owner;
static struct hv_agx_retained_mmio wire;
static struct hv_agx_gpuva_v5 gpuva_v5;
static struct hv_agx_gpuva_v5_wire gpuva_v5_wire;
static u64 root_base, root_length, asc_base, next_epoch;
static bool request_powered;
static struct hv_contract_snapshot launch_memory;
static bool launch_memory_valid;
static bool profile_inputs_valid;
extern u64 hv_ipa_to_pa(u64 ipa);

static unsigned char allocate_page(void *context, APPLE_AGX_UAT_PAGE *page)
{
    void *memory;
    (void)context;
    memory = memalign(0x4000, 0x4000);
    if (!memory) return 0;
    memset(memory, 0, 0x4000);
    page->Entries = memory;
    page->PhysicalAddress = (u64)memory;
    return 1;
}

static void release_page(void *context, const APPLE_AGX_UAT_PAGE *page)
{
    (void)context;
    free(page->Entries);
}

static unsigned long long translate_guest(void *context, unsigned long long ipa)
{
    u64 pa, offset, base = cur_boot_args.phys_base, length = cur_boot_args.mem_size;
    (void)context;
    if (ipa & 0x3fff) return 0;
    pa = hv_ipa_to_pa(ipa);
    /* Existing HVC guest-normal-RAM contract, with complete leaf validation.
     * SW/MMIO/private carveouts cannot resolve via hv_ipa_to_pa. */
    if (!pa || (pa & 0x3fff) || pa < base || length < 0x4000 ||
        pa - base > length - 0x4000 || pa >= (1ULL << 40) ||
        (pa < root_base + root_length && pa + 0x4000 > root_base))
        return 0;
    if (!launch_memory_valid || !hv_agx_retained_backing_allowed(&launch_memory,pa,0x4000))
        return 0;
    for (offset = 0; offset < 0x4000; offset += 0x1000)
        if (hv_ipa_to_pa(ipa + offset) != pa + offset ||
            hv_ipa_to_pa(ipa + offset + 0xfff) != pa + offset + 0xfff)
            return 0;
    return pa;
}

static void sync_tables(void *context)
{
    (void)context;
    /* Native UAT.flush_dirty uses context0 ASIDE1OS; order table stores before
     * invalidation and completion before mailbox grant or releasing storage. */
    /* ASIDE1OS encoding: SYS op1=0 CRn=8 CRm=1 op2=2 (Arm DDI0601).
     * Spell SYS because the pinned global -march=armv8.2-a assembler does not
     * name this v8.4 instruction; native EXP470 executed it on this J313. */
    __asm__ volatile("dsb oshst\n\tsys #0, c8, c1, #2, %0\n\tdsb osh\n\tisb"
                     :: "r"(0ULL) : "memory");
}

static uint64_t *gpuva_map_page(void *context, uint64_t ipa, uint64_t pa)
{
    (void)context; (void)ipa;
    return (uint64_t *)pa;
}
static uint64_t gpuva_translate(void *context, uint64_t ipa)
{
    return translate_guest(context, ipa);
}
static bool gpuva_read_slot(void *context, unsigned slot, uint64_t *low,
                            uint64_t *high)
{
    (void)context;
    if (slot >= 64 || !low || !high) return false;
    *low = read64(HV_AGX_G2_GPU_BASE + slot * 16u);
    *high = read64(HV_AGX_G2_GPU_BASE + slot * 16u + 8u);
    return true;
}
static bool gpuva_write_slot(void *context, unsigned slot, uint64_t low,
                             uint64_t high)
{
    (void)context;
    if (!slot || slot >= 64 || high) return false;
    write64(HV_AGX_G2_GPU_BASE + slot * 16u, low);
    write64(HV_AGX_G2_GPU_BASE + slot * 16u + 8u, 0);
    return true;
}
static bool gpuva_sync(void *context)
{
    (void)context;
    __asm__ volatile("dsb oshst" ::: "memory");
    return true;
}
static bool gpuva_invalidate(void *context, unsigned slot)
{
    uint64_t asid = (uint64_t)slot << 48;
    (void)context;
    if (!slot || slot >= 64) return false;
    __asm__ volatile("dsb oshst\n\tsys #0, c8, c1, #2, %0\n\tdsb osh\n\tisb"
                     :: "r"(asid) : "memory");
    return true;
}
static bool gpuva_prefix(void *context)
{
    (void)context;
    return root_owner.Active && hv_agx_retained_prefix_unchanged(&root_owner);
}
static bool gpuva_legacy_slot63(void *context)
{
    (void)context;
    return root_owner.Active;
}
static const struct hv_agx_gpuva_v5_ops gpuva_ops = {
    NULL, gpuva_translate, gpuva_map_page, gpuva_read_slot,
    gpuva_write_slot, gpuva_sync, gpuva_invalidate, gpuva_prefix,
    gpuva_legacy_slot63, NULL, 0};
/* M2-f6: the broker table the hypervisor sized from guest RAM at start
 * (hv_agx_retained_gpuva_storage); the built-in one while absent. */
static void *gpuva_storage;
static unsigned gpuva_storage_pages;

void hv_agx_retained_gpuva_storage(void *storage, unsigned system_pages)
{
    gpuva_storage = storage;
    gpuva_storage_pages = storage ? system_pages : 0u;
}

static bool gpuva_idle(void)
{
    unsigned i;
    for (i = 0; i < HV_AGX_GPUVA_V5_PROCESSES; ++i)
        if (gpuva_v5.processes[i].live) return false;
    for (i = 1; i < HV_AGX_GPUVA_V5_SLOTS; ++i)
        if (gpuva_v5.slots[i].occupied) return false;
    return true;
}

static bool gpuva_capacity_reported;
static void gpuva_execute(void *context, const AGX_GPUVA_V5_REQUEST *q,
                          AGX_GPUVA_V5_RESPONSE *r)
{
    enum hv_agx_gpuva_v5_result result = HV_AGX_GPUVA_V5_STALE;
    int owner;
    unsigned i;
    (void)context;
    if (!request_powered) goto done;
    result = hv_agx_gpuva_v5_verify(&gpuva_v5);
    if (result != HV_AGX_GPUVA_V5_OK) goto done;
    result = hv_agx_gpuva_v5_validate_envelope(
        &gpuva_v5, q->Epoch, q->Command, q->Flags);
    if (result != HV_AGX_GPUVA_V5_OK) goto done;
    switch (q->Command) {
    case AGX_GPUVA_V5_QUERY_CAPS:
    case AGX_GPUVA_V5_NEGOTIATE_LIVE:
        result = hv_agx_gpuva_v5_extension(&gpuva_v5, q, r);
        break;
    case AGX_GPUVA_V5_CREATE:
        result = hv_agx_gpuva_v5_create(&gpuva_v5,q->ProcessId,
            q->ProcessGeneration,q->TableIpa,q->Flags != 0);
        break;
    case AGX_GPUVA_V5_REGISTER_TABLE:
        result = hv_agx_gpuva_v5_register_table(&gpuva_v5,q->ProcessId,
            q->ProcessGeneration,q->AuxIpa,q->Index);
        break;
    case AGX_GPUVA_V5_REGISTER_BACKING:
        result = hv_agx_gpuva_v5_register_backing(&gpuva_v5,q->ProcessId,
            q->ProcessGeneration,q->AllocationGeneration,q->AuxIpa);
        break;
    case AGX_GPUVA_V5_REGISTER_SHARED_BACKING:
        result = hv_agx_gpuva_v5_register_shared_backing(&gpuva_v5,q->ProcessId,
            q->ProcessGeneration,q->AllocationGeneration,q->AuxIpa);
        break;
    case AGX_GPUVA_V5_UPDATE_PARENT:
        result = hv_agx_gpuva_v5_update_parent(&gpuva_v5,q->ProcessId,
            q->ProcessGeneration,q->TableIpa,q->Index,q->AuxIpa);
        break;
    case AGX_GPUVA_V5_UPDATE_LEAF: {
        uint64_t logical[4] = {q->LogicalIpa[0], q->LogicalIpa[1],
                               q->LogicalIpa[2], q->LogicalIpa[3]};
        result = hv_agx_gpuva_v5_update_leaf(&gpuva_v5,q->ProcessId,
            q->ProcessGeneration,q->TableIpa,q->Index,logical,
            q->AllocationGeneration,q->Flags,q->ValidMask,q->WritableMask);
        break;
    }
    case AGX_GPUVA_V5_RELOCATE_ROOT:
        result = hv_agx_gpuva_v5_relocate_root(&gpuva_v5,q->ProcessId,
            q->ProcessGeneration,q->TableIpa);
        break;
    case AGX_GPUVA_V5_LEASE: {
        uint64_t token = 0;
        result = hv_agx_gpuva_v5_lease(&gpuva_v5,q->ProcessId,
            q->ProcessGeneration,q->Slot,&token);
        r->Token = token;
        break;
    }
    case AGX_GPUVA_V5_JOB_BEGIN:
        result = hv_agx_gpuva_v5_job_begin(&gpuva_v5,q->Slot,q->Token);
        break;
    case AGX_GPUVA_V5_JOB_END:
        result = hv_agx_gpuva_v5_job_end(&gpuva_v5,q->Slot,q->Token);
        break;
    case AGX_GPUVA_V5_RELEASE:
        result = hv_agx_gpuva_v5_release(&gpuva_v5,q->Slot,q->Token);
        break;
    case AGX_GPUVA_V5_DESTROY:
        result = hv_agx_gpuva_v5_destroy(&gpuva_v5,q->ProcessId,
            q->ProcessGeneration);
        break;
    case AGX_GPUVA_V5_REVOKE_BACKING:
        result = hv_agx_gpuva_v5_revoke_backing(&gpuva_v5,q->ProcessId,
            q->ProcessGeneration,q->AllocationGeneration,q->AuxIpa);
        break;
    case AGX_GPUVA_V5_REVOKE_TABLE:
        result = hv_agx_gpuva_v5_revoke_table(&gpuva_v5,q->ProcessId,
            q->ProcessGeneration,q->TableIpa,q->Index);
        break;
    case AGX_GPUVA_V5_FLUSH_TLB:
        if (q->AuxIpa || q->AllocationGeneration || q->Token || q->Slot ||
            q->Index || q->ValidMask || q->WritableMask ||
            q->LogicalIpa[2] || q->LogicalIpa[3]) {
            result = HV_AGX_GPUVA_V5_INVALID;
            break;
        }
        result = hv_agx_gpuva_v5_flush_tlb(&gpuva_v5,q->ProcessId,
            q->ProcessGeneration,q->TableIpa,q->LogicalIpa[0],
            q->LogicalIpa[1]);
        break;
    case AGX_GPUVA_V5_ATTACH_MAILBOX: {
        uint64_t pa = 0;
        if (q->ProcessId || q->ProcessGeneration || q->TableIpa ||
            q->AllocationGeneration || q->Token || q->Slot || q->Index ||
            q->ValidMask || q->WritableMask || q->LogicalIpa[0] ||
            q->LogicalIpa[1] || q->LogicalIpa[2] || q->LogicalIpa[3]) {
            result = HV_AGX_GPUVA_V5_INVALID;
            break;
        }
        result = hv_agx_gpuva_v5_attach_mailbox(&gpuva_v5, q->AuxIpa, &pa);
        if (result == HV_AGX_GPUVA_V5_OK)
            gpuva_v5_wire.mailbox = pa ? (unsigned char *)pa : NULL;
        break;
    }
    default:
        result = HV_AGX_GPUVA_V5_INVALID;
        break;
    }
done:
    if (request_powered &&
        hv_agx_gpuva_v5_verify(&gpuva_v5) == HV_AGX_GPUVA_V5_TAINTED)
        result = HV_AGX_GPUVA_V5_TAINTED;
    if (result == HV_AGX_GPUVA_V5_CAPACITY && !gpuva_capacity_reported) {
        /* EXP1323 receipt-only: the first CAPACITY of the boot with the live
         * table records by level and by owner, once. */
        unsigned live = 0, level[3] = {0, 0, 0}, per_owner[HV_AGX_GPUVA_V5_PROCESSES];
        memset(per_owner, 0, sizeof(per_owner));
        for (i = 0; i < HV_AGX_GPUVA_V5_TABLES; ++i)
            if (gpuva_v5.tables[i].live) {
                ++live;
                if (gpuva_v5.tables[i].level < 3) ++level[gpuva_v5.tables[i].level];
                if (gpuva_v5.tables[i].owner < HV_AGX_GPUVA_V5_PROCESSES)
                    ++per_owner[gpuva_v5.tables[i].owner];
            }
        gpuva_capacity_reported = true;
        printf("HV: GPUVA v5 CAPACITY cmd=%u pid=%lu tables live=%u/%u l0=%u l1=%u l2=%u\n",
               q->Command, (unsigned long)q->ProcessId, live, HV_AGX_GPUVA_V5_TABLES,
               level[0], level[1], level[2]);
        for (i = 0; i < HV_AGX_GPUVA_V5_PROCESSES; ++i)
            if (gpuva_v5.processes[i].live || per_owner[i])
                printf("HV: GPUVA v5 owner %u id=%lu gen=%lu tables=%u backings=%lu\n", i,
                       (unsigned long)gpuva_v5.processes[i].identity,
                       (unsigned long)gpuva_v5.processes[i].generation, per_owner[i],
                       (unsigned long)gpuva_v5.owner_backings[i]);
    }
    r->Status = result;
    r->Epoch = gpuva_v5.epoch;
    for (i = 0; i < HV_AGX_GPUVA_V5_PROCESSES; ++i) {
        owner = (int)i;
        if (gpuva_v5.processes[owner].live &&
            gpuva_v5.processes[owner].identity == q->ProcessId &&
            gpuva_v5.processes[owner].generation == q->ProcessGeneration) {
            r->RootGeneration = gpuva_v5.processes[owner].root_generation;
            r->MapGeneration = gpuva_v5.processes[owner].map_generation;
            break;
        }
    }
    r->Flags = gpuva_v5.tainted ? 1u : 0u;
}

static bool cpu_stopped(void)
{
    return asc_base && !(read32(asc_base + 0x44) & BIT(4));
}

static const unsigned char *read_profile_property(void *context,const char *name,
                                                 unsigned int *length)
{
    return adt_getprop(adt,*(int *)context,name,length);
}

static void execute(void *context, const AGX_RR_REQUEST *q, AGX_RR_RESPONSE *r)
{
    static const struct hv_agx_retained_ops ops = {
        NULL, allocate_page, release_page, translate_guest, sync_tables};
    int status = HV_AGX_RETAINED_STATE;
    (void)context;
    if (!root_base || !request_powered) goto done;
    switch (q->Command) {
    case AGX_RR_PREPARE:
        if (q->Epoch || q->Va || q->Ipa || q->Length || q->Handle ||
            !cpu_stopped() || next_epoch == ~0ULL || !profile_inputs_valid) break;
        status = hv_agx_retained_prepare(&root_owner, root_base, (void *)root_base,
                                         root_length, ++next_epoch, &ops);
        break;
    case AGX_RR_ACTIVATE:
        if (q->Epoch != root_owner.Epoch || cpu_stopped() ||
            q->Va || q->Ipa || q->Length || q->Handle) break;
        status = hv_agx_retained_activate(&root_owner);
        if (!status) {
            write64(HV_AGX_G2_GPU_BASE, root_owner.Roots.Ttbr0PhysicalAddress | 1ULL);
            write64(HV_AGX_G2_GPU_BASE + 8, root_base | 1ULL);
            sync_tables(NULL);
            printf("HV: retained root ACTIVE root=0x%lx prefix=%llx/%llx epoch=%llu\n",
                   root_base, root_owner.PrivatePrefix[0], root_owner.PrivatePrefix[1],
                   root_owner.Epoch);
            status = hv_agx_retained_io_prepare(&root_owner,root_owner.Epoch);
            printf("HV: retained firmware IO prepare status=%d epoch=%llu\n",
                   status,root_owner.Epoch);
            if (!status) {
                enum hv_agx_gpuva_v5_result v5_status;
                /* A new broker epoch starts with no mailbox page attached. */
                gpuva_v5_wire.mailbox = NULL;
                struct hv_agx_gpuva_v5_ops ops = gpuva_ops;
                ops.storage = gpuva_storage;
                ops.storage_system_backings = gpuva_storage_pages;
                v5_status = hv_agx_gpuva_v5_init(&gpuva_v5,root_owner.Epoch,&ops);
                uint64_t public_base = 0, public_bytes = 0;
                if (v5_status == HV_AGX_GPUVA_V5_OK &&
                    hv_agx_local_public_span(&public_base,&public_bytes))
                    v5_status = hv_agx_gpuva_v5_configure_local(
                        &gpuva_v5,public_base,public_bytes,UINT64_C(0x4000000));
                else
                    v5_status = HV_AGX_GPUVA_V5_INVALID;
                if (v5_status != HV_AGX_GPUVA_V5_OK)
                    gpuva_v5.active = false;
                printf("HV: GPUVA broker v5 init=%u epoch=%llu\n",
                       v5_status,root_owner.Epoch);
            }
        }
        break;
    case AGX_RR_MAP:
        if (q->Handle) break;
        status = hv_agx_retained_map(&root_owner,q->Epoch,q->Va,q->Ipa,q->Length,&r->Handle);
        break;
    case AGX_RR_QUERY:
        status = hv_agx_retained_query(&root_owner,q->Epoch,q->Handle,q->Va,q->Ipa,
                                       q->Length,&r->Pa);
        r->Handle = q->Handle;
        break;
    case AGX_RR_UNMAP:
        status = hv_agx_retained_unmap(&root_owner,q->Epoch,q->Handle,q->Va,q->Ipa,q->Length);
        break;
    case AGX_RR_VERIFY_ABSENT:
        status = hv_agx_retained_verify_absent(&root_owner,q->Epoch,q->Handle,q->Va,q->Ipa,q->Length);
        r->Handle = q->Handle;
        break;
    case AGX_RR_QUERY_ARENA:
        if (!q->Epoch || q->Ipa || q->Length || q->Handle || q->Va > 0xffffffffULL)
            break;
        {
            AGX_RR_ARENA_DESCRIPTOR arena = {0};
            status = hv_agx_retained_query_arena(
                &root_owner, q->Epoch, (unsigned int)q->Va, &arena);
            if (!status) {
                r->ArenaVersion = arena.Version;
                r->ArenaClass = arena.Class;
                r->ArenaVa = arena.Va;
                r->ArenaBytes = arena.Bytes;
            }
        }
        break;
    case AGX_RR_QUERY_TABLE_HASH:
        if (q->Va || q->Ipa || q->Length || q->Handle) break;
        {
            unsigned int pages = 0;
            status = hv_agx_retained_table_hash(&root_owner, q->Epoch,
                                                 &r->Pa, &pages);
            if (!status) r->Count = pages;
        }
        break;
    case AGX_RR_CLOSE:
        if (!q->Epoch || q->Epoch != root_owner.Epoch || q->Va || q->Ipa || q->Length || q->Handle ||
            !cpu_stopped() || !gpuva_idle()) break;
        /* Root1 retains physical identity. Retire owned root0 before freeing. */
        if (root_owner.Prepared) {
            write64(HV_AGX_G2_GPU_BASE, 0);
            sync_tables(NULL);
        }
        status = hv_agx_retained_close(&root_owner,q->Epoch,1);
        if (!status) {
            memset(&gpuva_v5,0,sizeof(gpuva_v5));
            memset(&gpuva_v5_wire,0,sizeof(gpuva_v5_wire));
        }
        break;
    }
done:
    if (root_owner.Active &&
        (read64(HV_AGX_G2_GPU_BASE + 8) != (root_base | 1ULL) ||
         read64(HV_AGX_G2_GPU_BASE) != (root_owner.Roots.Ttbr0PhysicalAddress | 1ULL))) {
        root_owner.Tainted = 1;
        status = HV_AGX_RETAINED_TAINTED;
    }
    r->Status = status;
    r->Epoch = root_owner.Epoch;
    r->Root = root_owner.Active ? (read64(HV_AGX_G2_GPU_BASE + 8) & 0xffffffc000ULL)
                               : root_base;
    r->Ttbr0 = root_owner.Roots.Ttbr0PhysicalAddress;
    r->SystemVa = root_owner.SystemVa;
    r->SystemBytes = root_owner.SystemBytes;
    hv_agx_retained_finalize_count(r, q->Command, root_owner.MappingCount);
    r->Flags = (root_owner.Prepared ? AGX_RR_FLAG_PREPARED : 0) |
               (root_owner.Active ? AGX_RR_FLAG_ACTIVE : 0) |
               (hv_agx_retained_prefix_unchanged(&root_owner) ? AGX_RR_FLAG_PREFIX_UNCHANGED : 0);
    printf("HV: retained op=%u seq=%llu status=%u root=0x%lx epoch=%llu va=%llx count=%u flags=%x\n",
           q->Command,q->Sequence,r->Status,root_base,r->Epoch,q->Va,root_owner.MappingCount,r->Flags);
}

bool hv_agx_retained_platform_init(u64 root, u64 length)
{
    int path[8];
    int node = adt_path_offset_trace(adt, "/arm-io/gfx-asc", path);
    u64 size;
    if (chip_id != 0x8103 || board_id != 0x26) return false;
    {
        int sgx = adt_path_offset(adt,"/arm-io/sgx");
        profile_inputs_valid = sgx >= 0 && os_firmware.version == V13_5 &&
                              AgxHwdataInputsMatch(read_profile_property,&sgx);
        printf("HV: retained Hwdata input profile matched=%u\n",profile_inputs_valid);
    }
    if (node < 0 || adt_get_reg(adt,path,"reg",0,&asc_base,&size) < 0 || size < 0x48)
        return false;
    root_base = root; root_length = length;
    /* Assisted launches may reclaim the unused gap below the fixed FD.
     * The launcher also excludes its proxy heap and new ADT carveouts. */
    launch_memory_valid = cur_boot_args.mem_size <= UINT64_MAX - cur_boot_args.phys_base &&
        hv_launch_j313_capture_base(HV_CONTRACT_PRE_HV_INIT,1,&launch_memory) &&
        hv_agx_retained_ram_valid(&launch_memory, (u64)heapblock_alloc(0),
                                 cur_boot_args.phys_base + cur_boot_args.mem_size,
                                 J313_AUTONOMOUS_LAYOUT.firmware_base);
    if (!launch_memory_valid) return false;
    printf("HV: retained backing guest RAM=0x%llx..0x%llx reserved regions=%u\n",
           (unsigned long long)launch_memory.boot.ram_base,
           (unsigned long long)(launch_memory.boot.ram_base+launch_memory.boot.ram_size),
           launch_memory.region_count);
    return root_base != 0;
}

bool hv_agx_retained_platform_mmio(u64 offset, u64 *value, bool write,
                                  unsigned width, bool powered)
{
    unsigned long long data = *value;
    request_powered = powered;
    bool result = hv_agx_retained_mmio(&wire,offset,&data,write,width,execute,NULL);
    if (result && !write) *value = data;
    return result;
}

bool hv_agx_retained_platform_gpuva_v5(u64 offset, u64 *value, bool write,
                                      unsigned width, bool powered)
{
    request_powered = powered;
    if (!gpuva_v5.active || !root_owner.Active) return false;
    return hv_agx_gpuva_v5_mmio(&gpuva_v5_wire,offset,value,write,width,
                                 gpuva_execute,NULL);
}

bool hv_agx_retained_platform_io(u64 offset,u64 *value,bool write,
                                 unsigned width,bool powered)
{
    AGX_FW_IO_MANIFEST manifest = {0};
    unsigned long long data = *value;
    if (write || width > 3 || (offset & ((1u << width)-1)) ||
        offset > AGX_FW_IO_BYTES-(1u << width)) return false;
    if (powered && root_owner.Active &&
        read64(HV_AGX_G2_GPU_BASE + 8) == (root_base | 1ULL) &&
        read64(HV_AGX_G2_GPU_BASE) == (root_owner.Roots.Ttbr0PhysicalAddress | 1ULL))
        (void)hv_agx_retained_io_manifest(&root_owner,root_owner.Epoch,&manifest);
    if (!AgxFwIoReadWord(&manifest,offset,&data,0,width)) return false;
    *value = data;
    return true;
}

bool hv_agx_retained_platform_profile(u64 offset,u64 *value,bool write,
                                      unsigned width,bool powered)
{
    AGX_HWDATA_RECEIPT receipt = {0};
    AGX_FW_IO_MANIFEST manifest = {0};
    unsigned long long data = *value;
    if (write || width > 3 || (offset & ((1u << width)-1)) ||
        offset > AGX_HWDATA_RECEIPT_BYTES-(1u << width)) return false;
    if (profile_inputs_valid && powered && root_owner.Active &&
        read64(HV_AGX_G2_GPU_BASE + 8) == (root_base | 1ULL) &&
        read64(HV_AGX_G2_GPU_BASE) == (root_owner.Roots.Ttbr0PhysicalAddress | 1ULL) &&
        hv_agx_retained_io_manifest(&root_owner,root_owner.Epoch,&manifest) == 0) {
        receipt.Magic = AGX_HWDATA_RECEIPT_MAGIC;
        receipt.Version = 1; receipt.Bytes = sizeof(receipt); receipt.Chip = chip_id;
        receipt.Epoch = root_owner.Epoch; receipt.Root = root_base;
        memcpy(receipt.ProfileId,AgxHwdataProfileId,sizeof(receipt.ProfileId));
    }
    if (!AgxHwdataReceiptWord(&receipt,offset,&data,0,width)) return false;
    *value = data;
    return true;
}

bool hv_agx_retained_gpu_region(struct exc_info *ctx, u64 addr, u64 *value,
                                bool write, int width)
{
    u64 offset = addr - HV_AGX_G2_GPU_BASE;
    size_t bytes;
    (void)ctx;
    if (width < 0 || width > 3) return false;
    bytes = 1u << width;
    if (offset > HV_AGX_G2_GPU_SIZE - bytes || (offset & (bytes - 1))) return false;
    if (write && (!AgxRrGpuRegionWritable(offset,bytes) ||
                  (gpuva_v5.active && gpuva_v5.slots[63].occupied))) return false;
    if (write) memcpy((void *)addr,value,bytes);
    else { *value = 0; memcpy(value,(void *)addr,bytes); }
    dma_mb();
    return true;
}

/* M2-f2 pool window: a window page aliases plain guest RAM a GPU backing could
 * use, and leaves the window only while no broker backing, table or mailbox
 * holds its PA. Callers hold broker_lock. */
u64 hv_agx_retained_platform_ram_page(u64 ipa)
{
    return translate_guest(NULL, ipa);
}

bool hv_agx_retained_platform_page_busy(u64 pa)
{
    const struct hv_agx_gpuva_v5_backing *backing =
        hv_agx_gpuva_v5_backing_by_pa(&gpuva_v5, pa);
    unsigned i;
    if (backing && backing->live) return true;
    if (gpuva_v5.mailbox_pa && gpuva_v5.mailbox_pa == pa) return true;
    for (i = 0; i < HV_AGX_GPUVA_V5_TABLES; ++i)
        if (gpuva_v5.tables[i].live && gpuva_v5.tables[i].pa == pa) return true;
    return false;
}

bool hv_agx_retained_can_power_off(void)
{
    return !root_owner.Prepared && !root_owner.Active;
}

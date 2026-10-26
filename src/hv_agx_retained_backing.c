#include "hv_agx_retained_backing.h"
static bool overlaps(uint64_t a,uint64_t n,uint64_t b,uint64_t m)
{
    return m && a < b+m && b < a+n;
}
/* Boot inputs the launcher hands the guest (firmware image, its heap, ADT and
 * boot-args copies) become guest RAM once the guest runs: the OS allocates
 * them (EXP1251: Windows' RAM map starts at the guest RAM base and its pages
 * there were refused). Only regions the hypervisor or devices keep using
 * after launch -- the boot framebuffer, device DART tables, a published
 * ramdisk -- deny GPU backing. */
static bool boot_input(unsigned kind)
{
    return kind == HV_CONTRACT_REGION_HEAP || kind == HV_CONTRACT_REGION_FIRMWARE ||
           kind == HV_CONTRACT_REGION_ADT || kind == HV_CONTRACT_REGION_BOOT_ARGS;
}
bool hv_agx_retained_backing_allowed(const struct hv_contract_snapshot *s,
    uint64_t pa,uint64_t bytes)
{
    unsigned i;
    bool guest_region = false;
    if (!s || bytes != 0x4000 || (pa & 0x3fff) || pa > UINT64_MAX-bytes ||
        !s->region_count || s->region_count > HV_CONTRACT_MAX_REGIONS ||
        !s->boot.ram_size || s->boot.ram_base > UINT64_MAX-s->boot.ram_size ||
        pa < s->boot.ram_base || bytes > s->boot.ram_size ||
        pa-s->boot.ram_base > s->boot.ram_size-bytes)
        return false;
    for(i=0;i<s->region_count;++i) {
        const struct hv_contract_region *r=&s->regions[i];
        if (r->kind > HV_CONTRACT_REGION_RAMDISK || r->base > UINT64_MAX-r->size)
            return false;
        if(r->kind==HV_CONTRACT_REGION_GUEST_RAM) {
            if(pa>=r->base && bytes<=r->size && pa-r->base<=r->size-bytes)
                guest_region=true;
        } else if(r->kind!=HV_CONTRACT_REGION_LOW_MEMORY && !boot_input(r->kind) &&
                  overlaps(pa,bytes,r->base,r->size))
            return false;
    }
    return guest_region;
}

bool hv_agx_retained_ram_valid(const struct hv_contract_snapshot *s,
    uint64_t host_end, uint64_t platform_end, uint64_t firmware_entry)
{
    if (!s || !s->region_count || s->region_count > HV_CONTRACT_MAX_REGIONS ||
        !s->boot.ram_size || (s->boot.ram_base | s->boot.ram_size) & 0x3fff ||
        s->boot.ram_base < host_end || s->boot.ram_base >= platform_end ||
        s->boot.ram_size > platform_end - s->boot.ram_base ||
        s->boot.guest_entry != firmware_entry || firmware_entry < s->boot.ram_base ||
        firmware_entry - s->boot.ram_base >= s->boot.ram_size)
        return false;
    unsigned ram_regions = 0;
    for (unsigned i = 0; i < s->region_count; ++i) {
        const struct hv_contract_region *r = &s->regions[i];
        if (r->kind == HV_CONTRACT_REGION_GUEST_RAM) {
            if (r->flags || r->base != s->boot.ram_base || r->size != s->boot.ram_size)
                return false;
            ++ram_regions;
        }
    }
    return ram_regions == 1;
}

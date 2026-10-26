# SPDX-License-Identifier: MIT
"""Validate assisted RAM expansion independently of the fixed image anchor."""


def expanded_ram_base(utils, memory_base):
    """Return a safe RAM base; never use the launcher-overridden heap_top as a floor.

    Only the newly exposed interval is considered for ADT carveouts. Existing
    guest/device mappings retain their current ownership and exclusions.
    """
    load_base = utils.heap_top
    platform_base = utils.ba.phys_base
    platform_size = utils.ba.mem_size
    heap = utils.heap
    heap_end = heap.offset + heap.count * heap.block
    end = platform_base + platform_size
    if (not 0 <= platform_base < end < 1 << 64 or
            not platform_base <= heap.offset < heap_end <= memory_base <= load_base < end or
            (memory_base | load_base | end) & 0x3fff or
            memory_base >> 32 != load_base >> 32):
        raise ValueError('expanded RAM overlaps host heap or exceeds platform/load bounds')

    # The normal-RAM owner contains the entire interval, rather than carving
    # memory out of it. All other owners, including unknown IDs, are exclusions.
    carveouts = utils.adt['/chosen/carveout-memory-map']
    normal_seen = False
    for name, value in carveouts._properties.items():
        if not name.startswith('region-id-'):
            continue
        if not isinstance(value, (tuple, list)) or len(value) != 2:
            raise ValueError(f'invalid ADT carveout {name}')
        base, size = value
        if not 0 <= base <= base + size < 1 << 64:
            raise ValueError(f'invalid ADT carveout {name}')
        if (base, size) == (platform_base, platform_size):
            normal_seen = True
        elif size and base < load_base and memory_base < base + size:
            raise ValueError(f'expanded RAM overlaps ADT carveout {name}')
    if not normal_seen:
        raise ValueError('ADT carveout map does not confirm platform normal RAM')
    return memory_base

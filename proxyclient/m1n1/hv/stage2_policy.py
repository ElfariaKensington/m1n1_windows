# SPDX-License-Identifier: MIT
"""Stage-2 policy for DRAM above the guest's RAM top.

iBoot reports guest-usable RAM as ``[phys_base, phys_base + mem_size)``.  Everything from
that ``mem_top`` to the end of physical DRAM (``ram_base + mem_size_actual``) is firmware
carveout: TrustZone, DCP/DCPEXT data, the boot framebuffer, the iBoot-built DART
translation tables of dart-dcp/dart-disp0/dart-dcpext/dart-dispext0, AGX firmware
regions and others (``/chosen/carveout-memory-map``).  Asahi Linux marks these as
``no-map`` reserved memory; m1n1's standalone runtime already clamps the guest RAM map
to ``mem_top``.

``HV.load_raw()`` historically passed the whole window through to the guest.  For a
guest that does not own DCP (the J313 Windows guest: m1n1 owns DCP and its DARTs) this
lets guest CPU stores corrupt live DART tables.  This module computes, deterministically
from the ADT, which parts of that window stay mapped (the boot framebuffer and the AGX
regions the Windows AGX driver consumes) and which are removed from stage-2.

The functions here are pure: they read an ADT object and integers and return ranges.
"""

from dataclasses import dataclass

PAGE_SIZE = 0x4000

__all__ = [
    "PAGE_SIZE",
    "Stage2Range",
    "AboveRamPolicy",
    "StagePolicyError",
    "above_ram_keep_ranges",
    "above_ram_policy",
    "carveout_regions",
    "carveouts_overlapping",
    "in_ram_carveouts",
    "assert_disjoint",
]


class StagePolicyError(ValueError):
    pass


@dataclass(frozen=True, order=True)
class Stage2Range:
    start: int
    end: int
    label: str = ""

    @property
    def size(self):
        return self.end - self.start

    def __str__(self):
        return f"[{self.start:#x}..{self.end:#x}) {self.label}".rstrip()


@dataclass(frozen=True)
class AboveRamPolicy:
    window: Stage2Range
    keep: tuple
    exclude: tuple


def _getprop(node, name):
    try:
        return getattr(node, name.replace("-", "_"))
    except (AttributeError, KeyError):
        return None


def _node(adt, path):
    try:
        return adt[path]
    except (KeyError, AttributeError):
        return None


def _pair(value):
    if value is None:
        return None
    base, size = value[0], value[1]
    return int(base), int(size)


def carveout_regions(adt):
    """Return ``{"region-id-N": (base, size)}`` from /chosen/carveout-memory-map."""
    node = _node(adt, "/chosen/carveout-memory-map")
    if node is None:
        return {}
    regions = {}
    for name in node._properties:
        if not name.startswith("region-id-"):
            continue
        value = node._properties[name]
        try:
            base, size = _pair(value)
        except (TypeError, IndexError, ValueError):
            continue
        if base and size:
            regions[name] = (base, size)
    return regions


def carveouts_overlapping(adt, start, end):
    """Carveout ids overlapping ``[start, end)``, sorted by base (evidence/logging)."""
    hits = []
    for name, (base, size) in carveout_regions(adt).items():
        if base < end and start < base + size:
            hits.append((base, name))
    return [name for _, name in sorted(hits)]


def above_ram_keep_ranges(adt, video_base=None, video_size=None):
    """Ranges that must remain guest-visible, each labelled with its ADT source.

    * ``/vram`` reg 0: the boot framebuffer (carveout region-id-14 on J313);
    * the boot-args video surface, when the launcher points it above RAM;
    * ``/arm-io/sgx`` gfx-handoff, gfx-shared-region and gpu-region: the AGX firmware
      handoff page and regions exposed to the guest AGX driver.
    """
    keep = []

    vram = _node(adt, "/vram")
    if vram is not None:
        reg = _getprop(vram, "reg")
        if reg:
            entry = reg[0]
            if hasattr(entry, "addr"):
                base, size = int(entry.addr), int(entry.size)
            else:
                base, size = int(entry[0]), int(entry[1])
            if size:
                keep.append(Stage2Range(base, base + size, "framebuffer:/vram"))

    if video_base and video_size:
        keep.append(Stage2Range(video_base, video_base + video_size,
                                "framebuffer:boot-args.video"))

    sgx = _node(adt, "/arm-io/sgx")
    if sgx is not None:
        for prop, label in (("gfx-handoff", "agx:gfx-handoff"),
                            ("gfx-shared-region", "agx:gfx-shared-region"),
                            ("gpu-region", "agx:gpu-region")):
            base = _getprop(sgx, prop + "-base")
            size = _getprop(sgx, prop + "-size")
            if base and size:
                keep.append(Stage2Range(int(base), int(base) + int(size), label))

    return keep


def _align_down(value):
    return value & ~(PAGE_SIZE - 1)


def _align_up(value):
    return (value + PAGE_SIZE - 1) & ~(PAGE_SIZE - 1)


def above_ram_policy(adt, mem_top, dram_end, video_base=None, video_size=None):
    """Split ``[mem_top, dram_end)`` into kept and excluded stage-2 ranges.

    Kept ranges are clipped to the window and widened to 16 KiB pages (stage-2
    granule); everything else in the window is excluded.  The result is sorted,
    non-overlapping and covers the window exactly.
    """
    if mem_top % PAGE_SIZE or dram_end % PAGE_SIZE:
        raise StagePolicyError(
            f"window [{mem_top:#x}..{dram_end:#x}) is not 16 KiB aligned")
    if mem_top > dram_end:
        raise StagePolicyError(f"mem_top {mem_top:#x} is above DRAM end {dram_end:#x}")

    window = Stage2Range(mem_top, dram_end, "above-ram")

    clipped = []
    for r in above_ram_keep_ranges(adt, video_base, video_size):
        start = max(_align_down(r.start), mem_top)
        end = min(_align_up(r.end), dram_end)
        if start < end:
            clipped.append(Stage2Range(start, end, r.label))
    clipped.sort()

    # Merge overlapping/adjacent keep ranges, joining their labels.
    keep = []
    for r in clipped:
        if keep and r.start <= keep[-1].end:
            last = keep[-1]
            labels = last.label.split("+")
            if r.label not in labels:
                labels.append(r.label)
            keep[-1] = Stage2Range(last.start, max(last.end, r.end), "+".join(labels))
        else:
            keep.append(r)

    exclude = []
    cursor = mem_top
    for r in keep:
        if cursor < r.start:
            exclude.append(Stage2Range(cursor, r.start, "unmapped"))
        cursor = max(cursor, r.end)
    if cursor < dram_end:
        exclude.append(Stage2Range(cursor, dram_end, "unmapped"))

    labelled = []
    for r in exclude:
        ids = carveouts_overlapping(adt, r.start, r.end)
        labelled.append(Stage2Range(r.start, r.end, ",".join(ids) if ids else "no-carveout"))

    return AboveRamPolicy(window, tuple(keep), tuple(labelled))


# --------------------------------------------------------------------------- in-RAM carveouts


def in_ram_carveouts(adt, ram_start, ram_end, merge_gap=PAGE_SIZE):
    """iBoot carveouts that intersect guest RAM ``[ram_start, ram_end)``.

    A carveout that covers the whole guest window is the iBoot kernel-DRAM descriptor
    (region-id-24 on J313, equal to boot-args [phys_base, mem_top)), not a reservation,
    and is skipped.  The rest are clipped to the window, widened to 16 KiB pages and
    merged; labels list the contributing region ids.  Owners are not named in the ADT,
    so every such carveout is treated as not guest RAM.

    Ranges separated by at most ``merge_gap`` bytes of RAM are merged (the gap is lost
    to the guest): each separate range costs two stage-2 records and the launch-contract
    receipt holds 64.  Guest firmware must withhold a superset of the result from the OS.
    """
    if ram_start % PAGE_SIZE or ram_end % PAGE_SIZE or ram_start >= ram_end:
        raise StagePolicyError(f"bad guest RAM window [{ram_start:#x}..{ram_end:#x})")
    hits = []
    for name, (base, size) in carveout_regions(adt).items():
        end = base + size
        if base <= ram_start and ram_end <= end:
            continue
        start = max(_align_down(base), ram_start)
        stop = min(_align_up(end), ram_end)
        if start < stop:
            hits.append((start, stop, name))
    hits.sort()
    merged = []
    for start, stop, name in hits:
        if merged and start <= merged[-1].end + merge_gap:
            last = merged[-1]
            merged[-1] = Stage2Range(last.start, max(last.end, stop), last.label + "," + name)
        else:
            merged.append(Stage2Range(start, stop, name))
    return tuple(merged)


def assert_disjoint(ranges, occupied):
    """Raise if any launcher-owned window (name -> (start, end)) hits a carveout."""
    for name, (start, end) in occupied.items():
        for r in ranges:
            if start < r.end and r.start < end:
                raise StagePolicyError(
                    f"{name} [{start:#x}..{end:#x}) overlaps carveout {r}")

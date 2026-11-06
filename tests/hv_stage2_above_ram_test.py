#!/usr/bin/env python3
"""Stage-2 isolation of DRAM above the guest's RAM top (HV.load_raw RAM-HIGH).

Invariants caught here:
  * no page of the iBoot DART translation tables (pmap-io-ranges 'CARV', i.e.
    dart-dcp / dart-disp0 / dart-dcpext / dart-dispext0) or any other non-kept carveout
    above mem_top stays guest-mapped once isolation is enabled;
  * the boot framebuffer, the AGX handoff / shared / gpu-region pages and all guest RAM
    below mem_top keep exactly their previous stage-2 mapping;
  * the legacy (isolation off) stage-2 sequence is bit-identical to the live
    EXP950 launch-contract receipt, so the default behaviour is unchanged;
  * the extra stage-2 records fit the 64-entry launch-contract receipt
    (HV_STAGE2_STATE_MAX_MAPPINGS) on top of the 51 records of GPU-profile launches.

Fixture: tests/data/j313_adt_above_ram.json is extracted from the ADT saved by
ANSDART001-R3 (sha256 recorded in the fixture).  Regenerate with
  python3 tests/hv_stage2_above_ram_test.py --regenerate /path/to/adt.bin
"""

import hashlib
import json
import sys
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "proxyclient"))

from m1n1.hv.stage2_policy import PAGE_SIZE, StagePolicyError, above_ram_policy  # noqa: E402

FIXTURE = Path(__file__).parent / "data" / "j313_adt_above_ram.json"


# --------------------------------------------------------------------------- fixture


def extract_fixture(adt_path):
    from m1n1.adt import load_adt

    blob = Path(adt_path).read_bytes()
    adt = load_adt(blob)
    carveouts = {}
    for name, value in adt["/chosen/carveout-memory-map"]._properties.items():
        if name.startswith("region-id-"):
            carveouts[name] = [hex(int(value[0])), hex(int(value[1]))]
    vram = adt["/vram"].reg[0]
    sgx = adt["/arm-io/sgx"]
    sgx_props = {}
    for prop in ("gfx-handoff", "gfx-shared-region", "gpu-region"):
        for suffix in ("base", "size"):
            key = f"{prop}-{suffix}"
            sgx_props[key] = hex(int(getattr(sgx, key.replace("-", "_"))))
    pmap = [
        {"addr": hex(e.addr), "size": hex(e.size), "flags": hex(e.flags), "name": e.name}
        for e in adt["/defaults"].pmap_io_ranges
        if e.addr >= 0x800000000
    ]
    return {
        "source": "ANSDART001-R3 adt.bin (J313, iBoot-11881.140.96.701.1, OS FW 13.5)",
        "adt_sha256": hashlib.sha256(blob).hexdigest(),
        "boot_args": {
            "phys_base": "0x802078000",
            "mem_size": "0x1dd690000",
            "mem_size_actual": "0x200000000",
            "ram_base": "0x800000000",
        },
        "mcc_tz_carveouts": [
            ["0x9df710000", "0x1c28000"],
            ["0x9df708000", "0x8000"],
            ["0x9e6e20000", "0x19000000"],
        ],
        "carveout_memory_map": carveouts,
        "vram": [hex(vram.addr), hex(vram.size)],
        "sgx": sgx_props,
        "pmap_io_ranges_dram": pmap,
    }


class FakeNode:
    def __init__(self, props):
        self._properties = props

    def __getattr__(self, name):
        props = self.__dict__["_properties"]
        key = name.replace("_", "-")
        if key in props:
            return props[key]
        if name in props:
            return props[name]
        raise AttributeError(name)


class FakeAdt:
    def __init__(self, fx, drop=()):
        h = lambda v: int(v, 16)
        self.nodes = {
            "/chosen/carveout-memory-map": FakeNode(
                {k: [h(b), h(s)] for k, (b, s) in fx["carveout_memory_map"].items()}),
            "/vram": FakeNode({"reg": [SimpleNamespace(addr=h(fx["vram"][0]),
                                                      size=h(fx["vram"][1]))]}),
            "/arm-io/sgx": FakeNode({k: h(v) for k, v in fx["sgx"].items()}),
        }
        for path in drop:
            del self.nodes[path]

    def __getitem__(self, path):
        return self.nodes[path]


def load_fixture():
    return json.loads(FIXTURE.read_text())


def window(fx):
    ba = fx["boot_args"]
    mem_top = int(ba["phys_base"], 16) + int(ba["mem_size"], 16)
    dram_end = int(ba["ram_base"], 16) + int(ba["mem_size_actual"], 16)
    return mem_top, dram_end


def dart_table_pages(fx):
    return [(int(e["addr"], 16), int(e["size"], 16)) for e in fx["pmap_io_ranges_dram"]
            if e["name"] == "CARV"]


# --------------------------------------------------------------------------- policy


def test_policy_exact_ranges():
    fx = load_fixture()
    mem_top, dram_end = window(fx)
    assert (mem_top, dram_end) == (0x9df708000, 0xa00000000)
    pol = above_ram_policy(FakeAdt(fx), mem_top, dram_end)
    assert [(r.start, r.end) for r in pol.keep] == [
        (0x9e2638000, 0x9e5394000),  # /vram = region-id-14, boot framebuffer
        (0x9fff70000, 0x9fff74000),  # sgx gfx-handoff = region-id-56
        (0x9fff78000, 0x9fffbc000),  # sgx gfx-shared-region (52) + gpu-region (8)
    ], pol.keep
    assert [(r.start, r.end) for r in pol.exclude] == [
        (0x9df708000, 0x9e2638000),
        (0x9e5394000, 0x9fff70000),
        (0x9fff74000, 0x9fff78000),
        (0x9fffbc000, 0xa00000000),
    ], pol.exclude
    assert pol.exclude[2].label == "no-carveout"


def test_policy_partitions_window():
    fx = load_fixture()
    mem_top, dram_end = window(fx)
    pol = above_ram_policy(FakeAdt(fx), mem_top, dram_end)
    parts = sorted(list(pol.keep) + list(pol.exclude))
    cursor = mem_top
    for r in parts:
        assert r.start == cursor and r.end > r.start, parts
        assert r.start % PAGE_SIZE == 0 and r.end % PAGE_SIZE == 0
        cursor = r.end
    assert cursor == dram_end


def test_every_dart_table_page_is_excluded():
    fx = load_fixture()
    mem_top, dram_end = window(fx)
    pol = above_ram_policy(FakeAdt(fx), mem_top, dram_end)
    tables = dart_table_pages(fx)
    # The ANSDART001-R3 census pages: dcp L1 0x9fff0c000, disp0 L1 0x9fff34000/0x9fff04000.
    assert {0x9fff0c000, 0x9fff34000, 0x9fff04000} <= {b for b, _ in tables}
    for base, size in tables:
        for page in range(base, base + size, PAGE_SIZE):
            assert any(r.start <= page < r.end for r in pol.exclude), hex(page)
            assert not any(r.start <= page < r.end for r in pol.keep), hex(page)


def test_every_non_kept_carveout_above_ram_is_excluded():
    fx = load_fixture()
    mem_top, dram_end = window(fx)
    pol = above_ram_policy(FakeAdt(fx), mem_top, dram_end)
    kept_ids = {"region-id-14", "region-id-56", "region-id-52", "region-id-8"}
    for name, (b, s) in fx["carveout_memory_map"].items():
        base, size = int(b, 16), int(s, 16)
        if base < mem_top:
            continue
        inside = [r for r in (pol.keep if name in kept_ids else pol.exclude)
                  if r.start <= base and base + size <= r.end]
        assert inside, name


def test_boot_args_video_above_ram_is_kept_and_in_ram_is_ignored():
    fx = load_fixture()
    mem_top, dram_end = window(fx)
    adt = FakeAdt(fx, drop=("/vram",))
    pol = above_ram_policy(adt, mem_top, dram_end, 0x9e2638000, 0x2800 * 1600)
    assert pol.keep[0].start == 0x9e2638000 and pol.keep[0].end == 0x9e2638000 + 0x2800 * 1600
    # run_uefi.py: guest framebuffer at 0x85f000000 lies in guest RAM -> no keep entry.
    pol = above_ram_policy(adt, mem_top, dram_end, 0x85f000000, 10240 * 1600)
    assert all(not r.label.startswith("framebuffer") for r in pol.keep)


def test_missing_sgx_and_vram_exclude_whole_window():
    fx = load_fixture()
    mem_top, dram_end = window(fx)
    pol = above_ram_policy(FakeAdt(fx, drop=("/vram", "/arm-io/sgx")), mem_top, dram_end)
    assert pol.keep == ()
    assert [(r.start, r.end) for r in pol.exclude] == [(mem_top, dram_end)]


def test_rejects_unaligned_or_inverted_window():
    fx = load_fixture()
    for args in ((0x9df708001, 0xa00000000), (0xa00000000, 0x9df708000)):
        try:
            above_ram_policy(FakeAdt(fx), *args)
        except StagePolicyError:
            continue
        raise AssertionError(f"accepted {args}")


# --------------------------------------------------------------------------- HV integration


class FakeProxy:
    def __init__(self, tz):
        self.calls = []
        self.tz = tz

    def hv_map(self, ipa, pa, size, incr):
        self.calls.append((ipa, pa, size, incr))
        return 0

    # unmap_carveouts(): MCC TZ list terminated by a zero base.
    def mcc_get_carveouts(self):
        return 0x1000

    def read64(self, addr):
        idx, word = divmod(addr - 0x1000, 16)
        if idx >= len(self.tz):
            return 0
        return self.tz[idx][word // 8]


def build_hv(fx, isolate, video_base=0x85f000000):
    from m1n1.hv import HV
    from m1n1.utils import irange

    h = lambda v: int(v, 16)
    ba = fx["boot_args"]
    tz = [(h(b), h(s)) for b, s in fx["mcc_tz_carveouts"]]
    p = FakeProxy(tz)
    u = SimpleNamespace(
        adt=FakeAdt(fx),
        ba=SimpleNamespace(phys_base=h(ba["phys_base"]), mem_size=h(ba["mem_size"]),
                           mem_size_actual=h(ba["mem_size_actual"])),
        inst=lambda *_: None,
    )
    hv = HV(None, p, u)
    hv.print_tracer = SimpleNamespace(log_file=None)
    hv.tba = SimpleNamespace(video=SimpleNamespace(base=video_base, stride=10240, height=1600))
    # Same RAM-LOW/RAM-HIGH arithmetic as HV.load_raw(), EXP950 pinned phys_base.
    phys_base = 0x850000000
    ram_base = phys_base & ~0xffffffff
    mem_top = u.ba.phys_base + u.ba.mem_size
    hv.add_tracer(irange(ram_base, u.ba.phys_base - ram_base), "RAM-LOW", 0)
    hv.add_tracer(irange(phys_base, u.ba.mem_size_actual - phys_base + ram_base), "RAM-HIGH", 0)
    hv.unmap_carveouts()
    policy = None
    if isolate:
        policy = hv.unmap_above_ram(mem_top, ram_base + u.ba.mem_size_actual)
    hv.pt_update()
    return hv, p, policy


def resolve(calls, page):
    """Last stage-2 write covering ``page``: ('hw', pa) or ('unmapped', None)."""
    state = None
    for ipa, pa, size, incr in calls:
        if ipa <= page < ipa + size:
            state = ("unmapped", None) if pa == 0 else ("hw", (pa & ~0x3fff & ((1 << 48) - 1))
                                                         + (page - ipa))
    return state


def window_calls(calls, mem_top, dram_end):
    return [c for c in calls if c[0] < dram_end and mem_top < c[0] + c[2]]


def test_hv_legacy_sequence_matches_exp950_receipt():
    fx = load_fixture()
    mem_top, dram_end = window(fx)
    _, p, _ = build_hv(fx, isolate=False)
    got = [(ipa, size, pa == 0) for ipa, pa, size, _ in window_calls(p.calls, mem_top, dram_end)]
    # EXP950 normal2-contract.bin checkpoint 3 (attributes 0 = unmap, 1 = HW).
    assert got == [
        (0x9df708000, 0x1c30000, True),
        (0x9e1338000, 0x5ae8000, False),
        (0x9e6e20000, 0x19000000, True),
        (0x9ffe20000, 0x1e0000, False),
    ], [(hex(a), hex(b), c) for a, b, c in got]


def test_hv_isolation_maps_only_kept_pages():
    fx = load_fixture()
    mem_top, dram_end = window(fx)
    _, legacy, _ = build_hv(fx, isolate=False)
    _, p, policy = build_hv(fx, isolate=True)
    for page in range(mem_top, dram_end, PAGE_SIZE):
        state = resolve(p.calls, page)
        if any(r.start <= page < r.end for r in policy.keep):
            assert state == resolve(legacy.calls, page) == ("hw", page), hex(page)
        else:
            assert state == ("unmapped", None), (hex(page), state)


def test_hv_isolation_leaves_guest_ram_unchanged():
    fx = load_fixture()
    mem_top, dram_end = window(fx)
    _, legacy, _ = build_hv(fx, isolate=False)
    _, p, _ = build_hv(fx, isolate=True)
    below = lambda calls: [c for c in calls if c[0] + c[2] <= mem_top]
    assert below(p.calls) == below(legacy.calls)
    assert below(p.calls), "guest RAM must still be mapped"


def test_hv_isolation_fits_launch_contract_receipt():
    fx = load_fixture()
    mem_top, dram_end = window(fx)
    _, legacy, _ = build_hv(fx, isolate=False)
    _, p, _ = build_hv(fx, isolate=True)
    extra = len(window_calls(p.calls, mem_top, dram_end)) - len(
        window_calls(legacy.calls, mem_top, dram_end))
    # EXP2196/EXP2194 GPU-profile receipts hold 51 of HV_STAGE2_STATE_MAX_MAPPINGS = 64;
    # an overflow makes capture_contract() abort the launch.
    assert 51 + extra <= 64, extra


def test_hv_default_is_legacy():
    src = (ROOT / "proxyclient/m1n1/hv/__init__.py").read_text()
    body = src[src.index("    def load_raw("):src.index("        print(f\"Loading kernel image")]
    assert 'os.environ.get("WOM1_HV_ISOLATE_ABOVE_RAM", "0") == "1"' in body
    assert body.index("self.unmap_carveouts()") < body.index("self.unmap_above_ram(")
    assert body.index('"RAM-HIGH", TraceMode.OFF)') < body.index("self.unmap_above_ram(")


def test_fixture_matches_saved_adt_when_available():
    fx = load_fixture()
    candidates = [Path(a) for a in sys.argv[1:] if a.endswith(".bin")] + [
        ROOT.parents[1] / ".worktrees/ans-native/.local/ans-native/ANSDART001-R3/adt.bin",
        ROOT.parent / ".worktrees/ans-native/.local/ans-native/ANSDART001-R3/adt.bin",
    ]
    for path in candidates:
        if path.is_file():
            assert extract_fixture(path) == fx, f"fixture drifted from {path}"
            return "checked"
    return "skipped"


def main():
    if len(sys.argv) == 3 and sys.argv[1] == "--regenerate":
        FIXTURE.write_text(json.dumps(extract_fixture(sys.argv[2]), indent=2) + "\n")
        print(f"wrote {FIXTURE}")
        return 0
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_")]
    for name, fn in tests:
        result = fn()
        print(f"ok {name}" + (f" ({result})" if isinstance(result, str) else ""))
    print(f"{len(tests)} tests passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())

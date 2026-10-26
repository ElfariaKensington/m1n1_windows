"""WHAT REAL BUG OR INVARIANT WILL THIS TEST CATCH?
RAM expansion must not move fixed Mu boot inputs or expose either host heap or
new ADT carveouts. Exercise the real loader; only hardware transport is fake.
"""
from copy import deepcopy
from types import SimpleNamespace as NS
import unittest
import gzip
import random
from unittest.mock import Mock

from proxyclient.m1n1.hv import HV
from proxyclient.m1n1.adt import ADTNode
from proxyclient.m1n1.malloc import Heap
from proxyclient.m1n1.tgtypes import BootArgs_r2
from proxyclient.m1n1.proxyutils import ProxyUtils


class GuestMemoryTests(unittest.TestCase):
    def loader(self):
        h = HV.__new__(HV)
        ba = BootArgs_r2.parse(bytes(BootArgs_r2.sizeof()))
        ba.revision = ba.version = 2
        ba.phys_base = 0x802078000
        ba.mem_size = 0x1dd690000
        ba.mem_size_actual = 0x200000000
        ba.devtree_size = 0x5c000
        a = ADTNode()
        for path in ('chosen', 'chosen/memory-map', 'chosen/carveout-memory-map', 'arm-io', 'cpus'):
            node = ADTNode()
            node.name = path.split('/')[-1]
            a[path] = node
        a['chosen/memory-map'].SEPFW = (0x806000000, 0x56c000)
        a['chosen/memory-map'].TrustCache = (0x805000000, 0x58000)
        a['chosen/carveout-memory-map']._properties['region-id-24'] = (ba.phys_base, ba.mem_size)
        h.u = NS(ba=ba, adt=a, heap=Heap(0x810d7c000, 0x814d7c000),
                 heap_top=0x850000000, compressed_writemem=Mock())
        h.adt = a
        h.tba = deepcopy(ba)
        h.p = Mock()
        h.p.heapblock_alloc.return_value = 0x808000000
        h.iface = Mock()
        h.add_tracer = Mock()
        h.unmap_carveouts = Mock()
        return h

    def test_expansion_preserves_boot_input_destinations(self):
        old, new = self.loader(), self.loader()
        payload = bytes(0x1e00000)
        old.load_raw(payload, entryoffset=0)
        # RED on the old loader: no way to expand RAM independently of FD.
        new.load_raw(payload, entryoffset=0, memory_base=0x818000000)
        self.assertEqual(new.guest_base, 0x8510b4000)
        self.assertEqual(new.adt_base, 0x851000000)
        self.assertEqual(new.tba.phys_base, 0x818000000)
        self.assertEqual(new.tba.mem_size, 0x1c7708000)
        self.assertEqual(new.tba.mem_size - old.tba.mem_size, 896 * 1024**2)
        self.assertEqual(new.tba.devtree - new.tba.virt_base + new.tba.phys_base, 0x851000000)
        self.assertEqual(new.u.compressed_writemem.call_args, old.u.compressed_writemem.call_args)
        self.assertEqual(new.p.memcpy8.call_args_list, old.p.memcpy8.call_args_list)
        self.assertEqual(new.tba.top_of_kernel_data, old.tba.top_of_kernel_data)
        self.assertEqual(new.iface.writemem.call_args.args[0], old.iface.writemem.call_args.args[0])
        uploaded = BootArgs_r2.parse(new.iface.writemem.call_args.args[1])
        self.assertEqual(uploaded.phys_base, 0x818000000)
        ram_high = new.add_tracer.call_args_list[1].args[0]
        self.assertEqual(ram_high.start, 0x818000000)
        new.unmap_carveouts.assert_called_once()

    def test_invalid_expansion_refuses_before_any_guest_write(self):
        for base in (0, 0x810000000, 0x818000001, 0x850004000, 1 << 64):
            with self.subTest(base=base):
                h = self.loader()
                with self.assertRaises(ValueError):
                    h.load_raw(b'test', memory_base=base)
                h.u.compressed_writemem.assert_not_called()
                h.add_tracer.assert_not_called()
                h.iface.writemem.assert_not_called()

    def test_newly_claimed_carveout_is_never_exposed(self):
        h = self.loader()
        h.u.adt['chosen/carveout-memory-map']._properties['region-id-999'] = (0x820000000, 0x4000)
        with self.assertRaisesRegex(ValueError, 'carveout'):
            h.load_raw(b'test', memory_base=0x818000000)
        h.u.compressed_writemem.assert_not_called()

    def test_runtime_heap_crossing_proxy_heap_refuses(self):
        h = self.loader()
        h.p.heapblock_alloc.return_value = 0x810d7c001
        with self.assertRaisesRegex(ValueError, 'heap'):
            h.load_raw(b'test', memory_base=0x818000000)
        h.u.compressed_writemem.assert_not_called()


    def test_compact_heap_uploads_incompressible_firmware_and_releases_scratch(self):
        # The fixed FD envelope is 30 MiB; gzip can be larger than its input.
        # Exercise real compression + allocation with no device I/O.
        payload = random.Random(30).randbytes(30 * 1024**2)
        u = ProxyUtils.__new__(ProxyUtils)
        u.heap = Heap(0x810d7c000, 0x814d7c000)
        u.heap.malloc(0x10000)  # persistent code buffer
        u.heap.malloc(512)      # persistent SIMD scratch
        writes = {}
        u.iface = NS(dev=NS(timeout=1),
                     writemem=lambda address, data, progress: writes.update({address: data}))
        def decompress(address, size, destination, capacity):
            self.assertEqual(destination, 0x8510b4000)
            self.assertEqual(capacity, len(payload))
            self.assertGreater(size, len(payload))
            self.assertEqual(gzip.decompress(writes[address]), payload)
            return len(payload)
        u.proxy = NS(gzdec=decompress)
        before = list(u.heap.blocks)
        u.compressed_writemem(0x8510b4000, payload)
        self.assertEqual(u.heap.blocks, before)
        self.assertEqual(u.iface.dev.timeout, 1)

    def test_invalid_platform_or_unconfirmed_carveout_map_refuses(self):
        for kind in ('overflow', 'short', 'missing-normal', 'malformed', 'negative'):
            with self.subTest(kind=kind):
                h = self.loader()
                props = h.u.adt['chosen/carveout-memory-map']._properties
                if kind == 'overflow':
                    h.u.ba.mem_size = 1 << 64
                elif kind == 'short':
                    h.u.ba.mem_size = 0x10000000
                elif kind == 'missing-normal':
                    del props['region-id-24']
                elif kind == 'malformed':
                    props['region-id-999'] = (1,)
                else:
                    props['region-id-999'] = (0x820000000, -0x4000)
                with self.assertRaises(ValueError):
                    h.load_raw(b'test', memory_base=0x818000000)
                h.u.compressed_writemem.assert_not_called()
                h.add_tracer.assert_not_called()


if __name__ == '__main__':
    unittest.main()

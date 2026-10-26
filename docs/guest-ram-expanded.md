# Expanded RAM for the assisted J313 Mu launch

The fixed firmware load anchor and the start of guest RAM are independent.
`HV.load_raw(..., memory_base=...)` preserves the image placement derived from
`ProxyUtils.heap_top`, but uses the validated memory base for boot arguments and
the high RAM stage-2 mapping. Omitting the argument preserves the original path.

The proposed launcher profile `--guest-ram-profile expanded` uses a 64 MiB Python
proxy heap and guest RAM base `0x818000000`. The image anchor remains
`0x850000000`, so the canonical Mu image still enters at `0x8510b4000`. The
existing 128 MiB native-heap allowance, low-memory alias, 384 MiB AGX reserve,
framebuffer, iBoot/TZ/DCP regions and platform RAM ceiling remain in place.
The new guest interval is identity mapped, including for GPU backing after the
broker validates the published RAM descriptor and actual stage-2 translations.

The launcher must call `expanded_ram_base(u, requested_base)` before publishing
the descriptor or initializing the hypervisor. Publish that result in BOTH the
boot RAM fields and the GUEST_RAM region; keep image/ADT/boot-input regions at
their old addresses. Pass the same result to `load_raw(memory_base=...)`, which
validates it again before any guest write. Never use the overridden `heap_top`
as the host-allocation floor: the real bounded `u.heap` owns that extent.

The validator refuses an overlapping heap, invalid platform bounds, unaligned
addresses, a different 4 GiB address band, missing normal-RAM confirmation, or
any non-normal ADT carveout intersecting the newly claimed interval. It does
not reclaim older platform reservations or change the autonomous RAM clamp.
The new profile must reject `--no-pin` and `M1N1HEAP` overrides. Native heap
high-water marks above the proxy heap base also abort the launch.

EXP1959's recorded layout yields +939,524,096 bytes (896 MiB). Applied to the
reported Windows total of 6,279,270,400 bytes, the expectation is approximately
7,218,794,496 bytes, subject to EFI/Windows page overhead. This is an offline
prediction, not a hardware-validated result. The full accounting and proposed
launcher/prepare/recovery patches are in the RC30 handoff REPORT.md.

Validate first with boot and RAM enumeration only, preserving a legacy-profile
lastgood launcher; only then run GravityMark D3D12 2k. A new Mach-O alone does
not select the profile: deploy matching Python proxyclient and launcher changes.

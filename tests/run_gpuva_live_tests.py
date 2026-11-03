#!/usr/bin/env python3
"""Compile real broker/wire in their normal sibling shared-header layout.

WHAT REAL BUG OR INVARIANT WILL THIS TEST CATCH?
Live additive publication cannot weaken legacy ownership or active-job guards.
AGX_SHARED_ROOT selects the matching public_windows tree for standalone worktrees.
"""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

parser = argparse.ArgumentParser()
parser.add_argument("--legacy-publication", action="store_true",
                    help="negative control: restore the old live publication refusal")
parser.add_argument("--unchecked-generation", action="store_true",
                    help="negative control: remove the generation overflow refusal")
args = parser.parse_args()
repo = Path(__file__).resolve().parents[1]
shared = Path(os.environ.get("AGX_SHARED_ROOT", str(repo.parent))) / "drivers/apple-agx/shared"
with tempfile.TemporaryDirectory(prefix="m1n1-gpuva-live-") as tmp:
    root = Path(tmp)
    target = root / "m1n1"
    shutil.copytree(repo / "src", target / "src")
    shutil.copytree(repo / "tests", target / "tests")
    (root / "drivers/apple-agx").mkdir(parents=True)
    (root / "drivers/apple-agx/shared").symlink_to(shared.resolve())
    # Execute the actual retained dispatch and its response epilogue. Only
    # platform-owned storage/power globals are supplied by the host fixture.
    platform = (repo / "src/hv_agx_retained_platform.c").read_text()
    dispatch = platform[platform.index("static void gpuva_execute("):
                        platform.index("static bool cpu_stopped(")]
    (target / "tests/retained_execute_fixture.h").write_text(
        "static bool request_powered = true;\n"
        "static bool gpuva_capacity_reported;\n"
        "static struct hv_agx_gpuva_v5_wire gpuva_v5_wire;\n"
        "#define gpuva_v5 (((struct fixture *)context)->broker)\n" +
        dispatch + "\n#undef gpuva_v5\n")
    if args.legacy_publication:
        source = target / "src/hv_agx_gpuva_v5.c"
        source.write_text(source.read_text().replace(
            "if (live && (!b->processes[owner].live_additive || old || !descriptor))",
            "if (live)"))
    if args.unchecked_generation:
        source = target / "src/hv_agx_gpuva_v5.c"
        source.write_text(source.read_text().replace(
            "    if (b->processes[owner].map_generation == UINT64_MAX)\n"
            "        return HV_AGX_GPUVA_V5_CAPACITY;\n", ""))
    legacy_wire = subprocess.run(["git", "show",
        "075e39804b17e8f1fa653e9cac8804c7a619ff80:src/hv_agx_gpuva_v5_mmio.c"],
        cwd=repo, check=True, capture_output=True, text=True).stdout
    (target / "src/hv_agx_gpuva_v5_legacy_mmio.c").write_text(legacy_wire)
    for name in ("hv_agx_gpuva_v5_test", "hv_agx_gpuva_v5_mmio_test", "hv_agx_gpuva_live_legacy_test", "hv_agx_gpuva_live_test"):
        sources = [target / "tests" / (name + ".c")]
        if name not in ("hv_agx_gpuva_v5_mmio_test", "hv_agx_gpuva_live_legacy_test"):
            sources += [target / "src/hv_agx_gpuva_v5.c", shared / "src/apple_agx_uat.c"]
        if name != "hv_agx_gpuva_v5_test":
            wire = "hv_agx_gpuva_v5_legacy_mmio.c" if name == "hv_agx_gpuva_live_legacy_test" else "hv_agx_gpuva_v5_mmio.c"
            sources += [target / "src" / wire]
        output = root / name
        subprocess.run([os.environ.get("CC", "clang"), "-std=c11", "-Wall", "-Wextra",
                        "-Werror", "-Wno-unused-function", "-ftrivial-auto-var-init=pattern",
                        "-fsanitize=address,undefined", "-I" + str(shared / "include"),
                        *map(str, sources), "-o", str(output)], check=True)
        subprocess.run([str(output)], check=True)
        print(name + " PASS", flush=True)

RUSTARCH ?= aarch64-unknown-none-softfloat

ifeq ($(shell uname),Darwin)
USE_CLANG ?= 1
$(info INFO: Building on Darwin)
BREW ?= $(shell command -v brew)
TOOLCHAIN ?= $(shell $(BREW) --prefix llvm)/bin/
ifeq ($(shell ls $(TOOLCHAIN)/ld.lld 2>/dev/null),)
LLDDIR ?= $(shell $(BREW) --prefix lld)/bin/
else
LLDDIR ?= $(TOOLCHAIN)
endif
$(info INFO: Toolchain path: $(TOOLCHAIN))
endif

ifeq ($(shell uname -m),aarch64)
ARCH ?=
else
ARCH ?= aarch64-linux-gnu-
endif

ifneq ($(TOOLCHAIN),$(LLDDIR))
$(info INFO: LLD path: $(LLDDIR))
endif

ifeq ($(USE_CLANG),1)
CC := $(TOOLCHAIN)clang --target=$(ARCH)
AS := $(TOOLCHAIN)clang --target=$(ARCH)
LD := $(LLDDIR)ld.lld
OBJCOPY := $(TOOLCHAIN)llvm-objcopy
CLANG_FORMAT ?= $(TOOLCHAIN)clang-format
EXTRA_CFLAGS ?=
else
CC := $(TOOLCHAIN)$(ARCH)gcc
AS := $(TOOLCHAIN)$(ARCH)gcc
LD := $(TOOLCHAIN)$(ARCH)ld
OBJCOPY := $(TOOLCHAIN)$(ARCH)objcopy
CLANG_FORMAT ?= clang-format
EXTRA_CFLAGS ?= -Wstack-usage=2048
endif

ifeq ($(V),)
QUIET := @
else
ifeq ($(V),0)
QUIET := @
else
QUIET :=
endif
endif

BASE_CFLAGS := -O2 -Wall -g -Wundef -Werror=strict-prototypes -fno-common -fno-PIE \
	-Werror=implicit-function-declaration -Werror=implicit-int \
	-Wsign-compare -Wunused-parameter -Wno-multichar \
	-ffreestanding -fpic -ffunction-sections -fdata-sections \
	-nostdinc -isystem $(shell $(CC) -print-file-name=include) -isystem sysinc \
	-fno-stack-protector -mstrict-align -march=armv8.2-a \
	$(EXTRA_CFLAGS)

CFLAGS := $(BASE_CFLAGS) -mgeneral-regs-only -I../drivers/apple-agx/shared/include

CFG :=
ifeq ($(J316S_SETUP),1)
CFG += M1N1_J316S_SETUP
endif
ifeq ($(RELEASE),1)
CFG += RELEASE
endif
ifeq ($(RUNTIME_DIAG_VERBOSE),1)
CFG += HV_RUNTIME_DIAG_VERBOSE
endif
ifeq ($(APPLE_INPUT),0)
CFG += HV_DISABLE_APPLE_INPUT
endif
ifeq ($(AGX_LOCAL_RESERVE_V2),1)
ifneq ($(IOMFB_FULL_OWNER),1)
$(error AGX_LOCAL_RESERVE_V2 requires IOMFB_FULL_OWNER=1)
endif
CFG += AGX_LOCAL_RESERVE_V2
CFLAGS += -DAGX_LOCAL_RESERVE_V2=
endif

ifeq ($(IOMFB_FULL_OWNER),1)
CFG += DCP_IOMFB_FULL_OWNER
endif
ifeq ($(EXP806_SCANOUT_PATTERN),1)
ifneq ($(IOMFB_FULL_OWNER),1)
$(error EXP806_SCANOUT_PATTERN requires IOMFB_FULL_OWNER=1)
endif
CFG += EXP806_SCANOUT_PATTERN
endif
ifeq ($(EXP807_SCANOUT_SNAPSHOT),1)
ifneq ($(IOMFB_FULL_OWNER),1)
$(error EXP807_SCANOUT_SNAPSHOT requires IOMFB_FULL_OWNER=1)
endif
CFG += EXP807_SCANOUT_SNAPSHOT
endif
ifeq ($(EXP808_SCANOUT_DELAYED),1)
ifneq ($(IOMFB_FULL_OWNER),1)
$(error EXP808_SCANOUT_DELAYED requires IOMFB_FULL_OWNER=1)
endif
CFG += EXP808_SCANOUT_DELAYED
endif
ifeq ($(EXP890_SCANOUT_WINDOW),1)
ifneq ($(IOMFB_FULL_OWNER),1)
$(error EXP890_SCANOUT_WINDOW requires IOMFB_FULL_OWNER=1)
endif
ifneq ($(EXP808_SCANOUT_DELAYED),1)
$(error EXP890_SCANOUT_WINDOW requires EXP808_SCANOUT_DELAYED=1)
endif
CFG += EXP890_SCANOUT_WINDOW
endif
ifeq ($(IOMFB_START_OBSERVER),1)
CFG += DCP_IOMFB_START_OBSERVER
endif
ifeq ($(IOMFB_EARLY_PIODMA_OBSERVER),1)
CFG += DCP_IOMFB_EARLY_PIODMA_OBSERVER
endif
ifeq ($(IOMFB_SET_SHMEM_OBSERVER),1)
CFG += DCP_IOMFB_SET_SHMEM_OBSERVER
endif
ifeq ($(IOMFB_A401_OBSERVER),1)
CFG += DCP_IOMFB_A401_OBSERVER
endif
ifeq ($(IOMFB_A426_OBSERVER),1)
CFG += DCP_IOMFB_A426_OBSERVER
endif
ifeq ($(IOMFB_A449_OBSERVER),1)
CFG += DCP_IOMFB_A449_OBSERVER
endif
ifeq ($(IOMFB_A456_OBSERVER),1)
CFG += DCP_IOMFB_A456_OBSERVER
endif
ifeq ($(IOMFB_A411_OBSERVER),1)
CFG += DCP_IOMFB_A411_OBSERVER
endif
ifeq ($(IOMFB_A472_OBSERVER),1)
CFG += DCP_IOMFB_A472_OBSERVER
endif
ifeq ($(IOMFB_A410_OBSERVER),1)
CFG += DCP_IOMFB_A410_OBSERVER
endif
ifeq ($(IOMFB_A412_OBSERVER),1)
CFG += DCP_IOMFB_A412_OBSERVER
endif
ifeq ($(IOMFB_SWAP_OBSERVER),1)
CFG += DCP_IOMFB_SWAP_OBSERVER
endif
ifeq ($(IOMFB_START_OBSERVER),1)
ifneq ($(IOMFB_FULL_OWNER),1)
$(error IOMFB_START_OBSERVER requires IOMFB_FULL_OWNER=1)
endif
endif
ifeq ($(IOMFB_EARLY_PIODMA_OBSERVER),1)
ifneq ($(IOMFB_FULL_OWNER),1)
$(error IOMFB_EARLY_PIODMA_OBSERVER requires IOMFB_FULL_OWNER=1)
endif
endif
ifeq ($(IOMFB_SET_SHMEM_OBSERVER),1)
ifneq ($(IOMFB_FULL_OWNER),1)
$(error IOMFB_SET_SHMEM_OBSERVER requires IOMFB_FULL_OWNER=1)
endif
endif
ifeq ($(IOMFB_A401_OBSERVER),1)
ifneq ($(IOMFB_FULL_OWNER),1)
$(error IOMFB_A401_OBSERVER requires IOMFB_FULL_OWNER=1)
endif
endif
ifeq ($(IOMFB_A426_OBSERVER),1)
ifneq ($(IOMFB_FULL_OWNER),1)
$(error IOMFB_A426_OBSERVER requires IOMFB_FULL_OWNER=1)
endif
endif
ifeq ($(IOMFB_A449_OBSERVER),1)
ifneq ($(IOMFB_FULL_OWNER),1)
$(error IOMFB_A449_OBSERVER requires IOMFB_FULL_OWNER=1)
endif
endif
ifeq ($(IOMFB_A456_OBSERVER),1)
ifneq ($(IOMFB_FULL_OWNER),1)
$(error IOMFB_A456_OBSERVER requires IOMFB_FULL_OWNER=1)
endif
endif
ifeq ($(IOMFB_A411_OBSERVER),1)
ifneq ($(IOMFB_FULL_OWNER),1)
$(error IOMFB_A411_OBSERVER requires IOMFB_FULL_OWNER=1)
endif
endif
ifeq ($(IOMFB_A472_OBSERVER),1)
ifneq ($(IOMFB_FULL_OWNER),1)
$(error IOMFB_A472_OBSERVER requires IOMFB_FULL_OWNER=1)
endif
endif
ifeq ($(IOMFB_A410_OBSERVER),1)
ifneq ($(IOMFB_FULL_OWNER),1)
$(error IOMFB_A410_OBSERVER requires IOMFB_FULL_OWNER=1)
endif
endif
ifeq ($(IOMFB_A412_OBSERVER),1)
ifneq ($(IOMFB_FULL_OWNER),1)
$(error IOMFB_A412_OBSERVER requires IOMFB_FULL_OWNER=1)
endif
endif
ifeq ($(IOMFB_SWAP_OBSERVER),1)
ifneq ($(IOMFB_FULL_OWNER),1)
$(error IOMFB_SWAP_OBSERVER requires IOMFB_FULL_OWNER=1)
endif
ifneq ($(filter 1,$(IOMFB_START_OBSERVER) $(IOMFB_EARLY_PIODMA_OBSERVER) $(IOMFB_SET_SHMEM_OBSERVER) $(IOMFB_A401_OBSERVER) $(IOMFB_A426_OBSERVER) $(IOMFB_A449_OBSERVER) $(IOMFB_A456_OBSERVER) $(IOMFB_A411_OBSERVER) $(IOMFB_A472_OBSERVER) $(IOMFB_A410_OBSERVER) $(IOMFB_A412_OBSERVER)),)
$(error IOMFB_SWAP_OBSERVER is mutually exclusive with admission observers)
endif
endif
ifeq ($(IOMFB_START_OBSERVER)$(IOMFB_EARLY_PIODMA_OBSERVER),11)
$(error IOMFB_START_OBSERVER and IOMFB_EARLY_PIODMA_OBSERVER are mutually exclusive)
endif
ifeq ($(IOMFB_START_OBSERVER)$(IOMFB_SET_SHMEM_OBSERVER),11)
$(error IOMFB_START_OBSERVER and IOMFB_SET_SHMEM_OBSERVER are mutually exclusive)
endif
ifeq ($(IOMFB_EARLY_PIODMA_OBSERVER)$(IOMFB_SET_SHMEM_OBSERVER),11)
$(error IOMFB_EARLY_PIODMA_OBSERVER and IOMFB_SET_SHMEM_OBSERVER are mutually exclusive)
endif
ifneq ($(filter 11,$(IOMFB_START_OBSERVER)$(IOMFB_A401_OBSERVER) $(IOMFB_EARLY_PIODMA_OBSERVER)$(IOMFB_A401_OBSERVER) $(IOMFB_SET_SHMEM_OBSERVER)$(IOMFB_A401_OBSERVER)),)
$(error IOMFB_A401_OBSERVER is mutually exclusive with earlier IOMFB observers)
endif
ifneq ($(filter 11,$(IOMFB_START_OBSERVER)$(IOMFB_A426_OBSERVER) $(IOMFB_EARLY_PIODMA_OBSERVER)$(IOMFB_A426_OBSERVER) $(IOMFB_SET_SHMEM_OBSERVER)$(IOMFB_A426_OBSERVER) $(IOMFB_A401_OBSERVER)$(IOMFB_A426_OBSERVER)),)
$(error IOMFB_A426_OBSERVER is mutually exclusive with earlier IOMFB observers)
endif
ifneq ($(filter 11,$(IOMFB_START_OBSERVER)$(IOMFB_A449_OBSERVER) $(IOMFB_EARLY_PIODMA_OBSERVER)$(IOMFB_A449_OBSERVER) $(IOMFB_SET_SHMEM_OBSERVER)$(IOMFB_A449_OBSERVER) $(IOMFB_A401_OBSERVER)$(IOMFB_A449_OBSERVER) $(IOMFB_A426_OBSERVER)$(IOMFB_A449_OBSERVER)),)
$(error IOMFB_A449_OBSERVER is mutually exclusive with earlier IOMFB observers)
endif
ifneq ($(filter 11,$(IOMFB_START_OBSERVER)$(IOMFB_A456_OBSERVER) $(IOMFB_EARLY_PIODMA_OBSERVER)$(IOMFB_A456_OBSERVER) $(IOMFB_SET_SHMEM_OBSERVER)$(IOMFB_A456_OBSERVER) $(IOMFB_A401_OBSERVER)$(IOMFB_A456_OBSERVER) $(IOMFB_A426_OBSERVER)$(IOMFB_A456_OBSERVER) $(IOMFB_A449_OBSERVER)$(IOMFB_A456_OBSERVER)),)
$(error IOMFB_A456_OBSERVER is mutually exclusive with earlier IOMFB observers)
endif
ifneq ($(filter 11,$(IOMFB_START_OBSERVER)$(IOMFB_A411_OBSERVER) $(IOMFB_EARLY_PIODMA_OBSERVER)$(IOMFB_A411_OBSERVER) $(IOMFB_SET_SHMEM_OBSERVER)$(IOMFB_A411_OBSERVER) $(IOMFB_A401_OBSERVER)$(IOMFB_A411_OBSERVER) $(IOMFB_A426_OBSERVER)$(IOMFB_A411_OBSERVER) $(IOMFB_A449_OBSERVER)$(IOMFB_A411_OBSERVER) $(IOMFB_A456_OBSERVER)$(IOMFB_A411_OBSERVER)),)
$(error IOMFB_A411_OBSERVER is mutually exclusive with earlier IOMFB observers)
endif
ifneq ($(filter 11,$(IOMFB_START_OBSERVER)$(IOMFB_A472_OBSERVER) $(IOMFB_EARLY_PIODMA_OBSERVER)$(IOMFB_A472_OBSERVER) $(IOMFB_SET_SHMEM_OBSERVER)$(IOMFB_A472_OBSERVER) $(IOMFB_A401_OBSERVER)$(IOMFB_A472_OBSERVER) $(IOMFB_A426_OBSERVER)$(IOMFB_A472_OBSERVER) $(IOMFB_A449_OBSERVER)$(IOMFB_A472_OBSERVER) $(IOMFB_A456_OBSERVER)$(IOMFB_A472_OBSERVER) $(IOMFB_A411_OBSERVER)$(IOMFB_A472_OBSERVER)),)
$(error IOMFB_A472_OBSERVER is mutually exclusive with earlier IOMFB observers)
endif
ifneq ($(filter 11,$(IOMFB_START_OBSERVER)$(IOMFB_A410_OBSERVER) $(IOMFB_EARLY_PIODMA_OBSERVER)$(IOMFB_A410_OBSERVER) $(IOMFB_SET_SHMEM_OBSERVER)$(IOMFB_A410_OBSERVER) $(IOMFB_A401_OBSERVER)$(IOMFB_A410_OBSERVER) $(IOMFB_A426_OBSERVER)$(IOMFB_A410_OBSERVER) $(IOMFB_A449_OBSERVER)$(IOMFB_A410_OBSERVER) $(IOMFB_A456_OBSERVER)$(IOMFB_A410_OBSERVER) $(IOMFB_A411_OBSERVER)$(IOMFB_A410_OBSERVER) $(IOMFB_A472_OBSERVER)$(IOMFB_A410_OBSERVER)),)
$(error IOMFB_A410_OBSERVER is mutually exclusive with earlier IOMFB observers)
endif
ifneq ($(filter 11,$(IOMFB_START_OBSERVER)$(IOMFB_A412_OBSERVER) $(IOMFB_EARLY_PIODMA_OBSERVER)$(IOMFB_A412_OBSERVER) $(IOMFB_SET_SHMEM_OBSERVER)$(IOMFB_A412_OBSERVER) $(IOMFB_A401_OBSERVER)$(IOMFB_A412_OBSERVER) $(IOMFB_A426_OBSERVER)$(IOMFB_A412_OBSERVER) $(IOMFB_A449_OBSERVER)$(IOMFB_A412_OBSERVER) $(IOMFB_A456_OBSERVER)$(IOMFB_A412_OBSERVER) $(IOMFB_A411_OBSERVER)$(IOMFB_A412_OBSERVER) $(IOMFB_A472_OBSERVER)$(IOMFB_A412_OBSERVER) $(IOMFB_A410_OBSERVER)$(IOMFB_A412_OBSERVER)),)
$(error IOMFB_A412_OBSERVER is mutually exclusive with earlier IOMFB observers)
endif
ifeq ($(IOMFB_LATCH_OBSERVER),1)
CFG += DCP_IOMFB_LATCH_OBSERVER
endif
ifeq ($(IOMFB_FULL_OWNER)$(IOMFB_LATCH_OBSERVER),11)
$(error IOMFB_FULL_OWNER and IOMFB_LATCH_OBSERVER are mutually exclusive)
endif

# Required for no_std + alloc for now
export RUSTC_BOOTSTRAP=1
RUST_LIB := librust.a
ifeq ($(CHAINLOADING),1)
CFG += CHAINLOADING
endif

ifeq ($(BUILDSTD),1)
CARGO_FLAGS := -Z build-std=alloc,core
else
CARGO_FLAGS :=
endif

LDFLAGS := -EL -maarch64elf --no-undefined -X -Bsymbolic \
	-z notext --no-apply-dynamic-relocs --orphan-handling=warn \
	-z nocopyreloc --gc-sections -pie

MINILZLIB_OBJECTS := $(patsubst %,minilzlib/%, \
	dictbuf.o inputbuf.o lzma2dec.o lzmadec.o rangedec.o xzstream.o)

TINF_OBJECTS := $(patsubst %,tinf/%, \
	adler32.o crc32.o tinfgzip.o tinflate.o tinfzlib.o)

DLMALLOC_OBJECTS := dlmalloc/malloc.o

LIBFDT_OBJECTS := $(patsubst %,libfdt/%, \
	fdt_addresses.o fdt_empty_tree.o fdt_ro.o fdt_rw.o fdt_strerror.o fdt_sw.o \
	fdt_wip.o fdt.o)

DCP_OBJECTS := $(patsubst %,dcp/%, \
	dpav_ep.o \
	dptx_phy.o \
	dptx_port_ep.o \
	parser.o \
	system_ep.o)

OBJECTS := \
	adt.o \
	afk.o \
	afk_raw_router.o \
	afk_command.o \
	afk_command_owner.o \
	afk_deferred_message.o \
	aic.o \
	asc.o \
	asc_tx_gate.o \
	bootlogo_48.o bootlogo_128.o bootlogo_256.o \
	boot_options.o \
	chainload.o \
	chainload_layout.o \
	chainload_asm.o \
	chickens.o \
	chickens_avalanche.o \
	chickens_blizzard.o \
	chickens_cyclone_typhoon.o \
	chickens_everest.o \
	chickens_firestorm.o \
	chickens_hurricane_zephyr.o \
	chickens_monsoon_mistral.o \
	chickens_icestorm.o \
	chickens_sawtooth.o \
	chickens_twister.o \
	clk.o \
	cpufreq.o \
	cpufreq_state.o \
	dapf.o \
	dart.o \
	dcp.o \
	dcp_endpoint_owner.o \
	dcp_iboot.o \
	dcp_iomfb_clock.o \
	dcp_iomfb_latch.o \
	dcp_iomfb_mode_select.o \
	dcp_iomfb_rpc.o \
	dcp_iomfb_v13_5_abi.o \
	dcp_iomfb_bootstrap.o \
	dcp_iomfb_properties.o \
	dcp_iomfb_present.o \
	dcp_iomfb_resources.o \
	dcp_iomfb_transport.o \
	devicetree.o \
	display.o \
	display_dcp_frontend.o \
	display_guest.o \
	exception.o exception_asm.o \
	fb.o font.o font_retina.o \
	firmware.o \
	gxf.o gxf_asm.o \
	heapblock.o \
	hv.o hv_vm.o hv_exc.o hv_guest_ipa_pa.o hv_fiq_fast_path.o hv_vuart.o hv_pl011.o hv_pci.o hv_nvme.o hv_nvme_fast_path.o hv_nvme_queue.o hv_fb_stream.o hv_diag.o hv_irq_routes.o hv_apple_input.o hv_agx_g2_policy.o hv_agx_local_reserve.o hv_agx_pool_window.o hv_agx_config_snapshot.o hv_agx_firmware_prefix.o hv_agx_retained_root.o hv_agx_retained_tables.o hv_agx_retained_mmio.o hv_agx_gpuva_v5.o hv_agx_gpuva_v5_mmio.o hv_agx_retained_platform.o hv_agx_retained_backing.o hv_agx_power_broker.o hv_agx_scanout_broker.o hv_agx_scanout_service.o hv_agx_power_platform.o hv_agx_power_mmio.o hv_sgi_diag.o hv_sgi_pending.o hv_xhci_handoff.o hv_bootstrap.o hv_bootstrap_manifest.o hv_autonomous_manifest.o hv_autonomous_memory.o hv_autonomous.o hv_autonomous_runtime.o hv_autonomous_boot.o hv_autonomous_boot_runtime.o hv_wdt.o hv_asm.o hv_aic.o hv_virtio.o hv_psci.o hv_vgic.o hv_vgic_diag.o hv_vgic_redist.o \
	hv_tick_policy.o \
	hv_autonomous_profile.o hv_assisted_layout.o hv_launch_golden_j313.o \
	i2c.o \
	iodev.o \
	iova.o \
	iova_aligned_fit.o \
	isp.o \
	kboot.o kboot_atc.o \
	main.o \
	mitigations.o \
	mcc.o \
	memory.o memory_asm.o \
	nvme.o \
	payload.o \
	pcie.o \
	pmgr.o \
	proxy.o \
	proxy_boot_identity.o \
	ringbuffer.o \
	rtkit.o \
	rtkit_endpoint_map.o \
	rtkit_deferred.o \
	sart.o \
	sep.o \
	sio.o \
	smc.o \
	smp.o \
	start.o \
	startup.o \
	string.o \
	tunables.o tunables_static.o \
	tps6598x.o \
	uart.o \
	uartproxy.o uartproxy_event.o \
	usb.o usb_dwc3.o usb_dwc3_bulk_state.o \
	utils.o utils_asm.o \
	vsprintf.o \
	wdt.o \
	hv_guest_cpu_state.o \
	hv_launch_contract.o \
	hv_launch_j313.o hv_launch_transport.o hv_pci_state.o hv_stage2_state.o hv_stage_role.o \
	hv_launch_snapshot.o hv_launch_preflight.o hv_watchdog_snapshot.o \
	$(DCP_OBJECTS) \
	$(MINILZLIB_OBJECTS) $(TINF_OBJECTS) $(DLMALLOC_OBJECTS) $(LIBFDT_OBJECTS) $(RUST_LIB)

FP_OBJECTS := \
	kboot_gpu.o \
	math/expf.o \
	math/exp2f_data.o \
	math/powf.o \
	math/powf_data.o

ifeq ($(J316S_SETUP),1)
OBJECTS += hv_j316s_setup.o
endif

BUILD_OBJS := $(patsubst %,build/%,$(OBJECTS))
BUILD_FP_OBJS := $(patsubst %,build/%,$(FP_OBJECTS))
BUILD_ALL_OBJS := $(BUILD_OBJS) $(BUILD_FP_OBJS)
NAME := m1n1
TARGET := m1n1.macho
TARGET_RAW := m1n1.bin

DEPDIR := build/.deps

.PHONY: all clean format host-tests invoke_cc always_rebuild FORCE
all: build/$(TARGET) build/$(TARGET_RAW)
host-tests:
	./tests/run_host_tests.sh
clean:
	rm -rf build/* build/.deps
format:
	$(CLANG_FORMAT) -i src/*.c src/dcp/*.c src/math/*.c src/*.h src/dcp/*.h src/math/*.h sysinc/*.h
format-check:
	$(CLANG_FORMAT) --dry-run --Werror src/*.c src/dcp/*.c src/math/*.c src/*.h src/dcp/*.h src/math/*.h sysinc/*.h
rustfmt:
	cd rust && cargo fmt
rustfmt-check:
	cd rust && cargo fmt --check

build/$(RUST_LIB): rust/src/* rust/*
	$(QUIET)echo "  RS    $@"
	$(QUIET)mkdir -p $(DEPDIR)
	$(QUIET)mkdir -p "$(dir $@)"
	$(QUIET)cargo build $(CARGO_FLAGS) --target $(RUSTARCH) --lib --release --manifest-path rust/Cargo.toml --target-dir build
	$(QUIET)cp "build/$(RUSTARCH)/release/${RUST_LIB}" "$@"

build/%.o: src/%.S
	$(QUIET)echo "  AS    $@"
	$(QUIET)mkdir -p $(DEPDIR)
	$(QUIET)mkdir -p "$(dir $@)"
	$(QUIET)$(AS) -c $(BASE_CFLAGS) -MMD -MF $(DEPDIR)/$(*F).d -MQ "$@" -MP -o $@ $<

$(BUILD_FP_OBJS): build/%.o: src/%.c
	$(QUIET)echo "  CC FP $@"
	$(QUIET)mkdir -p $(DEPDIR)
	$(QUIET)mkdir -p "$(dir $@)"
	$(QUIET)$(CC) -c $(BASE_CFLAGS) -MMD -MF $(DEPDIR)/$(*F).d -MQ "$@" -MP -o $@ $<

build/%.o: src/%.c build/build_tag.h build/build_cfg.h
	$(QUIET)echo "  CC    $@"
	$(QUIET)mkdir -p $(DEPDIR)
	$(QUIET)mkdir -p "$(dir $@)"
	$(QUIET)$(CC) -c $(CFLAGS) -MMD -MF $(DEPDIR)/$(*F).d -MQ "$@" -MP -o $@ $<

# special target for usage by m1n1.loadobjs
invoke_cc:
	$(QUIET)$(CC) -c $(CFLAGS) -Isrc -o $(OBJFILE) $(CFILE)

build/$(NAME).elf: $(BUILD_ALL_OBJS) m1n1.ld
	$(QUIET)echo "  LD    $@"
	$(QUIET)$(LD) -T m1n1.ld $(LDFLAGS) -o $@ $(BUILD_ALL_OBJS)

build/$(NAME)-raw.elf: $(BUILD_ALL_OBJS) m1n1-raw.ld
	$(QUIET)echo "  LDRAW $@"
	$(QUIET)$(LD) -T m1n1-raw.ld $(LDFLAGS) -o $@ $(BUILD_ALL_OBJS)

build/$(NAME).macho: build/$(NAME).elf
	$(QUIET)echo "  MACHO $@"
	$(QUIET)$(OBJCOPY) -O binary --strip-debug $< $@

ifeq ($(LOGO),)
build/$(NAME).bin: build/$(NAME)-raw.elf
	$(QUIET)echo "  RAW   $@"
	$(QUIET)$(OBJCOPY) -O binary --strip-debug $< $@

else
build/$(NAME)-asahi.bin: build/$(NAME)-raw.elf
	$(QUIET)echo "  RAW   $@"
	$(QUIET)$(OBJCOPY) -O binary --strip-debug $< $@

build/$(NAME).bin: build/$(NAME)-asahi.bin build/$(LOGO).logo
	$(QUIET)echo "  RAW   $@"
	$(QUIET)cat $^ > $@
endif

FORCE:

# These generated headers depend on command-line build options and the Git state,
# neither of which make can represent as an ordinary file prerequisite.  Run the
# cheap compare on every invocation; only replace the header when its contents
# changed so dependent objects are rebuilt exactly when required.  Do not use GNU
# make grouped targets (`&:`): the default make shipped by macOS parses them as an
# unrelated target named `&`, which previously allowed stale diagnostic objects to
# leak into release images.
build/build_tag.h: FORCE
	$(QUIET)mkdir -p build
	$(QUIET)./version.sh > build/build_tag.tmp
	$(QUIET)cmp -s build/build_tag.h build/build_tag.tmp 2>/dev/null || \
	( mv -f build/build_tag.tmp build/build_tag.h && echo "  TAG   build/build_tag.h" )
	$(QUIET)rm -f build/build_tag.tmp

build/build_cfg.h: FORCE
	$(QUIET)mkdir -p build
	$(QUIET)for i in $(CFG); do echo "#define $$i"; done > build/build_cfg.tmp
	$(QUIET)cmp -s build/build_cfg.h build/build_cfg.tmp 2>/dev/null || \
	( mv -f build/build_cfg.tmp build/build_cfg.h && echo "  CFG   build/build_cfg.h" )
	$(QUIET)rm -f build/build_cfg.tmp

build/%.bin: data/%.bin
	$(QUIET)echo "  IMG   $@"
	$(QUIET)mkdir -p "$(dir $@)"
	$(QUIET)cp $< $@

build/%.o: build/%.bin
	$(QUIET)echo "  BIN   $@"
	$(QUIET)mkdir -p "$(dir $@)"
	$(QUIET)$(OBJCOPY) -I binary -B aarch64 -O elf64-littleaarch64 $< $@

build/%.bin: font/%.bin
	$(QUIET)echo "  CP    $@"
	$(QUIET)mkdir -p "$(dir $@)"
	$(QUIET)cp $< $@

build/%.rgba: data/%.png
	$(eval SIZE := $(lastword $(subst _, ,$*)))
	$(QUIET)echo "  MAGIC $@"
	$(QUIET)mkdir -p "$(dir $@)"
	$(QUIET)magick $< -background black -flatten -depth 8 -crop $(SIZE)x$(SIZE) -resize $(SIZE)x$(SIZE) rgba:$@

build/%.logo: build/%_256.rgba build/%_128.rgba
	$(QUIET)echo "  PAYLOAD $@"
	$(QUIET)mkdir -p "$(dir $@)"
	$(QUIET)echo -n "m1n1_logo_256128" > $@
	$(QUIET)cat $^ >> $@

-include $(DEPDIR)/*

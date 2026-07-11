# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
#
# Makefile — build and run the BPS-V VLIW simulation tests.
#
# Targets:
#   make build   — compile the Verilator simulation binary (Vvliw)
#   make asm     — assemble all .asm tests in tests/
#   make test    — run all assembled tests
#   make all     — asm + build + test
#   make clean   — remove generated files
#
# Configurable: WIDTH (default 4 for the smoke tests).

ROOT     := $(CURDIR)/../..
RTL_DIR  := $(ROOT)/rtl
VENDOR   := $(ROOT)/vendor/lowrisc_ip
WIDTH    ?= 4
TOP       = ibex_vliw_dispatch
TB        = tb_vliw
SIM      := V$(TOP)

# Verilator warnings to suppress (ibex-wide set + the VLIW additions).
VWARN := -Wno-WIDTH -Wno-UNUSEDPARAM -Wno-UNOPTFLAT -Wno-DECLFILENAME \
         -Wno-ASCRANGE -Wno-MULTIDRIVEN -Wno-UNUSED -Wno-PINCONNECTEMPTY \
         -Wno-PINMISSING -Wno-UNDRIVEN -Wno-CASEINCOMPLETE -Wno-TIMESCALEMOD \
         -Wno-HIERBLOCK -Wno-INITIALDLY -Wno-UNOPT

VINC := -I$(VENDOR)/ip/prim/rtl -I$(VENDOR)/ip/prim_generic/rtl -I$(VENDOR)/dv/sv/dv_utils

# Core RTL files (exclude tracer/fpv/top_tracing which pull extra deps).
RTL_ALL := $(filter-out %tracer% %fpv% %top_tracing% %ibex_pkg.sv %ibex_bps_pkg.sv, \
           $(wildcard $(RTL_DIR)/*.sv))
# Packages must come first.
PKG_SRCS := $(RTL_DIR)/ibex_pkg.sv $(RTL_DIR)/ibex_bps_pkg.sv

# Prim packages + RAM primitives the VLIW core needs.
PRIM_PKGS := $(wildcard $(VENDOR)/ip/prim_generic/rtl/*_pkg.sv) \
             $(VENDOR)/ip/prim/rtl/prim_cipher_pkg.sv
PRIM_RAMS := $(VENDOR)/ip/prim_generic/rtl/prim_ram_2p.sv

# Assembler
ASM := $(ROOT)/util/bundle_asm

# Tests
TESTS := $(basename $(notdir $(wildcard tests/*.asm)))

.PHONY: all build asm test clean

all: asm build test

# ---- Assembler ----
$(ASM): $(ROOT)/util/bundle_asm.c
	cc -O2 -Wall -o $@ $<

asm: $(ASM)
	@for t in $(TESTS); do \
		echo "[asm] $$t"; \
		$(ASM) $(WIDTH) tests/$$t.asm tests/$$t || exit 1; \
	done

# ---- Verilator build ----
build:
	verilator --cc --exe --build --trace \
	  --top-module $(TOP) \
	  -Wno-UNSIGNED $(VWARN) $(VINC) \
	  -GSuperscalarWidth=$(WIDTH) \
	  $(PKG_SRCS) $(PRIM_PKGS) $(PRIM_RAMS) $(RTL_ALL) \
	  $(TB).cpp
	@echo "built obj_dir/$(SIM)"

# ---- Run tests ----
test: build asm
	@fail=0; pass=0; \
	for t in $(TESTS); do \
		echo "=== $$t ==="; \
		./obj_dir/$(SIM) tests/$$t.img tests/$$t.expect $(WIDTH) 20000 && pass=$$((pass+1)) || fail=$$((fail+1)); \
	done; \
	echo "------------------------"; \
	echo "PASS=$$pass FAIL=$$fail"; \
	[ $$fail -eq 0 ]

clean:
	rm -rf obj_dir
	rm -f tests/*.img tests/*.expect

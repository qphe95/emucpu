# emucpu

**A dynarec-driven, statically-scheduled VLIW RISC-V core with a banked address register file. No branch predictor. No mispredicts. No apologies.**

Built on [Ibex](https://github.com/lowRISC/ibex). Targeting the Pango Titan2 PG2T390H (AXP390 board).

---

## Branch predictors are a scam

Every modern "wide" CPU sold to you as innovation is the same story: an
ever-deeper pipeline fronted by an ever-more-elaborate guess-and-check engine
that bets your cycles on statistically predicting the future, then flushes
them when it's wrong — which it is, 5–15% of the time, on any code that isn't a
straight line. You pay for the predictor in silicon area, in power, in design
complexity, in Spectre-class security holes, and in the hundreds of cycles you
eat every time the crystal ball cracks. And the industry calls this *progress*.

**emucpu has no branch predictor because it has no speculation.**

The dynamic recompiler (dynarec) owns all scheduling and control flow. It lays
out instruction bundles — up to `W` independent operations per cycle — and the
core has a **program counter** that advances sequentially or redirects on a
**resolved** branch. No predication. No speculation. Nothing guesses.

The trick: the dynarec watches which way branches actually go at runtime and
**fuses** or **unfuses** code regions accordingly. A branch that consistently
falls through gets its fall-through code fused into the same bundle stream —
the branch resolves not-taken, the PC advances, zero redirect cost. A branch
that goes taken gets unfused: the dynarec splits at the branch, the taken
target becomes the next PC. This is the same trace-formation feedback loop
that HP Dynamo and Apple Rosetta use — applied to VLIW bundle packing.

There is nothing to mispredict because nothing predicts. The "mispredict
penalty" row in the architecture spec is empty, because the concept does not
exist.

This isn't a hack. It's the *original* VLIW promise — the one the industry
abandoned when it decided you, the programmer, were too stupid to schedule
code and a silicon fortune-teller should do it for you. We brought it back.

---

## What's in the box

| Feature | What it does |
|---|---|
| **Banked address register file (ARF)** | 128K×32 of address storage across 64 banks. Pins hot addresses into registers so `ldp`/`stp` skip the ALU address-compute cycle. |
| **`ldp.next`** | `S[si] ← MEM[S[si]]` — a pointer chase in one instruction, self-contained in the ARF, no GPR touched. |
| **Hot-address detector** | A hardware frequency counter that snoops the data bus and *recommends* (never forces) pinning runtime-hot addresses. The dynarec commits the pins. |
| **W-lane VLIW dispatch** | `W` lanes execute one bundle per cycle in lockstep. No rename, no ROB, no OoO issue, no wakeup CAM — the dynarec already ordered everything. |
| **Dynarec fuse/unfuse** | The dynarec fuses fall-through paths into bundles (branch resolves not-taken = zero redirect cost) and splits at taken branches. Adapts to observed branch direction at runtime. |
| **Banked LSU** | 64-bank data memory gives W parallel load/store paths. External-fallback path routes MMIO to the core's external bus. |
| **`wj*` instructions** | Wait-jump-on-MMIO-write: `wjeq pK, si, rs2, P` — block a lane until a device writes, then redirect. No polling, no branch, no wasted lane. |

Full architecture spec: **[DESIGN.md](DESIGN.md)**. Dynarec scheduling algorithms: **[DYNAREC.md](DYNAREC.md)**.

---

## Custom instruction set (for emulation/dynarec)

All custom ops use the RISC-V reserved `CUSTOM-0` (`0x0B`) and `CUSTOM-1` (`0x2B`)
opcodes, gated by the `AddrRegFile` parameter. They decode to `illegal_insn`
when the feature is off — zero overhead, no pollution of the base ISA.

### Pinning addresses to the ARF (the dynarec's bread and butter)

```
pina   rs1, rs2        # S[rs1] ← rs2 ; mark PINNED (long-lived)
unpin  rs1              # return PINNED slot to free pool
spalloc rd              # rd ← free slot index (0x1FFFF if full)
spfree rs1              # return slot to free pool
```

### Dereferenced memory access (skip the ALU address-compute)

```
ldp    rd, rs1          # rd  ← MEM[S[rs1]]       (full 128K index)
stp    rs2, rs1         # MEM[S[rs1]] ← rs2
ldpi   rd, imm12        # rd  ← MEM[S[imm12]]     (low 4K, static table)
stpi   rs2, imm12       # MEM[S[imm12]] ← rs2
ldp.next si             # S[si] ← MEM[S[si]]      ; p = *p, no GPR
ldpcap rd, rs1          # rd ← MEM[S[rs1]] ; S[rs1] ← MEM[S[rs1]]
```

### Branches (resolved in-place, dynarec fuse/unfuse)

```
beq/bne/blt/bge/bltu/bgeu rs1, rs2, P    # if cond: next bundle PC = P
jal   rd, P                              # rd <- pc+4; next bundle PC = P
jalr  rd, rs1, imm                       # rd <- pc+4; next bundle PC = rs1+imm
```

Branch conditions resolve in the same cycle the bundle executes. The dynarec
fuses hot fall-through paths so the common case pays zero redirect cost;
taken branches cause a single-cycle resolved PC redirect (with an optional
software delay slot filled by the dynarec). Standard RV32 instruction slots
(32-bit) — no predicate fields, no guard encoding.

### Memory-event control transfer (MMIO without polling)

```
wjeq  si, rs2, P    # watch S[si]; when MEM[S[si]]==rs2: goto P
wjne  si, rs2, P    # same, condition !=
wset  si, rs2, P    # same, condition (MEM[S[si]] & rs2) != 0
```

The issuing lane suspends (no data-bus traffic) and wakes when the LSU
observes a store to the watched address. No polling loop, no branch to
schedule around, no wasted lane per cycle.

### Hot-address introspection (the hybrid spill loop)

```
sphint  rd, rs1         # hint: rs1 is hot (feeds the detector)
splr   rd, rs1          # rd ← metadata/LRU for slot rs1
spflush                 # drain recommendation queue into the ARF directory
slotr  rd, rs1          # rd ← S[rs1] (raw read)
slotw  rs1, rs2         # S[rs1] ← rs2 (raw write)
```

The hardware detector finds runtime-hot addresses; the dynarec decides and
commits pins. Hardware recommends, software commits — deterministic and debuggable.

Full reference: **[DESIGN.md §14](DESIGN.md)**.

---

## On vendor lock-in and "industry-standard" APIs

The FPGA industry wants you to believe that the path from RTL to a flashing LED
on a board requires: a 40-gigabyte vendor IDE that won't launch without a
license daemon, a proprietary synthesis flow that speaks no standard format,
an IP catalog of black-box megafunctions tied to one silicon family, and a
constraint language that changes semantics between point releases.

Pango, Xilinx/AMD, Intel/Altera — they all play the same game: chain you to
their "ecosystem" so you can never leave. Your RTL is portable; their tools
aren't. And the instant you want something they didn't anticipate — like a
1000-wide VLIW or a banked SRAM at 2× clock — you're in a support-ticket
queue talking to someone who's never read a datasheet.

emucpu fights this where it can: the RTL uses the open lowRISC `prim_ram_2p`
generic model (which Pango PDS infers into Titan2 BSRAM) instead of a vendor
megafunction. The simulation runs on open Verilator, not a licensed simulator.
The assembler is 500 lines of portable C with no toolchain dependency. The
only piece you can't escape is the vendor P&R — and that's the piece that
makes you want to change careers.

---

## Emulation workflow

### 1. Assemble a test

The C assembler (`util/bundle_asm.c`) compiles with no dependencies:

```bash
cc -O2 -o util/bundle_asm util/bundle_asm.c
./util/bundle_asm <width> <test.asm> [<output_stem>]
```

Example — a 4-lane ALU test:

```asm
# tests/alu.asm
.bundle
addi x5,  x0, 0x10       # lane 0
addi x6,  x0, 0x20       # lane 1
addi x7,  x0, 0x30       # lane 2
addi x8,  x0, 0x40       # lane 3
.bundle
add  x9,  x5, x6
add  x10, x7, x8
sub  x11, x8, x5
xor  x12, x6, x7
.bundle
ebreak
nop
nop
nop
.expect x5  = 0x10
.expect x9  = 0x30
.expect x10 = 0x70
```

```bash
./util/bundle_asm 4 tests/alu.asm tests/alu
# → tests/alu.img, tests/alu.expect
```

### 2. Build and run the simulation

```bash
cd dv/vliw
make build WIDTH=4       # Verilator: compile the VLIW core + testbench
make test WIDTH=4        # assemble + run all tests

# Or run one test manually:
./obj_dir/Vibex_vliw_dispatch tests/alu.img tests/alu.expect 4 20000
```

The testbench:
- Loads the bundle image into the bundle cache via the fill port.
- Drives `fetch_enable`, models the external data bus for MMIO.
- Halts on `ebreak` (or `illegal_insn`) and checks GPR values against `.expect`.

VCD waveform tracing is enabled — inspect with:
```bash
gtkwave dv/vliw/vliw_trace.vcd
```

### 3. Build the assembler binary

```bash
cc -O2 -Wall -o util/bundle_asm util/bundle_asm.c
```

No autotools, no CMake, no `configure`, no `Makefile.am`, no 47-step
bootstrap that downloads the internet. One source file. One compiler
invocation. That's it.

---

## Synthesis (Pango PDS, PG2T390H)

Not yet integrated into a PDS project flow. To target the AXP390 board you'll
need:

1. **A `.sdc`** defining `clk_i` and `clk_sram_2x` (2× the core clock from a PLL).
2. **A `.pdc`** pin constraint file for the AXP390 board.
3. **A PLL instantiation** for `clk_sram_2x` — the RTL has a placeholder
   (`assign clk_sram_2x = clk_i`) that must be replaced with a Pango PLL
   primitive for real hardware.

BRAM budget at the target instance (`W=32`, ARF=128K):

| Use | BRAM blocks (of 480) |
|---|---|
| ARF (64 banks × 2048×32) | ~128 (27%) |
| Data memory (64 banks) | ~200 (42%) |
| Instruction cache (32 banks) | ~64 (13%) |
| **Total** | **~82%** |

The `bps` config is defined in `ibex_configs.yaml`. Build with:
```bash
python3 util/ibex_config.py bps fusesoc_opts
```

---

## Honest performance reality

This section exists because lying about benchmarks helps no one.

**Against other FPGA soft cores** (Microblaze, scalar Ibex): emucpu wins,
potentially by 10–50×, because those cores are single-issue in-order with no
memory-level parallelism and no address acceleration. This is where the design
shines.

**Against an Apple M1 or any modern desktop CPU on general code**: emucpu
loses, by 1–2 orders of magnitude, because a 100 MHz FPGA soft-core cannot
outrun a 3.2 GHz ASIC, and the M1 already does ~80-wide memory-level
parallelism out of the box. The "no mispredicts" advantage is real but the M1
rarely mispredicts on the workloads this core targets (pointer chasing), so
the advantage doesn't translate to a win on that comparison.

**As an ASIC**: the architecture's advantages compound at GHz clocks with
hundreds of lanes and on-chip SRAM. The FPGA is the prototype vehicle, not
the performance target.

If anyone tells you a 100 MHz FPGA soft-core "beats an M1," check whether they
measured wall-clock time on the same workload, or whether they're quoting
IPC/cycle-count and hoping you won't notice the 32× clock gap. We measured. We
know.

---

## Project layout

```
rtl/
  ibex_bps_pkg.sv           # BPS-V types, ARF params, instruction funct codes
  ibex_vliw_dispatch.sv     # W-lane VLIW engine top (fetch→decode→lanes→commit)
  ibex_bundle_cache.sv      # W-bank instruction memory (64-bit slots)
  ibex_lane.sv              # One lane: ALU + LSU + branch resolution (no predicate)
  ibex_predicate.sv         # (removed — no predication in v3.2)
  ibex_register_file_vliw.sv# Multi-ported data RF (W×2R + W×1W)
  ibex_crossbar.sv          # Generic W×N read crossbar
  ibex_banked_lsumem.sv     # Banked data memory + external-fallback LSU
  ibex_addr_regfile.sv      # 128K×32 banked ARF + free-pool directory
  ibex_hot_addr_detect.sv   # Snoop-based hot-address detector + rec queue
  ibex_mem_event_watch.sv   # Per-lane MMIO wait/wake (wj* instructions)
util/
  bundle_asm.c             # Self-contained C bundle assembler
dv/vliw/
  tb_vliw.cpp              # Verilator C++ testbench
  Makefile                 # Build + run tests
  tests/                   # .asm test programs
DESIGN.md                  # Full architecture spec
DYNAREC.md                 # Scheduling + register allocation algorithms
```

---

## License

Apache 2.0 (inherited from Ibex / lowRISC).

---

## Final word

The CPU industry spent forty years adding transistors to a guess engine and
calling it innovation. emucpu deletes the guess engine, hands scheduling to a
dynarec that actually knows what the code does, and spends the saved silicon
on lanes and address registers that do real work.

No predictor. No mispredict. No pipeline flush. No apology.

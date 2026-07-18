# DESIGN — Ibex "BPS-V": Dynarec-Driven, 1000‑Lane Statically‑Scheduled Core with a Pinned‑Address ARF

> Status: proposal. Layered onto the **`small`** (2‑stage, `WritebackStage = 0`) Ibex configuration.
> All module/signal references are to the current tree (`master`, commit `c6edaa40`).
>
> **v3 — the defining shift.** This revision removes **all hardware branch prediction and all hardware
> instruction scheduling**. The **dynarec** lays out the instruction schedule (which lane executes what,
> in which cycle) and resolves control flow. The core is a **1000‑wide statically‑dispatched (VLIW‑style)
> engine**: one 1000‑instruction **bundle** per cycle, lanes execute in lockstep, there is no rename, no
> OoO scheduler, no wakeup/select CAM, no ROB. With no speculation there is **no misprediction by
> construction** — which is the strongest possible form of the original goal ("no pipeline stalls for
> mispredicts"). Branches are handled by **predication** (run both paths in the 1000 lanes and select) or,
> where the dynarec prefers, by a one‑cycle statically‑scheduled redirect.
>
> v2's OoO superscalar design is **superseded**; v3 keeps the ARF (which is unchanged and still good) and
> the 1000‑lane width, but replaces the engine underneath them.

---

## 1. Overview and goals

A RISC‑V core where:

1. **No prediction, no mispredicts.** The core makes no speculative control‑flow guesses. Therefore there
   is nothing to mispredict and no pipeline to flush on a wrong guess. Branches are either (a) eliminated
   by **predication** — both arms run simultaneously across the 1000 lanes and the result is selected — or
   (b) resolved and redirected in a single cycle that the dynarec has already scheduled around. *This is
   the literal realization of the original "no pipeline stalls for mispredicts" requirement.*

2. **The dynarec is the scheduler.** All instruction scheduling — dependency ordering, lane assignment,
   latency slotting, slot/bank conflict avoidance, control‑flow flattening — is done by the dynamic
   recompiler when it emits code. The core does **not** schedule; it **dispatches** one bundle per cycle.
   Concretely the core contains **no rename, no OoO issue, no CAM‑based wakeup/select, no ROB** (only a
   trivial in‑order commit register). This is what makes width 1000 buildable: the O(N²) scheduler logic
   that killed v2 is simply absent.

3. **The 500K (512K, 2^19) double‑clocked SRAM array holds all addresses.** Pinned addresses *and* live
   pointer‑chase temporaries live in the array's **free slots**; the integer RF holds only data. The
   pointer advance is `ldp.next si` (`S[si] <- MEM[ S[si] ]`), self‑contained in the ARF. Because the
   dynarec assigns slots, it can **guarantee no ARF bank conflicts by construction** (assign lanes to
   distinct banks within a bundle).

4. **`W` lanes exploit pointer‑level parallelism.** The dynarec packs `W` independent operations
   (typically `W` independent walk advances) into each bundle; the engine runs them in lockstep. Throughput
   approaches `min(W, data_mem_banks)` pointer advances per cycle. (`W` is parameterized; the abstract spec
   target is 1000, but the **PG2T390H FPGA instance is `W = 32`** — see §13. The remaining "1000" references
   in the intro and §3 describe the abstract architecture.)

**Non‑goals (unchanged).** Not a data cache — the ARF holds addresses, never data, so every `ldp`/`stp`
still reaches real memory and there is no coherence obligation. Not an OoO machine — there is no hardware
scheduling at all.

---

## 2. Baseline architecture (what we build on)

The "simplest one‑cycle" Ibex is the **`small`** config in `ibex_configs.yaml` (`WritebackStage=0`,
`BranchPredictor=0`, `BranchTargetALU=0`, `ICache=0`, `RV32Zca` only). Key facts we lean on:

- `ibex_id_stage.sv` is a 2‑state in‑order FSM (`FIRST_CYCLE`/`MULTI_CYCLE`, `:765`) with single‑issue
  stalls (`stall_branch`/`stall_jump` at `:824`/`:837`; assembled into `stall_id` at `:880`). **v3 replaces
  this FSM with a 1000‑lane static dispatcher** (§7); the existing `branch_decision_o = alu_cmp_result`
  (`ibex_ex_block.sv:92`) and the LSU are reused but widened.
- An experimental *static* predictor exists (`ibex_branch_predict.sv`, doc `:8-10`; instance
  `ibex_if_stage.sv:603-696`) with a resolution path (`nt_branch_mispredict`, `ibex_controller.sv:594-600`).
  **v3 does not use it** — `BranchPredictor = 0` permanently. The predictor module is left in the tree but
  uninstantiated.
- LSU (`ibex_load_store_unit.sv`): effective addr `= adder_result_ex_i` (`:111`), word‑aligned (`:517`),
  drives the Ibex bus (`:24-35`) through a 5‑state FSM (`:105-107`).
- Two external bus ports on `ibex_top.sv:76-96`; the only on‑core RAMs today are iCache tag/data banks
  (`ibex_top.sv:600-759`); **no data SRAM / TCM exists.**
- Custom‑instruction mechanism: add opcodes to `opcode_e` (`ibex_pkg.sv:67-78`), dispatch in the decoder
  (`ibex_decoder.sv:240`); RISC‑V reserves `CUSTOM-0..3` (today illegal). Gated by compile‑time parameters.
- SRAM config (commit `c6edaa40`): generic opaque req/rsp sideband (`ram_2p_cfg_req_t`, `Ram2pReqWidth=7`)
  used per‑way by icache RAMs (`ibex_top.sv:68-71`). The ARF reuses this pattern.

---

## 3. Control flow: a PC‑driven VLIW with dynarec fuse/unfuse

The engine has a **program counter** and resolves branches in hardware — there is **no branch prediction**
and **no predication**. The dynarec owns scheduling and adapts to observed branch behavior by **fusing and
unfusing** code regions into and out of bundles at runtime.

### 3.1 How it works: the PC and resolved branches

The core has a single bundle PC. Each cycle, the engine fetches the bundle at `pc`, executes all `W` lanes,
and advances:

- **No branch taken in the bundle:** `pc ← pc + bundle_bytes` (sequential).
- **A branch taken in the bundle:** `pc ← resolved_target` (set by the lane that evaluated the branch).

Branch conditions resolve **in the same cycle the bundle executes** (single‑cycle ALU + comparison). The
resolved target — either the fall‑through or the branch target — becomes the next bundle's PC. There is no
prediction, so there is never a wrong‑path fetch to squash. The redirect is deterministic: it fires exactly
when the branch condition is true, which is a resolved fact by the time the PC updates.

This is a **VLIW with a program counter** — the same model as classic VLIW (Multiflow, TI C6000): bundles
are the atomic issue unit, but control flow is sequential with resolved branches, not predicated.

### 3.2 Dynarec fuse/unfuse: adapting to branch direction at runtime

The dynarec observes which way branches go and re‑lays‑out the code accordingly:

**Fusing (branch resolves not‑taken → fall‑through).** When the dynarec sees a conditional branch that
consistently falls through, it **fuses** the fall‑through code into the same bundle stream as the code
before the branch. The branch instruction still executes (it evaluates the condition), but because it
resolves not‑taken, `pc` advances sequentially and the fall‑through bundle executes next with **zero
redirect cost**. The dynarec packs independent ops from both sides of the (not‑taken) branch into the same
bundles, maximizing lane utilization.

**Unfusing (branch resolves taken → split).** When a branch goes taken, the dynarec **unfuses** — it emits
the branch as the last op in its bundle, and the taken target becomes the next bundle's PC (a 1‑cycle
resolved redirect). The dynarec places independent work in the redirect's delay slot if available, or
accepts the single‑cycle bubble.

**Hot‑path optimization.** The dynarec's trace selector identifies hot paths (loops, common branches) and
fuses them into long straight‑line bundle sequences. Cold paths stay unfused (split at the branch). As
runtime behavior shifts, the dynarec re‑fuses or re‑unfuses — this is the same binary‑translation feedback
loop used by dynamic optimizers (HP Dynamo, Apple's Rosetta trace formation).

### 3.3 Why this satisfies the original requirement

| Mechanism | Stall on the branch? | Mispredict? |
|---|---|---|
| Fused fall‑through (branch resolves not‑taken) | **0 cycles** (sequential PC advance, both sides packed) | impossible — nothing guessed |
| Unfused taken branch + dynarec delay slot | **0 cycles** (delay slot filled with useful work) | impossible — nothing guessed |
| Unfused taken branch, unfilled | 1 cycle | impossible |

There is no row for "mispredict penalty" because the concept does not exist in this machine. The dynarec
optimizes for the *observed* branch direction — fusing the hot path so it runs at zero redirect cost —
and the hardware simply resolves the branch in‑place. No predictor, no speculation, no flush.

### 3.4 Memory‑event control transfer: wait/jump on MMIO write

For **memory‑mapped I/O and device handshakes** — "block until address `A` is written, then go to PC `P`" —
the core provides a hardware wait/jump instead of a polling loop:

- **`wjeq  si, rs2, P`** — watch ARF slot `si`; when `MEM[ S[si] ] == rs2`, set the next bundle PC to `P`.
  Until then, the issuing lane suspends (emits no data‑memory request). `si` holds the MMIO address;
  `rs2` is the sentinel value the device writes when ready.
- **`wjne si, rs2, P`** — same, condition `!=`.
- **`wset si, rs2, P`** — same, bit‑set form `(MEM[S[si]] & rs2) != 0`.

These are **not polls**. The hardware holds the lane in a suspended state and is woken when the LSU
observes a store to `S[si]` arriving on the `data_*` bus. No per‑cycle memory traffic, no branch to
schedule around. Like all control flow in this machine, `wj*` never guesses — it transfers control in
response to an observed event.

---

## 4. The double‑clocked pinned‑address SRAM array (banked for W lanes)

> **Sizing note (v3.1).** The abstract spec in earlier drafts was 512K entries × 1024 banks, aimed at an
> ASIC‑scale 1000‑lane machine. On the actual target part (Pango Titan2 **PG2T390H**), 512K×32 = 16.78 Mbit
> is **97% of the chip's 17.28 Mbit of block RAM** and would leave almost nothing for instruction/data
> memory — infeasible. The geometry below is **sized to the PG2T390H**: 128K entries, 64 banks, W = 32 lanes.
> See §13 for the full resource budget and the bring‑up vs. stretch trade. The abstract architecture is
> width/depth‑parameterized; these are the chosen instance values.

### 4.1 Geometry

| Property | Value | Notes |
|---|---|---|
| Capacity | **128K entries** (2^17) | sized to PG2T390H BRAM budget (§13); parameter `AddrRegFileDepth` |
| Word width | **32 bits** | one entry = one address |
| Total storage | **512 KB (4.19 Mbit)** | **24% of the chip's 17.28 Mbit BRAM** |
| **Banking** | **64 banks × 2048 entries** | `#banks = 2·W` for comfort (W = 32) — keeps the prefix‑sum scheduler valid with headroom (DYNAREC §A.4.5, §B.4) |
| Ports/bank | 2‑port true‑dual‑port BRAM (`prim_ram_2p`), **double‑clocked** at `clk_sram_2x` | ~1R + 1W per bank on each of 2 phases |
| Contents | *addresses only* | never data → no coherence obligation |

Each bank is 2048 × 32 = 8 KB = 64 Kbit, which is **2 of the chip's 36 Kbit BSRAM blocks** → **the whole ARF
uses 128 of the 480 BSRAM blocks (27%)**. Concrete and fits.

### 4.2 Logical regions (all in the one banked array)

| Region | Indices | Size | Purpose |
|---|---|---|---|
| Explicit table | `0x00000..0x003FF` | 1 K | dynarec‑pinned addresses (jump tables, globals); imm12‑addressable |
| HW‑managed working set | `0x00400..0x01FFF` | ~7.75 K | addresses promoted by the spill engine from the recommendation queue |
| **Free / spill pool** | `0x02000..0x1FFEF` | **~122 K** | **live pointer‑chase temporaries** (the "free" entries) |
| Metadata stripe | `0x1FFF0..0x1FFFF` | 16 | valid/free/LRU bits, free‑list, hash tags — **also in the array**, no separate RF/directory |

The metadata that v1 kept in flip‑flops now lives in the array's metadata stripe, so there is **no second
register file for address management** — the banked array is the single address store. ~122 K free‑pool slots
is still vastly more than any realistic pointer‑chase working set.

### 4.3 Double‑clocked phases

Both ports of every bank run from `clk_sram_2x_i` (2× `clk_core`); each core cycle each bank delivers, on
its 2× clock: 1 read + 1 write on the **phase‑A (datapath)** port and 1 read + 1 write on the
**phase‑B (management)** port. Phase A serves the lanes' `ldp`/`stp`/`ldp.next`/`slotr`/`slotw`/`pina`;
phase B serves the spill engine, the dynarec's bulk management, the free‑list, and metadata updates.

### 4.4 Bank conflict avoidance — now the dynarec's job

With **#banks (64) ≥ 2·W (2×32 = 64)**, a bundle where each lane touches a distinct slot can always be routed
one‑per‑bank. **Because the dynarec assigns slots, it can guarantee no conflicts by construction**: when
packing a bundle it assigns the 32 lanes' working slots from 32 distinct banks. The core therefore needs
**no dynamic conflict‑serialization logic** — another scheduler the hardware doesn't have to implement. (If a
dynarec‑emitted bundle does collide two lanes onto one bank, the core signals a stall trap and the dynarec
repacks; this is a correctness backstop, never on the hot path for well‑generated code.) The `2·W` margin
(not the bare minimum `#banks = W`) leaves the bank‑coloring graph (DYNAREC §B.4) trivially satisfiable even
when non‑chase ops also touch the ARF in the same bundle.

### 4.5 Core integration

New module `ibex_addr_regfile.sv` wraps the 64 banked `prim_ram_2p` macros, the lane↔bank crossbar, and
phase‑A/phase‑B arbitration. New ports on `ibex_core.sv` (exposed on `ibex_top.sv` like the icache RAMs):

```
output logic                 arf_clk_2x_o,
// datapath (phase A) — W concurrent slot accesses, one per lane
output logic [W-1:0]         arf_dreq_o,
output logic [W-1:0][16:0]   arf_didx_o,        // 17-bit index for 128K
input  logic [W-1:0][31:0]   arf_drdata_i,
output logic [W-1:0]         arf_dwe_o,
output logic [W-1:0][31:0]   arf_dwdata_o,
// management (phase B) — spill engine / system
input  logic [M-1:0]         arf_mreq_i, ...
// SRAM vendor sideband (c6edaa40 generic types), per bank
input  prim_ram_2p_pkg::ram_2p_cfg_req_t  arf_cfg_i [64],
output prim_ram_2p_pkg::ram_2p_cfg_rsp_t  arf_cfg_o [64]
```

Parameter `AddrRegFile` (default 0) gates the feature (mirrors `ICache`/`PMPEnable`); when 0, all ARF
instructions decode `illegal_insn` and ports are tied off.

---

## 5. Unified pointer‑chase spill into free SRAM slots

### 5.1 The walk primitive: `ldp.next` — advance through a slot

The classic chase `p = p->next` becomes a single instruction that reads the address from a free slot,
dereferences it, and writes the result back into the **same** slot — never touching the GPR file:

```
ldp.next  si          # S[si] <- MEM[ S[si] ]     ; p = *p, fully in the ARF
ldpcap rd, rs1        # rd <- MEM[S[rs1]] ; S[rs1] <- MEM[S[rs1]]   (advance + GPR copy)
```

A walk with a body computation:

```
# head pinned into a free‑pool slot (e.g. 0x2000) by the dynarec
loop:
    ldp.next  0x2000        # advance p = p->next           (no GPR, no addr‑compute)
    ldp  t0, 0x2000         # load p->data into a GPR       (address from slot, data to RF)
    ... use t0 ...
    bnez ?, loop            # resolved branch; dynarec fuses the loop body
```

The ALU address‑compute cycle of `lw` is gone, **and** the working pointer never occupies a GPR. With `W`
lanes, the dynarec runs **`W` independent walks** (slots `0x2000..0x201F`, each in its own bank) as one
bundle — that is the parallelism the wide engine consumes. (At the PG2T390H instance, `W = 32`.)

### 5.2 Slot lifecycle — allocate from / return to the free pool

Because metadata is in‑array, slot management is free‑list allocation over the free‑pool region:

- **`spalloc rd`** — allocate a free slot → `rd` (from the in‑array free list); sentinel `0x1FFFF` if full.
- **`spfree rs1`** — return slot `rs1` to the free list; contents become stale (harmless — reads go to memory).
- **`pina rs1, rs2`** — pin: `S[rs1] <- rs2`, mark PINNED (long‑lived; not LRU‑reclaimed). For the explicit
  table and the HW working set.
- **`unpin rs1`** — drop a PINNED slot back to the free pool.

A walk's lifetime: `spalloc` a working slot → seed with `pina`/`slotw` → loop on `ldp.next`/`ldp` →
`spfree`. The walk's live pointer state is, at every moment, a free‑pool slot.

### 5.3 What "no separate registers" means concretely

| State | v1 location | v3 location |
|---|---|---|
| Pinned addresses | big SRAM | big SRAM |
| Live walk pointer | GPR | **free SRAM slot** (`ldp.next` self‑updates) |
| Management directory | 64 FF | **SRAM metadata stripe** |
| Free list | n/a | **SRAM metadata stripe** |

The integer RF holds only data operands. The banked array is the single address store.

### 5.4 Consistency / correctness

- No data cached; `ldp`/`stp` always reach memory → DMA, multi‑core, self‑modifying code unaffected.
- Stale address in a freed slot → a redundant but correct memory access.
- Address validity best‑effort; software that cares uses `pina`/`spalloc`.
- The ARF holds addresses (a side channel); a `spflush` + free‑pool reset on privilege change is the
  tentative isolation policy (open, §11).

---

## 6. The dynarec as the scheduler (first‑class)

In v3 the dynarec is not a helper — it **is** the scheduler. Its responsibilities when translating a trace
into ARF+VLIW code:

1. **Dependency analysis & bundling.** Pack up to `W` independent ops into each `W`‑lane bundle (default
   scheduler: the online hash‑map scheduler of DYNAREC.md §A.5). Slots with unfilled work carry explicit
   NOPs (the lane idles).
2. **Latency slotting.** All operand latencies are known and fixed: ALU/ARF ops = 1 bundle; memory ops = a
   fixed, exposed latency `Lmem` bundles (the dynarec places the consumer exactly `Lmem` bundles after the
   producer, or uses the load‑valid interlock as a backstop). **Latency is exposed to software**, as in any
   VLIW.
3. **Lane/bank assignment.** Assign the bundle's working slots so each lane hits a distinct ARF bank
   (§4.4) — guaranteeing no bank conflicts without any hardware arbitration.
4. **Control‑flow flattening.** Replace short conditionals with predication (§3.1); emit resolved branches
   with delay slots for large diverges (§3.2). The dynarec, not the hardware, decides which.
5. **ARF slot management.** Assign free‑pool slots to walks; emit `spalloc`/`spfree` lifetimes; pin static
   addresses with `pina`; emit `ldp.next` for every pointer advance.
6. **Hint consumption.** Read the hardware hot‑address recommendation queue (`splr`/`spflush`, §8) and
   promote hot addresses to PINNED slots; re‑emit code against the pinned slots.

The core's contract with the dynarec is minimal and **fully static**: "execute exactly the bundle I give
you, in order, one per cycle; resolve branches in‑place; don't reorder, don't speculate, don't skip." Every
dynamic decision the dynarec would rather make itself, it makes.

---

## 7. The statically‑dispatched execution engine (width `W`)

The engine is a **VLIW dispatcher**, not a scheduler. Width is a parameter `W` (**instance value `W = 32` on
PG2T390H, §13**; the architecture is width‑parameterized, so an ASIC target could be wider).

### 7.1 What is NOT in the core

| v2 OoO structure | v3 status |
|---|---|
| Branch predictor | **removed** (§3) |
| Rename / physical RF mapping | **removed** — dynarec assigns resources |
| OoO issue + CAM wakeup/select | **removed** — dynarec orders |
| ROB | **removed** — only a trivial in‑order commit register |
| Load/store queue (memory disambiguation) | **removed** — dynarec orders; a simple in‑bundle address‑collision check remains only as a safety trap |

This removal is what makes the engine *buildable*: the per‑lane logic is an ALU + ARF read port + LSU port,
with **no wakeup CAM and no speculation state**. The cost is the dynarec, which is software. (At a given `W`,
per‑lane cost is roughly constant; the limiting factor on FPGA is routing congestion and the lane↔bank
crossbar, not logic — see §13.)

### 7.2 What IS in the core

- **Instruction store: a `W`‑bank, multi‑ported instruction memory — see §7.2.1.** Delivers one
  `W`‑instruction **bundle** per cycle (32 instructions = 128 B/cycle at the instance width). (This is fetch
  *bandwidth*, not speculation — the next bundle PC is the dynarec‑dictated one: a resolved branch target or
  the next sequential bundle.)
- **`W`‑lane decode.** Each bundle is `W` slots; each slot decodes one RV32 (+ compressed expansion) or
  ARF op, plus a 3‑bit predicate guard.

#### 7.2.1 Instruction fetch: banked multi‑read memory (not a single cache)

Fetching `W` instructions per cycle **cannot be done from a single SRAM/cache.** A conventional instruction
cache has 1–2 read ports; each port returns one word/cycle. Getting `W` reads/cycle out of one monolithic
SRAM would require `W` read ports, and SRAM area scales as roughly **O(ports²)** — a 1000‑port SRAM is
physically impossible (it would be almost entirely wiring and wouldn't close timing). ASIC SRAM compilers
top out around 2–8 ports; FPGA block RAM is **dual‑port** (2 reads) per block.

**The solution is *banking*:** split the instruction memory into `W` banks, give each bank its own read
port, and read one instruction from each bank per cycle. Now `W` reads/cycle is mechanically possible —
you've traded one impossible `W`‑port SRAM for `W` ordinary 1‑port SRAMs. This is how every wide/VLIW fetch
path works in practice (Itanium, GPUs, DSPs), and it's the same banking trick the ARF uses (§4).

**Bundle layout → bank mapping.** The dynarec lays out each `W`‑instruction bundle **linearly across the
banks**: instruction `i` of a bundle goes to bank `i mod W`. A fetch at bundle PC `b` then issues one read
to each bank at its own address for that bundle, and all `W` banks return in parallel. Because the bundle is
the atomic fetch unit, there are no cross‑bundle bank conflicts — every fetch is exactly one read per bank.

**Branch / non‑sequential fetch.** Because there is **no branch prediction** (§3), the fetch unit never
guesses. The next bundle PC is either `current + 1` (sequential) or a **resolved** branch target computed in
the same cycle the branch executes. The bank‑address decoders recompute every cycle from the new bundle PC,
so a redirect takes effect the very next fetch (at most one bundle bubble if the dynarec didn't fill a delay
slot). No wrong‑path fetch to squash.

**Bank ports and double‑clocking.** Each instruction bank is a `prim_ram_2p` dual‑port block:
- **Port A (fetch):** 1 read/cycle, supplying instruction `i` of the current bundle to lane `i`.
- **Port B (fill/scratch):** used by the dynarec/loader to write new bundles into the code region, and to
  read out code for deopt/inspection. Not on the fetch hot path.

Like the ARF (§4.3), both ports run from `clk_sram_2x` so fetch (A) and fill (B) don't contend. The fetch
read is registered and available the same core cycle the bundle executes — **fetch latency = 1 cycle, no
fetch stalls.**

**FPGA instantiation (PG2T390H).** The chip has 480 × 36 Kb block‑RAM blocks. Budget:

| Use | Blocks |
|---|---|
| ARF (128K × 32, 64 banks × 2 blocks) | 128 |
| Data memory (banked) | ~200 |
| **Instruction banks** | **~150** |
| (total) | 478 / 480 |

With dual‑port BRAM, **~150 instruction blocks → ~300 simultaneous reads/cycle → a ~300‑instruction/cycle
fetch ceiling** on this part. At the target `W = 32`, fetch needs 32 banks (~32–64 blocks), comfortably
inside budget with headroom. The ~300‑read ceiling only matters if you tried to push `W` toward ~300 on this
FPGA, at which point the lane↔bank crossbar routing (§13) would already have collapsed the clock well before
fetch bandwidth became the limiter. On a larger FPGA (Stratix 10 GX 10M, VU19P), scaling the instruction
banks toward 1000 is a BRAM‑budget question, not an architectural one.

**Why this is necessary but not sufficient for throughput.** Wide fetch is *required* to feed `W` lanes
(e.g. 1000 independent `ldp.next` ops filling 1000 lanes needs a 1000‑bank fetch), but on a latency‑bound
workload like a single pointer chase, the machine fetches a handful of instructions and then waits on
memory — fetch bandwidth is not the bottleneck there. Fetch bandwidth becomes the limiter only on
wide, compute‑heavy, straight‑line code (the §C multi‑stream case), which is exactly the workload this
machine targets.
- **`W` ALU lanes** in lockstep. Each lane: ALU → optional ARF slot read/write → optional data‑memory
  request. **All lanes always execute** (no predication) — the dynarec ensures every slot in a bundle has
  useful work (or an explicit NOP).
- **Banked data memory: `#banks ≥ W`** (same double‑clocked banking as the ARF) feeding the lanes' LSU
  ports. Stores commit in program (bundle) order; loads issue as the dynarec scheduled them.
- **Single in‑order commit register** per architectural GPR: the last writer in a bundle wins; results land
  at the end of the bundle. No reorder needed because the dynarec already ordered everything.
- **Resolved branch redirect**: if any lane's branch evaluates taken, the next bundle PC is set to the
  resolved target (§3.1). The dynarec fuses fall‑through paths so hot branches pay zero redirect cost.

### 7.3 Bundle execution semantics

A bundle is the atomic unit of forward progress:

- All `W` lanes' operations execute in the cycle the bundle dispatches.
- **Bundle completion = slowest operation in it completes.** Because memory latency is exposed and
  variable, a bundle containing a load holds the engine until that load's data returns (§11 risk). The
  dynarec mitigates by (a) scheduling consumers `Lmem` bundles later and (b) keeping each bundle's loads on
  independent chains so no single slow access needlessly gates the others — but the engine itself does not
  advance until the bundle's loads resolve.
- Branch resolution within a bundle sets the next bundle PC; there is no wrong‑path execution to discard.

### 7.4 Reuse from Ibex

| Ibex module | Fate in v3 |
|---|---|
| `ibex_if_stage.sv` (prefetch/icache) | replaced by the `W`‑bank bundle cache |
| `ibex_id_stage.sv` (in‑order FSM) | replaced by the `W`‑lane static dispatcher |
| `ibex_ex_block.sv` (1 ALU) | replicated into `W` lanes |
| `ibex_load_store_unit.sv` | widened to `W` LSU ports over the banked data memory |
| `ibex_controller.sv` | retained for exception/redirect, fed by resolved (non‑speculative) branches |
| `ibex_branch_predict.sv` | **uninstantiated** (`BranchPredictor = 0` permanently) |
| **new** | `ibex_addr_regfile.sv`, `ibex_hot_addr_detect.sv`, `ibex_bundle_cache.sv`, `ibex_lane.sv`, `ibex_mem_event_watch.sv` (per‑lane MMIO‑write wait/wake, §3.4) |

---

## 8. Hybrid hot‑address detection (still useful; feeds the dynarec)

Even though the dynarec schedules, it benefits from runtime hot‑address hints it couldn't see statically.
A small observer block `ibex_hot_addr_detect.sv` snoops the data bus (the same `data_*` signals the LSU
drives) into a 64‑entry saturating‑counter table keyed by `hash(data_addr)`; addresses crossing
`HOT_THRESH` (default 8) are pushed to a recommendation queue. The dynarec drains it with `spflush`/`splr`
and re‑emits code with those addresses pinned. Hardware still **never mutates the address map** — it only
*recommends*; the dynarec commits pins. This keeps behavior deterministic and the dynarec in full control.

---

## 9. Synergy: why static‑wide + ARF‑spill belong together

- The **ARF spill model** turns a pointer chase into independent, slot‑local ops with no shared GPR base —
  the only way to find `W`‑way parallelism in linked‑structure code. Without it, all lanes would depend on
  one base register and the engine would run 1‑wide.
- The **static‑wide engine** amortizes the ARF's `#banks` cost and, crucially, needs no scheduler — which
  is what keeps per‑lane cost low and lets the width be set by available FPGA resources rather than by
  scheduler complexity.
- The **dynarec** closes the loop: it is what packs `W` independent advances per bundle, assigns slots to
  avoid bank conflicts, flattens branches into predication, and consumes hot‑address hints.

For parallel graph / BFS / N‑list workloads, throughput approaches `min(W, data_mem_banks)` pointer
advances per cycle. For serial single‑chain walks, the engine is memory‑latency bound (~1 useful
advance/cycle) and the idle lanes are a utilization cost (§11) — but the ARF still removes the
address‑compute bubble and there are still never any mispredicts.

---

## 10. Microarchitecture (block diagram)

```
                ┌──── ibex_core (v3.2, W=32 on PG2T390H, PC-driven VLIW, no predication) ──┐
                │                                                                     │
  clk_sram_2x ──┼─▶ ┌──── ibex_addr_regfile: 64 banks × 2048 × 32 (128K entries) ───┐│
                │   │   metadata stripe (valid/free/LRU/free‑list) in‑array         ││
                │   │   phase A (datapath) ◀── W=32 lanes' slot r/w, 1 per bank     ││
                │   │   phase B (mgmt)     ◀── spill engine / dynarec              ││
                │   └───────────────────────────────────────────────────────────────┘│
                │   ┌── ibex_hot_addr_detect ── snoop data bus ─▶ recommend Q       │
                │                                                                     │
                │   BUNDLE CACHE (W‑bank i‑mem) ─▶ W‑lane decode                    │
                │       next bundle PC = dynarec‑dictated (resolved branch or       │
                │       sequential); NO predictor                                    │
                │                          ▼                                          │
                │   W LANES (lockstep): ALU → ARF r/w → LSU                          │
                │       resolved branch → PC redirect (dynarec fuse/unfuse)          │
                │                          ▼                                          │
                │   BANKED LSU (W ports) ─▶ #banks‑bank data mem ─▶ data bus        │
                │                          ▼                                          │
                │   COMMIT (in‑order, last‑writer‑wins per GPR; bundle atomic)      │
                │                                                                     │
                │   NO rename · NO OoO scheduler · NO wakeup CAM · NO ROB           │
                │   NO branch predictor · NO speculation                             │
                └─────────────────────────────────────────────────────────────────────┘
                       ▲
                       │ emits bundles (schedule, slot/bank assignment,
                       │ predication, delay slots, ARF lifetimes)
                 ┌─────┴──────┐
                 │  DYNAREC   │  ◀── reads hot‑addr recommend Q (spflush/splr)
                 └────────────┘
```

---

## 11. Risks and open questions

**Width is resource‑limited; `W = 32` is the PG2T390H instance.** The architecture is width‑parameterized;
the on‑FPGA build is `W = 32` (§13). Pushing wider (64) is possible but bounded by **routing congestion and
the lane↔bank crossbar**, not by raw logic — a static engine has no O(N²) scheduler, so the obstruction is
area/wire, not algorithmic infeasibility. The dynarec‑driven model degrades gracefully: fewer lanes just pack
fewer ops per bundle.

**Bundle‑completion gating (the main v3 cost).** "A bundle completes when its slowest op completes" means a
single slow load can stall all `W` lanes (classic VLIW memory‑stall exposure). Mitigations: (a) the dynarec
schedules consumers `Lmem` later so latency is hidden across bundles; (b) keep each bundle's loads on
independent chains; (c) optionally split the engine into a few independent sub‑engines that don't gate each
other (a middle ground between full lockstep and full OoO) — the `N × W = 32` multi-cluster variant of §13 is
exactly this split. This is the design's central performance risk.

**Restored, not lost: the mispredict property.** v2 reintroduced a large mispredict penalty. v3 **removes
misprediction entirely** — there is no predictor to be wrong. Branches are resolved in‑place (§3.1) and
the dynarec fuses hot fall‑through paths to eliminate redirect cost.
This is the strongest form of the original requirement; the trade is that predication wastes lanes on the
not‑taken arm and the dynarec must be good at scheduling.

**Dynarec dependence.** Scheduling quality now lives in software. A naive dynarec produces bundles full of
NOPs (poor utilization) and may even stall on memory; a good one approaches the engine's peak. This is the
inherent cost of moving scheduling out of silicon.

**Bank‑conflict backstop.** If the dynarec emits a bundle that collides two lanes on one ARF bank, the core
must trap+repack. Well‑generated code never hits this, but the core must still implement the check.

**2× SRAM clock availability.** Both ARF and data‑mem banks need `clk_sram_2x` from the SoC. Fallback:
single‑clocked 1R1W halves per‑bank throughput (→ more banks or lower effective width).

**Security / side channel.** ARF holds addresses (and live walk pointers) → side channel. Tentative policy:
mandatory `spflush` + free‑pool reset on privilege change (open).

**Compressed encodings.** All ARF/predicate ops are 32‑bit. A future phase could add compressed forms via
the reserved custom slot at `ibex_compressed_decoder.sv:512`; out of scope for v1.

**Naming.** The pre‑c6edaa40 SRAM config struct had an unrelated `rf_cfg` field; the ARF reuses the new
generic `ram_2p_cfg_req_t` and avoids the name.

---

## 12. Integration plan (phased; each phase leaves the tree building)

`AddrRegFile = 0`, `SuperscalarWidth = 1` (a single lane) by default — the default config is unchanged Ibex.

**Phase 0 — ARF skeleton + raw slot ops (single lane, in‑order).** Add `CUSTOM-0/1` to `opcode_e`
(`ibex_pkg.sv:67-78`); `AddrRegFile` param; `ibex_addr_regfile.sv` wrapping one `prim_ram_2p` bank;
`slotr`/`slotw` only. Decode in `ibex_decoder.sv:240`.

**Phase 1 — Spill model.** Free pool, metadata stripe, `spalloc`/`spfree`/`pina`/`unpin`; deref primitives
`ldp`/`stp`/`ldpi`/`stpi`/`ldp.next`/`ldpcap`; route deref addresses into the existing LSU.

**Phase 2 — Hybrid detector.** `ibex_hot_addr_detect.sv` + recommendation queue + `sphint`/`splr`/`spflush`.

**Phase 3 — Bank the ARF (still single issue).** Scale `ibex_addr_regfile.sv` to 64 banks (the PG2T390H
instance); lane↔bank crossbar; phase‑A/B arbitration. Proves the banking in‑isolation.

**Phase 4 — Memory‑event waits + branch resolution.** Add the `wj*` wait/jump instructions and the
per‑lane suspend + bus‑write‑wake logic in the LSU; wire branch decision/target from the lanes to the
dispatcher for resolved PC redirect (§3.1). Still single lane.

**Phase 5 — Widen to the static dispatcher.** `SuperscalarWidth` param; `W`‑bank bundle cache
(`ibex_bundle_cache.sv`); `W`‑lane decode + `W` `ibex_lane`s in lockstep; banked data memory; widened LSU.
Bring‑up at `W = 8`; **target instance `W = 32` on PG2T390H (§13)**; stretch at `W = 64`.

**Phase 6 — Dynarec.** Assembler patterns for all ARF + predicate ops; the dynarec's dependence‑graph
packer, latency slotter, bank‑conflict‑free slot assigner, predication pass, and hot‑address promotion.

**Phase 7 — Verification.** DV: walk correctness vs `lw`/`sw` reference; bundle‑atomic execution; bank‑conflict
trap; predication equivalence; consistency under interrupts and privilege change. Formal: extend the
custom‑opcode excludes (cf. recent counter‑alias commits) to ARF ops and branch resolution.

---

## 13. FPGA instantiation on Pango Titan2 PG2T390H (AXP390 board)

This is the concrete target. The part's total budget, from Pango's Titan‑2 family page:

| Resource | PG2T390H total | BPS‑V instance (W=32, ARF=128K) | % of part |
|---|---|---|---|
| LUT6 | 243,600 | ~16–22K (controller + 32 lanes + crossbar + decode) | ~7–9% |
| Flip‑flops | 487,200 | ~30–45K (pipeline regs, lane state) | ~7–9% |
| Block RAM (36 Kb) | 480 (≈17.28 Mbit) | **128 ARF + ~64 instruction banks + ~200 data mem ≈ 392** | ARF 27% + instr 13% + data 42% ≈ **82%** |
| DSP (APM, 18×25) | 840 | few (mul in lanes, if needed) | <5% |
| PLL | 20 (10 GPLL + 10 PPLL) | 1 (for `clk_sram_2x`) | 5% |

**Instruction fetch mapping (§7.2.1).** `W = 32` instruction banks, each a dual‑port 36 Kb block
(1 Kb × 32, i.e. 256 instructions deep) → ~32–64 blocks → **~64 reads/cycle of fetch headroom** (dual port),
comfortably above the 32 reads/cycle the target width needs. If `W` were pushed toward the BRAM ceiling
(~300 fetch reads/cycle on this part), the fetch would not be the limiter — lane routing/crossbar congestion
(§13) would collapse the clock first.

**ARF mapping.** 128K × 32 = 4.19 Mbit = 117 × 36 Kb blocks; organized as **64 banks × 2048 × 32**, each bank
= 2 × 36 Kb BSRAM (one 32‑bit‑wide × 2 K‑deep true‑dual‑port block). This is the §4.1 geometry, realized in
the chip's actual 36 Kb BRAMs.

**Why `W = 32` and not wider.** Per‑lane cost (~400–600 LUT incl. ARF port + LSU port) is not the
binding constraint — routing congestion and the **`W`×`#banks` lane↔bank crossbar** are. At `W = 32`,
`#banks = 64`: a 32×64 crossbar that place‑and‑route can close timing on. Each doubling of `W` roughly
quadruples the crossbar congestion; `W = 64` is a stretch (re‑floorplan + pipelined crossbar), `W = 128` is
not realistic on a single PG2T390H. The architecture is parameterized so an ASIC or multi‑FPGA target could
go wider.

**Why `ARF = 128K` and not larger.** Each doubling of ARF depth doubles BRAM use: 128K = 27%, 256K = 49%,
512K = 97% (leaving almost nothing for instruction/data memory). 128K still gives ~122 K free‑pool slots —
vastly more than any realistic pointer‑chase working set — and leaves ~13 Mbit for the instruction bundle
cache and data memory. Bring‑up can drop to 32K/64K for more memory headroom.

**Double‑clocking.** Titan2 BSRAM is true‑dual‑port with an **independent clock per port** — the §4.3 scheme
(drive both ports from `clk_sram_2x`) is directly supported. `clk_sram_2x` comes from one PLL at 2× the core
clock; feasible as long as `fmax(BRAM) ≥ 2·fmax(core)` (Titan2 BSRAM ≈ 300–500 MHz vs. a realistic
~100–150 MHz core). 20 PLLs on the part → no scarcity.

**`#banks ≥ W` and the scheduler precondition.** DYNAREC.md §A.4.5 / §B.4 require `#banks ≥ W` for the
prefix‑sum scheduler to be valid and the bank constraint to be satisfiable by RA alone. The instance uses
`#banks = 64 = 2·W`, the comfort margin recommended there.

**Bring‑up ladder.** `W = 1` (single‑lane, in‑order, proves ARF + custom ISA) → `W = 8` (proves banking,
predication, double‑clock) → **`W = 32`** (the target instance) → `W = 64` (stretch, if timing closes).
Each step is a parameter change, not a redesign.

**Multi-cluster variant (`N × W = 32`).** A single `W = 32` cluster is not the part's ceiling — lane logic is
only ~8% of the LUTs; the ceilings are BRAM geometry and the lane↔bank crossbar. The alternative to the
monolithic `W = 64` stretch is `N` independent clusters of `W = 32`, each with its own lanes, GPRs, ARF,
instruction banks, and data-memory partition, plus a pipelined interconnect with a fixed, exposed hop latency
`Lhop` for cross-cluster traffic. The dynarec schedules `Lhop` exactly like `Lmem` (DYNAREC.md §E). ARF
geometry follows the §4.1 rule that one bank = one 36 Kb block at full depth (1024 × 32 per block):

| Config | ARF blocks (geometry) | Instr blocks | Data mem | Total / 480 | Bank margin |
|---|---|---|---|---|---|
| 1 × W32 (this §) | 128 (64 banks × 2048) | ~64 | ~200 | 392 | `#banks = 2·W` ✓ |
| 2 × W32 | 128 (2 × 64 banks × 1024) | 2 × 32 | ~200 | 392 | `2·W` ✓ — identical footprint to one cluster |
| 3 × W32 | 96 (3 × 32 banks × 1024) | 3 × 32 | ~200 | 392 | `#banks = W` exactly |
| 4 × W32 | 128 (4 × 32 banks × 1024) | 4 × 32 | ~200 | 456 | `#banks = W` exactly |

Instruction banks are quoted at minimum geometry (one dual-port block per lane → 64 reads/cycle of headroom,
§7.2.1); doubling for code depth costs 32 blocks/cluster (`N = 2` still fits, at 456). LUT cost stays modest:
~10–14K per incremental cluster (32 lanes + a 32×64 crossbar + LSU ports) on top of the shared
controller/decode — 4 clusters ≈ 25% of the part. The one `clk_sram_2x` PLL serves all clusters.

Three structural caveats, in order of importance:

1. **The data memory decides whether clustering helps.** Throughput is still `min(lanes, data_mem_banks)`
   (§9) — clusters multiply lanes, not memory concurrency. Partition the data banks per cluster and keep
   walks partition-local (a remote walk crosses the interconnect at `Lhop`). Do **not** build a shared
   `(N·W) × #banks` crossbar: that is the O(W²) congestion that ruled out monolithic `W = 128`, re-introduced
   one level up.
2. **BRAM granularity is per bank, not per bit.** Every bank costs a whole 36 Kb block even when shallow, so
   `N = 3` drops aggregate ARF capacity to 96K entries (2048/3 isn't integral) and `N = 4` fits only at
   `#banks = W` with no `2·W` bank-coloring margin (DYNAREC.md §A.4.5 / §B.4). `N = 2` is the sweet spot:
   identical 392-block footprint, full margin, no capacity loss.
3. **Bundle gating improves.** A slow load gates only its own cluster's bundle; the other clusters keep
   dispatching — the §11 "independent sub-engines" mitigation falls out of the cluster model for free. A
   consumer waiting on an `Lhop` transfer from a gated cluster inherits the gate through the dependence
   edge (correct — a true dependence — not a new hazard).

**2 × W32 vs. the monolithic W = 64 stretch.** Same 64 lanes. The clustered form replaces one hostile 64×128
crossbar with two friendly 32×64 ones and keeps the bank margin; the monolith keeps a flat memory model with
no `Lhop` traffic. Run both through P&R before committing Phase 5's stretch goal (§12) — if the monolith
misses timing, clusters still deliver the lanes. The dynarec side (stream-level partitioning, transfer
insertion, spill tiers) is specified in DYNAREC.md §E.

---

## 14. ISA quick reference

```
# All gated by AddrRegFile; illegal_insn otherwise. si = rs1[16:0] (17-bit, 128K) or imm12.
# No predication — all lanes always execute (NOPs fill empty slots).

# Raw slot access (full 128K)
slotr  rd, rs1            # rd  <- S[rs1]
slotw  rs1, rs2           # S[rs1] <- rs2

# Pin / free‑pool lifecycle (metadata in the SRAM stripe)
pina   rs1, rs2           # S[rs1] <- rs2 ; mark PINNED
unpin  rs1                # return PINNED slot to free pool
spalloc rd                # rd <- free slot index (0x1FFFF if full)
spfree rs1                # return slot to free pool

# Dereferenced memory access (address pre‑staged in ARF)
ldp    rd, rs1            # rd  <- MEM[ S[rs1] ]    (full index)
stp    rs2, rs1           # MEM[ S[rs1] ] <- rs2
ldpi   rd, imm12          # rd  <- MEM[ S[imm12] ]  (low 4K)
stpi   rs2, imm12         # MEM[ S[imm12] ] <- rs2

# Pointer‑chase primitives — advance through a free slot, no GPR
ldp.next si              # S[si] <- MEM[ S[si] ]            ; p = *p
ldpcap rd, rs1           # rd <- MEM[S[rs1]] ; S[rs1] <- MEM[S[rs1]]

# Memory‑event control transfer — wait/jump on MMIO write (no polling)
wjeq  si, rs2, P          # watch S[si]; when MEM[S[si]]==rs2: goto P  (lane suspends until)
wjne  si, rs2, P          # same, condition !=
wset  si, rs2, P          # same, condition = (MEM[S[si]] & rs2) != 0   (bit‑set form)

# Branches (resolved in‑place, §3.1; dynarec fuses/unfuses)
beq/bne/blt/bge/bltu/bgeu rs1, rs2, P    # if cond: next bundle PC = P
jal   rd, P                              # rd <- pc+4; next bundle PC = P
jalr  rd, rs1, imm                       # rd <- pc+4; next bundle PC = rs1+imm

# Hybrid‑spill introspection (CUSTOM-1)
sphint rd, rs1            # hint: rs1 is hot
splr   rd, rs1            # read metadata/LRU for slot rs1
spflush                  # drain recommendation queue into metadata stripe
```


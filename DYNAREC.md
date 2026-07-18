# DYNAREC — Scheduling and Register Allocation for the Ibex BPS‑V VLIW Core

> Companion to `DESIGN.md` (v3). The core is a **dynarec‑driven, 1000‑lane statically‑dispatched (VLIW)
> engine**: the hardware does no scheduling and no branch prediction — the dynarec lays out every bundle
> and resolves all control flow. This document answers four questions:
>
> 1. **What is the best algorithm for determining what can be scheduled in (VLIW) superscalar order?**
> 2. **What is the best algorithm for assigning registers?**
> 3. **When should a value be spilled to SRAM, and to which memory?** (§D)
> 4. **How does allocation change when the engine is `N` clusters instead of one?** (§E)
>
> All four are answered for *this* machine specifically — exposed memory latency, 1024‑bank ARF with a bank
> constraint, a separate ARF‑slot address class, predication instead of branches, and a target workload of
> pointer‑chasing loops.

---

## 0. Scope and assumptions

- **Source = standard RV32 binary** (the program as compiled by a normal RISC‑V toolchain). The dynarec
  *lifts* hot traces into an internal IR and *lowers* them to **VLIW bundles** of the extended ISA from
  `DESIGN.md` §12 (RV32 + ARF ops + predicates). This is the usual "same‑ISA dynarec" model (à la HP Dynamo
  / Transmeta‑style binary translation), not JIT‑from‑bytecode.
- **The dynarec runs at runtime, so per‑trace translation cost must be small** (it is amortized over many
  executions of a hot trace). This rules out optimal schedulers and slow register allocators in favor of
  linear‑time‑ish heuristics that still hit ~95% of the quality of the expensive ones.
- **Latency is exposed.** ALU/ARF ops = 1 bundle; a load's data is valid `Lmem` bundles later. The dynarec
  knows `Lmem` (constant, or queried at bring‑up) and must place consumers accordingly.
- **Hardware gives the dynarec a fixed contract** (`DESIGN.md` §6): "execute exactly the bundle I give you,
  in order; resolve branches in place; don't reorder or speculate." Everything below operates within that.

### The dynarec pipeline (minimal framing)

The two asked algorithms live in the shaded middle stages. I keep the rest brief so the doc stays focused.

```
profile (hot PCs/edges)  ─▶  trace selection  ─▶  lift to IR  ─▶
   ┌──────────────────────────────────────────────────────────┐
   │  ◆ SCHEDULING   (this doc §A)   →  packet/bundle list    │
   │  ◆ REGISTER ALLOCATION (§B)      →  GPR + ARF slot assign │
   │  ◆ control‑flow flattening (predication), latency slotting│
   │  ◆ bank‑safe slot placement                              │
   └──────────────────────────────────────────────────────────┘
   ─▶  emit VLIW bundles  ─▶  code cache  ─▶  (deopt / OSR on guard miss)
```

---

## A. Scheduling: what can go in one bundle?

### A.1 The best algorithm is **list scheduling on a data‑dependence graph (DDG)**

For VLIW, list scheduling is the workhorse — it is what Bulldog (Ellis 1986, the foundational VLIW
compiler) and every VLIW/DSP codegen since uses. It is greedy, near‑linear in the graph size, and its main
heuristic (**critical‑path priority**) is the best single priority function known. Optimal scheduling is
NP‑hard, and list scheduling with a good priority routinely lands within a few percent of optimal.

**Step 1 — Build the DDG.** Nodes = IR ops. Edges = dependences with latency weights:

- **True (flow) dependence** `a → b` with weight `lat(a)`: `b` reads what `a` writes. This is the only
  edge that is *mandatory* in a correct schedule; everything below is added to make the hardware's static
  contract hold.
- **Anti (WAR) / output (WAW)** dependences: **in the first pass we do not add them** (see §B — we schedule
  on *virtual* registers so false dependences don't exist yet). They reappear only if/when a real register
  is reused, and the allocator guarantees they're respected.
- **Resource constraints** modeled per‑bundle: `#ops ≤ 1000` (lanes), `#ARF‑reads/writes per bank ≤ 1`,
  `#mem‑ports ≤ 1024`, one predicate write per `pK`. These are checked at placement time, not as edges.

**Step 2 — Compute priorities (critical path).** For each node compute:
- `ASAP(n)` = earliest bundle it can land in (longest weighted path from a source).
- `ALAP(n)` = latest it can land without lengthening the schedule (longest weighted path to a sink,
  subtracted from the schedule length).
- **`mobility(n) = ALAP(n) − ASAP(n)`**; **priority = −mobility**, i.e. **critical‑path length** of the
  node. Low mobility → high priority. This is the dominant heuristic: the op on the longest chain goes
  first, because delaying it delays everything downstream. (Secondary keys, used to break ties: resource
  scarcity — ops using the scarcest resource first; then latency — longer ops first.)

**Step 3 — Greedy cycle‑by‑cycle placement.** Maintain a *ready set* (all predecessors scheduled, their
latency satisfied). Each bundle slot `c`:

```
bundle = []
for each op in ready_set (highest CP priority first):
    if op's resource needs fit in bundle (lanes, banks, mem ports, predicate write)
       AND all its flow‑deps are satisfied at cycle c:
        place op in bundle; remove from ready; mark its resources used
        for each successor s: decrement pending‑dep count; if s now ready, add it
emit bundle at cycle c; c += 1; refill ready_set
```

That's the whole scheduler. It naturally packs independent ops into the same bundle (they're all ready and
don't conflict) and serializes dependent ones across bundles.

**Step 4 — Latency slotting (this machine's exposed‑latency wrinkle).** Because memory latency is exposed,
a load placed in bundle `c` does not make its consumers ready until bundle `c + Lmem`. List scheduling
already handles this — the edge weight is `Lmem`, so `ASAP(consumer) ≥ c + Lmem` automatically. Between the
load and its consumer the dynarec should fill the gap with independent work (other streams' loads) rather
than NOPs; list scheduling does this for free because those ops are ready and in the ready set.

### A.2 Region formation: **trace scheduling** + **hyperblocks** (for predication)

List scheduling operates on a *region*; what goes in the region decides how much parallelism it can see.
Two classical region forms, both used:

- **Trace scheduling (Fisher 1981).** Pick the most likely path through a region (the *trace*); schedule
  it straight‑line for maximum ILP; emit **compensation code** at each side exit to fix up the off‑trace
  cases. This is the original VLIW region model and is the right default for our acyclic hot code.
- **Hyperblocks (Mahlke et al. 1992) / predicated regions.** Where `DESIGN.md` §3.1 says "predicate both
  arms," the dynarec forms a hyperblock: it *if‑converts* short divergences into straight‑line predicated
  code (each op tagged with a predicate guard), so list scheduling sees one region with no branch. This is
  how we get the "both arms in one bundle, zero redirect" behavior. The trade (set in `DESIGN.md` §11) is
  that the not‑taken arm's ops still consume lanes — so if‑convert only when the arms are short and the
  predicate is unpredictable.

The **critical region‑selection insight for this CPU**: to fill 1000 lanes on a graph workload, the trace
selector must form **horizontal traces that span independent streams**, not just vertical traces down one
control path. If each linked list walk is its own trace, list scheduling sees a serial chain and fills one
lane. If `k` independent walks are fused into one superblock, list scheduling sees `k` independent chains
with **no inter‑stream edges** — so all `k` root ops are ready at once and the greedy packer puts them in
one bundle. (See the worked example, §C.) Trace fusion across independent call sites / loop nests is
therefore a first‑class dynarec pass.

### A.3 Loops: **modulo scheduling (software pipelining)** — and where it does NOT help here

For loops, the best algorithm is **modulo scheduling (Rau 1994)**, which overlaps iterations to hide
latency. The dynarec computes the **minimum initiation interval**:

- **ResMII** = max over resources of `⌈ops‑of‑that‑type per iteration / available‑per‑cycle⌉`.
- **RecMII** = max over loop‑carried recurrence cycles of `⌈Σ latency / dependence distance⌉`.
- **MII = max(ResMII, RecMII)**; schedule the kernel so iteration `i+1` starts `MII` cycles after `i`;
  emit prologue/epilogue (we have no hardware register rotation in v3, so the dynarec tracks modulo
  variants itself, or unrolls to materialize them).

**The pointer‑chase caveat (central to this CPU).** A *single* linked‑list walk is
`S[si] ← MEM[S[si]]` — a **loop‑carried dependence of distance 1 through memory**, latency `Lmem`.
So **RecMII ≥ Lmem + 1 ≈ Lmem**, and modulo scheduling a single chain **cannot** overlap its iterations
(each pointer depends on the previous load). This is a hard, fundamental bound: one serial list is
memory‑latency bound at ~1 advance/`Lmem` no matter how clever the scheduler.

The parallelism that fills the 1000 lanes is **inter‑stream, not intra‑stream**: `k` *independent* lists
are `k` independent recurrences with no edges between them, so list scheduling (A.1) on a fused region
(A.2) packs their `k` `ldp.next` ops into one bundle, delivering `min(k, 1000)` advances per cycle.
Modulo scheduling is still the right tool for *ordinary* loops in the body (histograms, reductions over the
loaded data) — just not for advancing a single pointer chain.

### A.4 A prefix‑sum–shaped VLIW list scheduler (the optimized form for this CPU)

The list scheduler in §A.1 is *sequential*: a cycle‑by‑cycle loop with a priority queue, where each placement
mutates the ready set. That is the general‑purpose form. For **this** machine it collapses to a
**data‑parallel, prefix‑sum–shaped computation** — the GPU "scan → scatter" pattern — because of one
machine‑specific fact:

> **On the BPS‑V core, lane capacity (≤ `W` ops/bundle) is the single dominant scheduling resource.**
> The ARF bank constraint is provably satisfiable by the allocator whenever `W ≤ #banks` (§B.4), and
> predicate‑port conflicts are rare and fixable in a post‑pass. So the scheduler reduces to
> **single‑resource bin packing with arrival times** — and that is exactly a prefix sum.

**Why single‑resource ⇒ prefix‑sum.** Classical list scheduling's priority queue is *adaptive*: the order in
which ops are considered depends on prior placements (the ready set changes). That adaptivity is what makes
it look sequential. Remove it in two steps:

1. **Pre‑compute readiness** as `r(v) = ASAP(v)`, the longest latency‑weighted path from a source (DAG
   longest path — itself a parallel computation, §A.4.1). Readiness no longer depends on placement.
2. **Pre‑determine priority** (critical‑path / mobility) — also placement‑independent.

Once the consideration order `(r(v) asc, priority(v) desc)` is fixed *a priori*, greedy first‑fit
bin‑packing into bins of size `W = 1000` is no longer adaptive: op `v` is delayed only by ops ahead of it in
the fixed order, and that delay is exactly the **cumulative count of earlier ops** — a prefix sum. That is
the whole reduction.

#### A.4.1 Phase R — readiness (parallel longest path)

`r(v) = ASAP(v) = max over preds p of (r(p) + lat(p))`, with `r = 0` at sources. This is longest‑path on a
DAG, computed by:

- **Layered/levelized sweep**: process ops grouped by topological level; each level's `r` is a max‑reduce
  over its predecessors' `r + lat`. Fully parallel *within* a level. Work O(V+E).
- Or a **(max,+) relaxation / parallel Bellman–Ford** if levels are inconvenient to materialize.

This is the dependence‑respecting floor; it is what guarantees correctness. `Lmem` enters here: an edge from
a load to its consumer carries weight `Lmem`, pushing the consumer's `r` out by the load latency — the §A.1
"latency slotting" emerges for free.

#### A.4.2 Phase P — packing as a segmented prefix sum

**Sort** all ops by `(r(v) asc, priority(v) desc)` — a **radix sort on `r`** (r is a small integer, the
bundle count, typically a few hundred for a trace) with a priority tiebreak. Radix sort on a small key is
O(V) work, O(log V) span. Now two regimes:

**Dense regime (the target workload — fused k ≥ 1000 streams).** Here the ready curve is full
(`count[r] ≥ W` for the early bundles); no op is ever starved by an empty bundle, so readiness never forces a
gap, and the assignment is simply:

```
c(v) = ⌊ rank(v) / W ⌋          // rank = position in the sorted array; W = 1000
```

That is the whole scheduler — **an integer divide of each op's sorted‑index by the bundle width**, O(1) per
op, embarrassingly parallel. This is the cleanest "prefix‑sum on GPU" form: the rank *is* the inclusive
prefix sum of the all‑ones array, and dividing by W buckets ops into bundles exactly like mapping GPU
work‑items to work‑groups/CTAs. For 1000 fused independent walks whose roots all have `r = 0`, the 1000 roots
take ranks `0..999` → bundle 0; their latency‑gated successors take later rank ranges → later bundles
(gated by `r` already baked into the sort key). Optimal packing, zero sequencing, zero priority‑queue.

**General regime (sparse ready curve — e.g. one wide fork that narrows).** Some bundles are starved
(`count[r] < W`), so `⌊rank/W⌋` can land *before* an op's ready bundle and violate correctness (`c(v) <
r(v)`). The fix is a **carry between segments** — the classic GPU "segmented scan with cross‑segment carry":

1. **Histogram** `count[r]` = #ops ready at bundle r; **inclusive prefix sum** `cum[r] = Σ_{r'≤r} count[r']`
   over the (few) distinct ready‑bundles.
2. **Sweep over ready‑bundles** carrying overflow. The bundle range for ready‑group r is
   `start[r] = max( r, ⌈cum[r−1] / W⌉ )`, and its ops fill contiguously from `start[r]`. This is a monotone
   forward scan over the distinct r values (≤ schedule length, small) — sequential *over bundles* but
   **vectorized over ops** within each group.
3. **Scatter**: the k‑th op (priority order) in group r lands at `start[r] + ⌊(k + spill[r]) / W⌋`, where
   `spill[r]` is the partial‑fill offset inherited from the previous group. A local index‑divide,
   data‑parallel per group.

The whole Phase P is therefore **sort + scan + scatter** — the canonical GPU data‑parallel pipeline. The
`max(r, ⌈cum/W⌉)` clamp is the only piece that isn't a pure scan, and it runs over the small bundle
dimension, not the op dimension.

#### A.4.3 Correctness sketch

- **Respect dependence:** `c(v) ≥ r(v)` by construction — dense regime because the full ready curve keeps
  `⌊rank/W⌋ ≥ r`; general regime by the explicit `max(r, …)` clamp.
- **Respect capacity:** each bundle receives exactly the ops whose assigned index falls in its
  `[b·W, (b+1)·W)` rank window — at most W, by construction of the divide.
- **Optimality (dense regime):** when `count[r] ≥ W` throughout, no bundle is starved, every bundle is
  exactly full, and the schedule length equals `⌈V/W⌉` — the information‑theoretic minimum given the
  capacity — while still respecting every `r(v)`. (Length optimality subject to readiness is the best any
  scheduler can claim; CP priority additionally keeps critical chains short within that bound.)

#### A.4.4 Complexity (work / span, GPU‑style)

| Step | Work | Span (depth) | Notes |
|---|---|---|---|
| Phase R (longest path) | O(V+E) | O(D) | D = topological depth (max‑reduce per level) |
| Sort by (r, −prio) | O(V) | O(log V) | radix sort on integer r |
| Dense assign `⌊rank/W⌋` | O(V) | O(1) | one divide per op |
| General carry‑scan | O(B + V) | O(B + log V) | B = #distinct ready‑bundles (small) |

Contrast with sequential list scheduling's O(V log V) work *and* O(V) span — the priority‑queue loop is fully
serial. The prefix‑sum form has **O(log V) span**, so it parallelizes. That matters because the dynarec runs
at runtime: a 1000‑wide fused trace can have hundreds of thousands of ops, and a serial O(V) scheduler would
dominate translation time. With this form the dynarec can schedule a wide trace on a few cores (or vector
units) in time proportional to log‑depth, not op‑count.

#### A.4.5 Why this is valid *here* — and the width threshold where it breaks

The collapse to single‑resource (and hence to prefix‑sum) is **specific to the BPS‑V design choices**, and
it has a **hard width threshold** that the doc's earlier "banks are not a scheduler resource" glossed over.

**The threshold: `W ≤ #banks`.** The prefix‑sum form is valid **only while lane capacity is the sole binding
resource**, which requires that the register allocator can always hand out bank‑distinct slots — i.e. that
the number of ARF‑touching ops in any bundle never exceeds the bank count (§B.4). At the spec width:

- **`W = 1000`, `#banks = 1024`:** `1000 ≤ 1024` ✓. Banks are *not* a scheduling resource; one binding
  dimension (lanes); `⌊rank/W⌋` works. (Tight: only 24 banks of slack if *every* op in a bundle touches
  the ARF, which a pure pointer‑chase bundle does. Practically you'd want `#banks ≥ 2·W` for comfort.)

**What breaks at `W = 10,000` with `#banks = 1024`.** Now `W > #banks`, and by the pigeonhole principle
any *full* bundle has ⌈10,000/1024⌉ ≈ 10 ARF‑touching ops per bank — **violating the "≤1 access/bank/bundle"
rule**. The bank constraint stops being something the allocator fixes after the fact and becomes a **second,
simultaneously‑binding scheduling dimension** alongside lane capacity. At that point:

- An op consumes a **capacity vector** `(1 lane, 1 bank-slot, and a specific bank that must differ from
  every other op's)`, and the bundle has a capacity vector `(10,000 lanes, 1024 bank-slots, 1‑per‑bank)`.
- You can't divide a 1‑D rank by a capacity vector, so `c(v) = ⌊rank(v)/W⌋` is dead. "Which subset of ops
  fits together" becomes **vector bin packing** (NP‑hard) **plus a graph‑coloring sub‑problem** (assign
  distinct banks within the chosen subset). This is exactly the multi‑dimensional bin packing the general
  VLIW form (§A.1) exists to handle.

**The two sane responses at `W = 10,000`:**

1. **Scale the banks with the width** — keep `#banks ≥ W` (≥10,000 banks) and the collapse is restored:
   banks stay non‑binding, the prefix‑sum stays valid. The cost is SRAM area and crossbar ports, **not
   algorithmic complexity.** This is the clean architectural answer and almost certainly the right one at
   10,000‑wide: if you're paying for 10,000 lanes, paying for 10,000+ banks keeps the dynarec fast.
2. **Accept vector bin packing** and give up the prefix‑sum. Practical algorithms for the dense pointer‑chase
   case: a **hierarchical / two‑level scan** — tile the bundle into bank‑groups, prefix‑scan lanes *within*
   each group and prefix‑scan ARF‑ops *across* groups (this is GPU work‑group tiling generalized to two
   levels, and it stays O(log V) span while respecting both dimensions). For irregular regions, **vector
   First‑Fit‑Decreasing** (greedy, near‑linear, constant‑factor approximation) or the §A.1 priority‑queue
   scheduler. In all of these the schedule length grows by the load‑factor `⌈W/#banks⌉ ≈ 10×` because the
   bank dimension throttles throughput — you have 10,000 lanes but only 1024 can advance a pointer per
   bundle, so the extra width is wasted on pointer chases. That waste is the real argument for response (1).

**The other validity conditions** (unchanged): one FU class dominates (homogeneous lanes; a VLIW with scarce
mixed FUs can't collapse); latency is fixed & known (variable latency forces the adaptive priority queue
back).

So the corrected rule: **§A.1 is the general algorithm; §A.4 is its optimized, parallel, prefix‑sum form,
valid iff `W ≤ #banks` (and comfortable at `#banks ≥ 2·W`).** Use §A.4 on hot fused‑stream traces at spec
width; either scale banks with width, or fall back to §A.1 / the two‑level scan, if width outgrows banks.

> **PG2T390H instance note.** The target build (DESIGN.md §13) is `W = 32`, `#banks = 64 = 2·W`, comfortably
> inside the prefix‑sum's validity region — the default online hash scheduler (§A.5) is the right choice
> there; §A.4 only earns its keep on traces large enough that serial lookups dominate translation time, which
> at `W = 32` is rare. The `W > #banks` collapse described above is an ASIC‑scale or mis‑configured concern,
> not something the on‑FPGA build hits.

#### A.4.6 Extension: a few secondary resources via per‑resource max

If predicate‑port conflicts (≤1 write/pK/bundle) or a future FU limit can't be ignored, generalize: compute
a *separate* rank‑divide `c_t(v) = ⌊rank_t(v)/cap_t⌋` for each resource t, and take `c(v) = max_t c_t(v)`.
This over‑approximates slightly (it assumes resources pack independently) but stays prefix‑sum‑shaped and is
exact when resources are nearly orthogonal — which they are here (lanes vs. the rare predicate write). A tiny
sequential fixup pass repairs the few residual conflicts; it touches O(conflicts) ops, not O(V).

> Note this is *not* the W‑outgrows‑banks case of §A.4.5: predicate ports are scalar capacities (≤1/pK) that
> compose by a max; banks are a *coupled* capacity (the specific bank chosen for op A removes it from op B's
> options), which is what forces the two‑level scan / vector bin packing. Don't conflate the two.

### A.5 Online hash‑map scheduler — the simple default

§A.1 (list scheduling) and §A.4 (prefix‑sum) are both *offline*: they build a dependence graph first, then
schedule against it. For a **runtime** dynarec that's overkill for almost every trace. The simpler, faster,
and **default** algorithm is an **online hash‑map scheduler**: stream the IR ops in trace order once, and for
each op, hash its dependence key to find the earliest bundle it can land in, then append it there. **O(1)
amortized per op, single pass, no DDG, no sort, no graph coloring.**

This is what real JIT dynarecs (HP Dynamo, QEMU's TCG, Web engines) actually do for the common case; the
graph‑based forms are reserved for huge hot regions.

#### A.5.1 The algorithm

State: a hash map `M: depKey → latestBundleInWhichItWasWritten`, plus the growing bundle list `B[]` each
holding up to `W` ops. The `depKey` is whatever identifies a value uniquely within the trace: the SSA value
name, or (for ARF‑slot values) the **slot index** — see §A.5.3 for why the slot index is the crucial key.

```
for each op v in trace order:
    earliest = 0
    for each operand key k of v:
        m[k]  = M.lookup(k)          // bundle in which k was last written
        lat   = producerLatency(k)   // = Lmem if k's producer was a load, else 1
        earliest = max(earliest, m[k] + lat)

    // pick the bundle: earliest legal, that isn't already full
    b = earliest
    while full(B[b]): b += 1         // skip past full bundles (rare; see A.5.5)

    place v in B[b], assign its lane within the bundle
    M[key_of_v_output] = b           // remember where v's result lands, for its consumers
```

That's the whole scheduler. The hash lookup gives the **dependence‑respecting floor** (replacing §A.1's
DDG); the full‑bundle skip gives the **capacity constraint** (replacing §A.4's `⌊rank/W⌋`); the append is
just adding to a list. No priority queue, no sort, no scan, no graph.

**Correctness** falls out directly:
- *Dependence*: `earliest ≥ producerBundle + lat` for every operand, so v never lands before its inputs are
  ready — this is the §A.1 latency constraint, obtained by a hash lookup instead of a DDG edge walk.
- *Capacity*: the `full()` skip guarantees ≤ W ops/bundle — this is §A.4's `⌊rank/W⌋`, obtained by a scan
  instead of a divide.
- *Termination*: each op advances at least to `earliest`, which is monotonic in trace order for
  well‑formed (acyclic) IR; the bundle list grows as needed.

#### A.5.2 When to use it vs. §A.1 / §A.4

| Situation | Scheduler | Why |
|---|---|---|
| **Most traces (default)** | **Online hash (A.5)** | O(1)/op, single pass; quality is ~greedy‑list and good enough for runtime |
| Small irregular region | §A.1 list scheduling | the sort/graph cost isn't worth it for small N, and the hash version is fine too |
| **Huge hot trace** (10^5+ ops, fused 1000‑stream) | **§A.4 prefix‑sum** | needs *parallelism in the dynarec itself* to keep translation time bounded; O(log V) span |
| Tight steady‑state loop | §A.3 modulo scheduling | hash scheduler handles loops but can't overlap iterations; modulo can (where RecMII allows) |

The hash scheduler is the **90% case**. §A.4 is the specialization for when a single trace is so large that
serial O(V) scheduling — even at O(1)/op — dominates translation time. §A.1 is the reference/fallback.

#### A.5.3 Why ARF pinning (the freq counter) makes this work for pointer chases

Here is the connection to the hardware frequency counter that your question is really getting at — and it's
the key reason this algorithm is a good fit for this CPU.

**The problem without pinning.** A generic load `ld rd, addr` keys on the *address* value, which in a
pointer chase is a different 32‑bit number every iteration (`addr` = the just‑loaded pointer). So the
scheduling map `M[addr]` almost never hits — every `ld` looks independent to the scheduler and they all
want bundle 0. The dynarec then has to do expensive alias/points‑to analysis to reconstruct the true
serialization, or get it wrong and emit overlapping loads that violate the chase dependency.

**Pin the hot address, and the key becomes stable.** The hardware freq counter
(`DESIGN.md` §8 / `ibex_hot_addr_detect.sv`) observes `hash(memaddr) → count` *at runtime*; addresses above
threshold get **pinned into an ARF slot `si`**. The chase then becomes `ldp.next si` (`S[si] ← MEM[S[si]]`),
and its dependence key is the **slot index `si`** — a small, stable integer that is identical across every
iteration of the walk. Now `M[si]` **hits every iteration**: iteration k's `ldp.next si` looks up where
iteration k−1's `ldp.next si` landed (`b_{k-1}`), and schedules itself at `b_{k-1} + Lmem` — **the correct
serial dependency, found by a single hash lookup, with zero alias analysis.**

So the two mechanisms compose:

- **Freq counter (HW) → pinning → stable slot keys (RA).** The hardware detector identifies the hot
  address; the dynarec pins it to a slot; now the dependence has a hashable, stable name.
- **Hash scheduler (dynarec) consumes those keys.** `M[si]` returns the producer's bundle; the scheduler
  places the consumer at `+ Lmem`. No DDG, no alias analysis, no points‑to.

**Important: these are two different maps.** Don't conflate them:
- `freq: hash(memaddr) → count` — populated **at runtime** by hardware, **past‑tense**, drives **pinning**.
- `M: depKey(slot/SSA‑name) → bundleIndex` — populated **at compile time** by the dynarec as it emits ops,
  drives **scheduling**.

They differ in time, in key space (memory address vs. slot/SSA name), and in purpose (pinning vs.
scheduling). The freq counter does **not** run the scheduler. What it does is **make the scheduler's key
stable and cheap**, by turning a per‑iteration‑unique address into a per‑trace‑stable slot index. Without
pinning, the hash scheduler can't see pointer‑chase dependencies; with pinning, it sees them as O(1)
lookups. **That is the real and only coupling**, and it's a good one.

(They also share infrastructure: the dynarec uses the same `hash()` primitive for both; and freq‑table
recency can *seed* `M` at trace entry, so the most recently hot addresses are pre‑mapped to early bundles —
a minor optimization, not the core mechanism.)

#### A.5.4 Worked example: `W` independent walks, hashed

Each walk `i` is pinned to a distinct free‑pool slot `s_i` in a distinct bank (e.g. `s_i = 0x2000 + i`, banks
`0..W−1`). Trace = `W` `ldp.next s_i` in source order:

- Op `ldp.next s_0`: `M.lookup(s_0)` misses (first use) → `earliest = 0` → bundle 0; `M[s_0] = 0`.
- Op `ldp.next s_1`: `M.lookup(s_1)` misses → `earliest = 0`, bundle 0 not full (1/W) → bundle 0;
  `M[s_1] = 0`.
- … all `W` roots land in bundle 0 (each keys on a distinct slot, no inter‑walk dependency).
- Next iteration: `ldp.next s_0` → `M[s_0] = 0`, `Lmem` → `earliest = 0 + Lmem` → bundle `Lmem`. Same for
  all `W`. Bundle `Lmem` holds the second advance of all `W` walks.

Result: `W` advances/bundle, schedule length `≈ iterations × Lmem`, **discovered by `W` hash lookups per
iteration with no graph, no sort, no scan.** Identical to the §A.4 prefix‑sum answer, arrived at by the much
simpler machinery — because for *independent* streams the online greedy order is already the optimal order.
The prefix‑sum only earns its keep when the trace is large enough that the lookups themselves (still serial
in the dynarec) become the bottleneck. (On the PG2T390H instance, `W = 32` — see DESIGN.md §13.)

#### A.5.5 Caveats and knobs

- **The full‑bundle skip** (`while full(B[b]): b += 1`) is the only place capacity bites, and it's rare for
  the dense‑stream target (bundles stay full by construction). In the worst case it's a linear walk over
  bundles, but in practice you track `first‑not‑full bundle` as a running cursor → O(1) amortized.
- **Hash collisions / aliasing.** Two unrelated values hashing to the same `depKey` would create a false
  dependence (over‑serialization, safe but wasteful). Mitigate with a few bits of the value/SSA id in the
  key, or accept the rare stall — correctness is never at risk, only packing quality.
- **No critical‑path priority.** The online scheduler processes ops in trace (source) order, not CP order,
  so it can occasionally defer a critical op behind a non‑critical one that happens to come first. This is
  the quality gap vs. §A.1; it's small for fused‑stream traces (the work is naturally independent) and
  usually irrelevant at runtime. For a region where it bites, re‑order the IR by a cheap CP estimate before
  feeding the hash scheduler, or fall back to §A.1.
- **Loops.** The hash scheduler handles loops (the slot key is stable across iterations, §A.5.3) but
  serializes them — it cannot overlap iterations the way §A.3 modulo scheduling can. Use §A.3 for tight
  steady‑state loops where overlapping iterations is legal and profitable.

### A.6 Scheduling, summarized

| Code shape | Algorithm | Why |
|---|---|---|
| **Most traces (the default)** | **Online hash‑map scheduler** (§A.5): hash depKey → producer bundle, append at `max(prod+lat)`, skip full bundles | O(1)/op, single pass, no DDG; quality ~greedy; relies on ARF pinning for stable keys on pointer chases |
| Acyclic hot code | **List scheduling, critical‑path priority** (§A.1) on a **trace** (§A.2) | Best practical heuristic; greedy & fast; CP is the dominant priority |
| **Huge hot fused‑stream trace** | **Prefix‑sum list scheduler** (§A.4): sort + scan + `⌊rank/W⌋` scatter | Single‑resource ⇒ collapses to prefix sum; O(log V) span, data‑parallel; only worth it when §A.5's serial lookups dominate translation time |
| Short divergences | **If‑convert to hyperblock**, then list schedule (§A.2) | Eliminates the branch → predication (no mispredict), keeps one region |
| Ordinary loops | **Modulo scheduling** (§A.3) | Hides latency across iterations |
| `k` independent pointer walks | **Fuse into one superblock** + online hash scheduler (§A.2 + §A.5) | Streams are independent → pack `k` advances/bundle; slot keys from pinning make the chase dependency an O(1) lookup; single‑chain is RecMII ≈ Lmem bound |

---

## B. Register allocation: which register holds which value?

### B.1 Two register classes, allocated separately

This machine has **two disjoint register files**, and they are allocated by different algorithms:

| Class | File | Size | Pressure | Algorithm |
|---|---|---|---|---|
| **Data** | integer GPRs (`x0..x31`) | 32 | real (small file) | **Linear scan on SSA** (§B.2) |
| **Address** | ARF slots (`0x00000..0x1FFFF`) | 128K (default; `AddrRegFileDepth` param); free pool ~122K | essentially none | **Bank‑aware slot assignment** (§B.4) |

Addresses and data live in different files by design (`DESIGN.md` §5.3 — the GPR holds only data in
steady‑state chase code), so the two allocations don't interfere except through the **cross‑class copies**
that load/store values bridge (a `ldp` reads an address slot, writes a data GPR). Those copies are just IR
edges with a fixed port mapping; they don't create inter‑file conflicts.

### B.2 Best algorithm for the GPRs: **linear scan on SSA** (Poletto‑Sarkar + Wimmer‑Franz)

The two serious candidates:

- **Graph coloring (Chaitin 1981 / Briggs 1992).** Best *quality*, but it builds and repeatedly simplifies
  an interference graph and iterates on spilling — too slow for a runtime dynarec, and the quality margin
  over linear scan on SSA is small in practice.
- **Linear scan (Poletto & Sarkar 1999), specifically the **SSA‑based variant (Wimmer & Franz 2010)**.**
  **This is the right choice for a dynarec.** It is `O(n log n)` (one sort of live ranges by start point,
  then a single sweep), produces code within a few percent of graph coloring, is what production JITs
  (LLVM's, HotSpot's) actually use, and SSA‑form gives it the quality boost that closes most of the gap.

**How it runs on the data values:**

1. **Live‑range analysis** over the *scheduled* bundle list (allocation happens *after* scheduling — see
   §B.3). Because latency is exposed, a value's live range spans from its defining bundle to the last
   bundle that reads it, *including the `Lmem`‑bundle gap* a load's result sits idle in.
2. **Sort live ranges** by start bundle.
3. **Sweep**, keeping active live ranges in a priority structure keyed by end bundle:
   - On a new range starting: free any expired ranges (end < now) back to the free pool of GPRs; assign
     the new range a free GPR. If none free, **spill** the range whose end is farthest in the future
     (Belady's rule — optimal on a fixed schedule; the *target* is chosen by the two-tier policy of §D:
     an ARF scratch slot via `slotw`, or the stack via `sw`/`lw` for long distances).
   - On a range ending: free its GPR.
4. **Insert moves at SSA φ‑block boundaries** (the standard split‑edge move insertion; on this machine a
   move is one lane op, easily co‑scheduled).

Because the file is only 32 GPRs, spills do happen on data‑heavy bodies; the dynarec should coalesce moves
and keep the live‑set small by re‑reading from ARF‑backed addresses rather than holding many pointers as
GPRs (which is exactly the `DESIGN.md` design — addresses are *not* in the GPR file).

### B.3 Order of scheduling and allocation (the VLIW interaction)

Scheduling and allocation interact: scheduling *before* allocation exposes ILP but can raise register
pressure (long live ranges across parallel bundles) → spills; allocating *before* scheduling introduces
false (anti/output) dependences → constrains the schedule. The standard, correct ordering for VLIW is:

> **Schedule first on virtual (unbounded) registers → allocate → if spills are excessive, re‑schedule
> the spilled region with anti/output edges added, then re‑allocate.** Usually one pass suffices.

We lean on this: §A runs on a DDG with **only true dependences** (infinite virtual regs), which is what
lets list scheduling see maximum parallelism. §B then back‑fills the real assignments. For the address
class, pressure is ~zero (~122K free slots at the PG2T390H instance), so the ordering problem barely exists there.

### B.4 Best algorithm for ARF slots: **bank‑aware assignment** (not classic RA)

The ARF is not a normal register file, so classic RA is the wrong frame:

- **There is no scarcity.** ~122K free slots (128K ARF, PG2T390H instance) vs. a live‑set of, at most, a few
  thousand active address values → the allocator essentially **never spills an address**. "Which slot" is a
  *placement* decision, not a contention decision.
- **The real constraint is the bank rule** (`DESIGN.md` §4.4): within one bundle, two slot accesses must
  land in **distinct banks** (`#banks` banks, one access/bank/bundle). Since the dynarec assigns slots, it
  can satisfy this *by construction*.
- **There is a fixed (precolored) subset**: the explicit table `0x000..0x3FF` and any PINNED addresses are
  statically assigned by the dynarec and must not be moved.

So the algorithm is **constraint‑based placement**:

1. **Classify each address value:** PINNED (static, e.g. a jump‑table base) → fixed slot, precolored;
   runtime‑hot (promoted from the recommendation queue, `DESIGN.md` §8) → PINNED into the HW working set;
   **pointer‑chase working pointer** → a free‑pool slot, allocated by `spalloc` and managed by the dynarec.
2. **Build a co‑bundle interference graph** for the free‑pool values: an edge between two address values
   iff they are **both accessed in the same bundle**. (Values only ever used in different bundles never
   conflict on banks.)
3. **Color the graph with bank‑colors** so adjacent values differ in bank. With `#banks ≥ W` this is trivial
   — a greedy first‑fit in bundle order always succeeds; the graph's chromatic number is bounded by the
   number of distinct slots accessed in any one bundle, which is ≤ `W` ≤ `#banks`.
4. **Pick a concrete slot** within the assigned bank from the free list (or reuse a just‑freed slot).

Because the chromatic number is bounded by the number of distinct slots accessed in any one bundle, **the
bank constraint is satisfiable iff that number ≤ #banks.** At the instance width this holds: ≤`W` ARF‑touching
ops/bundle ≤ `#banks` → **the dynarec never has to serialize for banks** *if it assigns slots with this
pass*; the core's bank‑conflict trap (`DESIGN.md` §4.4) is only a backstop for a buggy allocator.

**This is W‑dependent.** If width grows past the bank count (e.g. `W = 10,000`, `#banks = 1024`), a full
bundle has ⌈10,000/1024⌉ ≈ 10 ARF‑touching ops/bank — the graph is **no longer (1024‑)colorable**,
allocation can't fix it after the fact, and the bank constraint becomes a second binding scheduling
dimension (see §A.4.5). The fix is architectural (scale `#banks ≥ W`) or algorithmic (the two‑level scan /
vector bin packing of §A.4.5), not allocator‑side. So this §B.4 pass presumes `W ≤ #banks`; that precondition
is the same one §A.4's prefix‑sum needs.

> Practical note: the simplest correct policy is **"assign each free‑pool pointer its own dedicated slot in
> a dedicated bank for the whole trace."** `W` lanes × 1 walk each = `W` slots in `W` banks, assigned once at
> trace entry (`spalloc` ×`W`), freed at exit. Zero intra‑trace reassignment, zero conflicts. This is what
> the worked example (§C) does.

### B.5 Register allocation, summarized

| Class | Algorithm | Spills? | Notes |
|---|---|---|---|
| Data GPRs | **Linear scan on SSA** (§B.2), *after* scheduling (§B.3) | yes — per §D (remat → ARF scratch → stack); keep live‑set small by leaving addresses in ARF | `O(n log n)`; production‑JIT quality |
| Address ARF slots | **Bank‑aware placement** (§B.4): precolor PINNED; first‑fit bank‑color the co‑bundle interference graph | ~never (~122K free slots) | satisfiable iff `W ≤ #banks` (instance: 32 ≤ 64); dedicated‑slot‑per‑walk is the simple policy |

---

## C. Worked example: scheduling & allocating `W` independent list walks

Source (the canonical case this CPU exists for):

```c
for (int i = 0; i < W; i++)               // W independent linked lists (W = 32 on PG2T390H)
    while (list[i]) { use(list[i]->data); list[i] = list[i]->next; }
```

**(1) Region formation (§A.2).** The dynarec recognizes `W` independent walks and **fuses them into one
superblock**. The DDG has `W` connected components, each a serial chain
(`ldp.next s_i` → use → `ldp.next s_i`), with **no edges between components**.

**(2) Slot allocation (§B.4).** Assign each walk a dedicated free‑pool slot in a dedicated bank:
`s_i = 0x2000 + i`, banks `0..W−1`. PINNED/seed each with its list head via `pina` at trace entry. No
intra‑trace reallocation; bank constraint satisfied trivially (distinct banks). Data values (`list[i]->data`
into a GPR) go through linear scan — `W` short live ranges, fits in 32 GPRs across the body.

**(3) Scheduling (§A.5 online hash scheduler).** All `W` `ldp.next s_i` ops are ready at cycle 0 (no
inter‑stream edges), each needs one lane + one mem port + one bank, all distinct → **all `W` pack into
bundle 0** (each keys on a distinct slot, no hash collisions). The bundle completes when the slowest of the
`W` loads returns (`Lmem` later — the §11 bundle‑gating cost). Consumers (the `use` bodies) are scheduled
into bundles `Lmem .. Lmem+k` as their keys dictate. Throughput: **`W` list advances per `Lmem` bundles** ≈
`W/Lmem` advances/cycle, i.e. ~lane‑saturation until data‑mem bandwidth is the bound.

**(4) Control flow (§3 of DESIGN.md).** Each walk's `while` exit is a predicate; the superblock is a
predicated loop with no branch on the hot path. When all `W` lists are exhausted, a single resolved
branch exits the trace.

**Contrast — a single walk:** DDG is one chain; list scheduling fills 1 lane, throughput ≈ 1 advance/`Lmem`
(memory‑bound), the other `W−1` lanes idle. This is the fundamental serial bound; no scheduler beats it. The
wide engine's value is entirely in the multi‑stream case above.

---

## D. Spilling to SRAM: when and what

This section resolves the open knob (§F) into the concrete spill policy §B.2 defers to: when to spill at
all, which value, and to which memory. Three machine-specific facts shape the answer:

- **The schedule is fixed when the allocator runs** (§B.3), so every value's def and use bundles are known
  exactly. Spill choice on a fixed schedule is the paging problem, whose optimal victim is **furthest next
  use** (Belady 1966) — optimal for a fixed reference string with uniform replacement cost. Linear scan's
  stock heuristic (spill the active range with the latest end) approximates Belady; on SSA with use lists
  the dynarec computes the exact next-use distance, so it can run *literal* Belady — a luxury no dynamic
  machine has, because its use distances are estimates.
- **Spill cost is a two-tier latency hierarchy, not a constant.** Tier 1: an ARF free-pool slot used as a
  data scratch via `slotw`/`slotr`. This is the deliberate widening of the "addresses only" rule (DESIGN.md
  §4.1), and it is safe: scratch slots never alias architecturally visible memory, so the no-coherence
  property (DESIGN.md §5.4) is untouched — the dynarec must simply never *use* a scratch value as an
  address. Cost: 1 bundle for the store, 1 for the reload — but both are ARF-touching ops that must satisfy
  the bank rule (DESIGN.md §4.4) in their bundles. Tier 2: the stack via `sw`/`lw`. The store costs an LSU
  port; the reload costs an LSU port and must issue `Lmem` bundles before the use to be free.
- **Lanes are abundant; bank slots and LSU ports are scarce.** Bundles are rarely full (§A.5.5), so a spare
  lane costs ~nothing. This inverts the classic spill-vs-rematerialize trade.

### D.1 The algorithm: remat-first, Belady victim, admission-controlled insertion

When the §B.2 sweep needs a GPR and none is free:

1. **Rematerialize before spilling anything.** If any candidate value is recomputable in the consumer's
   bundle from values live there — constants, 1-latency ALU functions of live values, or a re-`ldp` of a
   pinned slot (the address is already in the ARF, so the re-load needs no address remat) — drop the value
   and re-issue the defining op at the use. Remat spends the cheap resource (a lane) instead of the scarce
   ones (bank slots, LSU ports); on this machine it outranks spilling, not the other way around.
2. **Pick the victim by furthest next use (Belady), with admission-aware tie-breaks.** Compute `d(v)` =
   distance to next use for each active range; spill max `d(v)`. Tie-break toward: (a) clean values (a copy
   already exists — no store needed), (b) values whose reload bundle has a free ARF bank slot (a tier-1
   reload can't force a reschedule), (c) load-defined values (rematerializable per step 1, so spilling them
   is free).
3. **Pick the tier by distance.** If the use is ≥ `Lmem` bundles away and some bundle in `[now, use−Lmem]`
   has a free LSU port → **tier 2 (stack)**, and place the reload *early* at the first such bundle, so the
   memory latency is hidden off the critical path. Otherwise → **tier 1 (ARF scratch)**.
4. **Insert; don't reschedule.** The store needs a free lane (and, tier 1, a free bank slot) in the current
   bundle; the reload needs its slot/port in its bundle. Both are ARF-/LSU-touching ops and join the §B.4
   co-bundle coloring — the `2·W` bank margin (DESIGN.md §4.4) exists partly to absorb them. If the target
   bundle is full, take the next bundle with room for a store, or the nearest earlier legal bundle for a
   reload. Only when even that fails does §B.3's single re-schedule pass fire — capped, per §F.

### D.2 What is provable, what is heuristic

- Single tier, uniform cost, fixed schedule: furthest-next-use is **optimal** (Belady 1966). A dynarec gets
  to run a provably optimal memory algorithm because the future is known.
- Two tiers + bank/LSU admission + remat interact into an NP-hard whole (it contains resource-constrained
  scheduling). The policy above is the standard near-optimal practical form, and spill volume is low here
  anyway: addresses never enter the GPR file (§B.1), so only data pressure spills, and the free pool
  (~122K slots) makes tier-1 exhaustion a non-event.

---

## E. Multi-cluster: cycle-aware allocation across N clusters

The multi-cluster build (DESIGN.md §13, "Multi-cluster variant") replicates the §7 engine `N` times: each
cluster has its own `W` lanes, GPRs, ARF (`#banks ≥ W`), instruction banks, and data partition, and a
pipelined interconnect moves a 32-bit word between clusters in a fixed, exposed `Lhop` bundles. `Lhop` is a
compile-time constant, like `Lmem` — which is why this section is short: **everything the scheduler already
does with `Lmem`, it does with `Lhop`, unchanged.**

### E.1 Partition first: the fused DDG already names the clusters

Cluster assignment happens between region formation (§A.2) and scheduling (§A.5), on the DDG:

1. **Atoms = connected components.** After horizontal trace fusion the DDG is `k` mostly-disconnected stream
   components (§C). Components are the atoms — never split one stream's chain across clusters: its
   `ldp.next` recurrence would pay `Lhop` per advance, the §A.3 RecMII bound with a worse constant. (The §C
   worked example scales verbatim: fuse `k = N·W` streams, place `W` per cluster; each cluster's bundle 0
   holds its own `W` `ldp.next` roots.)
2. **Balanced assignment: LPT bin-packing.** Sort components by op count, place each on the least-loaded
   cluster — O(k log k), within 4/3 of optimal makespan (Graham 1969), and balance only needs to be loose
   (±10%) because clusters don't gate each other (DESIGN.md §13, caveat 3). Placement constraints: the
   cluster's ARF working set ≤ its free pool; its stream count ≤ its data banks.
3. **Cut refinement: greedy migration.** Where components interact (joins, reductions), repeatedly move the
   component whose migration most reduces Σ cut-edge weights (values crossing × uses), until no improvement
   — Kernighan–Lin truncated to one linear-ish pass, per the runtime budget (§0). The result is a local
   optimum of an objective (transfer traffic) that is itself an approximation of schedule length; do not
   spend more here.

### E.2 Transfers are DDG nodes: the scheduler does not change

Split every cut edge producer `p` (cluster `c_p`) → consumer `q` (cluster `c_q`) mechanically:

```
p @ c_p ──1──▶ send @ c_p ──Lhop──▶ recv @ c_q ──1──▶ q @ c_q
```

- `send` occupies a lane in `c_p`; `recv` occupies a lane in `c_q`, `Lhop` bundles later; the link itself is
  a scalar capacity (≤ words/link/bundle) that composes by `max` exactly like the predicate ports of §A.4.6.
- In the online hash scheduler (§A.5) the change is one line: after placing `send` in bundle `b`, record
  `M[key] = b + Lhop` **in `c_q`'s map**. The consumer then lands at `≥ b + Lhop + 1` by the ordinary
  lookup — the §A.4.1 latency-slotting fall-out, applied to a second latency class. Each cluster keeps its
  own bundle list, its own `M`, its own linear scan (§B.2), its own §B.4 bank coloring under its own
  `#banks ≥ W` precondition.
- **Cycle-aware by construction.** The allocator knows, for every cluster and every bundle, the exact
  lane/bank/link occupancy, because it placed every op. A received value's live range starts at its `recv`
  bundle (it is a def there); a value needed in `j` clusters is sent `j` times and holds a register in each,
  which the partitioner prices as cut weight.

### E.3 Allocation consequences

- **A walk never migrates mid-trace.** Its state is a slot in one cluster's ARF; moving it costs a transfer
  of the contents plus `spalloc`/`spfree` (≈ `Lhop` + 2 bundles) and buys nothing, because streams are
  long-lived and the partition was balanced. Walk→cluster assignment is fixed at trace entry; re-partition
  on re-translation (a deopt event), never mid-trace.
- **Spilling gains a third tier.** The §D hierarchy becomes: local ARF scratch (1 bundle) < remote ARF
  scratch (`Lhop` + 1 — a `slotw` forwarded over the interconnect) < stack (`Lmem`). Victim selection and
  admission control are unchanged; the remote tier is the overflow valve that lets the allocator degrade
  instead of fail. It should fire ~never: each cluster's free pool is ~122K/`N` slots against a working set
  of thousands.
- **There is no cross-cluster ARF access.** `ldp`/`stp`/`ldp.next` address only the local ARF. An address
  value needed in another cluster moves as data over the interconnect and is written into a local slot on
  the receive side (`slotw`/`pina`). The §4.4 bank rule stays a purely local property: per-cluster coloring,
  no global coordination.

### E.4 What is provable, what is heuristic

- Joint partition + schedule + allocation is NP-hard (it contains graph partitioning). The pipeline above —
  partition, then per-cluster §A.5 + §B — is the standard decomposition; each stage is a near-linear
  heuristic from this document.
- **The approximation lives in the partition, not in the cycle accounting.** With the partition fixed, each
  per-cluster schedule is exactly as good as §A.5 on the same ops, because `Lhop` enters as ordinary
  weighted edges and per-cluster resources are independent. LPT is within 4/3 of optimal balance; greedy
  migration is a local optimum. Nothing in §A/§B weakens.

---

## F. Knobs and open questions

- **`Lmem` assumption.** Scheduling quality depends on the dynarec's `Lmem` being accurate. If memory
  latency is variable (banked SRAM with contention), the dynarec should schedule to a conservative
  percentile and let the bundle‑completion interlock absorb the tail — or split into independent
  sub‑engines (`DESIGN.md` §11) so one slow load doesn't gate the other `W−1`.
- **Trace‑fusion heuristics.** How aggressively to fuse independent streams into one superblock is the
  single biggest scheduler knob for this workload. Too little → 1‑wide; too much → huge superblocks with
  poor I‑cache locality and imbalance (one long list gating many short ones). A length‑bounded,
  bank‑aware fusion is the starting point.
- **Spill policy for data GPRs.** **Resolved — see §D**: remat-first, Belady victim selection, and a tiered
  target (local ARF scratch → remote ARF → stack). With addresses out of the GPR file, data pressure is the
  only spill source.
- **Re‑scheduling on spill.** §B.3's re‑schedule pass is the main place translation cost can blow up; cap
  it (e.g., one re‑schedule, else accept the spill) to keep dynarec latency bounded.
- **Quality vs. translation speed.** List scheduling + linear scan + bank placement are all
  near‑linear; the dynarec should stay well under a few thousand cycles per emitted bundle even at width
  1000, which amortizes quickly for hot traces.

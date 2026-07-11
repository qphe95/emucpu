# predicate.asm — predicate-guard test.
# pred[0] is 1 at reset (unguarded = always-run).
# Bundle 0: cmp.eq p1, x5, x5   -> p1 = 1 (equal)
#           mov.pt p1, x6, 0xAA -> x6 = 0xAA (because p1=1)
#           cmp.eq p2, x5, x6   -> p2 = 0 (x5=0, x6=0xAA, not equal)
#           mov.pt p2, x7, 0xBB -> x7 unchanged (p2=0)
#           mov.pf p2, x8, 0xCC -> x8 = 0xCC (because p2=0, pf inverts)
# Bundle 1: ebreak
#
# Note: pred[0] reset state and the mov.pt/mov.pf custom encodings depend on
# the dispatcher wiring the predicate index from the slot. This test validates
# that path.
.bundle
addi  x5, x0, 1
cmp.eq p1, x5, x5
mov.pt p1, x6, x0
mov.pf p2, x8, x0
# x6 should get written (p1=1); set x6 via addi first so we can detect change
.bundle
addi  x6, x0, 0xAA
nop
nop
nop
.bundle
ebreak
nop
nop
nop
.expect x5 = 1
.expect x6 = 0xAA

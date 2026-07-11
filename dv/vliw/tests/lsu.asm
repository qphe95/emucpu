# lsu.asm — load/store test against the external memory model.
# The on-chip window is [0, 0x80000); anything outside hits the external bus.
# We store to an external address and load it back.
#   Bundle 0: set up addresses + values in GPRs.
#   Bundle 1: store x6 to the address in x5.
#   Bundle 2: load from the address in x5 into x7.
#   Bundle 3: ebreak.
.bundle
lui  x5, 0x10000     # x5 = 0x10000000 (external region)
addi x6, x0, 0x2A    # value to store
addi x8, x0, 0x00
nop
.bundle
sw   x6, 0(x5)       # MEM[0x10000000] <- 0x2A
nop
nop
nop
.bundle
lw   x7, 0(x5)       # x7 <- MEM[0x10000000]
nop
nop
nop
.bundle
ebreak
nop
nop
nop
.expect x5 = 0x10000000
.expect x6 = 0x2A
.expect x7 = 0x2A
.expect_mem 0x10000000 = 0x2A

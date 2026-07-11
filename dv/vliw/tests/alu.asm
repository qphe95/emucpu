# alu.asm — basic ALU test.  W=4 for a small smoke test.
# Bundle 0: four independent ADDIs that each write a distinct GPR.
#   x5 <- 0x10, x6 <- 0x20, x7 <- 0x30, x8 <- 0x40
# Bundle 1: add x9 = x5 + x6, add x10 = x7 + x8, sub x11 = x8 - x5, xor x12 = x6 ^ x7
# Bundle 2: ebreak (halt) on lane 0 only.
.bundle
addi x5,  x0, 0x10
addi x6,  x0, 0x20
addi x7,  x0, 0x30
addi x8,  x0, 0x40
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
.expect x6  = 0x20
.expect x7  = 0x30
.expect x8  = 0x40
.expect x9  = 0x30
.expect x10 = 0x70
.expect x11 = 0x30
.expect x12 = 0x10

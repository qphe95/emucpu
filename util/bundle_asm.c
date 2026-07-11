// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// bundle_asm.c — a self-contained assembler for BPS-V VLIW bundles.
//
// Takes a simple assembly file, encodes each instruction to 32-bit RV32I +
// the BPS-V ARF custom opcodes, packages W instructions into 64-bit bundle
// slots (32-bit instr | (pred_idx<<32) | (invert<<36)), and emits:
//   - <name>.img    : flat binary of 64-bit little-endian slots, bundle 0
//                     slot 0 first.
//   - <name>.expect : expected GPR register file at halt, one "xN=hex" per
//                     line, for the testbench to check.
//
// No binutils dependency — the encoder is built in.
//
// Usage: bundle_asm <width> <asm_file> [<out_stem>]
//
// See util/bundle_asm.py (deleted) or DESIGN.md §6/§12 for the instruction set.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <ctype.h>

// ---------------------------------------------------------------------------
// Opcodes
// ---------------------------------------------------------------------------
enum {
    OPC_OP_IMM = 0x13, OPC_OP = 0x33, OPC_LUI = 0x37, OPC_AUIPC = 0x17,
    OPC_JAL = 0x6f, OPC_JALR = 0x67, OPC_BRANCH = 0x63, OPC_LOAD = 0x03,
    OPC_STORE = 0x23, OPC_SYSTEM = 0x73,
    OPC_CUSTOM_0 = 0x0b, OPC_CUSTOM_1 = 0x2b,
};

// BPS-V funct3 values (must match ibex_bps_pkg).
enum {
    C0_SLOTR=0, C0_PINA=1, C0_UNPIN=2, C0_LDP=3,
    C0_STP=4, C0_LDPI=5, C0_STPI=6, C0_LDP_NEXT=7,
    C1_SPHINT=0, C1_SPLR=1, C1_SPFREE=2, C1_SPALLOC=3,
    C1_LDPCAP=4, C1_SPFLUSH=7, C1_WJ=5, C1_PCMP=6,
};

// ---------------------------------------------------------------------------
// Instruction encoders
// ---------------------------------------------------------------------------
static uint32_t enc_r(uint32_t f7, uint32_t rs2, uint32_t rs1,
                      uint32_t f3, uint32_t rd, uint32_t opc) {
    return (f7<<25)|(rs2<<20)|(rs1<<15)|(f3<<12)|(rd<<7)|opc;
}
static uint32_t enc_i(int32_t imm, uint32_t rs1, uint32_t f3,
                      uint32_t rd, uint32_t opc) {
    return ((imm & 0xfff)<<20)|(rs1<<15)|(f3<<12)|(rd<<7)|opc;
}
static uint32_t enc_s(int32_t imm, uint32_t rs2, uint32_t rs1,
                      uint32_t f3, uint32_t opc) {
    uint32_t i = imm & 0xfff;
    return (((i>>5)&0x7f)<<25)|(rs2<<20)|(rs1<<15)|(f3<<12)|((i&0x1f)<<7)|opc;
}
static uint32_t enc_u(int32_t imm, uint32_t rd, uint32_t opc) {
    return ((imm & 0xfffff)<<12)|(rd<<7)|opc;
}
static uint32_t enc_j(int32_t imm, uint32_t rd, uint32_t opc) {
    uint32_t i = imm & 0x1fffff;
    return (((i>>20)&1)<<31)|(((i>>1)&0x3ff)<<21)|(((i>>12)&0xff)<<12)|
           (((i>>11)&1)<<20)|(rd<<7)|opc;
}
static uint32_t enc_b(int32_t imm, uint32_t rs2, uint32_t rs1,
                      uint32_t f3, uint32_t opc) {
    uint32_t i = imm & 0x1fff;
    return ((i&1)<<31)|(((i>>5)&0x3f)<<25)|(rs2<<20)|(rs1<<15)|(f3<<12)|
           (((i>>1)&0xf)<<7)|(((i>>11)&1)<<20)|opc;
}

// ---------------------------------------------------------------------------
// Register parsing
// ---------------------------------------------------------------------------
static const char *ALIASES[] = {
    "zero","ra","sp","gp","tp","t0","t1","t2","s0","s1",
    "a0","a1","a2","a3","a4","a5","a6","a7",
    "s2","s3","s4","s5","s6","s7","s8","s9","s10","s11",
    "t3","t4","t5","t6", NULL
};

static int parse_reg(const char *tok) {
    // strip trailing comma/whitespace
    static char buf[64];
    int n = 0;
    while (tok[n] && tok[n] != ',' && tok[n] != ' ' && n < 63) {
        buf[n] = tok[n]; n++;
    }
    buf[n] = '\0';
    for (int i = 0; ALIASES[i]; i++)
        if (strcmp(buf, ALIASES[i]) == 0) return i;
    if (buf[0] == 'x' || buf[0] == 'X') return atoi(buf+1);
    // predicate register p0..p7 — return as 0..7 (for funct3=PCMP)
    if (buf[0] == 'p' || buf[0] == 'P') return atoi(buf+1);
    fprintf(stderr, "bad register: %s\n", buf); exit(1);
}

static int parse_imm(const char *tok) {
    static char buf[64]; int n = 0;
    while (tok[n] && tok[n] != ',' && tok[n] != ' ' && n < 63) {
        buf[n] = tok[n]; n++;
    }
    buf[n] = '\0';
    return (int)strtol(buf, NULL, 0);
}

// ---------------------------------------------------------------------------
// ALU funct3/funct7 tables
// ---------------------------------------------------------------------------
struct alu_r { const char *name; int f3, f7; };
static const struct alu_r ALU_R[] = {
    {"add",0,0b0000000},{"sub",0,0b0100000},{"sll",1,0b0000000},
    {"slt",2,0b0000000},{"sltu",3,0b0000000},{"xor",4,0b0000000},
    {"srl",5,0b0000000},{"sra",5,0b0100000},{"or",6,0b0000000},
    {"and",7,0b0000000},{NULL,0,0},
};
struct alu_i { const char *name; int f3; int is_sh; };
static const struct alu_i ALU_I[] = {
    {"addi",0,0},{"slti",2,0},{"sltiu",3,0},{"xori",4,0},
    {"ori",6,0},{"andi",7,0},{"slli",1,1},{"srli",5,1},{"srai",5,1},{NULL,0,0},
};

// ---------------------------------------------------------------------------
// Operand tokenizer: splits a comma-separated operand string into tokens.
// Returns count; fills argv[].
// ---------------------------------------------------------------------------
static int split_ops(char *ops_str, char *argv[], int max) {
    int n = 0;
    char *p = strtok(ops_str, ",");
    while (p && n < max) {
        while (*p == ' ') p++;
        argv[n++] = p;
        p = strtok(NULL, ",");
    }
    return n;
}

// ---------------------------------------------------------------------------
// Encode one instruction. ops[] are the raw operand tokens.
// ---------------------------------------------------------------------------
static uint32_t encode_one(const char *mnem, char *ops[], int nops) {
    // RV32I
    if (!strcmp(mnem,"nop"))    return enc_i(0,0,0,0,OPC_OP_IMM);
    if (!strcmp(mnem,"ebreak")) return enc_i(1,0,0,0,OPC_SYSTEM);
    if (!strcmp(mnem,"ecall"))  return enc_i(0,0,0,0,OPC_SYSTEM);
    if (!strcmp(mnem,"lui"))    return enc_u(parse_imm(ops[1]), parse_reg(ops[0]), OPC_LUI);
    if (!strcmp(mnem,"auipc"))  return enc_u(parse_imm(ops[1]), parse_reg(ops[0]), OPC_AUIPC);

    for (int i = 0; ALU_R[i].name; i++) {
        if (!strcmp(mnem, ALU_R[i].name))
            return enc_r(ALU_R[i].f7, parse_reg(ops[1]), parse_reg(ops[0]),
                         ALU_R[i].f3, parse_reg(ops[0]), OPC_OP);
    }
    for (int i = 0; ALU_I[i].name; i++) {
        if (!strcmp(mnem, ALU_I[i].name)) {
            int rd = parse_reg(ops[0]), rs1 = parse_reg(ops[1]);
            int imm = parse_imm(ops[2]);
            int f7 = 0;
            if (ALU_I[i].is_sh) {
                f7 = (mnem[2]=='r' && mnem[3]=='a') ? 0b0100000 : 0b0000000; // srai
                return enc_i((f7<<5)|(imm&0x1f), rs1, ALU_I[i].f3, rd, OPC_OP_IMM);
            }
            return enc_i(imm, rs1, ALU_I[i].f3, rd, OPC_OP_IMM);
        }
    }

    if (!strcmp(mnem,"jal"))
        return enc_j(parse_imm(ops[1]), parse_reg(ops[0]), OPC_JAL);
    if (!strcmp(mnem,"jalr"))
        return enc_i(parse_imm(ops[2])&0xfff, parse_reg(ops[1]), 0,
                     parse_reg(ops[0]), OPC_JALR);

    // Branches
    int bf3 = -1;
    if (!strcmp(mnem,"beq"))  bf3 = 0;
    if (!strcmp(mnem,"bne"))  bf3 = 1;
    if (!strcmp(mnem,"blt"))  bf3 = 4;
    if (!strcmp(mnem,"bge"))  bf3 = 5;
    if (!strcmp(mnem,"bltu")) bf3 = 6;
    if (!strcmp(mnem,"bgeu")) bf3 = 7;
    if (bf3 >= 0)
        return enc_b(parse_imm(ops[2]), parse_reg(ops[1]), parse_reg(ops[0]),
                     bf3, OPC_BRANCH);

    // Loads: lw rd, imm(rs1)
    int lf3 = -1;
    if (!strcmp(mnem,"lb"))  lf3 = 0;
    if (!strcmp(mnem,"lh"))  lf3 = 1;
    if (!strcmp(mnem,"lw"))  lf3 = 2;
    if (!strcmp(mnem,"lbu")) lf3 = 4;
    if (!strcmp(mnem,"lhu")) lf3 = 5;
    if (lf3 >= 0) {
        // ops[1] = "imm(rs1)" — parse manually
        char *paren = strchr(ops[1], '(');
        int off = (int)strtol(ops[1], NULL, 0);
        char *rs1tok = paren + 1;
        char *close = strchr(rs1tok, ')');
        if (close) *close = '\0';
        return enc_i(off & 0xfff, parse_reg(rs1tok), lf3, parse_reg(ops[0]), OPC_LOAD);
    }

    // Stores: sw rs2, imm(rs1)
    int sf3 = -1;
    if (!strcmp(mnem,"sb")) sf3 = 0;
    if (!strcmp(mnem,"sh")) sf3 = 1;
    if (!strcmp(mnem,"sw")) sf3 = 2;
    if (sf3 >= 0) {
        char *paren = strchr(ops[1], '(');
        int off = (int)strtol(ops[1], NULL, 0);
        char *rs1tok = paren + 1;
        char *close = strchr(rs1tok, ')');
        if (close) *close = '\0';
        return enc_s(off, parse_reg(ops[0]), parse_reg(rs1tok), sf3, OPC_STORE);
    }

    // BPS-V custom (CUSTOM-0)
    if (!strcmp(mnem,"slotr"))   return enc_i(0,parse_reg(ops[1]),C0_SLOTR,parse_reg(ops[0]),OPC_CUSTOM_0);
    if (!strcmp(mnem,"pina"))    return enc_r(0,parse_reg(ops[1]),parse_reg(ops[0]),C0_PINA,0,OPC_CUSTOM_0);
    if (!strcmp(mnem,"unpin"))   return enc_i(0,parse_reg(ops[0]),C0_UNPIN,0,OPC_CUSTOM_0);
    if (!strcmp(mnem,"ldp"))     return enc_i(0,parse_reg(ops[1]),C0_LDP,parse_reg(ops[0]),OPC_CUSTOM_0);
    if (!strcmp(mnem,"stp"))     return enc_s(0,parse_reg(ops[0]),parse_reg(ops[1]),C0_STP,OPC_CUSTOM_0);
    if (!strcmp(mnem,"ldpi"))    return enc_i(parse_imm(ops[1]),0,C0_LDPI,parse_reg(ops[0]),OPC_CUSTOM_0);
    if (!strcmp(mnem,"stpi"))    return enc_s(parse_imm(ops[1]),parse_reg(ops[0]),0,C0_STPI,OPC_CUSTOM_0);
    if (!strcmp(mnem,"ldp.next"))return enc_i(0,parse_reg(ops[0]),C0_LDP_NEXT,0,OPC_CUSTOM_0);

    // BPS-V custom (CUSTOM-1)
    if (!strcmp(mnem,"ldpcap"))  return enc_i(0,parse_reg(ops[1]),C1_LDPCAP,parse_reg(ops[0]),OPC_CUSTOM_1);
    if (!strcmp(mnem,"spalloc")) return enc_i(0,0,C1_SPALLOC,parse_reg(ops[0]),OPC_CUSTOM_1);
    if (!strcmp(mnem,"spfree"))  return enc_i(0,parse_reg(ops[0]),C1_SPFREE,0,OPC_CUSTOM_1);
    if (!strcmp(mnem,"sphint"))  return enc_i(0,parse_reg(ops[1]),C1_SPHINT,parse_reg(ops[0]),OPC_CUSTOM_1);
    if (!strcmp(mnem,"splr"))    return enc_i(0,parse_reg(ops[1]),C1_SPLR,parse_reg(ops[0]),OPC_CUSTOM_1);
    if (!strcmp(mnem,"spflush")) return enc_i(0,0,C1_SPFLUSH,0,OPC_CUSTOM_1);

    // Predicate ops: cmp.eq pK, ra, rb
    if (!strncmp(mnem,"cmp.",4)) {
        const char *cond = mnem+4;
        int w3 = 0;
        if (!strcmp(cond,"eq")) w3=0; else if (!strcmp(cond,"ne")) w3=1;
        else if (!strcmp(cond,"lt")) w3=4; else if (!strcmp(cond,"ge")) w3=5;
        else if (!strcmp(cond,"ltu")) w3=6; else if (!strcmp(cond,"geu")) w3=7;
        // pK in rd field, ra in rs1, rb in rs2; funct7 = condition code.
        return enc_r(w3, parse_reg(ops[2]), parse_reg(ops[1]), C1_PCMP,
                     parse_reg(ops[0]), OPC_CUSTOM_1);
    }
    if (!strcmp(mnem,"mov.pt"))
        return enc_r(0b0100000, parse_reg(ops[2]), 0, C1_PCMP, parse_reg(ops[0]), OPC_CUSTOM_1);
    if (!strcmp(mnem,"mov.pf"))
        return enc_r(0b0100001, parse_reg(ops[2]), 0, C1_PCMP, parse_reg(ops[0]), OPC_CUSTOM_1);

    fprintf(stderr, "unknown mnemonic: %s\n", mnem);
    exit(1);
}

// ---------------------------------------------------------------------------
// Parse a guard prefix [pK] or [~pK] from the line; return rest-of-line.
// ---------------------------------------------------------------------------
static const char *parse_guard(const char *line, int *pred, int *invert) {
    *pred = 0; *invert = 0;
    if (line[0] == '[') {
        const char *p = line + 1;
        if (*p == '~') { *invert = 1; p++; }
        if (*p == 'p' || *p == 'P') { *pred = atoi(p+1); }
        while (*p && *p != ']') p++;
        if (*p == ']') p++;
        while (*p == ' ') p++;
        return p;
    }
    return line;
}

// ---------------------------------------------------------------------------
// Output buffers (dynamic)
// ---------------------------------------------------------------------------
static uint64_t *img_slots = NULL;
static int img_count = 0, img_cap = 0;

struct expect_entry { int is_mem; uint32_t idx; uint32_t val; };
static struct expect_entry *exps = NULL;
static int exp_count = 0, exp_cap = 0;

static void img_push(uint64_t slot) {
    if (img_count >= img_cap) {
        img_cap = img_cap ? img_cap * 2 : 256;
        img_slots = realloc(img_slots, img_cap * sizeof(uint64_t));
    }
    img_slots[img_count++] = slot;
}
static void exp_push(int is_mem, uint32_t idx, uint32_t val) {
    if (exp_count >= exp_cap) {
        exp_cap = exp_cap ? exp_cap * 2 : 64;
        exps = realloc(exps, exp_cap * sizeof(struct expect_entry));
    }
    exps[exp_count].is_mem = is_mem;
    exps[exp_count].idx = idx;
    exps[exp_count].val = val;
    exp_count++;
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(int argc, char **argv) {
    if (argc < 3) {
        fprintf(stderr, "usage: %s <width> <asm_file> [<out_stem>]\n", argv[0]);
        return 2;
    }
    int width = atoi(argv[1]);
    const char *path = argv[2];
    char stem[1024];
    if (argc > 3) {
        strncpy(stem, argv[3], sizeof(stem)-1);
    } else {
        // strip extension from path
        strncpy(stem, path, sizeof(stem)-1);
        char *dot = strrchr(stem, '.');
        if (dot) *dot = '\0';
    }
    stem[sizeof(stem)-1] = '\0';

    FILE *f = fopen(path, "r");
    if (!f) { fprintf(stderr, "cannot open %s\n", path); return 1; }

    // Current bundle: array of (instr, pred, invert)
    uint32_t cur_instr[256]; int cur_pred[256]; int cur_inv[256]; int cur_n = 0;
    int nbundles = 0;

    char line[512];
    while (fgets(line, sizeof(line), f)) {
        // strip comment
        char *hash = strchr(line, '#');
        if (hash) *hash = '\0';
        // strip trailing whitespace
        int len = strlen(line);
        while (len > 0 && isspace((unsigned char)line[len-1])) line[--len] = '\0';
        // skip empty
        char *p = line;
        while (*p && isspace((unsigned char)*p)) p++;
        if (*p == '\0') continue;

        // Directives
        if (!strncmp(p, ".bundle", 7)) {
            // flush current bundle (skip empty — the leading .bundle before
            // any instructions is just a marker, not a real bundle).
            if (cur_n > 0) {
                for (int i = cur_n; i < width; i++) {
                    cur_instr[i] = enc_i(0,0,0,0,OPC_OP_IMM); cur_pred[i] = 0; cur_inv[i] = 0;
                }
                for (int i = 0; i < width; i++) {
                    uint64_t slot = (uint64_t)cur_instr[i] |
                                    ((uint64_t)cur_pred[i] << 32) |
                                    ((uint64_t)cur_inv[i] << 36);
                    img_push(slot);
                }
                nbundles++;
            }
            cur_n = 0;
            continue;
        }
        if (!strncmp(p, ".pad", 4)) {
            int n = atoi(p + 4);
            for (int i = 0; i < n; i++) {
                if (cur_n < 256) {
                    cur_instr[cur_n] = enc_i(0,0,0,0,OPC_OP_IMM);
                    cur_pred[cur_n] = 0; cur_inv[cur_n] = 0; cur_n++;
                }
            }
            continue;
        }
        if (!strncmp(p, ".expect_mem", 11)) {
            // .expect_mem 0xADDR = 0xVAL
            uint32_t addr = 0, val = 0;
            char *eq = strchr(p, '=');
            if (eq) {
                addr = (uint32_t)strtoul(p + 11, NULL, 0);
                val = (uint32_t)strtoul(eq + 1, NULL, 0);
            }
            exp_push(1, addr, val);
            continue;
        }
        if (!strncmp(p, ".expect", 7)) {
            // .expect xN = 0xVAL
            char *eq = strchr(p, '=');
            if (eq) {
                uint32_t idx = (uint32_t)parse_reg(p + 8);
                uint32_t val = (uint32_t)strtoul(eq + 1, NULL, 0);
                exp_push(0, idx, val);
            }
            continue;
        }

        // Instruction
        int pred = 0, invert = 0;
        const char *rest = parse_guard(p, &pred, &invert);
        // Split mnemonic + operands
        char buf[512];
        strncpy(buf, rest, sizeof(buf)-1); buf[sizeof(buf)-1] = '\0';
        char *space = strchr(buf, ' ');
        char *mnem, *ops_str = NULL;
        if (space) { *space = '\0'; mnem = buf; ops_str = space + 1; }
        else { mnem = buf; }

        char *ops[8]; int nops = 0;
        if (ops_str) nops = split_ops(ops_str, ops, 8);

        uint32_t instr = encode_one(mnem, ops, nops);
        if (cur_n < 256) {
            cur_instr[cur_n] = instr;
            cur_pred[cur_n] = pred;
            cur_inv[cur_n] = invert;
            cur_n++;
        }
    }
    fclose(f);

    // Flush last bundle
    if (cur_n > 0) {
        for (int i = cur_n; i < width; i++) {
            cur_instr[i] = enc_i(0,0,0,0,OPC_OP_IMM); cur_pred[i] = 0; cur_inv[i] = 0;
        }
        for (int i = 0; i < width; i++) {
            uint64_t slot = (uint64_t)cur_instr[i] |
                            ((uint64_t)cur_pred[i] << 32) |
                            ((uint64_t)cur_inv[i] << 36);
            img_push(slot);
        }
        nbundles++;
    }

    // Write .img
    char outpath[1152];
    snprintf(outpath, sizeof(outpath), "%s.img", stem);
    FILE *of = fopen(outpath, "wb");
    if (!of) { fprintf(stderr, "cannot write %s\n", outpath); return 1; }
    for (int i = 0; i < img_count; i++) {
        uint64_t s = img_slots[i];
        for (int b = 0; b < 8; b++) fputc((s >> (b*8)) & 0xff, of);
    }
    fclose(of);

    // Write .expect
    snprintf(outpath, sizeof(outpath), "%s.expect", stem);
    of = fopen(outpath, "w");
    if (!of) { fprintf(stderr, "cannot write %s\n", outpath); return 1; }
    for (int i = 0; i < exp_count; i++) {
        if (exps[i].is_mem)
            fprintf(of, "mem[0x%08x]=0x%08x\n", exps[i].idx, exps[i].val);
        else
            fprintf(of, "x%u=0x%08x\n", exps[i].idx, exps[i].val);
    }
    fclose(of);

    printf("assembled %d bundles (%d slots, %d bytes) -> %s.img\n",
           nbundles, nbundles*width, nbundles*width*8, stem);
    printf("%d expectations -> %s.expect\n", exp_count, stem);

    return 0;
}

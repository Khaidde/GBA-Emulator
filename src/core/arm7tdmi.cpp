#include "arm7tdmi.hpp"

#include <cstdarg>

#include "memory.hpp"

#define ARM_ERR(...)                   \
    do {                               \
        printf("...at PC=%08x\n", PC); \
        FATAL(__VA_ARGS__);            \
    } while (0)

#ifndef NDEBUG
#define ARM_ASSERT(cond, ...)     \
    do {                          \
        if (!(cond)) {            \
            ARM_ERR(__VA_ARGS__); \
        }                         \
    } while (0)
#else
#define ARM_ASSERT(cond, ...) \
    do {                      \
    } while (0)
#endif

enum class InstrType {
    FLUSHED = 0,
    PENDING,
    B_BL,
    BX,
    SWI,
    BKPT,
    MULT,
    PSR,
    DATA_PROCCESS,
    SINGLE_DATA_TRANSFER,
    HW_DW_SIGNED_DATA_TRANSFER,
    BLOCK_TRANSFER,
    SWAP,
};

enum PSRMask : u32 {
    N = 0x80000000,
    Z = 0x40000000,
    C = 0x20000000,
    V = 0x10000000,

    IRQn = 0x00000080,
    FIQn = 0x00000040,
    THUMB = 0x00000020,
    MODE = 0x1F,
};

enum ModeBits : char {
    USR = 0x10,
    FIQ = 0x11,
    IRQ = 0x12,
    SVC = 0x13,
    ABT = 0x17,
    UND = 0x18,
    SYS = 0x1F,
};

static int sign_extend(char signBit, u32 val) {
#ifndef NDEBUG
    if (signBit >= 31) {
        FATAL("Can't sign extend number with signBit >= 31: val=%08x", val);
    }
#endif
    return (int)(~((val & (1 << signBit)) - 1) | val);
}

ARM::ARM(Bus* bus) : bus(bus) {
    for (int i = 0; i < 8; i++) {
        r[i] = &genRegLo[i];
    }
    for (int i = 0; i < 5; i++) {
        r[i + 8] = &genRegHi[i];
    }
    r[13] = &bankedSP.usr;
    r[14] = &bankedLR.usr;
    r[15] = &PC;
    CPSR = 0;
    SPSR = nullptr;
    bankedSPSR.fiq = 0;
    bankedSPSR.svc = 0;
    bankedSPSR.abt = 0;
    bankedSPSR.irq = 0;
    bankedSPSR.und = 0;
}

void ARM::x_regs() {
    for (int i = 0; i < 8; i++) {
        printf("  r%1d=%08x", i, genRegLo[i]);

        if (i < 5) {
            printf("  r%02d=%08x", i + 8, genRegHi[i]);
            printf("  %7s=%08x", ("r" + std::to_string(i + 8) + "_fiq").c_str(), fiqReg[i]);
        }
        if (i == 5) {
            printf("  r13=%08x", bankedSP.usr);
            printf("  r13_fiq=%08x", bankedSP.fiq);
            printf("  r13_svc=%08x", bankedSP.svc);
            printf("  r13_abt=%08x", bankedSP.abt);
            printf("  r13_irq=%08x", bankedSP.irq);
            printf("  r13_und=%08x", bankedSP.und);
        }
        if (i == 6) {
            printf("  r14=%08x", bankedLR.usr);
            printf("  r14_fiq=%08x", bankedLR.fiq);
            printf("  r14_svc=%08x", bankedLR.svc);
            printf("  r14_abt=%08x", bankedLR.abt);
            printf("  r14_irq=%08x", bankedLR.irq);
            printf("  r14_und=%08x", bankedLR.und);
        }
        if (i == 7) {
            printf("  r15=%08x", PC);
        }

        printf("\n");
    }
    printf("CPSR=%08x", CPSR);
    printf("              ");
    printf(" SPSR_fiq=%08x", bankedSPSR.fiq);
    printf(" SPSR_svc=%08x", bankedSPSR.svc);
    printf(" SPSR_abt=%08x", bankedSPSR.abt);
    printf(" SPSR_irq=%08x", bankedSPSR.irq);
    printf(" SPSR_und=%08x\n", bankedSPSR.und);
}

void ARM::reset() {
    for (u32 i = 0; i < 16; i++) {
        *r[i] = i + 0x100;
    }

    PC = 0x80000000;
    CPSR = 0;
    SPSR = nullptr;

    pipelineIdx = 0;
    flush_pipeline();
}

void ARM::cycle() {
    log("--");
    ARM_ASSERT((PC & 0x3) == 0, "Last instruction misaligned PC");
    ARM_ASSERT(PC >= 0x80000000, "Invalid PC which is less than 0x80000000");

    // Fetch
    pipeline[pipelineIdx].opcode = bus->mem->read32(PC);
    pipeline[pipelineIdx].type = InstrType::PENDING;

    // Decode
    u8 decodePipeIdx = (pipelineIdx + 2) % 3;
    u32 opcode = pipeline[decodePipeIdx].opcode;
    if (pipeline[decodePipeIdx].type != InstrType::FLUSHED) {
        if ((opcode & 0x0E000010) == 0x06000010) {
            ARM_ERR("TODO undefined not implemented: %08x", opcode);
        } else if ((opcode & 0x0E000000) == 0x0A000000) {
            pipeline[decodePipeIdx].type = InstrType::B_BL;
        } else if ((opcode & 0x0FFFFF00) == 0x012FFF00) {
            pipeline[decodePipeIdx].type = InstrType::BX;
        } else if ((opcode & 0x0F000000) == 0x0F000000) {
            pipeline[decodePipeIdx].type = InstrType::SWI;
        } else if ((opcode & 0x0C000000) == 0) {
            if ((opcode & 0x0FB00FF0) == 0x01000090) {
                pipeline[decodePipeIdx].type = InstrType::SWAP;
            } else if ((opcode & 0x0E0000F0) == 0x90) {
                pipeline[decodePipeIdx].type = InstrType::MULT;
            } else if ((opcode & 0x0E000090) == 0x90) {
                pipeline[decodePipeIdx].type = InstrType::HW_DW_SIGNED_DATA_TRANSFER;
            } else if ((opcode & 0x01900000) == 0x01000000) {
                pipeline[decodePipeIdx].type = InstrType::PSR;
            } else {
                pipeline[decodePipeIdx].type = InstrType::DATA_PROCCESS;
            }
        } else if ((opcode & 0x0C000000) == 0x04000000) {
            pipeline[decodePipeIdx].type = InstrType::SINGLE_DATA_TRANSFER;
        } else if ((opcode & 0x0E000000) == 0x08000000) {
            pipeline[decodePipeIdx].type = InstrType::BLOCK_TRANSFER;
        } else {
            ARM_ERR("TODO unknown decode for instruction: %08x", opcode);
        }
    }

    // Execute
    u8 executePipeIdx = (pipelineIdx + 1) % 3;
    execOpcode = pipeline[executePipeIdx].opcode;
    if (pipeline[executePipeIdx].type != InstrType::FLUSHED) {
        log("%08x:\t%08x: ", PC - 8, execOpcode);
        if (check_cond(execOpcode >> 28)) {
            switch (pipeline[executePipeIdx].type) {
                case InstrType::B_BL: arm_b_bl(); break;
                case InstrType::BX: arm_bx(); break;
                case InstrType::DATA_PROCCESS: arm_data_proc(); break;
                case InstrType::PSR: arm_psr(); break;
                case InstrType::MULT: arm_mul(); break;
                case InstrType::SINGLE_DATA_TRANSFER: arm_single_data(); break;
                case InstrType::HW_DW_SIGNED_DATA_TRANSFER: arm_halfword_signed_data(); break;
                case InstrType::SWAP: arm_swap(); break;
                case InstrType::BLOCK_TRANSFER: arm_block_transfer(); break;
                case InstrType::FLUSHED: break;
                default: ARM_ERR("TODO unknown execute for instruction");
            }
        } else {
            log("(NZCV=%01x) (cond=%01x) skipped instruction...\n", CPSR >> 28, execOpcode >> 28);
        }
    } else {
        log("\n");
    }
    if (pipeline[pipelineIdx].type != InstrType::FLUSHED) {
        PC += 4;
    }
    pipelineIdx = (pipelineIdx + 1) % 3;
}

void ARM::log(const char* format, ...) {
#ifndef NDEBUG
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
#endif
}

void ARM::update_mode(u32 psr) {
    if ((CPSR ^ psr) & 0x1F) {
        u8 mode = psr & 0x1F;
        if (mode != ModeBits::FIQ) {
            for (int i = 0; i < 5; i++) {
                ref.reg_8_12[i] = &genRegHi[i];
            }
        }
        switch (mode) {
            case ModeBits::SYS:
            case ModeBits::USR:
                ref.SP = &bankedSP.usr;
                ref.LR = &bankedLR.usr;
                SPSR = nullptr;
                break;
            case ModeBits::FIQ:
                for (int i = 0; i < 5; i++) {
                    ref.reg_8_12[i] = &fiqReg[i];
                }
                ref.SP = &bankedSP.fiq;
                ref.LR = &bankedLR.fiq;
                SPSR = &bankedSPSR.fiq;
                break;
            case ModeBits::SVC:
                ref.SP = &bankedSP.svc;
                ref.LR = &bankedLR.svc;
                SPSR = &bankedSPSR.svc;
                break;
            case ModeBits::ABT:
                ref.SP = &bankedSP.abt;
                ref.LR = &bankedLR.abt;
                SPSR = &bankedSPSR.abt;
                break;
            case ModeBits::IRQ:
                ref.SP = &bankedSP.irq;
                ref.LR = &bankedLR.irq;
                SPSR = &bankedSPSR.irq;
                break;
            case ModeBits::UND:
                ref.SP = &bankedSP.und;
                ref.LR = &bankedLR.und;
                SPSR = &bankedSPSR.und;
                break;
            default: ARM_ERR("TODO unknown mode which can't be switched to");
        }
    }
}

u32 ARM::mask_op(u8 offset, u32 mask) { return (execOpcode >> offset) & mask; }

bool ARM::bit_op(u8 offset) { return (execOpcode & (1 << offset)) != 0; }

bool ARM::check_cond(u8 cond) {
    switch (cond) {
        case 0x0:  // EQ
            return z_flag();
        case 0x1:  // NE
            return !z_flag();
        case 0x2:  // CS/HS
            return c_flag();
        case 0x3:  // CC/LO
            return !c_flag();
        case 0x4:  // MI
            return n_flag();
        case 0x5:  // PL
            return !n_flag();
        case 0x6:  // VS
            return v_flag();
        case 0x7:  // VC
            return !v_flag();
        case 0x8:  // HI
            return c_flag() && !z_flag();
        case 0x9:  // LS
            return !c_flag() && z_flag();
        case 0xA:  // GE
            return n_flag() == v_flag();
        case 0xB:  // LT
            return n_flag() ^ v_flag();
        case 0xC:  // GT
            return !z_flag() && (n_flag() == v_flag());
        case 0xD:  // LE
            return z_flag() || (n_flag() ^ v_flag());
        case 0xE: return true;
        case 0xF: ARM_ERR("TODO 'NEVER' condition code is rarely used"); return false;
        default: ARM_ERR("Condition code for instruction should always 4-bits");
    }
}

bool ARM::n_flag() { return CPSR & PSRMask::N; }

bool ARM::z_flag() { return CPSR & PSRMask::Z; }

bool ARM::c_flag() { return CPSR & PSRMask::C; }

bool ARM::v_flag() { return CPSR & PSRMask::V; }

void ARM::update_logical_flags(u32 value) {
    CPSR &= 0x1FFFFFFF;
    CPSR |= (value >> 31) * PSRMask::N;
    CPSR |= (value == 0) * PSRMask::Z;
    CPSR |= carry * PSRMask::C;
}

void ARM::update_add_carry(u32 op1, u32 op2) {
    CPSR &= 0xDFFFFFFF;
    CPSR |= (op1 > 0xFFFFFFFF - op2) * PSRMask::C;
}

void ARM::update_sub_carry(u32 op1, u32 op2) {
    CPSR &= 0xDFFFFFFF;
    CPSR |= (op1 >= op2) * PSRMask::C;
}

void ARM::update_arithmetic_flags(u32 result, u32 op1, u32 op2) {
    CPSR &= 0x2FFFFFFF;
    CPSR |= (result >> 31) * PSRMask::N;
    CPSR |= (result == 0) * PSRMask::Z;

    bool op1Sign = op1 >> 31;
    bool op2Sign = op2 >> 31;
    bool resSign = result >> 31;
    CPSR |= (!(op1Sign ^ op2Sign) & (op1Sign ^ resSign)) * PSRMask::V;
}

u32 ARM::lsl(u8 shift, u32 val) {
    if (shift > 0) {
        carry = val & (1 << (32 - shift));
    }
    return shift >= 32 ? 0 : (val << shift) & 0xFFFFFFFF;
}

u32 ARM::lsr(u8 shift, u32 val) {
    if (shift > 0) {
        carry = val & (1 << (shift - 1));
        return shift >= 32 ? 0 : (val >> shift) & 0xFFFFFFFF;
    } else {
        carry = (val >> 31) & 1;
        return 0;
    }
}

u32 ARM::asr(u8 shift, u32 val) {
    bool neg = (val >> 31) & 1;
    if (0 < shift && shift < 32) {
        carry = val & (1 << (shift - 1));

        u32 signExtension = (u32) ~((neg << (32 - shift)) - 1);
        val = (val >> shift) & 0xFFFFFFFF;
        return signExtension | val;
    } else {
        carry = neg;
        return neg ? 0xFFFFFFFF : 0;
    }
}

u32 ARM::ror(u8 shift, u32 val) {
    if (shift == 0) {
        carry = val & 1;
        return (u32)(c_flag() << 31) | val >> 1;
    } else {
        shift %= 32;
        carry = val & (1 << (shift - 1));
        return (val << (32 - shift)) | (val >> shift);
    }
}

u32 ARM::rotate_immediate_addressing() {
    u8 shift = mask_op(8, 0xF) << 1;
    u8 imm8 = execOpcode & 0xFF;

    u32 res = imm8 >> shift;
    if (shift < 32) {
        res |= (u32)imm8 << (32 - shift);
    }
    if (shift) {
        carry = res & 0x80000000;
        log("(0x%02x, ror#%d)=>(0x%08x)", imm8, shift, res);
    } else {
        log("0x%02x", imm8);
    }
    return res;
}

void ARM::arm_b_bl() {
    bool isLink = bit_op(24);
    int signedOffset = sign_extend(23, mask_op(0, 0xFFFFFF)) << 2;

    if (isLink) {
        *ref.LR = PC - 4;
    }
    PC = (long long)PC + signedOffset;
    log("b%s to %08x, CPSR=%08x\n", isLink ? "l" : "", PC, CPSR);
    flush_pipeline();
}

void ARM::arm_bx() {
    ARM_ASSERT(mask_op(4, 0xF) == 0b0001, "Only bx supported and not bxj or blx");
    u8 rn = execOpcode & 0xF;
    u32 op = *r[rn];
    if (op & 0x1) {
        ARM_ERR("TODO implement switching to thumb mode");
    }
    ARM_ASSERT((op & 0x3) == 0, "bx instruction must be aligned to word (4-bytes)");

    PC = op;
    log("bx to %08x, CPSR=%08x\n", PC, CPSR);
    flush_pipeline();
}

static const char* DAT_PROC_MAP[0x10] = {
    "and", "eor", "sub", "rsb", "add", "adc", "sbc", "rsc", "tst", "teq", "cmp", "cmn", "orr", "mov", "bic", "mvn",
};

static const char* SHIFT_TYPE_MAP[4] = {"lsl", "lsr", "asr", "ror"};

void ARM::arm_data_proc() {
    ARM_ASSERT(mask_op(26, 0x3) == 0, "Bit [27:26] must be 0 for data processing instruction");

    bool isImmediate = bit_op(25);
    u8 opcode = mask_op(21, 0xF);
    bool setcc = bit_op(20);

    u8 rn = mask_op(16, 0xF);
    u8 rd = mask_op(12, 0xF);

    log("%s", DAT_PROC_MAP[opcode]);
    if (opcode == 0xD || opcode == 0xF) {
        ARM_ASSERT(rn == 0, "1st operand must be 0 for MOV and MVN");
        log("%s r%d(%08x), ", setcc ? "s" : "", rd, *r[rd]);
    } else if (0x8 <= opcode && opcode <= 0xB) {
        ARM_ASSERT(setcc, "Bit 20 must be set for TST, TEQ, CMP and CMN");
        ARM_ASSERT(rd == 0 || rd == 0b1111, "rd must be 0 or 15 for TST, TEQ, CMP and CMN");
        log(" r%d(%08x), ", rn, *r[rn]);
    } else {
        log("%s r%d(%08x), r%d(%08x), ", setcc ? "s" : "", rd, *r[rd], rn, *r[rn]);
    }

    carry = c_flag();
    u32 op1 = *r[rn];
    u32 op2;
    if (isImmediate) {
        op2 = rotate_immediate_addressing();
        log("\n");
    } else {
        bool isRegShift = bit_op(4);
        u8 shiftType = mask_op(5, 0x3);
        u8 rm = execOpcode & 0xF;
        op2 = *r[rm];

        u8 shiftAmt;
        if (isRegShift) {
            u8 rs = mask_op(8, 0xF);
            ARM_ASSERT(rs <= 14, "register shift for data processing instructions can't be PC");
            ARM_ASSERT(!bit_op(7), "Bit 7 must be 0 for data proccess with shift by register");
            // TODO: cleanup this quirk when rm == PC or rn == PC
            if (rn == 15) {
                op1 = PC + 4;
            }
            if (rm == 15) {
                op2 = PC + 4;
            }
            shiftAmt = *r[rs] & 0xFF;
        } else {
            shiftAmt = mask_op(7, 0x1F);
        }
        if (!isRegShift || shiftAmt > 0) {
            switch (shiftType) {
                case 0x0: op2 = lsl(shiftAmt, op2); break;
                case 0x1: op2 = lsr(shiftAmt, op2); break;
                case 0x2: op2 = asr(shiftAmt, op2); break;
                case 0x3: op2 = ror(shiftAmt, op2); break;
            }
        }
        log("(r%d(%08x), %s#%d)=>(%08x)\n", rm, *r[rm], SHIFT_TYPE_MAP[shiftType], shiftAmt, op2);
    }

    switch (opcode) {
        case 0x0:  // AND
            *r[rd] = op1 & op2;
            if (setcc) update_logical_flags(*r[rd]);
            break;
        case 0x1:  // EOR
            *r[rd] = op1 ^ op2;
            if (setcc) update_logical_flags(*r[rd]);
            break;
        case 0x2:  // SUB
            *r[rd] = op1 - op2;
            if (setcc) {
                update_arithmetic_flags(*r[rd], op1, -op2);
                update_sub_carry(op1, op2);
            }
            break;
        case 0x3:  // RSB
            *r[rd] = op2 - op1;
            if (setcc) {
                update_arithmetic_flags(*r[rd], op2, -op1);
                update_sub_carry(op2, op1);
            }
            break;
        case 0x4:  // ADD
            *r[rd] = op1 + op2;
            if (setcc) {
                update_arithmetic_flags(*r[rd], op1, op2);
                update_add_carry(op1, op2);
            }
            break;
        case 0x5:  // ADC
            *r[rd] = op1 + op2 + c_flag();
            if (setcc) {
                update_arithmetic_flags(*r[rd], op1, op2 + c_flag());
                update_add_carry(op1, op2 + c_flag());
            }
            break;
        case 0x6:  // SBC
            *r[rd] = op1 - op2 + c_flag() - 1;
            if (setcc) {
                update_arithmetic_flags(*r[rd], op1, -op2 + c_flag() - 1);
                update_sub_carry(op1, op2 - c_flag() + 1);
            }
            break;
        case 0x7:  // RSC
            *r[rd] = op2 - op1 + c_flag() - 1;
            if (setcc) {
                update_arithmetic_flags(*r[rd], op2, -op1 + c_flag() - 1);
                update_sub_carry(op2, op1 - c_flag() + 1);
            }
            break;
        case 0x8:  // TST
            update_logical_flags(op1 & op2);
            break;
        case 0x9:  // TEQ
            update_logical_flags(op1 ^ op2);
            break;
        case 0xA:  // CMP
            update_arithmetic_flags(op1 - op2, op1, -op2);
            update_sub_carry(op1, op2);
            break;
        case 0xB:  // CMN
            update_arithmetic_flags(op1 + op2, op1, op2);
            update_add_carry(op1, op2);
            break;
        case 0xC:  // ORR
            *r[rd] = op1 | op2;
            if (setcc) update_logical_flags(*r[rd]);
            break;
        case 0xD:  // MOV
            *r[rd] = op2;
            if (setcc) update_logical_flags(*r[rd]);
            break;
        case 0xE:  // BIC
            *r[rd] = op1 & ~op2;
            if (setcc) update_logical_flags(*r[rd]);
            break;
        case 0xF:  // MVN
            *r[rd] = ~op2;
            if (setcc) update_logical_flags(*r[rd]);
            break;
    }

    if (rd == 15 && (SPSR || !(0x8 <= opcode && opcode <= 0xB))) {
        flush_pipeline();

        if (setcc) {
            if ((CPSR & PSRMask::MODE) == ModeBits::USR) {
                ARM_ERR("s flag and rd == PC not allowed in usr mode");
            }
            if (SPSR) {
                u32 oldSPSR = *SPSR;
                update_mode(oldSPSR);
                CPSR = oldSPSR;
            }
        }
    }
}

void ARM::arm_psr() {
    ARM_ASSERT(mask_op(26, 0x3) == 0, "Bit [27:26] must be 0 for psr instruction");
    bool isImmediate = bit_op(25);
    ARM_ASSERT(mask_op(23, 0x3) == 0b10, "Bit [24:23] must be 0b10 for psr instruction");
    bool isSPSRSel = bit_op(22);
    u32* PSR;
    u8 curMode = CPSR & PSRMask::MODE;
    if (isSPSRSel) {
        ARM_ASSERT(SPSR, "No SPSR exists in user/sys mode");
        PSR = SPSR;
    } else {
        PSR = &CPSR;
    }
    bool isStore = bit_op(21);

    ARM_ASSERT(!bit_op(20), "Bit 20 must be cleared for psr instruction");
    if (isStore) {
        log("msr %s", isSPSRSel ? "spsr" : "cpsr");
        bool writeFlags = bit_op(19);

        ARM_ASSERT(!bit_op(18), "Bit 18 attempts to set reserved PSR bits for msr instruction");
        ARM_ASSERT(!bit_op(17), "Bit 17 attempts to set reserved PSR bits for msr instruction");
        bool writeCnt = bit_op(16);

        ARM_ASSERT(!writeCnt || curMode != ModeBits::USR, "Can't change control bits in usr mode");
        ARM_ASSERT(mask_op(12, 0xF) == 0xF, "Bit [15:12] must be 0b1111 for msr instruction");

        if (writeFlags && writeCnt) {
            log("_fc");
        } else if (writeFlags) {
            log("_f");
        } else if (writeCnt) {
            log("_c");
        }
        log(", ");

        u32 op;
        if (isImmediate) {
            op = rotate_immediate_addressing();
        } else {
            ARM_ASSERT(!mask_op(4, 0xFF), "Bits [11:4] must be 0 for msr immediate instruction");
            u8 rm = execOpcode & 0xF;
            op = *r[rm];
            log("r%d(%08x)", rm, *r[rm]);
        }
        // Note: bit 5 (thumb/arm state mode) can't be changed
        u32 mask = (writeFlags ? 0xFF000000 : 0x00) | (writeCnt ? 0xDF : 0x0);
        if (!isSPSRSel && writeCnt) {
            update_mode(op);
        }
        *PSR = (~mask & *PSR) | (mask & op);
        if (!isSPSRSel) {
            log(" (CPSR=%08x)", CPSR);
        }
        log("\n");
    } else {
        ARM_ASSERT(!isImmediate, "Bit 25 must be 0 for mrs instruction");
        ARM_ASSERT(mask_op(16, 0xF) == 0xF, "Bits [19:16] must be 15 for mrs instruction");
        u8 rd = mask_op(12, 0xF);
        ARM_ASSERT((execOpcode & 0xFFF) == 0, "Bits [11:0] must be cleared for mrs instruction");
        *r[rd] = *PSR;
        log("mrs r%d(%08x) %s\n", rd, *r[rd], isSPSRSel ? "SPSR" : "CPSR");
    }
}

static const char* MULT_MAP[8] = {"mul", "mla", "umaal(invalid)", "invalild", "umull", "umlal", "smull", "smlal"};

void ARM::arm_mul() {
    ARM_ASSERT(mask_op(25, 0x7) == 0, "Bits [27:25] must be 0 for mult instruction");
    u8 opcode = mask_op(21, 0xF);
    ARM_ASSERT(opcode < 0b1000, "Only opcodes 0b0000-0b0111 supported in ARMv4 for multiplication");
    bool setcc = bit_op(20);

    u8 rd = mask_op(16, 0xF);
    u8 rn = mask_op(12, 0xF);
    u8 rs = mask_op(8, 0xF);
    u8 rm = execOpcode & 0xF;
    ARM_ASSERT(rd <= 14, "rd must be reg 0-14 inclusive for mult instrutions");
    ARM_ASSERT(rn <= 14, "rn must be reg 0-14 inclusive for mult instrucitons");
    ARM_ASSERT(rs <= 14, "rs must be reg 0-14 inclusive for mult instructions");
    ARM_ASSERT(rm <= 14, "rm must be reg 0-14 inclusive for mult instructions");
    ARM_ASSERT(rd != rm, "rd cannot be the same as rd for mult instructions");

    ARM_ASSERT(mask_op(4, 0x7) == 0b001, "Bits [6:4] must be 0b001 for mult instructions");

    ARM_ASSERT(bit_op(7), "Bit 7 must be set for mult instructions");

    carry = c_flag();
    log("%s%s ", MULT_MAP[opcode], setcc ? "s" : "");
    switch (opcode) {
        case 0b0000:  // MUL
            *r[rd] = ((*r[rm]) * (*r[rs])) & 0xFFFFFFFF;
            if (setcc) update_logical_flags(*r[rd]);
            log("r%d[%08x], r%d[%08x], r%d[%08x]\n", rd, *r[rd], rm, *r[rm], rs, *r[rs]);
            break;
        case 0b0001:  // MLA
            *r[rd] = ((*r[rm]) * (*r[rs]) + (*r[rn])) & 0xFFFFFFFF;
            if (setcc) update_logical_flags(*r[rd]);
            log("r%d[%08x], r%d[%08x], r%d[%08x], r%d[%08x]\n", rd, *r[rd], rm, *r[rm], rs, *r[rs], rn, *r[rn]);
            break;
        case 0b0100: {  // UMULL
            u64 res = ((u64)*r[rm]) * ((u64)*r[rs]);
            *r[rd] = res >> 32;
            *r[rn] = res & 0xFFFFFFFF;
            if (setcc) {
                CPSR &= 0x3FFFFFFF;
                CPSR |= (res >> 63) * PSRMask::N;
                CPSR |= (res == 0) * PSRMask::Z;
            }
            log("r%d[%08x], r%d[%08x], r%d[%08x], r%d[%08x]\n", rn, *r[rn], rd, *r[rd], rm, *r[rm], rs, *r[rs]);
            break;
        }
        case 0b0101: {  // UMLAL
            u64 res = ((u64)*r[rm]) * ((u64)*r[rs]) + (((u64)*r[rd] << 32) | *r[rn]);
            *r[rd] = res >> 32;
            *r[rn] = res & 0xFFFFFFFF;
            if (setcc) {
                CPSR &= 0x3FFFFFFF;
                CPSR |= (res >> 63) * PSRMask::N;
                CPSR |= (res == 0) * PSRMask::Z;
            }
            log("r%d[%08x], r%d[%08x], r%d[%08x], r%d[%08x]\n", rn, *r[rn], rd, *r[rd], rm, *r[rm], rs, *r[rs]);
            break;
        }
        case 0b0110: {  // SMULL
            s64 res = ((s32)*r[rm]) * ((s32)*r[rs]);
            *r[rd] = (u32)(res >> 32);
            *r[rn] = res & 0xFFFFFFFF;
            if (setcc) {
                CPSR &= 0x3FFFFFFF;
                CPSR |= (res >> 63) * PSRMask::N;
                CPSR |= (res == 0) * PSRMask::Z;
            }
            log("r%d[%08x], r%d[%08x], r%d[%08x], r%d[%08x]\n", rn, *r[rn], rd, *r[rd], rm, *r[rm], rs, *r[rs]);
            break;
        }
        case 0b0111: {  // SMLAL
            s64 res = ((s32)*r[rm]) * ((s32)*r[rs]) + (s64)(((u64)*r[rd] << 32) | *r[rn]);
            *r[rd] = (u32)(res >> 32);
            *r[rn] = res & 0xFFFFFFFF;
            if (setcc) {
                CPSR &= 0x3FFFFFFF;
                CPSR |= (res >> 63) * PSRMask::N;
                CPSR |= (res == 0) * PSRMask::Z;
            }
            log("r%d[%08x], r%d[%08x], r%d[%08x], r%d[%08x]\n", rn, *r[rn], rd, *r[rd], rm, *r[rm], rs, *r[rs]);
            break;
        }
        default: ARM_ERR("Unknown mult instruction opcode: %01x", opcode);
    }
}

void ARM::arm_single_data() {
    ARM_ASSERT(mask_op(26, 0x3) == 0b01, "Bits [27:26] must be 0b01 for single data instruction");
    bool isShiftedRegister = bit_op(25);
    bool isPreOffset = bit_op(24);
    bool isAddOffset = bit_op(23);
    bool isByteTransfer = bit_op(22);

    bool isWriteBack;
    if (isPreOffset) {
        isWriteBack = bit_op(21);
    } else {
        isWriteBack = true;
        ARM_ASSERT(bit_op(21) == 0, "Forced non-priviledged access not supported for single data instruction");
    }
    bool isLoad = bit_op(20);
    log("%s%s ", isLoad ? "ldr" : "str", isByteTransfer ? "b" : "");

    u8 rn = mask_op(16, 0xF);
    u8 rd = mask_op(12, 0xF);
    log("r%d(%08x), [r%d(%08x)", rd, *r[rd], rn, *r[rn]);

    u32 offset;
    if (isShiftedRegister) {
        u8 shiftAmt = mask_op(7, 0x1F);
        u8 shiftType = mask_op(5, 0x3);
        ARM_ASSERT(bit_op(4) == 0, "Bit 4 must be cleared for single data instruction");
        u8 rm = execOpcode & 0xF;
        ARM_ASSERT(rm <= 14, "rm cannot be 15 for single data instruction instruction");

        switch (shiftType) {
            case 0x0: offset = lsl(shiftAmt, *r[rm]); break;
            case 0x1: offset = lsr(shiftAmt, *r[rm]); break;
            case 0x2: offset = asr(shiftAmt, *r[rm]); break;
            case 0x3: offset = ror(shiftAmt, *r[rm]); break;
        }
        if (isPreOffset) {
            log(" %sr%d(%08x), %s#%d]%s\n", isAddOffset ? "+" : "-", rm, *r[rm], SHIFT_TYPE_MAP[shiftType], shiftAmt,
                isWriteBack ? "!" : "");
        } else {
            log("] %sr%d(%08x), %s#%d\n", isAddOffset ? "+" : "-", rm, *r[rm], SHIFT_TYPE_MAP[shiftType], shiftAmt,
                isWriteBack ? "!" : "");
        }
    } else {
        offset = execOpcode & 0xFFF;
        if (isPreOffset) {
            log(" #%s0x%03x]%s\n", isAddOffset ? "+" : "-", offset, isWriteBack ? "!" : "");
        } else {
            log("], #%s0x%03x\n", isAddOffset ? "+" : "-", offset);
        }
    }

    u32 address = *r[rn];
    if (isPreOffset) {
        address += isAddOffset ? offset : -offset;
    }
    if (isLoad) {
        if (isByteTransfer) {
            *r[rd] = bus->mem->read8(address);
        } else {
            u32 data;
            if (address & 0x3) {
                data = bus->mem->read32(address & ~(u32)0x3);
                data = ror((address & 0x3) << 3, data);
            } else {
                data = bus->mem->read32(address);
            }
            *r[rd] = data;
        }
        if (rd == 15) {
            flush_pipeline();
        }
    } else {
        u32 storeData = rd == 15 ? PC + 4 : *r[rd];
        if (isByteTransfer) {
            bus->mem->write8(address, storeData & 0xFF);
        } else {
            address &= ~(u32)0x3;
            bus->mem->write32(address, storeData);
        }
    }
    if (!isPreOffset) {
        address += isAddOffset ? offset : -offset;
    }
    if (isWriteBack && (!isLoad || rd != rn)) {
        *r[rn] = address;
    }
}

static const char* HALFWORD_SIGNED_SUFFIX_MAP[8] = {"invalid", "h", "sb", "sh"};

void ARM::arm_halfword_signed_data() {
    ARM_ASSERT(mask_op(25, 0x7) == 0, "Bits [27:25] must be cleared for halfword signed data transfer");
    bool isPreOffset = bit_op(24);
    bool isAddOffset = bit_op(23);
    bool isImmediateOffset = bit_op(22);

    bool isWriteBack;
    if (isPreOffset) {
        isWriteBack = bit_op(21);
    } else {
        ARM_ASSERT(bit_op(21) == 0, "Bit 21 must be cleared for post offset halfword signed data transfer");
        isWriteBack = true;
    }
    bool isLoad = bit_op(20);

    u8 rn = mask_op(16, 0xF);
    u8 rd = mask_op(12, 0xF);

    u8 opcode = mask_op(5, 0x3);
    log("%s%s", isLoad ? "ldr" : "str", HALFWORD_SIGNED_SUFFIX_MAP[opcode]);
    log(" r%d(%08x), [r%d(%08x)", rd, *r[rd], rn, *r[rn]);

    u32 offset;
    if (isImmediateOffset) {
        u8 upperOff = mask_op(8, 0xF) << 4;
        u8 lowerOff = execOpcode & 0xF;
        offset = (u32)(upperOff << 4) | lowerOff;
        if (isPreOffset) {
            log(" #%s0x%03x]%s\n", isAddOffset ? "+" : "-", offset, isWriteBack ? "!" : "");
        } else {
            log("], #%s0x%03x\n", isAddOffset ? "+" : "-", offset);
        }
    } else {
        ARM_ASSERT(mask_op(8, 0xF) == 0, "Bits [11:8] must be 0 for register offset halfword signed data transfer");
        u8 rm = execOpcode & 0xF;
        offset = *r[rm];
        if (isPreOffset) {
            log(" %sr%d(%08x)]%s\n", isAddOffset ? "+" : "-", rm, *r[rm], isWriteBack ? "!" : "");
        } else {
            log("] %sr%d(%08x)\n", isAddOffset ? "+" : "-", rm, *r[rm]);
        }
    }

    u32 address = *r[rn];
    if (isPreOffset) {
        address += isAddOffset ? offset : -offset;
    }

    switch (opcode) {
        case 0b00: ARM_ERR("Reserved%s", isLoad ? " for swp instruction" : ""); break;
        case 0b01:  // STRH/LDRH
            if (isLoad) {
                u32 data;
                if (address & 0x1) {
                    data = bus->mem->read16(address & ~(u32)0x1);
                    data = ror(8, data);
                } else {
                    data = bus->mem->read16(address);
                }
                *r[rd] = data;
                if (rd == 15) {
                    flush_pipeline();
                }
            } else {
                u32 storeData = rd == 15 ? PC + 4 : *r[rd];
                address &= ~(u32)0x3;
                bus->mem->write16(address, storeData & 0xFFFF);
            }
            break;
        case 0b10: {  // LDRSB
            ARM_ASSERT(isLoad, "ldrd instruction not supported");

            u32 data = bus->mem->read8(address & ~(u32)0x3);
            if (data & 0x80) {
                *r[rd] = 0xFFFFFF00 | data;
            } else {
                *r[rd] = data;
            }
            if (rd == 15) {
                flush_pipeline();
            }
            break;
        }
        case 0b11: {  // LDRSH
            ARM_ASSERT(isLoad, "strd instruction not supported");
            u16 data;
            if (address & 0x1) {
                data = bus->mem->read8(address);
                if (data & 0x80) {
                    *r[rd] = 0xFFFFFF00 | data;
                } else {
                    *r[rd] = data;
                }
            } else {
                data = bus->mem->read16(address);
                if (data & 0x8000) {
                    *r[rd] = 0xFFFF0000 | data;
                } else {
                    *r[rd] = data;
                }
            }
            if (rd == 15) {
                flush_pipeline();
            }
            break;
        }
    }

    if (!isPreOffset) {
        address += isAddOffset ? offset : -offset;
    }
    if (isWriteBack && (!isLoad || rd != rn)) {
        *r[rn] = address;
    }
}

void ARM::arm_swap() {
    ARM_ASSERT(mask_op(23, 0x1F) == 0x2, "Bits [27:23] must be 0b00010 for swap instruction");
    bool swapByte = bit_op(22);
    ARM_ASSERT(mask_op(20, 0x3) == 0, "Bits [21:20] must be 0 for swap instruction");
    u8 rn = mask_op(16, 0xF);
    u8 rd = mask_op(12, 0xF);

    ARM_ASSERT(mask_op(4, 0xFF) == 0x9, "Bits [11:4] must be 0b00001001 for swap instruction");
    u8 rm = execOpcode & 0xF;

    ARM_ASSERT(rn != 15, "rn cannot be PC for swap instruction");
    ARM_ASSERT(rd != 15, "rd cannot be PC for swap instruction");
    ARM_ASSERT(rm != 15, "rm cannot be PC for swap instruction");

    log("swp%s r%d(%08x), r%d(%08x), [r%d(%08x)]\n", swapByte ? "b" : "", rd, *r[rd], rm, *r[rm], rn, *r[rn]);

    u32 address = *r[rn];
    if (swapByte) {
        u8 writeVal = *r[rm] & 0xFF;
        *r[rd] = bus->mem->read8(address);
        bus->mem->write8(address, writeVal);
    } else {
        u32 data;
        if (address & 0x3) {
            data = bus->mem->read32(address & ~(u32)0x3);
            data = ror((address & 0x3) << 3, data);
        } else {
            data = bus->mem->read32(address);
        }
        u32 writeVal = *r[rm];
        *r[rd] = data;
        bus->mem->write32(address & ~(u32)0x3, writeVal);
    }
}

void ARM::arm_block_transfer() {
    ARM_ASSERT(mask_op(25, 0x7) == 0b100, "Bits [27:25] must be 0b100 for block transfer instruction");
    bool isPreOffset = bit_op(24);
    bool isAddOffset = bit_op(23);

    bool isForceUser = bit_op(22);
    bool isWriteBack = bit_op(21);
    ARM_ASSERT(!isForceUser || !isWriteBack, "Write back should be 0 for forced user in block transfer instruction");
    bool isLoad = bit_op(20);

    u8 rn = mask_op(16, 0xF);
    ARM_ASSERT(rn != 15, "rn cannot be PC for block transfer instruction");

    u32 oldCPSR = CPSR;
    if (isForceUser) {
        ARM_ASSERT(!isLoad || !bit_op(15), "Unimplemented LDM and R15 mode change for block transfer instruction");
        for (int i = 0; i < 5; i++) {
            ref.reg_8_12[i] = &genRegHi[i];
        }
        ref.SP = &bankedSP.usr;
        ref.LR = &bankedLR.usr;
        CPSR = ModeBits::USR;
    }

    log("%s%s%s ", isLoad ? "ldm" : "stm", (isPreOffset ^ isLoad) ? "f" : "e", (isAddOffset ^ isLoad) ? "a" : "d");
    log("r%d(%08x)%s, {", rn, *r[rn], isWriteBack ? "!" : "");

    u16 regList = execOpcode & 0xFFFF;
    u8 numRegs = BYTE_SET_COUNT[regList >> 8] + BYTE_SET_COUNT[regList & 0xFF];

    u32 baseAddress = *r[rn];
    u32 oldBaseAddress = baseAddress;
    u32 newBaseAddress = baseAddress + (u32)(isAddOffset ? 4 : -4) * numRegs;

    if (regList == 0) {
        // Strange behavior if there are no registers in the reg list
        if (isLoad) {
            flush_pipeline();
            PC = bus->mem->read32(baseAddress & ~(u32)0x3);
        } else {
            u32 offsetAddress = baseAddress;
            if (!isAddOffset) {
                offsetAddress -= 0x40;
            }
            if (!(isAddOffset ^ isPreOffset)) {
                offsetAddress += 0x4;
            }
            bus->mem->write32(offsetAddress & ~(u32)0x3, PC + 4);
        }
        if (isAddOffset) {
            *r[rn] = baseAddress + 0x40;
        } else {
            *r[rn] = baseAddress - 0x40;
        }
    } else {
        if (!isAddOffset) {
            baseAddress = newBaseAddress;
        }
        if (!isAddOffset && isWriteBack) {
            *r[rn] = baseAddress;
        }
        u8 countedRegs = 0;
        for (int i = 0; i <= 15; i++) {
            if (bit_op(i)) {
                if (!(isAddOffset ^ isPreOffset)) {
                    baseAddress += 4;
                }
                if (isLoad) {
                    if (i == 15) {
                        flush_pipeline();
                    }
                    if (i == rn) {
                        isWriteBack = false;
                    }
                    *r[i] = bus->mem->read32(baseAddress & ~(u32)0x3);
                } else {
                    u32 storeData = i == 15 ? PC + 4 : *r[i];
                    if (i == rn) {
                        if (countedRegs == 0) {
                            storeData = oldBaseAddress;
                        } else {
                            storeData = newBaseAddress;
                        }
                    }
                    bus->mem->write32(baseAddress & ~(u32)0x3, storeData);
                }
                if (isAddOffset ^ isPreOffset) {
                    baseAddress += 4;
                }
                if (countedRegs == 0) {
                    log("r%d", i);
                } else {
                    log(", r%d", i);
                }
                countedRegs++;
            }
            if (countedRegs == numRegs) {
                break;
            }
        }
        if (isAddOffset && isWriteBack) {
            *r[rn] = baseAddress;
        }
    }
    log("}%s\n", isForceUser ? "^" : "");
    if (isForceUser) {
        update_mode(oldCPSR);
        CPSR = oldCPSR;
    }
}

void ARM::flush_pipeline() { pipeline[0] = pipeline[1] = pipeline[2] = {0, InstrType::FLUSHED}; }

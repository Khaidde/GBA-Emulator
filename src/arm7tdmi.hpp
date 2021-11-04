#pragma once

#include "emulator.hpp"

enum class InstrType {
    UNKNOWN = 0,
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

enum Mode : char {
    USR = 0,
    FIQ = 1,
    SVC = 2,
    ABT = 3,
    IRQ = 4,
    UND = 5,
};

class ARM {
public:
    ARM(Bus* bus) : bus(bus) {
        for (size_t i = 0; i < 8; i++) {
            r[i] = &genRegLo[i];
        }
        for (size_t i = 0; i < 5; i++) {
            r[i + 8] = &genRegHi[i];
        }
        r[13] = &SP.m[Mode::USR];
        r[14] = &LR.m[Mode::USR];
        r[15] = &PC;
        PC++;
    }

    void reset();
    void cycle();

    void arm_data_proc();

private:
    u32 execOpcode;

    struct {
        u32 opcode;
        InstrType type;
    } pipeline[3];
    u8 pipelineIdx;

    Bus* bus;

    union {
        u32* r[16];
        struct {
            u32* genReg[12];
            u32* SP;
            u32* LR;
            u32* PC;
        } ref;
    };

    struct BankableReg {
        union {
            u32 usr;
            u32 fiq;
            u32 svc;
            u32 abt;
            u32 irq;
            u32 und;
        };
        u32 m[6];
    };
    struct {
        u32 genRegLo[8];

        u32 genRegHi[5];
        u32 fiqReg[5];  // R[8:14]_fiq

        BankableReg SP;
        BankableReg LR;

        u32 PC;
    };

    u32 CPSR;
    union {
        struct {
            u32 fiq;
            u32 svc;
            u32 abt;
            u32 irq;
            u32 und;
        };
        u32 m[5];
    } SPSR;
};

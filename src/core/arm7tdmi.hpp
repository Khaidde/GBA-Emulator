#pragma once

#include "emulator.hpp"

enum class InstrType;

class ARM {
public:
    ARM(Bus* bus);
    void x_regs();
    bool end_test() { return *r[10] == 0xFF; }

    void reset();
    void cycle();

private:
    void log(const char* format, ...);

    void update_mode(u32 psr);

    u32 mask_op(u8 offset, u32 mask);
    bool bit_op(u8 offset);

    bool check_cond(u8 cond);
    bool n_flag();
    bool z_flag();
    bool c_flag();
    bool v_flag();
    void update_logical_flags(u32 value);
    void update_add_carry(u32 op1, u32 op2);
    void update_sub_carry(u32 op1, u32 op2);
    void update_arithmetic_flags(u32 result, u32 op1, u32 op2);

    bool carry;
    u32 lsl(u8 shift, u32 val);
    u32 lsr(u8 shift, u32 val);
    u32 asr(u8 shift, u32 val);
    u32 ror(u8 shift, u32 val);

    u32 rotate_immediate_addressing();

    void arm_b_bl();
    void arm_bx();
    void arm_data_proc();
    void arm_psr();
    void arm_mul();
    void arm_single_data();
    void arm_halfword_signed_data();
    void arm_swap();
    void arm_block_transfer();

    void flush_pipeline();

    u32 execOpcode;

    struct {
        u32 opcode;
        InstrType type;
    } pipeline[3];
    u8 pipelineIdx;

    Bus* bus;

    union {
        struct {
#ifdef GBA_LITTLE_ENDIAN
            u32* reg_0_7[8];
            u32* reg_8_12[5];
            u32* SP;
            u32* LR;
            u32* PC;
#elif defined GBA_BIG_ENDIAN
            u32* PC;
            u32* LR;
            u32* SP;
            u32* genReg[12];
#endif
        } ref;
        u32* r[16];
    };

    struct BankableReg {
        u32 usr;
        u32 fiq;
        u32 irq;
        u32 svc;
        u32 abt;
        u32 und;
    };
    struct {
        u32 genRegLo[8];

        u32 genRegHi[5];
        u32 fiqReg[5];  // R[8:12]_fiq

        BankableReg bankedSP;
        BankableReg bankedLR;

        u32 PC;
    };

    u32 CPSR;
    u32* SPSR;
    union {
        u32 fiq;
        u32 svc;
        u32 abt;
        u32 irq;
        u32 und;
    } bankedSPSR;
};

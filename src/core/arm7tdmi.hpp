#pragma once

#include "emulator.hpp"

enum class InstrType;

class ARM {
public:
    ARM(Bus* bus);
    void x_regs();

    void reset();
    void cycle();

    bool swiInterrupt = false;

private:
    void log(const char* format, ...);

    bool is_thumb_mode();
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

    bool shiftCarry;
    u32 lsl(u8 shift, u32 val);
    u32 lsr(u8 shift, u32 val);
    u32 asr(u8 shift, u32 val);
    u32 ror(u8 shift, u32 val);

    u32 rotate_immediate_addressing(u8 shift, u8 nn);

    void arm_b_bl();
    void arm_bx();
    void arm_data_proc();
    void arm_psr();
    void arm_mul();
    void arm_single_data();
    void arm_halfword_signed_data();
    void arm_swap();
    void arm_block_transfer();
    void arm_swi();

    void thumb_move_shifted();
    void thumb_add_sub();
    void thumb_immediate();
    void thumb_alu();
    void thumb_hi_reg_bx();
    void thumb_ldr_pc();
    void thumb_ldr_str_reg();
    void thumb_ldr_str_signed_byte_hw();
    void thumb_ldr_str_immediate();
    void thumb_ldr_str_halfword();
    void thumb_ldr_str_sp();
    void thumb_add_pc_sp();
    void thumb_add_sp_off();
    void thumb_push_pop();
    void thumb_ldm_stm();
    void thumb_cond_br();
    void thumb_uncond_br();
    void thumb_bl_hi();
    void thumb_bl_lo();

    void execute_data_proc(u8 opcode, bool setcc, u32* rd, u32 op1, u32 op2);
    void execute_ldr(u8 rd, u32 address);
    void execute_byte_load(u8 rd, u32 address);
    void execute_str(u8 rd, u32 address);
    void execute_byte_store(u8 rd, u32 address);
    void execute_halfword_memory(bool isLoad, u8 rd, u32 address);
    void execute_signed_byte_load(u8 rd, u32 address);
    void execute_signed_halfword_load(u8 rd, u32 address);
    void execute_block_transfer(bool isPreOffset, bool isAddOffset, bool isWriteBack, bool isLoad, u8 rn, u16 regList);

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

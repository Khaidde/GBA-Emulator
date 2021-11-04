#include "arm7tdmi.hpp"

#include "memory.hpp"

void ARM::reset() {
    PC = 0x00;
    CPSR = 0;
    (void)SPSR;

    pipelineIdx = 0;
    pipeline[0] = pipeline[1] = pipeline[2] = {0, InstrType::UNKNOWN};
}

void ARM::cycle() {
    printf("---------------------");
    printf("cycle...\n");
    // Fetch
    pipeline[pipelineIdx].opcode = bus->mem->read32(PC);
    pipeline[pipelineIdx].type = InstrType::PENDING;
    PC += 4;

    // Decode
    u8 decodePipeIdx = (pipelineIdx + 2) % 3;
    u32 opcode = pipeline[decodePipeIdx].opcode;
    if (pipeline[decodePipeIdx].type != InstrType::UNKNOWN) {
        if ((opcode & 0x0FFFFF00) == 0x0320F0) {
            fatal("arm11:hint not implemented");
        } else if ((opcode & 0xFFF000F0) == 0xE1200070) {
            fatal("arm9:bkpt not implemented");
        }
        if ((opcode & 0x0E000010) == 0x06000010) {
            fatal("undefined not implemented");
        } else if ((opcode & 0x0E000000) == 0x0A000000) {
            pipeline[decodePipeIdx].type = InstrType::B_BL;
        } else if ((opcode & 0x0FFFFF00) == 0x012FFF00) {
            pipeline[decodePipeIdx].type = InstrType::BX;
        } else if ((opcode & 0x0F000000) == 0x0F000000) {
            pipeline[decodePipeIdx].type = InstrType::SWI;
        } else if ((opcode & 0x0C000000) == 0) {
            if ((opcode & 0x0E0000F0) == 0x90) {
                pipeline[decodePipeIdx].type = InstrType::MULT;
            } else if ((opcode & 0x01900000) == 0x01000000) {
                pipeline[decodePipeIdx].type = InstrType::PSR;
            } else {
                pipeline[decodePipeIdx].type = InstrType::DATA_PROCCESS;
            }
        } else if ((opcode & 0x0C000010) == 0x04000000) {
            pipeline[decodePipeIdx].type = InstrType::SINGLE_DATA_TRANSFER;
        } else if ((opcode & 0x0E0000900) == 0x90) {
            pipeline[decodePipeIdx].type = InstrType::HW_DW_SIGNED_DATA_TRANSFER;
        } else if ((opcode & 0x0E0000000) == 0x08000000) {
            pipeline[decodePipeIdx].type = InstrType::BLOCK_TRANSFER;
        } else if ((opcode & 0x0FB000000) == 0x01000000) {
            pipeline[decodePipeIdx].type = InstrType::SWAP;
        } else {
            fatal("TODO unknown decode for instruction: %08x", opcode);
        }
    }

    // Execute
    u8 executePipeIdx = (pipelineIdx + 1) % 3;
    execOpcode = pipeline[executePipeIdx].opcode;
    if (pipeline[executePipeIdx].type != InstrType::UNKNOWN) {
        // clang-format off
        switch (pipeline[executePipeIdx].type) {
            case InstrType::DATA_PROCCESS: arm_data_proc(); break;
            default:
                fatal("TODO unknown execute for instruction");
        }
        // clang-format on
    }

    pipelineIdx = (pipelineIdx + 1) % 3;
}

void ARM::arm_data_proc() {
    bool isImmediate = (execOpcode >> 25) & 1;
    u8 opcode = (execOpcode >> 21) & 0xF;
    // bool setcc = (execOpcode >> 20) & 0x1;

    // u8 rn = (execOpcode >> 16) & 0xF;
    // u8 rd = (execOpcode >> 12) & 0xF;

    u32 op2;
    if (isImmediate) {
        u8 ror = (execOpcode >> 8) & 0xF;
        u8 imm8 = execOpcode & 0xFF;
        op2 = (u32)(imm8 >> (ror << 1));
        if (32 > ror << 1) {
            op2 |= (u32)(imm8 << (32 - (ror << 1)));
        }
        fatal("Verify that ror on immediate value works correctly");
    } else {
    }

    switch (opcode) {
        case 0x00:  // AND
            break;
        case 0x01:  // EOR
            break;
    }
}

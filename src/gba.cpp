#include <stdio.h>

#include "arm7tdmi.hpp"
#include "emulator.hpp"
#include "memory.hpp"

int main() {
    Bus bus;
    Memory memory;
    ARM armCpu(&bus);
    bus.armCpu = &armCpu;
    bus.mem = &memory;

    constexpr u8 NUM_INSTR = 3;
    u32 instructions[NUM_INSTR] = {0x02000147};
    for (int i = 0; i < NUM_INSTR; i++) {
        memory.write32((u32)(i << 2), instructions[i]);
    }
    armCpu.reset();
    try {
        armCpu.cycle();  // First fetch
        for (int i = 0; i < 4; i++) {
            armCpu.cycle();
        }
    } catch (const std::exception& e) {
        printf("%s\n", e.what());
    }

    printf("GBA: The start of something great...\n");

    return 0;
}

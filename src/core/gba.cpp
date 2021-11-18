#include <stdio.h>

#include <fstream>

#include "arm7tdmi.hpp"
#include "emulator.hpp"
#include "memory.hpp"

int main() {
    std::ifstream file("./test/gba-tests/arm/arm.gba", std::ios::binary | std::ios::ate);
    // std::ifstream file("./test/gba-tests/thumb/thumb.gba", std::ios::binary | std::ios::ate);

    if (!file.is_open()) {
        FATAL("Cannot open file\n");
        return -1;
    }

    std::streamsize fileSize = file.tellg();
    u8 res[fileSize];
    file.seekg(0);
    file.read((char*)&res[0], fileSize);
    file.close();

    Bus bus;
    Memory memory;
    ARM armCpu(&bus);
    bus.armCpu = &armCpu;
    bus.mem = &memory;

    for (u32 i = 0; i < fileSize; i += 4) {
        u32 word = *(u32*)&res[i];
        memory.write32(i + 0x80000000, word);
    }
    armCpu.reset();
    try {
        while (!armCpu.swiInterrupt) {
            armCpu.cycle();
        }
        armCpu.x_regs();
    } catch (const std::exception& e) {
        printf("%s\n", e.what());
    }

    printf("GBA: The start of something great...\n");

    return 0;
}

#pragma once

#include "emulator.hpp"

class Memory {
public:
    Memory();

    u32 read32(u32 address);
    void write32(u32 address, u32 value);

private:
    static constexpr size_t MEMORY_SIZE = 0x100000000;
    u8* memory;  // TODO temporary
};

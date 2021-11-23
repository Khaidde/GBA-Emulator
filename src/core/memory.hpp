#pragma once

#include "general.hpp"

namespace gba {

class Memory {
public:
    Memory();

    void reset();

    u32 read32(u32 address);
    u16 read16(u32 address);
    u8 read8(u32 address);

    void write32(u32 address, u32 value);
    void write16(u32 address, u16 value);
    void write8(u32 address, u8 value);

private:
    static constexpr size_t MEMORY_SIZE = 0x100000000;
    u8* memory;

    u32 dma3Dest;
    u32 dma3Src;
    bool ongoingDMA;
};

}  // namespace gba

#include "memory.hpp"

#include "gba.hpp"

namespace gba {

enum IORegister : u32 {
    DISPCNT = 0x04000000,
    VCOUNT = 0x04000006,

    DMA3SAD = 0x040000d4,
    DMA3DAD = 0x040000d8,
    DMA3CNT = 0x040000dc,

    KEYINPUT = 0x04000130,
    IME = 0x04000208,
};

static bool in_range(u32 lo, u32& address, u32 hi) { return lo <= address && address < hi; }

static bool valid_address(u32& address) {
    if (address & 0xF0000000) {
        return true;
    }
    return in_range(0x02000000, address, 0x0203FFFF) || in_range(0x03000000, address, 0x03007FFF) ||
           in_range(0x08000000, address, 0x0A000000);
}

Memory::Memory() : memory(new u8[MEMORY_SIZE]) {}

void Memory::reset() {
    dma3Src = 0;
    dma3Dest = 0;
    ongoingDMA = false;
}

u32 Memory::read32(u32 address) {
    if (address == KEYINPUT) {
        return key.get_key_input();
    } else if (valid_address(address)) {
        return *((u32*)&memory[address]);
    } else {
        FATAL("Invalid u32 read address=%08x", address);
    }
}

u16 Memory::read16(u32 address) {
    switch (address) {
        case IORegister::VCOUNT: return lcd.read_vcount();
    }
    if (valid_address(address)) {
        return *((u16*)&memory[address]);
    } else {
        FATAL("Invalid u16 read address=%08x", address);
    }
}

u8 Memory::read8(u32 address) {
    if (valid_address(address)) {
        return memory[address];
    } else {
        FATAL("Invalid u8 read address=%08x", address);
    }
}

void Memory::write32(u32 address, u32 value) {
    switch (address) {
        case IORegister::IME:
            if (value & 0x1) {
                printf("Master enabled interrupts\n");
            } else {
                printf("Master disabled interrupts\n");
            }
            return;
        case IORegister::DMA3SAD: dma3Src = value; return;
        case IORegister::DMA3DAD: dma3Dest = value; return;
        case IORegister::DMA3CNT: {
            u16 hi = value >> 16;
            char dma3DestAdjust;
            char destCnt = (hi >> 5) & 0x3;
            switch (destCnt) {
                case 0b00: dma3DestAdjust = 2; break;
                case 0b01: dma3DestAdjust = -2; break;
                case 0b10: dma3DestAdjust = 0; break;
                default: FATAL("Unknown dma3 destination address control: %01x", destCnt);
            }
            char dma3SrcAdjust;
            char srcCnt = (hi >> 7) & 0x3;
            switch (srcCnt) {
                case 0b00: dma3SrcAdjust = 2; break;
                case 0b01: dma3SrcAdjust = -2; break;
                case 0b10: dma3SrcAdjust = 0; break;
                default: FATAL("Unknown dma3 source address control: %01x", srcCnt);
            }
            if ((hi & 0x7E1F) != 0) {
                FATAL("Unsupported dma3 type: %08x", hi);
            }
            if ((hi & 0x8000) == 0) {
                FATAL("dma enable bit must be set when writing to dma3cnt");
            }

            u16 wordCount = value & 0xFFFF;
            if (wordCount == 0) {
                FATAL("Unsupported 0 length wordcount for dma3");
            }
            ongoingDMA = true;
            for (size_t i = 0; i < wordCount; i++) {
                write16(dma3Dest, read16(dma3Src));
                dma3Src = (u32)((int)dma3Src + dma3SrcAdjust);
                dma3Dest = (u32)((int)dma3Dest + dma3DestAdjust);
            }
            ongoingDMA = false;
            return;
        }
    }
    if (valid_address(address)) {
        *((u32*)&memory[address]) = value;
    } else {
        FATAL("Invalid u32 write address=%08x", address);
    }
}

void Memory::write16(u32 address, u16 value) {
    switch (address) {
        case IORegister::DISPCNT: lcd.write_dispcnt(value); return;
    }
    if (in_range(0x06000000, address, 0x07000000)) {
        if (address < 0x06017FFF) {
            lcd.write_vram(address, value);
        } else if (!ongoingDMA) {
            FATAL("Write to unused memory address: %08x", address);
        }
    } else if (valid_address(address)) {
        *((u16*)&memory[address]) = value;
    } else {
        FATAL("Invalid u16 write address=%08x", address);
    }
}

void Memory::write8(u32 address, u8 value) {
    if (valid_address(address)) {
        memory[address] = value;
    } else {
        FATAL("Invalid u8 write address=%08x", address);
    }
}

}  // namespace gba

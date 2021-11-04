#include "memory.hpp"

Memory::Memory() : memory(new u8[MEMORY_SIZE]) {}

u32 Memory::read32(u32 address) { return *((u32*)&memory[address]); }

void Memory::write32(u32 address, u32 value) { *((u32*)&memory[address]) = value; }

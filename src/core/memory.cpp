#include "memory.hpp"

Memory::Memory() : memory(new u8[MEMORY_SIZE]) {}

u32 Memory::read32(u32 address) { return *((u32*)&memory[address]); }

u16 Memory::read16(u32 address) { return *((u16*)&memory[address]); }

u8 Memory::read8(u32 address) { return *((u8*)&memory[address]); }

void Memory::write32(u32 address, u32 value) { *((u32*)&memory[address]) = value; }

void Memory::write16(u32 address, u16 value) { *((u16*)&memory[address]) = value; }

void Memory::write8(u32 address, u8 value) { memory[address] = value; }

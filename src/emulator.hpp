#pragma once

#include <stdint.h>
#include <stdio.h>

#include <stdexcept>

#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
#define GBA_BIG_ENDIAN
#endif
#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define GBA_LITTLE_ENDIAN
#endif

#define fatal(...)                                                                     \
    char buff[0xFF] = "FATAL ERROR: ";                                                 \
    snprintf(buff + 13 * sizeof(char), sizeof(buff) - 13 * sizeof(char), __VA_ARGS__); \
    throw std::runtime_error(buff)

using u32 = uint32_t;
using u16 = uint16_t;
using u8 = uint8_t;

class Memory;
class ARM;
struct Bus {
    Memory* mem;
    ARM* armCpu;
};

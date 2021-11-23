#pragma once

#include <stdint.h>
#include <stdio.h>

#include <array>
#include <stdexcept>

#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
#define GBA_BIG_ENDIAN
#endif
#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define GBA_LITTLE_ENDIAN
#endif

#define UNUSED(expr) (void)(expr);

#define FATAL(...)                                           \
    do {                                                     \
        char buff[0xFF] = "FATAL ERROR: ";                   \
        snprintf(buff + 13, sizeof(buff) - 13, __VA_ARGS__); \
        throw std::runtime_error(buff);                      \
    } while (0)

using s64 = int64_t;
using u64 = uint64_t;

using s32 = int32_t;
using u32 = uint32_t;

using u16 = uint16_t;
using u8 = uint8_t;

constexpr auto POP_COUNT = [] {
    std::array<int, 256> arr = {};
    arr[0] = 0;
    for (size_t i = 0; i < 256; i++) {
        arr[i] = ((char)i & 1) + arr[i >> 1];
    }
    return arr;
}();

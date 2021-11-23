#pragma once

#include "general.hpp"

namespace gba {

enum class LCDState {
    VISIBLE,
    H_BLANK,
    V_BLANK,
};

enum Timing {
    VISIBLE = 240 << 2,
    H_BLANK = 68 << 2,
    LINE_CYCLES = VISIBLE + H_BLANK,

    VISIBLE_LINES = 160,
    V_BLANK_LINES = 68,
    TOTAL_LINES = VISIBLE_LINES + V_BLANK_LINES,
    V_BLANK = V_BLANK_LINES * LINE_CYCLES,
};

class LCD {
public:
    void reset();

    void draw_to(u32* pixelBuffer);
    void cycle();

    void write_vram(u32 address, u16 value);

    void write_dispcnt(u16 value);
    u16 read_vcount();

private:
    u32* pixelBuffer = nullptr;
    int cycleCnt;
    LCDState state;

    u16 dispCnt;
    u16 vcount;
};

}  // namespace gba

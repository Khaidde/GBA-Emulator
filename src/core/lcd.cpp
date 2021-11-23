#include "lcd.hpp"

#include "gba.hpp"

#ifdef DEBUG
#define LCD_ASSERT(cond, ...)   \
    do {                        \
        if (!(cond)) {          \
            FATAL(__VA_ARGS__); \
        }                       \
    } while (0)
#else
#define LCD_ASSERT(cond, ...) \
    do {                      \
    } while (0)
#endif

namespace gba {

enum DispCnt : u16 {
    BG_MODE_3 = 0x00000003,
    BG2_ENABLE = (1 << 10),
};

void LCD::reset() {
    dispCnt = BG_MODE_3 | BG2_ENABLE;
    vcount = 0;
    state = LCDState::VISIBLE;
}

void LCD::draw_to(u32* pixelBuffer) { this->pixelBuffer = pixelBuffer; }

void LCD::cycle() {
    LCD_ASSERT(pixelBuffer, "Pixel buffer is null");
    cycleCnt++;
    switch (state) {
        case LCDState::VISIBLE:
            if (cycleCnt >= VISIBLE) {
                cycleCnt = 0;
                state = LCDState::H_BLANK;
            }
            break;
        case LCDState::H_BLANK:
            if (cycleCnt >= H_BLANK) {
                cycleCnt = 0;
                vcount++;
                if (vcount == VISIBLE_LINES) {
                    state = LCDState::V_BLANK;
                } else {
                    state = LCDState::VISIBLE;
                }
            }
            break;
        case LCDState::V_BLANK:
            if (cycleCnt >= LINE_CYCLES) {
                cycleCnt = 0;
                vcount++;
                if (vcount == TOTAL_LINES) {
                    vcount = 0;
                    state = LCDState::VISIBLE;
                }
            }
            break;
    }
}

static constexpr double rgb888Ratio = (double)0xFF / 0x1F;

void LCD::write_vram(u32 address, u16 value) {
    LCD_ASSERT(0x06000000 <= address && address < 0x06017FFF,
               "Write to vram must be within range 0x06000000 to 0x06017FFF");
    u8 r = value & 0x1F;
    u8 g = (value >> 5) & 0x1F;
    u8 b = (value >> 10) & 0x1F;
    u8 r8 = (u8)(r * rgb888Ratio);
    u8 g8 = (u8)(g * rgb888Ratio);
    u8 b8 = (u8)(b * rgb888Ratio);

    pixelBuffer[(address - 0x06000000) >> 1] = (u32)((r8 << 16) | (g8 << 8) | b8);
}

void LCD::write_dispcnt(u16 value) {
    if (value != (BG2_ENABLE | BG_MODE_3)) {
        FATAL("Invalid write to dispcnt not allowed: %04x", value);
    }
    dispCnt = value;
}

u16 LCD::read_vcount() { return vcount; }

}  // namespace gba

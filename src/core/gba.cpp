#include "gba.hpp"

namespace gba {

LCD lcd;
Memory mem;
ARM cpu;
Keypad key;

void reset() {
    lcd.reset();
    mem.reset();
    cpu.reset();
    key.reset();
}

void cycle() {
    for (int i = 0; i < Timing::LINE_CYCLES * Timing::TOTAL_LINES; i++) {
        lcd.cycle();
        cpu.cycle();
        if (gba::cpu.swiInterrupt) {
            return;
        }
    }
}

}  // namespace gba

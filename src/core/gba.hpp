#pragma once

#include "arm7tdmi.hpp"
#include "keypad.hpp"
#include "lcd.hpp"
#include "memory.hpp"

namespace gba {

extern LCD lcd;
extern Memory mem;
extern ARM cpu;
extern Keypad key;

void reset();
void cycle();

}  // namespace gba

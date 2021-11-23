#include "keypad.hpp"

namespace gba {

void Keypad::reset() { keyinput = 0xFFFF; }

void Keypad::up_key(Button button) { keyinput |= button; }

void Keypad::down_key(Button button) { keyinput &= ~button; }

u16 Keypad::get_key_input() { return keyinput; }

}  // namespace gba

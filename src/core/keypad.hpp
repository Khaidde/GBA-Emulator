#pragma once

#include "general.hpp"

namespace gba {

enum Button : u16 {
    BUTTON_A = (1 << 0),
    BUTTON_B = (1 << 1),
    SELECT = (1 << 2),
    START = (1 << 3),
    RIGHT = (1 << 4),
    LEFT = (1 << 5),
    UP = (1 << 6),
    DOWN = (1 << 7),
    BUTTON_R = (1 << 8),
    BUTTON_L = (1 << 9),
};

class Keypad {
public:
    void reset();
    void up_key(Button button);
    void down_key(Button button);

    u16 get_key_input();

private:
    u16 keyinput;
};

}  // namespace gba

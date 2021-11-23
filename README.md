# GBA-Emulator
Very simple Game Boy Advance emulator with limited features. Created to run the GBA project in CS 2110 at Georgia Tech.

# Features
- Fully featured ARM7tdmi emulation (ARM/THUMB mode)
- Mode 3 Background Mode
- DMA3 Channel
- Keypad Input

# Controls
| keyboard | gba keypad |
| -------- | :--------: |
| wasd     | direction  |
| k        | button A   |
| j        | button B   |
| f        | start      |
| v        | select     |
| q        | button L   |
| o        | button R   |

# Make Targets
- build: (default) compile emulator for release
- build-debug: compile with debug symbols and detailed console output
- clean: remove all files from build directory

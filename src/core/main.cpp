#include <SDL.h>
#include <stdio.h>

#include <fstream>

#include "gba.hpp"

static void handle_key(bool isDown, u8 button) {
    gba::Button gbaButton;
    switch (button) {
        case SDLK_k: gbaButton = gba::Button::BUTTON_A; break;
        case SDLK_j: gbaButton = gba::Button::BUTTON_B; break;
        case SDLK_v: gbaButton = gba::Button::SELECT; break;
        case SDLK_f: gbaButton = gba::Button::START; break;
        case SDLK_d: gbaButton = gba::Button::RIGHT; break;
        case SDLK_a: gbaButton = gba::Button::LEFT; break;
        case SDLK_w: gbaButton = gba::Button::UP; break;
        case SDLK_s: gbaButton = gba::Button::DOWN; break;
        case SDLK_o: gbaButton = gba::Button::BUTTON_R; break;
        case SDLK_q: gbaButton = gba::Button::BUTTON_L; break;
        default: return;
    }
    if (isDown) {
        gba::key.down_key(gbaButton);
    } else {
        gba::key.up_key(gbaButton);
    }
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        printf("Usage: gbaemu [romFile.gba]\n");
        return EXIT_FAILURE;
    }

    std::ifstream file(argv[1], std::ios::binary | std::ios::ate);
    if (file.is_open()) {
        std::streamsize fileSize = file.tellg();
        u8 res[fileSize];
        file.seekg(0);
        file.read((char*)res, fileSize);
        file.close();

        gba::reset();
        for (u32 i = 0; i < fileSize; i += 4) {
            u32 word = *(u32*)&res[i];
            gba::mem.write32(i + 0x08000000, word);
        }
    } else {
        printf("ERR: Cannot open file: %s\n", argv[1]);
        return EXIT_FAILURE;
    }

    static constexpr int SCALE = 3;
    static constexpr int WIDTH = 240;
    static constexpr int HEIGHT = 160;

    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    SDL_Texture* bufferTexture = nullptr;

    try {
        SDL_Init(SDL_INIT_VIDEO);
        window = SDL_CreateWindow("GBA Emulator", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WIDTH * SCALE,
                                  HEIGHT * SCALE, SDL_WINDOW_SHOWN);
        if (!window) {
            FATAL("Window could not be created! SDL_Error: %s\n", SDL_GetError());
        }
        renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE);
        bufferTexture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGB888, SDL_TEXTUREACCESS_STREAMING, WIDTH, HEIGHT);
        if (!bufferTexture) {
            FATAL("Buffer texture could not be created! SDL_Error: %s\n", SDL_GetError());
        }

        u32* pixelBuffer;

        int pitch = 0;
        bool running = true;
        SDL_Event e;
        int nextFrame = (int)SDL_GetTicks();
        while (running) {
            while (SDL_PollEvent(&e)) {
                switch (e.type) {
                    case SDL_QUIT: running = false; break;
                    case SDL_KEYDOWN: handle_key(true, e.key.keysym.sym); break;
                    case SDL_KEYUP: handle_key(false, e.key.keysym.sym); break;
                }
            }

            SDL_RenderClear(renderer);
            if (SDL_LockTexture(bufferTexture, nullptr, (void**)&pixelBuffer, &pitch)) {
                FATAL("Failed to lock texture! SDL_error: %s\n", SDL_GetError());
            }
            pitch /= sizeof(u32);

            gba::lcd.draw_to(pixelBuffer);

            gba::cycle();
            if (gba::cpu.swiInterrupt) {
                running = false;
                gba::cpu.x_regs();
                printf("(BIOS and software interrupts are not implemented)\n");
            }

            SDL_UnlockTexture(bufferTexture);
            SDL_RenderCopy(renderer, bufferTexture, nullptr, nullptr);
            SDL_RenderPresent(renderer);

            nextFrame += 16;
            int delay = nextFrame - (int)SDL_GetTicks();
            if (delay > 0) {
                SDL_Delay((u32)delay);
            }
        }
    } catch (const std::exception& e) {
        printf("%s\n", e.what());
        SDL_DestroyTexture(bufferTexture);
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);

        SDL_Quit();
        return EXIT_FAILURE;
    }

    SDL_DestroyTexture(bufferTexture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);

    SDL_Quit();
    return EXIT_SUCCESS;
}

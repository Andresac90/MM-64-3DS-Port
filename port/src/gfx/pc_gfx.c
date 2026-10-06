/*
 * pc_gfx.c — glue between the port scheduler and the fast3d renderer.
 * Provides the strong PortGfx_RunTask that overrides sched_shim's weak stub.
 * NOTE: no glReadPixels here — readback is unreliable/unstable under WSLg.
 * Diagnostics use pure stderr logging from gfx_pc.c instead.
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <SDL2/SDL_opengl.h>
#include <PR/gbi.h>
#include "ultra64/sptask.h"
#include "gfx_pc.h"
#include "gfx_sdl.h"
#include "gfx_opengl.h"

static int sInited = 0;

void PortGfx_Init(void) {
    gfx_init(&gfx_sdl, &gfx_opengl_api);
    sInited = 1;
    fprintf(stderr, "[gfx] SDL2+OpenGL renderer initialized\n");
}

/* called from gfx_pc.c after gfx_flush(), before the buffer swap.
   Under Xvfb+llvmpipe glReadPixels is reliable, so a one-shot GL_BACK grab
   at PORT_DUMP_FRAME writes /tmp/frame.ppm then exits. */
void PortGfx_FrameReady(void) {
    if (getenv("PORT_SCAN")) {
        static unsigned sf = 0;
        if (++sf % 20 == 0) {
            int w = 320, h = 240;
            static unsigned char b[320*240*3];
            glReadBuffer(GL_BACK);
            glFinish();
            glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, b);
            unsigned long t = 0; unsigned mx = 0;
            for (int i = 0; i < w*h*3; i++) { t += b[i]; if (b[i] > mx) mx = b[i]; }
            fprintf(stderr, "[scan] f%u mean=%.1f max=%u\n", sf, (double)t/(w*h*3), mx);
        }
        return;
    }
    const char* fs = getenv("PORT_DUMP_FRAME");
    if (fs == NULL) return;
    static unsigned frame = 0;
    if (++frame != (unsigned)atoi(fs)) return;
    int w = 640, h = 480;
    unsigned char* px = (unsigned char*)malloc(w * h * 3);
    glReadBuffer(GL_BACK);
    glFinish();
    glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, px);
    FILE* f = fopen("/tmp/frame.ppm", "wb");
    fprintf(f, "P6\n%d %d\n255\n", w, h);
    for (int y = h - 1; y >= 0; y--) fwrite(px + y * w * 3, 1, w * 3, f);
    fclose(f);
    fprintf(stderr, "[gfx] dumped frame to /tmp/frame.ppm\n");
    exit(0);
}

extern unsigned int gfx_port_tri_count;
static unsigned int sTaskCount = 0;

void PortGfx_RunTask(OSTask* task) {
    if (!sInited) {
        PortGfx_Init();
    }
    if ((++sTaskCount % 60) == 0) {
        fprintf(stderr, "[gfx] task %u: %u tris last frame\n", sTaskCount, gfx_port_tri_count);
    }
    gfx_port_tri_count = 0;
    gfx_start_frame();
    gfx_run((Gfx*)task->t.data_ptr);
    gfx_end_frame();
}

/* ------------------------------------------------------------------ */
/* Keyboard input -> N64 controller. gfx_sdl2 calls these on key events;
 * PortInput_GetPad (called by osContGetReadData) builds the OSContPad.
 * Key map:  Z=A  X=B  Enter=Start  LShift=Z-trig  A=L  S=R
 *           arrows=analog stick   I/K/J/L=C-up/down/left/right
 *           T/F/G/H=D-pad up/left/down/right                       */
#include <SDL2/SDL_scancode.h>

#define BTN_A_      0x8000
#define BTN_B_      0x4000
#define BTN_Z_      0x2000
#define BTN_START_  0x1000
#define BTN_DUP_    0x0800
#define BTN_DDOWN_  0x0400
#define BTN_DLEFT_  0x0200
#define BTN_DRIGHT_ 0x0100
#define BTN_L_      0x0020
#define BTN_R_      0x0010
#define BTN_CUP_    0x0008
#define BTN_CDOWN_  0x0004
#define BTN_CLEFT_  0x0002
#define BTN_CRIGHT_ 0x0001

static unsigned char sKeyState[512];

bool keyboard_on_key_down(int scancode) {
    if (scancode >= 0 && scancode < 512) sKeyState[scancode] = 1;
    return true;
}
bool keyboard_on_key_up(int scancode) {
    if (scancode >= 0 && scancode < 512) sKeyState[scancode] = 0;
    return true;
}
void keyboard_on_all_keys_up(void) {
    for (int i = 0; i < 512; i++) sKeyState[i] = 0;
}

static int kd(int sc) { return (sc >= 0 && sc < 512) ? sKeyState[sc] : 0; }

/* Fill button bits + analog stick from current key state. Returns button mask;
 * writes stick deltas through pointers. */
unsigned short PortInput_GetPad(signed char* outX, signed char* outY) {
    unsigned short b = 0;
    if (kd(SDL_SCANCODE_Z))      b |= BTN_A_;
    if (kd(SDL_SCANCODE_X))      b |= BTN_B_;
    if (kd(SDL_SCANCODE_RETURN)) b |= BTN_START_;
    if (kd(SDL_SCANCODE_LSHIFT) || kd(SDL_SCANCODE_RSHIFT)) b |= BTN_Z_;
    if (kd(SDL_SCANCODE_A))      b |= BTN_L_;
    if (kd(SDL_SCANCODE_S))      b |= BTN_R_;
    if (kd(SDL_SCANCODE_I))      b |= BTN_CUP_;
    if (kd(SDL_SCANCODE_K))      b |= BTN_CDOWN_;
    if (kd(SDL_SCANCODE_J))      b |= BTN_CLEFT_;
    if (kd(SDL_SCANCODE_L))      b |= BTN_CRIGHT_;
    if (kd(SDL_SCANCODE_T))      b |= BTN_DUP_;
    if (kd(SDL_SCANCODE_G))      b |= BTN_DDOWN_;
    if (kd(SDL_SCANCODE_F))      b |= BTN_DLEFT_;
    if (kd(SDL_SCANCODE_H))      b |= BTN_DRIGHT_;

    int x = 0, y = 0;
    if (kd(SDL_SCANCODE_LEFT))  x -= 80;
    if (kd(SDL_SCANCODE_RIGHT)) x += 80;
    if (kd(SDL_SCANCODE_UP))    y += 80;
    if (kd(SDL_SCANCODE_DOWN))  y -= 80;
    if (outX) *outX = (signed char)x;
    if (outY) *outY = (signed char)y;
    return b;
}

bool configFullscreen = false;

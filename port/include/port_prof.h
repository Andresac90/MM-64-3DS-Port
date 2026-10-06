#ifndef PORT_PROF_H
#define PORT_PROF_H
/* PORT (2026-09-30): sampling profiler for hardware (3ds_main.c Port3ds_ProfThread). The code stores the
 * stage it is in into one byte (no system call: the tick timers of perf_stages cost a kernel trap each
 * and distorted the profile); a thread on another core samples that byte every 250 us and the perf report
 * logs the share of samples per stage. Enabled with prof=1 in settings.txt (the byte stores are always
 * compiled in: one store per stage change). */
enum {
    PROF_GAME,       /* game logic + display-list building (outside the render task) */
    PROF_DL,         /* display-list walk: command decode, state commands */
    PROF_VTX,        /* G_VTX: vertex transform + lighting */
    PROF_TRI,        /* triangle: reject/cull, render-state setup */
    PROF_TRI_BUILD,  /* triangle: packed vertices (uv, colour) */
    PROF_EMIT,       /* triangle: clip/split + write to the VBO */
    PROF_TEX,        /* texture import (decode + upload) */
    PROF_RECT,       /* texture / fill rectangles */
    PROF_MTX,        /* G_MTX */
    PROF_FLUSH,      /* draw submission to the backend */
    PROF_AUDIO,      /* audio pumps (audio engine on the main thread) */
    PROF_PACE,       /* waiting for retraces */
    PROF_SWAP,       /* frame end: GPU submit, readbacks */
    PROF_GPUWAIT,    /* C3D_FrameBegin: waiting for the GPU */
    PROF_REPLAY,     /* 60 fps replay: positions + re-issue */
    PROF_INPUT,      /* input, touch panel */
    PROF_VTX_LIGHT,  /* G_VTX: lighting + texgen part */
    PROF_SUBMIT,     /* citro3d draw call (state upload + command emission) */
    PROF_SPLIT,      /* N64-exact triangle splitting/clipping (gfx_emit_tri_one's slow path) */
    PROF_GPU_PAL,    /* GPU path: matrix palette lookup/insert per triangle */
    PROF_GPU_PACK,   /* GPU path: packing vertices new to the batch (uv, colour, palette index) */
    /* PORT (2026-10-04): finer stages for the Old 3DS hardware profile */
    PROF_VTX_BOX,    /* G_VTX: the load's bounding box (off-screen rejection, raw-path near/depth test) */
    PROF_VTX_RAW,    /* G_VTX: raw-path load (pointers only) */
    PROF_RAW_EMIT,   /* raw-path triangle: palette, parameters, 16-byte vertex copies */
    PROF_C3D_CTX,    /* citro3d C3Di_UpdateContext: dirty render state, textures, combiners, program */
    PROF_C3D_UNIF,   /* citro3d C3D_UpdateUniforms */
    PROF_C3D_DRAW,   /* citro3d draw command (fixed register sequence) */
    PROF_COUNT
};
extern volatile unsigned char gPortProf;
#define PROF_SET(s) (gPortProf = (unsigned char)(s))
#define PROF_PUSH(s) unsigned char _profPrev = gPortProf; gPortProf = (unsigned char)(s)
#define PROF_POP() (gPortProf = _profPrev)
#endif

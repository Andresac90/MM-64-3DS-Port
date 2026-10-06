#ifdef TARGET_N3DS

#include <3ds.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <citro3d.h>
#include "gfx_3ds.h"

/* PORT PERF (2026-10-04): CPU cache maintenance through the kernel directly. GSPGPU_FlushDataCache / DSP_FlushDataCache
 * are requests to the GPU and DSP system services, which run on the system core (1); the kernel call they end up making
 * is available to the title itself (port/oot.rsf FlushProcessDataCache / InvalidateProcessDataCache). Hardware v54: with
 * a large share of that core reserved for the audio mixer, those requests slowed everything (the audio engine 3-4x). The
 * per-frame flushes go straight to the kernel; the service request remains the fallback if the kernel call fails. */
u32 gPortPerfCacheFallbacks;
void Port3ds_CacheFlush(const void* p, u32 size) {
    if (size == 0) return;
    if (R_FAILED(svcFlushProcessDataCache(CUR_PROCESS_HANDLE, (u32)p, size))) {
        gPortPerfCacheFallbacks++;
        GSPGPU_FlushDataCache(p, size);
    }
}
void Port3ds_CacheInvalidate(const void* p, u32 size) {
    if (size == 0) return;
    if (R_FAILED(svcInvalidateProcessDataCache(CUR_PROCESS_HANDLE, (u32)p, size))) {
        gPortPerfCacheFallbacks++;
        GSPGPU_InvalidateDataCache(p, size);
    }
}

// #define DISPLAY_TRANSFER_FLAGS_NORMAL \
// 	(GX_TRANSFER_FLIP_VERT(0) | GX_TRANSFER_OUT_TILED(0) | GX_TRANSFER_RAW_COPY(0) | \
// 	GX_TRANSFER_IN_FORMAT(GX_TRANSFER_FMT_RGBA8) | GX_TRANSFER_OUT_FORMAT(GX_TRANSFER_FMT_RGB8) | \
// 	GX_TRANSFER_SCALING(GX_TRANSFER_SCALE_NONE))

// #define DISPLAY_TRANSFER_FLAGS_AA_12 \
// 	(GX_TRANSFER_FLIP_VERT(0) | GX_TRANSFER_OUT_TILED(0) | GX_TRANSFER_RAW_COPY(0) | \
// 	GX_TRANSFER_IN_FORMAT(GX_TRANSFER_FMT_RGBA8) | GX_TRANSFER_OUT_FORMAT(GX_TRANSFER_FMT_RGB8) | \
// 	GX_TRANSFER_SCALING(GX_TRANSFER_SCALE_X))

// #define DISPLAY_TRANSFER_FLAGS_AA_22 \
// 	(GX_TRANSFER_FLIP_VERT(0) | GX_TRANSFER_OUT_TILED(0) | GX_TRANSFER_RAW_COPY(0) | \
// 	GX_TRANSFER_IN_FORMAT(GX_TRANSFER_FMT_RGBA8) | GX_TRANSFER_OUT_FORMAT(GX_TRANSFER_FMT_RGB8) | \
// 	GX_TRANSFER_SCALING(GX_TRANSFER_SCALE_XY))

// #ifdef N3DS_USE_ANTIALIASING

// #define DISPLAY_TRANSFER_FLAGS \
// 	(GX_TRANSFER_FLIP_VERT(0) | GX_TRANSFER_OUT_TILED(0) | GX_TRANSFER_RAW_COPY(0) | \
// 	GX_TRANSFER_IN_FORMAT(GX_TRANSFER_FMT_RGBA8) | GX_TRANSFER_OUT_FORMAT(GX_TRANSFER_FMT_RGB8) | \
// 	GX_TRANSFER_SCALING(GX_TRANSFER_SCALE_XY))

// #else

// #define DISPLAY_TRANSFER_FLAGS \
// 	(GX_TRANSFER_FLIP_VERT(0) | GX_TRANSFER_OUT_TILED(0) | GX_TRANSFER_RAW_COPY(0) | \
// 	GX_TRANSFER_IN_FORMAT(GX_TRANSFER_FMT_RGBA8) | GX_TRANSFER_OUT_FORMAT(GX_TRANSFER_FMT_RGB8) | \
// 	GX_TRANSFER_SCALING(GX_TRANSFER_SCALE_NONE))

// #endif

static C3D_RenderTarget* sTarget;
/* PORT (2026-09-30): stereoscopic 3D. With the 3D slider up the top screen switches from the mono mode
 * (800px wide + AA on hardware; the wide mode cannot show 3D) to two 400x240 eyes rendered side by side
 * into one 240x800 target: every draw is issued twice with a different viewport and eye uniform
 * (gfx_citro3d.c), so the display list is processed once and no render-target switching is needed. Two
 * output-only targets alias the halves, so citro3d's FrameEnd transfers them to the left/right
 * framebuffers itself. Slider down: back to the mono mode, zero extra cost. */
float gPortStereoSep;
/* convergence (clip w at screen depth) for the stereo shader: Link's distance from the camera, set by
 * the game each frame (gPortStereoFocusW, z_play.c; 0 when there is no player), smoothed so camera
 * cuts and Link's movement don't make the depth jump. */
float gPortStereoFocusW;
int gPortStereoFlatScene; /* set by menu gamestates (file select) each frame; cleared after the frame */
/* PORT (2026-10-03): the renderer's copies of the game's per-frame requests, taken when the frame is handed over
 * (Port3ds_ApplyFrameRequests): with the render thread (3ds_main.c) the game already builds the next frame and sets
 * them again while this one is drawn */
float gPortStereoFocusWR;
int gPortStereoFlatSceneR;
int gPortRenderThreaded; /* 3ds_main.c: frames are drawn by the render thread (the game thread never touches the GPU) */
/* the render watchdog's markers (3ds_main.c Port3ds_RenderWaitIdle logs the last one if a frame takes over 2 s) */
extern void Port3ds_RenderPhase(int phase);
extern int Port3ds_RenderPhaseGet(void);
#define PHASE_IN(n) const int _phasePrev = Port3ds_RenderPhaseGet(); Port3ds_RenderPhase(n)
#define PHASE_OUT() Port3ds_RenderPhase(_phasePrev)
float gPortStereoConv = 150.0f;
static Gfx3DSMode sMonoMode;
static bool sMonoAA;
static void gfx_3ds_mono_config(bool aa);
static void flip_init(void); /* the flip presenter (below) */
static void flip_stop(void);
/* settings aa=0/1 (hardware only: AA is part of N3DS_USE_ANTIALIASING builds). PORT PERF (2026-10-03): off by
 * default - hardware v41 (New 3DS, 2D): GPU 11.6 -> 4.6-5.0 ms per frame without it, frames displayed 37.5 -> 52.6-53.8
 * per second with the present gate; the 2x2 (2x vertical in 800-pixel mode) supersampling more than doubled the GPU
 * work, which was the frame-rate limit */
int gPortAA = 0;
static u32 sMonoFlags;
static int sMonoW, sMonoH;
static bool sMonoWide;
static C3D_RenderTarget* sEyeOut[2];

/* the region readbacks use: one eye in stereo (the left half, laid out like the normal mode) */
static int ViewW(void) {
    return sTarget->frameBuf.width;
}
static int ViewH(void) {
    return gGfx3DSMode == GFX_3DS_MODE_STEREO ? STEREO_EYE_OFFSET : sTarget->frameBuf.height;
}

Gfx3DSMode gGfx3DSMode;

static bool checkN3DS()
{
    bool isNew3DS = false;

	if (R_SUCCEEDED(APT_CheckNew3DS(&isNew3DS)))
		return isNew3DS;

	return false;
}

static void gfx_3ds_init(void) 
{
    extern int gPortO3dsSim;
    if (checkN3DS() && !gPortO3dsSim)
		osSetSpeedupEnable(true);

    /* PORT (2026-09-30): gfx is already initialized by main() (for the boot log). A second gfxInitDefault
     * took a second GSP reference (libctru reference-counts gspInit) and allocated unused framebuffers;
     * the single gfxExit at shutdown then left GSP's event thread running while exit() unmapped the heap
     * holding its stack: HOME -> Close crashed in gspEventThreadMain (hardware v17, v18). */
    consoleInit(GFX_BOTTOM, NULL);
    /* PORT PERF (2026-10-03): 1 MB instead of 256 KB. With the overlap (gPortOverlap) an update's frames share one
     * citro3d frame, so the buffer holds up to three frames of commands, and running out is a panic (libctru
     * GPUCMD_AddInternal svcBreak), not a dropped draw. The perf report logs the peak use. */
    C3D_Init(0x100000);

    bool useAA = false;
#ifdef N3DS_USE_ANTIALIASING
    useAA = gPortAA != 0;
#endif
    bool useWide = false;
#ifdef N3DS_USE_WIDE_800PX
    u8 model;
    CFGU_GetSystemModel(&model);
    useWide = model != 3; //wide is not possible on o2ds
    /* PORT (2026-09-24): Citra-family emulators (Azahar) don't implement the 800px wide
     * top-screen mode -- they show only the left 400 columns at 2x, which looked like an
     * off-center camera and pushed centered UI (the pause menu) off-screen. Detect the
     * emulator via its emulator-only system-info type (0x20000); real hardware returns an
     * error there, so hardware keeps the sharper wide mode. */
    {
        s64 emu = 0;
        if (R_SUCCEEDED(svcGetSystemInfo(&emu, 0x20000, 0)))
            useWide = false;
    }
#endif

    sMonoWide = useWide;
    gfx_3ds_mono_config(useAA);
    sTarget = C3D_RenderTargetCreate(sMonoW, sMonoH, GPU_RB_RGBA8, GPU_RB_DEPTH24_STENCIL8);
	C3D_RenderTargetSetOutput(sTarget, GFX_TOP, GFX_LEFT, sMonoFlags);
    gGfx3DSMode = sMonoMode;
    flip_init();

    if (useWide)
        gfxSetWide(true);
}

/* the mono target's size, output transfer and mode for anti-aliasing on/off (2x2 supersampling, or 1x2 in the
 * 800px wide mode) */
static void gfx_3ds_mono_config(bool aa) {
    u32 flags = GX_TRANSFER_FLIP_VERT(0) | GX_TRANSFER_OUT_TILED(0) | GX_TRANSFER_RAW_COPY(0) |
                GX_TRANSFER_IN_FORMAT(GX_TRANSFER_FMT_RGBA8) | GX_TRANSFER_OUT_FORMAT(GX_TRANSFER_FMT_RGB8);
    if (aa && !sMonoWide)
        flags |= GX_TRANSFER_SCALING(GX_TRANSFER_SCALE_XY);
    else if (aa && sMonoWide)
        flags |= GX_TRANSFER_SCALING(GX_TRANSFER_SCALE_X);
    else
        flags |= GX_TRANSFER_SCALING(GX_TRANSFER_SCALE_NO);
    sMonoFlags = flags;
    sMonoW = aa ? 480 : 240;                /* target height (the screen's short axis) */
    sMonoH = aa || sMonoWide ? 800 : 400;   /* target width */
    sMonoMode = !aa ? (sMonoWide ? GFX_3DS_MODE_WIDE : GFX_3DS_MODE_NORMAL)
                    : (sMonoWide ? GFX_3DS_MODE_WIDE_AA_12 : GFX_3DS_MODE_AA_22);
    sMonoAA = aa;
}

/* PORT (2026-10-01): anti-aliasing at run time (settings aa=0/1; aa_ab=1 alternates it every 2 perf reports
 * for a hardware A/B). The AA target is 4x the pixels of the screen and the 60 fps mode draws three frames per
 * update: hardware v30 spent 7.5 ms per update in C3D_FrameBegin waiting for the GPU. Rebuilt between frames
 * like the stereo switch (citro3d cannot delete a target inside a frame); no change while 3D is on. */
static void gfx_3ds_update_aa(void) {
    if (gGfx3DSMode == GFX_3DS_MODE_STEREO || (gPortAA != 0) == sMonoAA) {
        return;
    }
    { extern void PortDbg(const char*); PortDbg(gPortAA ? "[gfx] anti-aliasing on" : "[gfx] anti-aliasing off"); }
    C3D_RenderTargetDelete(sTarget);
    gfx_3ds_mono_config(gPortAA != 0);
    sTarget = C3D_RenderTargetCreate(sMonoW, sMonoH, GPU_RB_RGBA8, GPU_RB_DEPTH24_STENCIL8);
    C3D_RenderTargetSetOutput(sTarget, GFX_TOP, GFX_LEFT, sMonoFlags);
    gGfx3DSMode = sMonoMode;
    {
        extern void gfx_3ds_drop_readbacks(void);
        gfx_3ds_drop_readbacks();
    }
}

/* Called before C3D_FrameBegin: citro3d breaks (svcBreak) on C3D_RenderTargetDelete inside a frame, and
 * the delete waits for the GPU queue itself. The mono and stereo targets never coexist (VRAM: the
 * 480x800 AA target alone is 3 MB). */
static void gfx_3ds_update_stereo(void) {
    float slider = osGet3DSliderState();
    bool want;
    { /* test aid: stereo_test=1 in settings.txt switches 3D on and off every 2 s (no slider needed) */
        extern int gPortStereoTest;
        static unsigned sTestFrames;
        if (gPortStereoTest) slider = ((++sTestFrames / 120) & 1) ? 1.0f : 0.0f;
    }
    want = slider > 0.02f;
    bool on = gGfx3DSMode == GFX_3DS_MODE_STEREO;
    int e;

    /* full slider: 0.04 NDC per eye = ~16 px of disparity at infinity (v1 had 0.03 / 12 px and a
     * fixed convergence: "not deep enough" on hardware) */
    gPortStereoSep = want ? slider * 0.04f : 0.0f;
    {
        float target = gPortStereoFocusWR;
        float border = 0.0f;
        { /* the nearest 3D surface at the screen borders (gfx_pc.c stereo_probe_tri): the 3rd-nearest
           * of the 13 border probes, so one particle near the lens doesn't drag the screen plane */
            extern float gPortStereoProbeW[13];
            float v[13];
            int n = 0, a, b;
            for (a = 0; a < 13; a++) {
                if (gPortStereoProbeW[a] > 0.0f) v[n++] = gPortStereoProbeW[a];
                gPortStereoProbeW[a] = 0.0f;
            }
            for (a = 1; a < n; a++) { /* insertion sort */
                float x = v[a];
                for (b = a - 1; b >= 0 && v[b] > x; b--) v[b + 1] = v[b];
                v[b + 1] = x;
            }
            if (n >= 3) border = v[2];
            else if (n > 0) border = v[n - 1];
        }
        if (border > 0.0f && border < target) target = border; /* keep border geometry out of the air */
        if (target < 10.0f) target = 10.0f;  /* the camera skimming the ground (title intro: w = 16) */
        if (target > 600.0f) target = 600.0f;
        {
            /* pre-rendered rooms (a S2DEX background was drawn last frame): converge at half Link's
             * distance so he sits ~50% deep, inside the room picture (drawn at 75%) */
            extern int gPortPrerenderedFrame;
            if (gPortPrerenderedFrame) target *= 0.5f;
        }
        /* asymmetric: pull the screen plane nearer fast (border geometry must never pop out), let it
         * recede slowly (no depth "breathing" when the border estimate jumps between frames) */
        gPortStereoConv += (target - gPortStereoConv) * (target < gPortStereoConv ? 0.3f : 0.04f);
    }
    if (want == on) {
        return;
    }
    { extern void PortDbg(const char*); PortDbg(want ? "[stereo] enter" : "[stereo] leave"); }
    if (want) {
        C3D_RenderTarget* st;
        static int sRetryWait;
        if (sRetryWait > 0) { /* a failed switch retries once a second, not every frame */
            sRetryWait--;
            gPortStereoSep = 0.0f;
            return;
        }
        /* free the mono target FIRST: VRAM cannot hold both (plus the pause menu's off-screen targets,
         * which stay allocated once used; creating the stereo target next to the mono one failed on
         * hardware after a pause, and 3D never came back) */
        C3D_RenderTargetDelete(sTarget); /* also unlinks it from the top screen */
        sTarget = NULL;
        st = C3D_RenderTargetCreate(240, 2 * STEREO_EYE_OFFSET, GPU_RB_RGBA8, GPU_RB_DEPTH24_STENCIL8);
        for (e = 0; e < 2 && st != NULL; e++) {
            sEyeOut[e] = C3D_RenderTargetCreate(240, STEREO_EYE_OFFSET, GPU_RB_RGBA8, -1);
            if (sEyeOut[e] != NULL) {
                /* alias half e of the stereo target (the tiled buffer is contiguous along the long axis) */
                vramFree(sEyeOut[e]->frameBuf.colorBuf);
                sEyeOut[e]->frameBuf.colorBuf = (u8*)st->frameBuf.colorBuf + e * 240 * STEREO_EYE_OFFSET * 4;
                sEyeOut[e]->ownsColor = false;
            }
        }
        if (st == NULL || sEyeOut[0] == NULL || sEyeOut[1] == NULL) {
            { extern void PortDbg(const char*); PortDbg("[stereo] no VRAM for the stereo target, staying mono"); }
            for (e = 0; e < 2; e++) {
                if (sEyeOut[e] != NULL) C3D_RenderTargetDelete(sEyeOut[e]), sEyeOut[e] = NULL;
            }
            if (st != NULL) C3D_RenderTargetDelete(st);
            sTarget = C3D_RenderTargetCreate(sMonoW, sMonoH, GPU_RB_RGBA8, GPU_RB_DEPTH24_STENCIL8);
            C3D_RenderTargetSetOutput(sTarget, GFX_TOP, GFX_LEFT, sMonoFlags);
            gPortStereoSep = 0.0f;
            sRetryWait = 60;
            return;
        }
        sTarget = st;
        {
            u32 flags = GX_TRANSFER_FLIP_VERT(0) | GX_TRANSFER_OUT_TILED(0) | GX_TRANSFER_RAW_COPY(0) |
                        GX_TRANSFER_IN_FORMAT(GX_TRANSFER_FMT_RGBA8) | GX_TRANSFER_OUT_FORMAT(GX_TRANSFER_FMT_RGB8) |
                        GX_TRANSFER_SCALING(GX_TRANSFER_SCALE_NO);
            C3D_RenderTargetSetOutput(sEyeOut[0], GFX_TOP, GFX_LEFT, flags);
            C3D_RenderTargetSetOutput(sEyeOut[1], GFX_TOP, GFX_RIGHT, flags);
        }
        gfxSetWide(false);
        gfxSet3D(true);
        gGfx3DSMode = GFX_3DS_MODE_STEREO;
    } else {
        for (e = 0; e < 2; e++) {
            C3D_RenderTargetDelete(sEyeOut[e]); /* not the owner of the color buffer */
            sEyeOut[e] = NULL;
        }
        C3D_RenderTargetDelete(sTarget); /* first, same VRAM reason as above */
        sTarget = C3D_RenderTargetCreate(sMonoW, sMonoH, GPU_RB_RGBA8, GPU_RB_DEPTH24_STENCIL8);
        C3D_RenderTargetSetOutput(sTarget, GFX_TOP, GFX_LEFT, sMonoFlags);
        gfxSet3D(false);
        gfxSetWide(sMonoWide);
        gGfx3DSMode = sMonoMode;
    }
    /* the depth/color readbacks were sized for the old target */
    {
        extern void gfx_3ds_drop_readbacks(void);
        gfx_3ds_drop_readbacks();
    }
}

static void gfx_3ds_main_loop(void (*run_one_game_iter)(void)) 
{
    while (aptMainLoop())
        run_one_game_iter();

    flip_stop();
    ndspExit();
    C3D_Fini();
	gfxExit();
}

static void gfx_3ds_get_dimensions(uint32_t *width, uint32_t *height) 
{
    *width = 400;
    *height = 240;
}

/* PORT (2026-09-29): service the HOME button / sleep / power once per frame (the game's own loop never
 * reaches gfx_3ds_main_loop, so nothing called aptMainLoop and HOME did nothing). aptMainLoop blocks
 * while the HOME menu is open and returns false when the user closes the software. */
static void gfx_3ds_handle_events(void)
{
    if (!aptMainLoop()) {
        { extern void PortSram_FlushNow(void); PortSram_FlushNow(); } /* a save the game just made */
        { extern void Port3ds_LogFlush(void); Port3ds_LogFlush(); } /* log lines the writer has not written */
        /* PORT (2026-09-30): shut graphics down before exit(). exit() unmaps the app heap, and libctru's
         * GSP event thread (stack on that heap) was still running: HOME -> Close crashed with a data
         * abort in gspEventThreadMain (hardware, v17). gfxExit stops that thread. */
        flip_stop(); /* its thread presents through GSP: stopped before graphics shut down */
        ndspExit();
        C3D_Fini();
        gfxExit();
        exit(0);
    }
}

/* Depth readback for the game's CPU reads of the N64 z-buffer (point-light glows in z_lights.c, the
 * sun's lens-flare test in z_kankyo.c). When a frame's render completes, its D24S8 depth buffer is copied (GPU display transfer, detiled) into linear memory; the copy
 * only runs while the game has asked for depth recently. port/src/zbuffer_port.c converts samples to
 * N64 z-buffer words. */
static u32* sDepthLinear[2]; /* same slot convention as the color readback below */
static int sDepthSlot = -1;   /* slot of the most recent depth readback */
static int sDepthWantFrames;
static bool sDepthValid;
static int sFrameSlot;        /* slot this frame's readbacks go to (alternates each frame) */
static bool sInFrame;         /* between C3D_FrameBegin and C3D_FrameEnd */
static unsigned sFramesDone;  /* frames ended so far */
static unsigned sDepthOfFrame = ~0u; /* sFramesDone when the current copy was taken */
static void gfx_3ds_depth_lazy(void);
static void depth_async_wait(void);
/* PORT PERF (2026-10-02): EARLY depth copy with 60 fps replays. The lazy copy below waits for the GPU to
 * finish the logic frame's last shown frame; with the present gate that frame is handed to the GPU right at
 * the update boundary, so the game's first depth read (Navi's glow test, almost every update) waited for most
 * of its GPU time: game logic 8.9 -> 14.4 ms per update (hardware v39). Instead, when the game read depth in
 * the previous update, the logic frame's FIRST shown frame (the walk's in-between frame) is copied before the
 * next frame starts, where C3D_FrameBegin would wait for it anyway: only the copy itself costs. That depth is
 * up to 2/3 of an update older than the logic frame's own (the N64 reads the previous frame's). */
static unsigned sDepthReadAt = ~0u; /* sFramesDone at the game's latest depth read */
static bool sLogicStarted = true;   /* the next in-between frame is a logic frame's first */
static bool sDepthEarlyPending;     /* that frame was just submitted: copy its depth before the next one */

void Port3ds_RequestDepth(void) {
    sDepthWantFrames = 60;
}

static const u32* depth_read_threaded(int* width, int* height);
static int sColorWantFrames; /* (defined with the color readback below) */
const u32* Port3ds_GetDepth(int* width, int* height) {
    if (gPortRenderThreaded && sColorWantFrames == 0) { /* (tools: every frame is copied and waited for) */
        return depth_read_threaded(width, height);
    }
    sDepthReadAt = sFramesDone;
    depth_async_wait();
    gfx_3ds_depth_lazy();
    *width = ViewW();
    *height = ViewH();
    return (sDepthValid && sDepthSlot >= 0) ? sDepthLinear[sDepthSlot] : NULL;
}

/* back = 0: most recent frame, 1: the one before (tools/statediff dumps both with the colors) */
const u32* Port3ds_GetDepthSlot(int back, int* width, int* height) {
    *width = ViewW();
    *height = ViewH();
    if (sDepthSlot < 0) {
        return NULL;
    }
    return sDepthLinear[back ? (sDepthSlot ^ 1) : sDepthSlot];
}

/* synchronous depth copy after C3D_FrameEnd (waits for the GPU): the v23 behaviour, default */
static void gfx_3ds_read_back_depth(void) {
    size_t size = (size_t)ViewW() * ViewH() * 4;

    if (sDepthWantFrames <= 0) {
        sDepthValid = false;
        return;
    }
    sDepthWantFrames--;
    if (sDepthLinear[sFrameSlot] == NULL) {
        sDepthLinear[sFrameSlot] = linearAlloc(size);
        if (sDepthLinear[sFrameSlot] == NULL) {
            return;
        }
    }
    C3D_SyncDisplayTransfer((u32*)sTarget->frameBuf.depthBuf,
                            GX_BUFFER_DIM(ViewW(), ViewH()), sDepthLinear[sFrameSlot],
                            GX_BUFFER_DIM(ViewW(), ViewH()),
                            GX_TRANSFER_FLIP_VERT(0) | GX_TRANSFER_OUT_TILED(0) | GX_TRANSFER_RAW_COPY(0) |
                                GX_TRANSFER_IN_FORMAT(GX_TRANSFER_FMT_RGBA8) |
                                GX_TRANSFER_OUT_FORMAT(GX_TRANSFER_FMT_RGBA8) |
                                GX_TRANSFER_SCALING(GX_TRANSFER_SCALE_NO));
    GSPGPU_InvalidateDataCache(sDepthLinear[sFrameSlot], size);
    sDepthSlot = sFrameSlot;
    sDepthValid = true;
}

/* PORT PERF (2026-10-01): LAZY depth copy. The game reads depth (Navi's glow, the sun's lens flare) during
 * its next update; the copy used to run right after C3D_FrameEnd, where C3D_SyncDisplayTransfer first
 * waits for the GPU to finish the whole frame (hardware v23 profile: "swap" 15-24% of New 3DS time, 9-13%
 * of Old 3DS). Now the same synchronous copy - the call v23 proved on hardware, outside any frame - runs
 * at the game's FIRST depth read of an update: the GPU finished long before (pacing + game logic ran in
 * between), so the wait is about the copy itself, and frames whose update reads no depth copy nothing.
 * It is still the depth of the frame just shown, which is what the N64 gives (see swap_buffers_begin).
 * (An asynchronous in-frame copy was tried in v24/v25 together with GX_CMDLIST_FLUSH: hardware froze.) */
static void gfx_3ds_depth_lazy(void) {
    if (sInFrame || sTarget == NULL || sDepthOfFrame == sFramesDone || sFramesDone == 0) {
        return;
    }
    sDepthWantFrames = 1;
    gfx_3ds_read_back_depth();
    sDepthOfFrame = sFramesDone;
}

/* Color readback for tools/statediff (renderer ground truth vs ares' RDP framebuffer): the last two
 * finished frames, detiled into linear RGBA8, kept alternately. Only runs while a comparison build asks
 * for it (Port3ds_RequestColor), so normal play pays nothing. */
static u32* sColorLinear[2];
static int sColorWantFrames;
static int sColorLatest = -1; /* slot holding the most recently finished frame */

void Port3ds_RequestColor(void) {
    sColorWantFrames = 60;
}

int Port3ds_DrawIdActive(void) {
    return sColorWantFrames > 0;
}

int Port3ds_ColorLatestSlot(void) {
    return sColorLatest;
}

/* back = 0: most recently finished frame; back = 1: the one before */
const u32* Port3ds_GetColor(int back, int* width, int* height) {
    int slot;
    *width = ViewW();
    *height = ViewH();
    if (sColorLatest < 0) {
        return NULL;
    }
    slot = back ? (sColorLatest ^ 1) : sColorLatest;
    return sColorLinear[slot];
}

static void gfx_3ds_read_back_color(void) {
    size_t size = (size_t)ViewW() * ViewH() * 4;
    int slot;

    if (sColorWantFrames <= 0) {
        return;
    }
    sColorWantFrames--;
    slot = sFrameSlot;
    if (sColorLinear[slot] == NULL) {
        sColorLinear[slot] = linearAlloc(size);
        if (sColorLinear[slot] == NULL) {
            return;
        }
    }
    C3D_SyncDisplayTransfer((u32*)sTarget->frameBuf.colorBuf,
                            GX_BUFFER_DIM(ViewW(), ViewH()), sColorLinear[slot],
                            GX_BUFFER_DIM(ViewW(), ViewH()),
                            GX_TRANSFER_FLIP_VERT(0) | GX_TRANSFER_OUT_TILED(0) | GX_TRANSFER_RAW_COPY(0) |
                                GX_TRANSFER_IN_FORMAT(GX_TRANSFER_FMT_RGBA8) |
                                GX_TRANSFER_OUT_FORMAT(GX_TRANSFER_FMT_RGBA8) |
                                GX_TRANSFER_SCALING(GX_TRANSFER_SCALE_NO));
    GSPGPU_InvalidateDataCache(sColorLinear[slot], size);
    sColorLatest = slot;
}

/* PORT (2026-09-28): framebuffer capture for PreRender (pause-menu background, transition tiles). On the
 * N64 the RDP copies the rendered frame into fbufSave (gZBuffer memory) and the CPU/RDP read it back
 * later. Here the frame is read back after the GPU finishes it, box-filtered from the 2x2 supersamples,
 * the pillarbox dropped, and written as 320x240 RGBA5551 in the texture memory layout (u64-swizzled:
 * 16-bit pixel k at native index k ^ 3), so the game's own restore draws it like any texture. */
static void* sCaptureDst;

static void* sCaptureReq; /* requested by the game for the frame it is building (latched at hand-over) */
void Port3ds_CaptureFrame5551(void* dst) {
    sCaptureReq = dst;
}

/* game thread, when a frame is handed over: its requests, cleared for the next frame the game builds */
void Port3ds_TakeFrameRequests(void** capture, int* flat, float* focusW) {
    *capture = sCaptureReq;
    *flat = gPortStereoFlatScene;
    *focusW = gPortStereoFocusW;
    sCaptureReq = NULL;
    gPortStereoFlatScene = 0; /* the next frame's gamestate sets it again if it is a menu */
}

/* renderer, before drawing that frame */
void Port3ds_ApplyFrameRequests(void* capture, int flat, float focusW) {
    sCaptureDst = capture;
    gPortStereoFlatSceneR = flat;
    gPortStereoFocusWR = focusW;
}

/* PORT (2026-10-03): in widescreen the frame captured for the pause background is 400 pixels wide but the game's
 * buffer holds the N64's 320: the paused picture shrank to 4:3 (hardware v41). The whole width is also kept here,
 * same layout, and gfx_pc.c draws it instead when the game copies its buffer to the screen. */
static u16* sWideCap;     /* 400x240 RGBA5551, pixel k at index k ^ 3 */
static const void* sWideCapOf; /* the game buffer it stands for */

const void* Port3ds_WideCapFor(const void* gameBuf) {
    return (sWideCap != NULL && gameBuf != NULL && gameBuf == sWideCapOf) ? sWideCap : NULL;
}

/* 60 fps interpolation: no in-between frames while a capture/readback is pending for this frame */
int Port3ds_InterpBlocked(void) {
    return sCaptureReq != NULL || sCaptureDst != NULL || sColorWantFrames > 0;
}

/* buffers sized for the previous target: dropped on a stereo switch, reallocated at the new size */
void gfx_3ds_drop_readbacks(void) {
    int i;
    for (i = 0; i < 2; i++) {
        if (sDepthLinear[i] != NULL) linearFree(sDepthLinear[i]), sDepthLinear[i] = NULL;
        if (sColorLinear[i] != NULL) linearFree(sColorLinear[i]), sColorLinear[i] = NULL;
    }
    sDepthValid = false;
    sDepthSlot = -1;
    sColorLatest = -1;
}

static void gfx_3ds_capture_frame(void) {
    extern void gfx_texture_cache_invalidate_range(const void* start, uint32_t size);
    static u32* sLin;
    static size_t sLinSize;
    int W = ViewW(), H = ViewH(), sx = H / 400, sy = W / 240, x, y, ox, oy;
    size_t size = (size_t)W * H * 4;
    uint16_t* dst = (uint16_t*)sCaptureDst;

    if (dst == NULL) {
        return;
    }
    sCaptureDst = NULL;
    /* sized for the current target; regrown when anti-aliasing is switched on at run time (gfx_3ds_update_aa) */
    if (sLin == NULL || sLinSize < size) {
        if (sLin != NULL) linearFree(sLin);
        sLin = linearAlloc(size);
        sLinSize = sLin != NULL ? size : 0;
        if (sLin == NULL) return;
    }
    C3D_SyncDisplayTransfer((u32*)sTarget->frameBuf.colorBuf, GX_BUFFER_DIM(W, H), sLin, GX_BUFFER_DIM(W, H),
                            GX_TRANSFER_FLIP_VERT(0) | GX_TRANSFER_OUT_TILED(0) | GX_TRANSFER_RAW_COPY(0) |
                                GX_TRANSFER_IN_FORMAT(GX_TRANSFER_FMT_RGBA8) |
                                GX_TRANSFER_OUT_FORMAT(GX_TRANSFER_FMT_RGBA8) | GX_TRANSFER_SCALING(GX_TRANSFER_SCALE_NO));
    GSPGPU_InvalidateDataCache(sLin, size);
    for (y = 0; y < 240; y++) {
        for (x = 0; x < 320; x++) {
            unsigned r = 0, g = 0, b = 0, n = (unsigned)(sx * sy);
            for (ox = 0; ox < sx; ox++) {
                for (oy = 0; oy < sy; oy++) {
                    u32 w = sLin[((x + 40) * sx + ox) * W + (W - 1 - (y * sy + oy))]; /* 0xRRGGBBAA */
                    r += (w >> 24) & 0xFF;
                    g += (w >> 16) & 0xFF;
                    b += (w >> 8) & 0xFF;
                }
            }
            r /= n;
            g /= n;
            b /= n;
            dst[(y * 320 + x) ^ 3] = (uint16_t)(((r >> 3) << 11) | ((g >> 3) << 6) | ((b >> 3) << 1) | 1);
        }
    }
    gfx_texture_cache_invalidate_range(dst, 320 * 240 * 2);
    sWideCapOf = NULL;
    {
        extern int gPortFrameWide; /* gfx_pc.c: this frame's 3D filled the whole width */
        if (gPortFrameWide && (sWideCap != NULL || (sWideCap = malloc(400 * 240 * 2)) != NULL)) {
            for (y = 0; y < 240; y++) {
                for (x = 0; x < 400; x++) {
                    unsigned r = 0, g = 0, b = 0, n = (unsigned)(sx * sy);
                    for (ox = 0; ox < sx; ox++) {
                        for (oy = 0; oy < sy; oy++) {
                            u32 w = sLin[(x * sx + ox) * W + (W - 1 - (y * sy + oy))];
                            r += (w >> 24) & 0xFF;
                            g += (w >> 16) & 0xFF;
                            b += (w >> 8) & 0xFF;
                        }
                    }
                    r /= n, g /= n, b /= n;
                    sWideCap[(y * 400 + x) ^ 3] = (uint16_t)(((r >> 3) << 11) | ((g >> 3) << 6) | ((b >> 3) << 1) | 1);
                }
            }
            gfx_texture_cache_invalidate_range(sWideCap, 400 * 240 * 2);
            sWideCapOf = dst;
        }
    }
}

/* PORT (2026-09-28): off-screen color images (the pause menu's Link preview, Player_DrawPause, and any other
 * gDPSetColorImage to a non-framebuffer address). Draws go to a 1x 320x240-space render target (portrait,
 * like the screen, no pillarbox); after the frame the drawn region (0..width x 0..height, from the
 * scissor) is read back into the game's buffer as RGBA5551 in texture layout, so the game draws or
 * processes it like the N64's RDP output - one frame later than on the N64. */
#define OFFSCREEN_MAX 4
static struct {
    void* addr;
    C3D_RenderTarget* rt;
    int width, height; /* image size in pixels written back */
    int used;          /* drawn this frame */
} sOff[OFFSCREEN_MAX];
static int sOffCur = -1; /* current target, -1 = screen */

int Port3ds_IsOffscreen(void) {
    return sOffCur >= 0;
}

/* gfx_citro3d.c, at the start of each replayed frame: the viewport C3D_FrameDrawOn gives a frame */
void Port3ds_ResetFrameViewport(void) {
    if (sOffCur < 0 && sTarget != NULL) {
        /* (a scissor rectangle first: C3D_SetViewport turns the scissor off but keeps its last rectangle, which would
         * differ with whatever frame came before) */
        C3D_SetScissor(GPU_SCISSOR_NORMAL, 0, 0, sTarget->frameBuf.width, sTarget->frameBuf.height);
        C3D_SetViewport(0, 0, sTarget->frameBuf.width, sTarget->frameBuf.height);
    }
}

/* the frame buffer draws go to (gfx_citro3d.c replay by copy: a captured frame only replays onto the same one) */
const void* Port3ds_DrawTargetId(void) {
    if (sOffCur >= 0) {
        return NULL;
    }
    return sTarget != NULL ? sTarget->frameBuf.colorBuf : NULL;
}

void Port3ds_SetDrawTarget(void* addr, int width, int height) {
    int i, slot = -1;
    if (addr == NULL) {
        if (sOffCur >= 0) {
            C3D_FrameDrawOn(sTarget);
            sOffCur = -1;
        }
        return;
    }
    for (i = 0; i < OFFSCREEN_MAX; i++) {
        if (sOff[i].addr == addr) {
            slot = i;
            break;
        }
    }
    if (slot < 0) {
        for (i = 0; i < OFFSCREEN_MAX && slot < 0; i++) {
            if (sOff[i].addr == NULL || !sOff[i].used) {
                slot = i;
            }
        }
        if (slot < 0) {
            return;
        }
        if (sOff[slot].rt == NULL) {
            sOff[slot].rt = C3D_RenderTargetCreate(240, 320, GPU_RB_RGBA8, GPU_RB_DEPTH24_STENCIL8);
            if (sOff[slot].rt == NULL) {
                return;
            }
        }
        sOff[slot].addr = addr;
        sOff[slot].used = 0;
    }
    if (width > 0) {
        sOff[slot].width = width > 320 ? 320 : width;
    }
    if (height > sOff[slot].height) {
        sOff[slot].height = height > 240 ? 240 : height;
    }
    if (!sOff[slot].used) {
        C3D_RenderTargetClear(sOff[slot].rt, C3D_CLEAR_ALL, 0x000000FF, 0xFFFFFFFF);
        sOff[slot].used = 1;
    }
    C3D_FrameDrawOn(sOff[slot].rt);
    sOffCur = slot;
}

static void gfx_3ds_read_back_offscreen(void) {
    extern void gfx_texture_cache_invalidate_range(const void* start, uint32_t size);
    static u32* sLin;
    int i, x, y;
    const int W = 240, H = 320;
    if (sLin == NULL && (sLin = linearAlloc(W * H * 4)) == NULL) {
        return;
    }
    for (i = 0; i < OFFSCREEN_MAX; i++) {
        uint16_t* dst = (uint16_t*)sOff[i].addr;
        int w = sOff[i].width, h = sOff[i].height;
        if (!sOff[i].used || dst == NULL || w <= 0 || h <= 0) {
            continue;
        }
        sOff[i].used = 0;
        C3D_SyncDisplayTransfer((u32*)sOff[i].rt->frameBuf.colorBuf, GX_BUFFER_DIM(W, H), sLin, GX_BUFFER_DIM(W, H),
                                GX_TRANSFER_FLIP_VERT(0) | GX_TRANSFER_OUT_TILED(0) | GX_TRANSFER_RAW_COPY(0) |
                                    GX_TRANSFER_IN_FORMAT(GX_TRANSFER_FMT_RGBA8) |
                                    GX_TRANSFER_OUT_FORMAT(GX_TRANSFER_FMT_RGBA8) |
                                    GX_TRANSFER_SCALING(GX_TRANSFER_SCALE_NO));
        GSPGPU_InvalidateDataCache(sLin, W * H * 4);
        for (y = 0; y < h; y++) {
            for (x = 0; x < w; x++) {
                u32 p = sLin[x * W + (W - 1 - y)]; /* 0xRRGGBBAA */
                dst[(y * w + x) ^ 3] = (uint16_t)(((((p >> 24) & 0xFF) >> 3) << 11) | ((((p >> 16) & 0xFF) >> 3) << 6) |
                                                  ((((p >> 8) & 0xFF) >> 3) << 1) | 1);
            }
        }
        gfx_texture_cache_invalidate_range(dst, (uint32_t)(w * h * 2));
    }
}

/* verification aid: with sdmc:/3ds/oot/capture_stereo present, every 300 frames while in stereo the
 * whole 240x800 target (both eyes, detiled RGBA8) goes to sdmc:/3ds/oot/stereo_fb_<n>.bin (header w, h) */
static void gfx_3ds_debug_dump_stereo(void) {
    static int sFrames;
    static u32* sLin;
    FILE* f;
    int W, H;
    if (gGfx3DSMode != GFX_3DS_MODE_STEREO || (++sFrames % 300) != 0) {
        return;
    }
    f = fopen("sdmc:/3ds/oot/capture_stereo", "rb");
    if (f == NULL) {
        return;
    }
    fclose(f);
    W = sTarget->frameBuf.width, H = sTarget->frameBuf.height;
    if (sLin != NULL && linearGetSize(sLin) < (u32)W * H * 4) { /* the target grew (3D off, AA on) */
        linearFree(sLin);
        sLin = NULL;
    }
    if (sLin == NULL && (sLin = linearAlloc((size_t)W * H * 4)) == NULL) {
        return;
    }
    C3D_SyncDisplayTransfer((u32*)sTarget->frameBuf.colorBuf, GX_BUFFER_DIM(W, H), sLin, GX_BUFFER_DIM(W, H),
                            GX_TRANSFER_FLIP_VERT(0) | GX_TRANSFER_OUT_TILED(0) | GX_TRANSFER_RAW_COPY(0) |
                                GX_TRANSFER_IN_FORMAT(GX_TRANSFER_FMT_RGBA8) |
                                GX_TRANSFER_OUT_FORMAT(GX_TRANSFER_FMT_RGBA8) | GX_TRANSFER_SCALING(GX_TRANSFER_SCALE_NO));
    GSPGPU_InvalidateDataCache(sLin, (size_t)W * H * 4);
    {
        char path[64];
        snprintf(path, sizeof path, "sdmc:/3ds/oot/stereo_fb_%d.bin", sFrames / 300);
        f = fopen(path, "wb");
    }
    if (f != NULL) {
        u32 hdr[2] = { (u32)W, (u32)H };
        fwrite(hdr, 4, 2, f);
        fwrite(sLin, 4, (size_t)W * H, f);
        fclose(f);
    }
}

/* verification aid for the 60 fps interpolation: with sdmc:/3ds/oot/capture_interp present, every 200th
 * logic frame's passes (in-between ones, then the exact one) go to sdmc:/3ds/oot/interp_fb_<n>_<pass>.bin */
static int sInterpDumpOn = -1; /* sdmc:/3ds/oot/capture_interp exists (checked once) */
int gPortInterpDumpSpan = 2;   /* settings interp_dump_span: consecutive logic frames dumped per 200 */
int gPortInterpDumpAt = -1;    /* settings interp_dump_at: dump logic frames at..at+span-1 instead (once) */
static bool interp_dump_on(void) {
    if (sInterpDumpOn < 0) {
        FILE* f = fopen("sdmc:/3ds/oot/capture_interp", "rb");
        sInterpDumpOn = f != NULL;
        if (f != NULL) fclose(f);
    }
    return sInterpDumpOn != 0;
}
static void gfx_3ds_debug_dump_interp(void) {
    extern int gPortInterpExtra;
    static int sLogic, sPass;
    static u32* sLin;
    FILE* f;
    int W, H;
    if (!interp_dump_on()) {
        return;
    }
    if (gPortInterpDumpAt >= 0 ? (sLogic >= gPortInterpDumpAt && sLogic < gPortInterpDumpAt + gPortInterpDumpSpan)
                               : (sLogic % 200) >= 200 - gPortInterpDumpSpan) {
        W = sTarget->frameBuf.width, H = sTarget->frameBuf.height;
        if (sLin != NULL && linearGetSize(sLin) < (u32)W * H * 4 * 2) { /* the target grew */
            linearFree(sLin);
            sLin = NULL;
        }
        if (sLin == NULL) sLin = linearAlloc((size_t)W * H * 4 * 2);
        if (sLin != NULL) {
            char path[64];
            C3D_SyncDisplayTransfer((u32*)sTarget->frameBuf.colorBuf, GX_BUFFER_DIM(W, H), sLin, GX_BUFFER_DIM(W, H),
                                    GX_TRANSFER_FLIP_VERT(0) | GX_TRANSFER_OUT_TILED(0) | GX_TRANSFER_RAW_COPY(0) |
                                        GX_TRANSFER_IN_FORMAT(GX_TRANSFER_FMT_RGBA8) |
                                        GX_TRANSFER_OUT_FORMAT(GX_TRANSFER_FMT_RGBA8) |
                                        GX_TRANSFER_SCALING(GX_TRANSFER_SCALE_NO));
            GSPGPU_InvalidateDataCache(sLin, (size_t)W * H * 4);
            snprintf(path, sizeof path, "sdmc:/3ds/oot/interp_fb_%d_%d.bin", sLogic, sPass);
            f = fopen(path, "wb");
            if (f != NULL) {
                u32 hdr[2] = { (u32)W, (u32)H };
                fwrite(hdr, 4, 2, f);
                fwrite(sLin, 4, (size_t)W * H, f);
                fclose(f);
            }
        }
    }
    sPass++;
    if (!gPortInterpExtra) {
        sLogic++;
        sPass = 0;
    }
}

/* PORT PERF (2026-10-02): presentation timing. citro3d swaps a frame's screen buffers the moment the GPU
 * finishes it (renderqueue.c onQueueFinish -> gfxScreenSwapBuffers) and the LCD takes the new buffer at the
 * next vblank. When two frames finish within one refresh, the first is never shown and the second's display
 * transfer writes into the buffer being scanned out. With 60 fps replays on a New 3DS (hardware v34) the
 * logic frame finished ~27 ms into its update, past the first in-between frame's refresh, and the
 * in-between frames followed back to back; "frames shown/s" counted frames submitted, not displayed.
 * Present gate (settings present_gate, default on): a frame is handed to the GPU only after the vblank at
 * which the previous one becomes visible, so every frame gets its own refresh. The previous frame's finish
 * comes from citro3d's GPU timer (C3D_GetDrawingTime, complete once its queue is done, i.e. after the next
 * C3D_FrameBegin) and vblank times from a 59.831 Hz clock anchored at the last gspWaitForVBlank
 * (3ds_main.c Port3ds_VBlankSeen). The same model counts frames replaced before any vblank showed them. */
#define VB_TICKS ((double)SYSCLOCK_ARM11 / 59.831)
#define MS_TICKS ((double)SYSCLOCK_ARM11 / 1000.0)
static volatile u64 sVbAnchor; /* tick of a recent top-screen vblank (written by the game thread) */
static u64 sSubmitTick; /* the previous frame: when C3D_FrameEnd handed it to the GPU (0 = accounted for) */
static u64 sShownVb;    /* the previous frame: the vblank at which it becomes visible (0 = unknown) */
int gPortPresentGate = 1;
unsigned gPortPerfReplaced; /* frames replaced by the next one before any vblank showed them */
unsigned gPortPerfPresented; /* frames whose display vblank was estimated */
u64 gPortPerfGateWait;      /* ticks spent in the gate (per report) */
u64 gPortGateTicksTotal;    /* the same, never reset: 3ds_main.c keeps it out of its frame-cost averages */
float gPortGpuDrawMsAvg;    /* GPU time per frame, running average (3ds_main.c: the logic frame's deadline) */
unsigned gPortPerfStereoFrames; /* frames drawn with the 3D slider up (each draw is issued for both eyes) */

void Port3ds_VBlankSeen(void) {
    sVbAnchor = svcGetSystemTick();
}

/* 3ds_main.c pacing: the latest vblank at or before tMs (milliseconds of svcGetSystemTick); tMs itself
 * without an anchor */
double Port3ds_VBlankAtOrBefore(double tMs) {
    u64 anchor, again;
    double a, p = VB_TICKS / MS_TICKS;
    do { /* written by the game thread, read by the render thread: a consistent 64-bit copy */
        anchor = sVbAnchor;
        again = sVbAnchor;
    } while (anchor != again);
    if (anchor == 0) {
        return tMs;
    }
    a = (double)anchor / MS_TICKS;
    return a + floor((tMs - a) / p) * p;
}

/* the first vblank after tick t (0 without an anchor) */
static u64 vblank_after(u64 t) {
    double n;
    if (sVbAnchor == 0) {
        return 0;
    }
    n = ceil((double)(s64)(t - sVbAnchor) / VB_TICKS);
    return sVbAnchor + (u64)(s64)(n * VB_TICKS);
}

/* 3ds_main.c: when (milliseconds of svcGetSystemTick) the frame just submitted should reach the screen - the first
 * vblank after its predicted GPU finish (average GPU time); 0 when unknown */
double Port3ds_PredictShownMs(void) {
    extern float gPortGpuDrawMsAvg;
    u64 vb;
    if (sSubmitTick == 0 || sVbAnchor == 0) {
        return 0.0;
    }
    vb = vblank_after(sSubmitTick + (u64)((gPortGpuDrawMsAvg + 0.5f) * MS_TICKS));
    return (double)vb / MS_TICKS;
}

/* ---- PORT PERF (2026-10-03): flip presenter ----
 * citro3d copies a finished frame to the LCD's buffer and swaps when the GPU ends it, so a frame reaches the
 * screen as soon as it is done: two frames finishing within one refresh lost one, and the 60 fps replays had to
 * wait for each refresh before being drawn (the present gate), holding up the game thread (~15 ms per update,
 * hardware v41). Here every frame is copied by the GPU into its own buffer of a ring (same queue, right after
 * the frame), the GPU then stamps the frame's number into a word of linear memory, and a small thread points the LCD at the
 * newest finished frame due at each vblank (gspPresentBuffer, latched by GSP at the vblank). The game thread
 * renders an update's frames back to back and never waits for a refresh; each frame carries the vblank it is due
 * at (3ds_main.c: the update's frames on its 4th, 5th and 6th vblank). A late frame repeats the previous one for a
 * refresh; nothing tears. settings flip=0: citro3d's own output, as before. */
#define FLIP_N 8
#define FLIP_BYTES (240 * 800 * 3) /* one 800x240 BGR8 frame, or two 400x240 eyes */
int gPortFlip = 1;
static int sFlipOn;                      /* the ring and the thread exist */
static u8* sFlipBuf[FLIP_N];
/* PORT (2026-10-03): the stamp lives in LINEAR memory, written by the GPU's copy engine (GX_TextureCopy from a per-buffer
 * source word). It was a VRAM word filled by GX_MemoryFill and read by the CPU: the CPU may not touch VRAM on a real
 * 3DS - v46 (the first hardware run of the presenter) died at boot with a data abort, "Permission - Section", on the
 * write that zeroed it. Azahar allows it. */
static volatile u32* sFlipStamp;         /* the last frame the GPU finished (16 bytes, linear) */
static u32* sFlipStampSrc;               /* per buffer: 16 bytes holding its frame id, copied to sFlipStamp */
static u64 sFlipSlot[FLIP_N];            /* per buffer: the vblank tick it is due at */
static u8 sFlipMode[FLIP_N];             /* 0 2D (400 wide), 1 800 wide, 2 stereo */
static volatile u32 sFlipQueued;         /* the last frame id handed to the GPU */
static volatile u32 sFlipShown;          /* the frame id on screen (latched) */
static volatile int sFlipRun;
static u64 sFlipNextSlot;                /* 3ds_main.c: the vblank the next frame is due at (0 = as soon as done) */
unsigned gPortPerfFlipShown, gPortPerfFlipSkipped, gPortPerfFlipRepeats;
static Thread sFlipThread;

/* debug (settings flipdump=1): the buffer on screen, once per perf report, to sdmc:/3ds/oot/flip_shown.bin */
int gPortFlipDump;
void Port3ds_FlipDump(void) {
    FILE* f;
    u32 id = sFlipShown, hdr[3];
    if (!sFlipOn || !gPortFlipDump || id == 0 || (f = fopen("sdmc:/3ds/oot/flip_shown.bin", "wb")) == NULL) {
        return;
    }
    hdr[0] = id, hdr[1] = sFlipMode[id % FLIP_N], hdr[2] = FLIP_BYTES;
    fwrite(hdr, 4, 3, f);
    GSPGPU_InvalidateDataCache(sFlipBuf[id % FLIP_N], FLIP_BYTES);
    fwrite(sFlipBuf[id % FLIP_N], 1, FLIP_BYTES, f);
    fclose(f);
}

void Port3ds_SetFrameSlot(double slotMs) {
    sFlipNextSlot = slotMs > 0.0 ? (u64)(slotMs * MS_TICKS) : 0;
}
int Port3ds_FlipActive(void) {
    return sFlipOn;
}
/* a consistent copy of the vblank anchor (written by the game thread) */
static u64 flip_anchor(void) {
    u64 a, b;
    do {
        a = sVbAnchor;
        b = sVbAnchor;
    } while (a != b);
    return a;
}

static void flip_thread(void* arg) {
    u32 pending = 0, presented = 0;
    u64 pendingVb = 0;
    int swap = 0;
    (void)arg;
    while (sFlipRun) {
        u64 anchor = flip_anchor(), now = svcGetSystemTick(), vb, wake;
        double n;
        if (anchor == 0) {
            svcSleepThread(2000000LL);
            continue;
        }
        /* the next vblank at least 1 ms away: present 1.5 ms before it */
        n = ceil((double)(s64)(now + (u64)(1.0 * MS_TICKS) - anchor) / VB_TICKS);
        vb = anchor + (u64)(s64)(n * VB_TICKS);
        wake = vb - (u64)(1.5 * MS_TICKS);
        if (now < wake) {
            svcSleepThread((s64)((double)(wake - now) * (1e9 / SYSCLOCK_ARM11)));
        }
        if (pending != 0 && svcGetSystemTick() > pendingVb) { /* the previous present latched at its vblank */
            sFlipShown = pending;
            pending = 0;
        }
        {
            u32 done, last, top, id, chosen = 0;
            Port3ds_CacheInvalidate((void*)sFlipStamp, 16); /* written by the GPU */
            done = *sFlipStamp, last = sFlipQueued, top = done < last ? done : last;
            for (id = top; id > presented && id + FLIP_N > last; id--) {
                if (sFlipSlot[id % FLIP_N] <= vb + (u64)(0.5 * MS_TICKS)) {
                    chosen = id;
                    break;
                }
            }
            if (chosen != 0) {
                int b = chosen % FLIP_N;
                const u8* fa = sFlipBuf[b];
                const u8* fb = sFlipMode[b] == 2 ? fa + FLIP_BYTES / 2 : fa;
                u32 mode = GSP_BGR8_OES | (sFlipMode[b] == 2 ? BIT(5) : sFlipMode[b] == 0 ? BIT(6) : 0) | (1 << 8);
                if (gspHasGpuRight()) {
                    gspPresentBuffer(GSP_SCREEN_TOP, swap, fa, fb, 240 * 3, mode);
                    swap ^= 1;
                }
                gPortPerfFlipShown++;
                gPortPerfFlipSkipped += chosen - presented - 1 < FLIP_N ? chosen - presented - 1 : 0;
                presented = chosen;
                pending = chosen;
                pendingVb = vb;
            } else if (presented != 0) {
                gPortPerfFlipRepeats++; /* nothing new due: the screen keeps its frame for this refresh */
            }
        }
        /* past this vblank before planning the next one */
        now = svcGetSystemTick();
        if (now < vb + (u64)(0.5 * MS_TICKS)) {
            svcSleepThread((s64)((double)(vb + (u64)(0.5 * MS_TICKS) - now) * (1e9 / SYSCLOCK_ARM11)));
        }
    }
}

static void flip_init(void) {
    int i;
    s32 prio = 0x30;
    if (!gPortFlip) {
        return;
    }
    for (i = 0; i < FLIP_N; i++) {
        sFlipBuf[i] = linearAlloc(FLIP_BYTES);
        if (sFlipBuf[i] == NULL) {
            break;
        }
        memset(sFlipBuf[i], 0, FLIP_BYTES);
        GSPGPU_FlushDataCache(sFlipBuf[i], FLIP_BYTES);
    }
    sFlipStamp = (volatile u32*)linearMemAlign(64, 64);
    sFlipStampSrc = (u32*)linearMemAlign(FLIP_N * 16, 64);
    if (i < FLIP_N || sFlipStamp == NULL || sFlipStampSrc == NULL) {
        extern void PortDbg(const char*);
        PortDbg("[gfx] flip presenter: no memory, citro3d output");
        return;
    }
    memset((void*)sFlipStamp, 0, 64);
    memset(sFlipStampSrc, 0, FLIP_N * 16);
    GSPGPU_FlushDataCache((void*)sFlipStamp, 64);
    GSPGPU_FlushDataCache(sFlipStampSrc, FLIP_N * 16);
    svcGetThreadPriority(&prio, CUR_THREAD_HANDLE);
    sFlipRun = 1;
    /* core 0 always: with the Old 3DS layout (3ds_main.c) graphics start on the game thread on core 1, whose time is
     * capped by the system; the presenter must wake on time for every vblank */
    sFlipThread = threadCreate(flip_thread, NULL, 8 * 1024, prio - 2, 0, false);
    if (sFlipThread == NULL) {
        sFlipRun = 0;
        return;
    }
    sFlipOn = 1;
    { extern void PortDbg(const char*); PortDbg("[gfx] flip presenter on"); }
}

static void flip_stop(void) {
    if (sFlipThread != NULL) {
        sFlipRun = 0;
        threadJoin(sFlipThread, 100000000ULL); /* at most one refresh away from noticing */
        threadFree(sFlipThread);
        sFlipThread = NULL;
    }
    sFlipOn = 0;
}

/* swap_buffers_begin, inside the frame: copy it into the next ring buffer after its commands, then stamp it */
static void flip_submit(u8 splitFlags) {
    u32 id = sFlipQueued + 1;
    int b = id % FLIP_N, e;
    {
        PHASE_IN(30); /* the presenter ring is full */
        while (sFlipShown + FLIP_N <= id && sFlipRun) { /* that buffer may still be on screen: wait for a flip */
            svcSleepThread(1000000LL);
        }
        PHASE_OUT();
    }
    C3D_FrameSplit(splitFlags); /* the frame's commands go into the queue first */
    if (gGfx3DSMode == GFX_3DS_MODE_STEREO) {
        u32 flags = GX_TRANSFER_FLIP_VERT(0) | GX_TRANSFER_OUT_TILED(0) | GX_TRANSFER_RAW_COPY(0) |
                    GX_TRANSFER_IN_FORMAT(GX_TRANSFER_FMT_RGBA8) | GX_TRANSFER_OUT_FORMAT(GX_TRANSFER_FMT_RGB8) |
                    GX_TRANSFER_SCALING(GX_TRANSFER_SCALE_NO);
        for (e = 0; e < 2; e++) { /* eye e: half e of the stereo target, to the left / right framebuffer */
            GX_DisplayTransfer((u32*)((u8*)sTarget->frameBuf.colorBuf + e * 240 * STEREO_EYE_OFFSET * 4),
                               GX_BUFFER_DIM(240, STEREO_EYE_OFFSET), (u32*)(sFlipBuf[b] + e * (FLIP_BYTES / 2)),
                               GX_BUFFER_DIM(240, STEREO_EYE_OFFSET), flags);
        }
        sFlipMode[b] = 2;
    } else {
        GX_DisplayTransfer((u32*)sTarget->frameBuf.colorBuf, GX_BUFFER_DIM(sTarget->frameBuf.width, sTarget->frameBuf.height),
                           (u32*)sFlipBuf[b], GX_BUFFER_DIM(sTarget->frameBuf.width, sTarget->frameBuf.height),
                           sMonoFlags);
        sFlipMode[b] = sMonoWide ? 1 : 0;
    }
    sFlipStampSrc[b * 4] = id;
    Port3ds_CacheFlush(&sFlipStampSrc[b * 4], 16);
    GX_TextureCopy(&sFlipStampSrc[b * 4], 0, (u32*)sFlipStamp, 0, 16, 8); /* after the frame's copy: it is done */
    sFlipSlot[b] = sFlipNextSlot;
    sFlipNextSlot = 0;
    __sync_synchronize();
    sFlipQueued = id;
    sTarget->used = false; /* citro3d's own copy and swap: not for this frame */
    if (sEyeOut[0] != NULL) sEyeOut[0]->used = sEyeOut[1]->used = false;
}

/* right after C3D_FrameBegin: the previous frame's queue is done, its GPU time known */
static void present_note_finished(void) {
    u64 done, vb;
    if (sSubmitTick == 0) {
        return;
    }
    /* + 0.5 ms: the swap request has to reach the GSP before the vblank to be taken there */
    done = sSubmitTick + (u64)(C3D_GetDrawingTime() * MS_TICKS);
    vb = vblank_after(done + (u64)(0.5 * MS_TICKS));
    if (vb != 0 && vb == sShownVb) {
        gPortPerfReplaced++; /* the frame before it never reached the screen */
    }
    gPortPerfPresented++;
    gPortGpuDrawMsAvg += (C3D_GetDrawingTime() - gPortGpuDrawMsAvg) * 0.1f;
    sShownVb = vb;
    sSubmitTick = 0;
}

/* right before C3D_FrameEnd: hold the frame until the previous one is on screen */
static void present_gate(void) {
    extern volatile unsigned char gPortProf;
    extern void Port3ds_MaybePumpAudio(void);
    u64 now, until;
    if (!gPortPresentGate || sShownVb == 0) {
        return;
    }
    until = sShownVb + (u64)(0.3 * MS_TICKS);
    now = svcGetSystemTick();
    if (now < until && until - now < (u64)(2.0 * VB_TICKS)) {
        unsigned char prev = gPortProf;
        gPortProf = 11; /* PROF_PACE (port_prof.h) */
        svcSleepThread((s64)((double)(until - now) * (1e9 / SYSCLOCK_ARM11)));
        gPortPerfGateWait += svcGetSystemTick() - now;
        gPortGateTicksTotal += svcGetSystemTick() - now;
        Port3ds_MaybePumpAudio(); /* one audio task per retrace, also across this wait */
        gPortProf = prev;
    }
}

/* PORT PERF (2026-10-03): CPU/GPU overlap within an update. Each frame used to be a citro3d frame of its own, and
 * C3D_FrameBegin waits until the GPU has finished the previous one (citro3d reuses one command buffer and clears
 * the GX queue there), so the CPU building an in-between frame and the GPU drawing the frame before took turns:
 * logic + walk + 3 x GPU + 2 x replay CPU per update (3D on hardware: ~6 + 19 + 3 x 8 + 2 x 6 = 61 of 50 ms).
 * With the GPU vertex path an in-between frame only re-issues the walk's draws with other matrices (uniforms in
 * the command stream; the vertex and index buffers are not rewritten), so the update's frames now share one
 * citro3d frame: after each frame its commands are split off with GX_CMDLIST_FLUSH (the GSP flushes them from the
 * CPU cache itself: unflushed splits froze hardware in v24/v25) and the GX queue is started at once, and the next
 * frame's commands are built while the GPU draws. The frame closes (C3D_FrameEnd) with the update's last frame;
 * the next update's walk still starts with C3D_FrameBegin's wait, so textures and the vertex buffer are never
 * rewritten under the GPU. Inside a citro3d frame C3D_SyncDisplayTransfer is NOT synchronous (it splits without
 * the flush and queues), so the early depth copy is queued here too and waited for at the game's read. Settings
 * overlap=0 disables it; overlap_ab=1 alternates it every 2 perf reports (3ds_main.c). */
int gPortOverlap = 1;
int gPortFramesFollow;          /* 3ds_main.c: frames of this update still to come after the one being drawn */
u32 gPortPerfOverlapFrames;     /* perf: frames that left the citro3d frame open for the next one */
float gPortPerfCmdBufMax;       /* perf: peak C3D_GetCmdBufUsage() */
static bool sFrameOpen;         /* the citro3d frame continues into the next frame of this update */
static bool sFrameOpenUpdate;   /* this update's citro3d frame was kept open before (the GX queue is live) */
static int sDepthAsyncIdx = -1; /* GX queue entry of a queued depth copy not yet waited for, -1 = none */

static inline volatile gxCmdQueue_s* c3d_queue(void) {
    extern unsigned char __C3D_Context[]; /* citro3d's context: its GX queue is the first member (internal.h) */
    return (volatile gxCmdQueue_s*)(void*)__C3D_Context;
}

typedef struct {
    int idx, slot, w, h; /* GX queue entry not yet waited for (-1: done), readback slot (-1: none), size */
} DepthPub;
static DepthPub sDepthPub = { -1, -1, 0, 0 };
static LightLock sDepthLock = 1; /* libctru: 1 = unlocked (LightLock_Init) */
u64 gPortPerfDepthWait; /* perf: ticks the game waited for a queued depth copy */
u32 gPortPerfDepthAsync; /* perf: queued depth copies read */

/* the early depth copy, queued behind the frame just split off (overlap: the next frame's clear comes after it) */
static void depth_copy_async(void) {
    size_t size = (size_t)ViewW() * ViewH() * 4;
    if (sDepthLinear[sFrameSlot] == NULL && (sDepthLinear[sFrameSlot] = linearAlloc(size)) == NULL) {
        return;
    }
    GX_DisplayTransfer((u32*)sTarget->frameBuf.depthBuf, GX_BUFFER_DIM(ViewW(), ViewH()), sDepthLinear[sFrameSlot],
                       GX_BUFFER_DIM(ViewW(), ViewH()),
                       GX_TRANSFER_FLIP_VERT(0) | GX_TRANSFER_OUT_TILED(0) | GX_TRANSFER_RAW_COPY(0) |
                           GX_TRANSFER_IN_FORMAT(GX_TRANSFER_FMT_RGBA8) | GX_TRANSFER_OUT_FORMAT(GX_TRANSFER_FMT_RGBA8) |
                           GX_TRANSFER_SCALING(GX_TRANSFER_SCALE_NO));
    sDepthAsyncIdx = c3d_queue()->numEntries - 1;
    sDepthSlot = sFrameSlot;
    sDepthValid = true;
    LightLock_Lock(&sDepthLock);
    sDepthPub.idx = sDepthAsyncIdx, sDepthPub.slot = sDepthSlot, sDepthPub.w = ViewW(), sDepthPub.h = ViewH();
    LightLock_Unlock(&sDepthLock);
}

/* render thread mode (gPortRenderThreaded): the game thread reads only copies the renderer queued and published */
static const u32* depth_read_threaded(int* width, int* height) {
    volatile gxCmdQueue_s* q = c3d_queue();
    DepthPub p;
    LightLock_Lock(&sDepthLock);
    p = sDepthPub;
    LightLock_Unlock(&sDepthLock);
    sDepthReadAt = sFramesDone;
    if (p.slot < 0 || sDepthLinear[p.slot] == NULL) {
        return NULL;
    }
    if (p.idx >= 0) { /* not read since it was queued: wait for it, then drop the CPU's stale cache lines */
        u64 t0 = svcGetSystemTick();
        while (q->numEntries > p.idx && q->lastEntry <= p.idx) {
            svcSleepThread(100000LL);
        }
        GSPGPU_InvalidateDataCache(sDepthLinear[p.slot], (size_t)p.w * p.h * 4);
        gPortPerfDepthWait += svcGetSystemTick() - t0;
        gPortPerfDepthAsync++;
        LightLock_Lock(&sDepthLock);
        if (sDepthPub.idx == p.idx && sDepthPub.slot == p.slot) {
            sDepthPub.idx = -1;
        }
        LightLock_Unlock(&sDepthLock);
    }
    *width = p.w;
    *height = p.h;
    return sDepthLinear[p.slot];
}

/* before the CPU reads a queued depth copy: wait for its queue entry (a cleared queue had finished it) */
static void depth_async_wait(void) {
    volatile gxCmdQueue_s* q = c3d_queue();
    u64 t0;
    if (sDepthAsyncIdx < 0) {
        return;
    }
    t0 = svcGetSystemTick();
    while (q->numEntries > sDepthAsyncIdx && q->lastEntry <= sDepthAsyncIdx) {
        svcSleepThread(100000LL);
    }
    gPortPerfDepthWait += svcGetSystemTick() - t0;
    gPortPerfDepthAsync++;
    if (sDepthSlot >= 0 && sDepthLinear[sDepthSlot] != NULL) {
        GSPGPU_InvalidateDataCache(sDepthLinear[sDepthSlot], (size_t)ViewW() * ViewH() * 4);
    }
    sDepthAsyncIdx = -1;
}

/* 3ds_main.c, after an update's last frame: end a citro3d frame left open (frames planned but not drawn) */
void Port3ds_EndUpdateFrames(void) {
    if (sFrameOpen) {
        sFrameOpen = false;
        C3D_FrameEnd(GX_CMDLIST_FLUSH);
        sInFrame = false;
    }
    sFrameOpenUpdate = false;
}

u64 gPortPerfGpuWait; /* ticks in C3D_FrameBegin: waiting for the previous frame's GPU work */
/* PORT PERF (2026-10-02): hardware v34 - anti-aliasing off (half the pixels) did not change the 11-14 ms of
 * GPU wait per update, so the GPU is not fill-bound. Measured per frame: time inside C3D_FrameEnd (it cleans
 * the CPU data cache over the whole linear heap when GX_CMDLIST_FLUSH is not passed), and citro3d's timers
 * processing / drawing timers. */
u64 gPortPerfFrameEnd;
int gPortCmdlistFlush;
float gPortPerfGpuProcMs, gPortPerfGpuDrawMs;
unsigned gPortPerfGpuFrames;

static bool gfx_3ds_start_frame(void)
{
    u64 t0;
    if (sFrameOpen) {
        /* overlap: the next frame of the same update, in the same citro3d frame (no wait for the GPU). The clear is
         * a GX memory fill queued behind the previous frame's copies. */
        sFrameOpen = false;
        sInFrame = true;
        C3D_RenderTargetClear(sTarget, C3D_CLEAR_ALL, 0x000000FF, 0xFFFFFFFF);
        C3D_FrameDrawOn(sTarget);
        return true;
    }
    if (sDepthEarlyPending) {
        sDepthEarlyPending = false;
        if (sFramesDone - sDepthReadAt <= 1 && sColorWantFrames == 0 && sTarget != NULL) {
            sDepthWantFrames = 1;
            gfx_3ds_read_back_depth(); /* waits for that frame (as C3D_FrameBegin would), then copies */
            sDepthOfFrame = sFramesDone + 1; /* stands for the logic frame ending in this update */
        }
    }
    {
        PHASE_IN(32); /* 3D on/off: render targets deleted and created */
        gfx_3ds_update_stereo(); /* outside a frame: citro3d refuses to delete targets inside one */
        PHASE_OUT();
    }
    gfx_3ds_update_aa();
    t0 = svcGetSystemTick();
    {
        extern volatile unsigned char gPortProf;
        unsigned char prev = gPortProf;
        gPortProf = 13; /* PROF_GPUWAIT (port_prof.h) */
        /* PORT PERF (2026-09-30): no C3D_FRAME_SYNCDRAW. That flag runs C3D_FrameSync, which waits for the
         * NEXT vblank on both screens (citro3d renderqueue.c) - on top of Port3ds_PaceFrame's own retrace
         * waits. Every frame lost up to a whole retrace there (hardware v21: "gpu wait" 9-15 ms per frame,
         * each 60 fps replay frame ~16.7 ms = exactly one retrace). Without it FrameBegin only waits for
         * the GPU to finish the previous frame; presentation stays retrace-paced by 3ds_main.c. */
        {
            PHASE_IN(31); /* C3D_FrameBegin: waiting for the GPU */
            C3D_FrameBegin(0);
            PHASE_OUT();
        }
        sInFrame = true;
        gPortProf = prev;
    }
    gPortPerfGpuWait += svcGetSystemTick() - t0;
    present_note_finished();
    C3D_RenderTargetClear(sTarget, C3D_CLEAR_ALL, 0x000000FF, 0xFFFFFFFF);
	C3D_FrameDrawOn(sTarget);
    return true;
}

static void gfx_3ds_swap_buffers_begin(void) 
{
    if (gGfx3DSMode == GFX_3DS_MODE_STEREO && !sFlipOn) {
        /* drawn through the stereo target: mark the aliased outputs for FrameEnd's transfers */
        sEyeOut[0]->used = sEyeOut[1]->used = true;
    }
    {
        extern void gfx_citro3d_flush_vbo(void);
        gfx_citro3d_flush_vbo(); /* CPU-written vertices must reach RAM before the GPU runs the frame */
    }
    {
        extern int gPortInterpExtra;
        (void)gPortInterpExtra;
    }
    /* C3D_FrameEnd(0) flushes the CPU data cache over the whole linear heap. v24 passed GX_CMDLIST_FLUSH
     * to skip that, but then the first part of a split command list (the in-frame depth copy splits the
     * frame) reached the GPU unflushed: hardware froze/crashed (v24, v25), Azahar has no cache to show it.
     * Kept as is until every GPU-read buffer, command lists included, is flushed explicitly. */
    {
        u64 t0;
        /* settings cmdlist_flush=1 (experiment, hardware): flush only the command list. Every other buffer the
         * GPU reads is flushed explicitly (VBO + indices above, textures at upload) and no frame is split any
         * more (all readbacks run outside frames), which is what froze v24/v25. Off by default until measured. */
        extern int gPortReplayBroken;
        extern int gPortInterpExtra;
        extern int gPortGpuVtx;
        /* overlap (see gPortOverlap): more frames of this update follow; frames with readbacks or captures, and
         * the CPU vertex path (its in-between frames rewrite the vertex buffer), keep the per-frame wait */
        bool keep = sFlipOn && gPortOverlap && gPortGpuVtx && gPortFramesFollow > 0 && !gPortReplayBroken &&
                    gPortInterpExtra && sColorWantFrames == 0 && sCaptureDst == NULL &&
                    !interp_dump_on(); /* its synchronous readback is not synchronous inside a citro3d frame */
        /* while the queue runs (an earlier frame of this update started it) every split must carry the flush: the
         * GPU may read it before C3D_FrameEnd's flush of the linear heap */
        bool queueLive = keep || sFrameOpenUpdate;
        if (sFlipOn) {
            flip_submit(queueLive ? GX_CMDLIST_FLUSH : 0); /* the flip thread presents it at its vblank: no gate */
        } else {
            present_gate();
        }
        gPortPerfStereoFrames += gGfx3DSMode == GFX_3DS_MODE_STEREO;
        {
            float use = C3D_GetCmdBufUsage();
            gPortPerfCmdBufMax = use > gPortPerfCmdBufMax ? use : gPortPerfCmdBufMax;
        }
        if (keep) {
            if (sLogicStarted && sFramesDone - sDepthReadAt <= (gPortRenderThreaded ? 2u : 1u) && sTarget != NULL) {
                depth_copy_async(); /* the walk's in-between frame: see sDepthReadAt */
                sDepthOfFrame = sFramesDone + 1;
            }
            {
                /* what C3D_FrameEnd(0) did for every frame: everything the CPU wrote (earlier splits of this frame
                 * included) reaches RAM before the GPU starts */
                extern u32 __ctru_linear_heap, __ctru_linear_heap_size;
                Port3ds_CacheFlush((void*)__ctru_linear_heap, __ctru_linear_heap_size);
            }
            gxCmdQueueRun((gxCmdQueue_s*)c3d_queue()); /* the GPU starts on this frame now */
            sFrameOpen = true;
            sFrameOpenUpdate = true;
            gPortPerfOverlapFrames++;
        } else {
            t0 = svcGetSystemTick();
            Port3ds_RenderPhase(33); /* C3D_FrameEnd */
            C3D_FrameEnd(gPortCmdlistFlush ? GX_CMDLIST_FLUSH : 0);
            sSubmitTick = svcGetSystemTick(); /* the queue starts at the end of C3D_FrameEnd */
            gPortPerfFrameEnd += sSubmitTick - t0;
            /* GPU timers of the frame that finished last (citro3d renderqueue.c): command processing, drawing */
            gPortPerfGpuProcMs += C3D_GetProcessingTime();
            gPortPerfGpuDrawMs += C3D_GetDrawingTime();
            gPortPerfGpuFrames++;
            sFrameOpenUpdate = false;
        }
    }
    if (!sFrameOpen) {
        sInFrame = false;
    }
    {
        extern int gPortInterpExtra;
        if (gPortRenderThreaded) {
            /* the game thread must not touch the GPU: the update's first frame is always copied here, queued */
            if (sLogicStarted && !sFrameOpen && sTarget != NULL && sColorWantFrames == 0 &&
                sFramesDone - sDepthReadAt <= 2u) {
                depth_copy_async();
            }
        } else if (gPortInterpExtra && sLogicStarted && !sFrameOpen) {
            sDepthEarlyPending = true; /* the walk's in-between frame: see sDepthReadAt */
        }
        sLogicStarted = !gPortInterpExtra;
        if (!gPortInterpExtra) {
            sFramesDone++; /* a frame the game may read depth from (gfx_3ds_depth_lazy) */
            if (sColorWantFrames > 0) {
                /* tools/statediff (color readback active) compares every frame: eager copy, as before */
                gfx_3ds_read_back_depth();
                sDepthOfFrame = sFramesDone;
            }
        }
    }
    {
        /* an in-between frame (60 fps interpolation) re-draws the same display list: the logic frame's
         * readbacks and captures belong to its exact pass, which comes after it */
        extern int gPortInterpExtra;
        if (gPortInterpExtra) {
            gfx_3ds_debug_dump_interp();
            sOffCur = -1;
            return;
        }
        gfx_3ds_debug_dump_interp();
    }
    /* (gPortStereoFlatScene: cleared when the game hands the frame over, Port3ds_TakeFrameRequests) */
    /* Depth readback right after this frame's render (C3D_SyncDisplayTransfer outside a frame waits
     * for the queued render first). The game samples it from Environment_GraphCallback, which the N64
     * runs once the previous frame's RDP work is done: reading it back here gives the same frame N-1
     * depth. (Reading at the next start_frame was one frame older - measured with tools/statediff:
     * Navi's glow in the adult Water Temple flipped.) */
    gfx_3ds_read_back_color();
    gfx_3ds_capture_frame();
    gfx_3ds_read_back_offscreen();
    gfx_3ds_debug_dump_stereo();
    sOffCur = -1;
    sFrameSlot ^= 1; /* next frame's readbacks go to the other slot */
    /* PORT (2026-09-24): no vblank wait here -- Port3ds_PaceFrame (3ds_main.c) paces updates to the
     * game's R_UPDATE_RATE retraces and pumps audio per retrace. */
}

static void gfx_3ds_swap_buffers_end(void) 
{
}

static double gfx_3ds_get_time(void) 
{
    return 0.0;
}

struct GfxWindowManagerAPI gfx_3ds =
{
    gfx_3ds_init,
    gfx_3ds_main_loop,
    gfx_3ds_get_dimensions,
    gfx_3ds_handle_events,
    gfx_3ds_start_frame,
    gfx_3ds_swap_buffers_begin,
    gfx_3ds_swap_buffers_end,
    gfx_3ds_get_time
};

#endif
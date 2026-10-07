#include <malloc.h>
/*
 * 3ds_main.c — Nintendo 3DS entry point (libctru). Replaces pc_main.c/pc_gfx.c.
 * Boots the port runtime, drives the OoT gamestate loop, and reads the real
 * 3DS buttons into the controller shim. Rendering goes through the citro3d
 * backend (gfx3ds/gfx_citro3d.c) via the same gfx_pc interface proven on PC.
 */
#include <3ds.h>
#include <stdio.h>
#include <string.h>
#include <dirent.h>
#include <math.h>

#include <PR/gbi.h>
#include "ultra64/sptask.h"
#include "gfx_pc.h"
#include "gfx_3ds.h"
#include "gfx_rendering_api.h"

/* N64 controller button bits (from include/controller.h) */
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

extern struct GfxWindowManagerAPI gfx_3ds;
extern struct GfxRenderingAPI gfx_citro3d_api;

extern void Main(void* arg);
extern void Graph_ThreadEntry(void* arg);
extern void PortDma_Init(const char* romPath);

/* VI config globals the boot path expects (idle.c) */
extern unsigned char gViConfigModeType;
extern void* osViModeNtscLan1;

/* read by osContGetReadData via the shim */
static unsigned short s3dsButtons;
static signed char s3dsStickX, s3dsStickY;

unsigned short PortInput_GetPad(signed char* outX, signed char* outY) {
    if (outX) *outX = s3dsStickX;
    if (outY) *outY = s3dsStickY;
    return s3dsButtons;
}

/* PORT (2026-09-30): widescreen option (gfx_pc.c gPortWidescreen): SELECT toggles it (the N64 pad has
 * no SELECT), saved in sdmc:/3ds/mm/settings.txt. Off = the N64's 4:3 picture with side bars. */
#define PORT_SETTINGS_PATH sSettingsPath
/* PORT (2026-10-04): holding L while the game starts reads (and saves) sdmc:/3ds/mm/settings_b.txt instead, when it
 * exists: a second set of settings for hardware tests, chosen on the console without editing the SD card */
static const char* sSettingsPath = "sdmc:/3ds/mm/settings.txt";
static int sO3dsSimSetting;
/* bench=1: A/B benchmark - frames alternate between the indexed and the array vertex path (gfx_pc.c
 * gPortLegacyVbo); display-list time is accumulated per variant and logged with each report. Leave the
 * title screen's attract demo running (same content every boot). */
static int sBench;
/* prof=1: sampling profiler (port_prof.h) */
#include "port_prof.h"
volatile unsigned char gPortProf;
static int sProfOn;
static int sLogMute; /* the periodic perf report without a measurement switch on (PortGfx_RunTask) */
static int sGpuAB; /* settings gpu_ab=1: the vertex path alternates every 2 perf reports */
static int sAaAB;  /* settings aa_ab=1: anti-aliasing alternates every 2 perf reports (gfx_3ds.c) */
static int sCmdflushAB; /* settings cmdflush_ab=1: C3D_FrameEnd flush mode alternates every 2 reports */
static int sPresentAB;  /* settings present_ab=1: the present gate (gfx_3ds.c) alternates every 4 reports */
static int sOverlapAB;  /* settings overlap_ab=1: the CPU/GPU overlap (gfx_3ds.c gPortOverlap) alternates every 2 reports */
static int sRenderThreadFile = 1;    /* render_thread as settings.txt has it (written back as read) */
static int sRenderThreadSetting = 1; /* settings render_thread=0/1: draw on another core (Port3ds_RenderJob); on by default
                                      * since hardware v49 (New 3DS: 2D unchanged at 59.7-59.8 shown, 3D up to 59.8) */
static int sRenderThreadAB;      /* settings render_thread_ab=1: jobs alternate thread / inline every 2 reports */
static int sAudioShareAB;         /* settings audio_share_ab=1: the Old 3DS mixer's system-core share cycles 30/55/80 */
static int sRawAB;                /* settings raw_vtx_ab=1: the raw vertex path alternates every 2 perf reports */
static int sSpeedRules = 1; /* settings speed_rules (raw_relax): 0 off, 1 while frame skip is on (Old 3DS), 2 always (tests).
                        * Active: gfx_pc.c gPortRawRelax (near-camera geometry stays raw) and no screen-linear shading
                        * splits - the per-triangle CPU work that dominates Old 3DS drawing on hardware (v58 profile) */
static int sReplayCopyAB;
static int sSpeedRulesAB;         /* settings speed_rules_ab=1: speed rules 1 / 0 alternate every 2 reports (Old 3DS A/B) */         /* settings replay_copy_ab=1: replay by copy (gfx_citro3d.c) alternates every 2 reports */
static int sO3dsLayout;          /* settings o3ds_layout=1: the Old 3DS thread layout on a New 3DS (with o3ds_sim=1) */
int Port3ds_OnRenderThread(void);
static int sCstickCamera = 1; /* settings cstick=0: the New 3DS C-stick presses the C buttons, not the camera */
int gPortStereoTest; /* settings stereo_test=1: 3D toggles every 2 s (gfx_3ds.c), freeze reproduction */
static volatile u32 sProfHist[PROF_COUNT];
static volatile int sProfRun;
static void Port3ds_ProfThread(void* arg) {
    (void)arg;
    while (sProfRun) {
        svcSleepThread(250000); /* 250 us */
        if (gPortProf < PROF_COUNT) sProfHist[gPortProf]++;
    }
}
static void Port3ds_ProfStart(void) {
    s32 prio = 0x30;
    if (!sProfOn) return;
    sProfRun = 1;
    svcGetThreadPriority(&prio, CUR_THREAD_HANDLE);
    /* another core (the main thread is on core 0): 1 = the system core on Old 3DS, also present on New */
    if (threadCreate(Port3ds_ProfThread, NULL, 4096, prio - 2, 1, true) == NULL) {
        threadCreate(Port3ds_ProfThread, NULL, 4096, prio - 2, -2, true);
    }
}
/* fps60=0 turns the 60 fps frame interpolation off (on by default; docs/3ds-60fps-plan.md) */
static int sInterp = 1;
/* frameskip: -1 = automatic (on for the Old 3DS and o3ds_sim / perf_ab's Old 3DS phase), 0 = off, 1 = on.
 * See PortGfx_RunTask. */
static int sFrameSkip = -1;
static int sSplitSet; /* split_* in settings.txt: fixed thresholds, not chosen per console */
/* PORT PERF (2026-10-02): coarse splits also on a New 3DS while the logic frame is about to miss its refresh.
 * With the present gate an update's three frames take three consecutive vblanks, the logic frame no later
 * than the second after the update starts: game logic + walk + GPU must stay under ~32.9 ms (hardware v34:
 * ~33.5). The coarse thresholds save about 3 ms (split share 6.5% -> 1.6%, 101-scene error 5.92 -> 5.99).
 * Chosen from the measured margin with hysteresis; settings split_auto=0 keeps the fine ones. */
static int sSplitAuto = 1, sCoarseAuto;
static double sDeadlineMarginMs = 10.0; /* running average */
static unsigned sCoarseUpdates;         /* per report */
static double sWalkDoneSum;             /* logic frame done (GPU included) after the update's start, per report */
static unsigned sWalkDoneN;
static int sChose; /* this update chose its in-between frames (gameplay; not menus, readbacks) */
static u64 sBenchDl[2];
static u32 sBenchFrames[2]; /* o3ds_sim as read from settings.txt (not perf_ab's run-time toggling) */
static void Port3ds_SaveSettings(void) {
    extern int gPortWidescreen;
    FILE* f = fopen(PORT_SETTINGS_PATH, "w");
    if (f != NULL) {
        extern int gPortHudTop;
        fprintf(f, "widescreen=%d\n", gPortWidescreen ? 1 : 0);
        fprintf(f, "hud=%d\n", gPortHudTop ? 1 : 0);
        {
            /* measurement switches are written back as they were read (perf_ab toggles gPortO3dsSim at
             * run time; saving that state made the next boot start in Old-3DS speed) */
            extern int gPortPerfStagesOn, gPortPerfAB;
            if (sO3dsSimSetting) fprintf(f, "o3ds_sim=1\n");
            if (gPortPerfStagesOn) fprintf(f, "perf_stages=1\n");
            if (gPortPerfAB) fprintf(f, "perf_ab=1\n");
            if (sProfOn) fprintf(f, "prof=1\n");
            if (sGpuAB) fprintf(f, "gpu_ab=1\n");
            if (sAaAB) fprintf(f, "aa_ab=1\n");
            { extern int gPortCmdlistFlush; if (gPortCmdlistFlush && !sCmdflushAB) fprintf(f, "cmdlist_flush=1\n"); }
            if (sCmdflushAB) fprintf(f, "cmdflush_ab=1\n");
            { extern int gPortPresentGate; if (!gPortPresentGate && !sPresentAB) fprintf(f, "present_gate=0\n"); }
            if (sPresentAB) fprintf(f, "present_ab=1\n");
            { extern int gPortFlip; if (!gPortFlip) fprintf(f, "flip=0\n"); }
            { extern int gPortOverlap; if (!gPortOverlap && !sOverlapAB) fprintf(f, "overlap=0\n"); }
            if (sOverlapAB) fprintf(f, "overlap_ab=1\n");
            if (!sRenderThreadFile) fprintf(f, "render_thread=0\n"); /* (as read: the Old 3DS layout forces it off) */
            if (sRenderThreadAB) fprintf(f, "render_thread_ab=1\n");
            if (sO3dsLayout) fprintf(f, "o3ds_layout=1\n");
            { extern int gPortRawVtxWant; if (!gPortRawVtxWant && !sRawAB) fprintf(f, "raw_vtx=0\n"); }
            if (sRawAB) fprintf(f, "raw_vtx_ab=1\n");
            { extern int gPortReplayCopy; if (!gPortReplayCopy && !sReplayCopyAB) fprintf(f, "replay_copy=0\n"); }
            if (sReplayCopyAB) fprintf(f, "replay_copy_ab=1\n");
            if (sSpeedRulesAB) fprintf(f, "speed_rules_ab=1\n");
            if (sSpeedRules) fprintf(f, "speed_rules=%d\n", sSpeedRules);
            { extern int gPortAudioMargin; if (gPortAudioMargin >= 0) fprintf(f, "audio_margin=%d\n", gPortAudioMargin); }
            { extern int gPortAudioShare; if (gPortAudioShare != 55) fprintf(f, "audio_share=%d\n", gPortAudioShare); }
            if (sAudioShareAB) fprintf(f, "audio_share_ab=1\n");
            if (!sCstickCamera) fprintf(f, "cstick=0\n");
            { extern int gPortAA; if (gPortAA && !sAaAB) fprintf(f, "aa=1\n"); }
            { extern int gPortGpuVtx; if (!gPortGpuVtx && !sGpuAB) fprintf(f, "gpu_vtx=0\n"); }
            if (sInterp != 1) fprintf(f, "fps60=%d\n", sInterp);
            if (sFrameSkip >= 0) fprintf(f, "frameskip=%d\n", sFrameSkip);
            { extern int gPortShadeSplit; if (!gPortShadeSplit) fprintf(f, "shade_split=0\n"); }
            if (!sSplitAuto) fprintf(f, "split_auto=0\n");
            { extern int gPortShadeLinear; if (gPortShadeLinear) fprintf(f, "shade_linear=1\n"); }
            if (sSplitSet) {
                extern float gPortSplitRatio, gPortSplitMinPx;
                extern int gPortSplitDepth;
                fprintf(f, "split_ratio=%d\nsplit_px=%d\nsplit_depth=%d\n", (int)(gPortSplitRatio * 100.0f + 0.5f),
                        (int)gPortSplitMinPx, gPortSplitDepth);
            }
        }
        fclose(f);
    }
}
static void Port3ds_LoadSettings(void) {
    extern int gPortWidescreen;
    char line[64];
    FILE* f = fopen(PORT_SETTINGS_PATH, "r");
    if (f == NULL) {
        return;
    }
    while (fgets(line, sizeof(line), f) != NULL) {
        int v;
        if (sscanf(line, "widescreen=%d", &v) == 1) {
            gPortWidescreen = v != 0;
        }
        if (sscanf(line, "hud=%d", &v) == 1) {
            extern int gPortHudTop;
            gPortHudTop = v != 0;
        }
        /* measurement switches (docs/3ds-60fps-plan.md P0), hand-edited in settings.txt:
         * o3ds_sim=1: New 3DS runs at the Old 3DS clock (268 MHz, no L2) - approximates an Old 3DS
         * perf_stages=1: per-stage frame timers in boot.log (one tick read per triangle) */
        if (sscanf(line, "o3ds_sim=%d", &v) == 1) {
            extern int gPortO3dsSim;
            gPortO3dsSim = v != 0;
            sO3dsSimSetting = v != 0;
        }
        if (sscanf(line, "bench=%d", &v) == 1) {
            sBench = v != 0;
        }
        if (sscanf(line, "prof=%d", &v) == 1) {
            sProfOn = v != 0;
        }
        if (sscanf(line, "cmdlist_flush=%d", &v) == 1) { extern int gPortCmdlistFlush; gPortCmdlistFlush = v != 0; }
        if (sscanf(line, "cmdflush_ab=%d", &v) == 1) { sCmdflushAB = v != 0; }
        if (sscanf(line, "present_gate=%d", &v) == 1) { extern int gPortPresentGate; gPortPresentGate = v != 0; }
        if (sscanf(line, "present_ab=%d", &v) == 1) { sPresentAB = v != 0; }
        if (sscanf(line, "flip=%d", &v) == 1) { extern int gPortFlip; gPortFlip = v != 0; }
        if (sscanf(line, "overlap=%d", &v) == 1) { extern int gPortOverlap; gPortOverlap = v != 0; }
        if (sscanf(line, "overlap_ab=%d", &v) == 1) { sOverlapAB = v != 0; }
        if (sscanf(line, "render_thread=%d", &v) == 1) { sRenderThreadSetting = sRenderThreadFile = v != 0; }
        if (sscanf(line, "render_thread_ab=%d", &v) == 1) { sRenderThreadAB = v != 0; }
        if (sscanf(line, "o3ds_layout=%d", &v) == 1) { sO3dsLayout = v != 0; }
        if (sscanf(line, "raw_vtx=%d", &v) == 1) { extern int gPortRawVtxWant; gPortRawVtxWant = v != 0; }
        if (sscanf(line, "raw_vtx_ab=%d", &v) == 1) { sRawAB = v != 0; }
        if (sscanf(line, "replay_copy_ab=%d", &v) == 1) { sReplayCopyAB = v != 0; }
        if (sscanf(line, "speed_rules_ab=%d", &v) == 1) { sSpeedRulesAB = v != 0; }
        if (sscanf(line, "raw_relax=%d", &v) == 1 || sscanf(line, "speed_rules=%d", &v) == 1) { sSpeedRules = v < 0 ? 0 : v > 2 ? 2 : v; }
        if (sscanf(line, "audio_margin=%d", &v) == 1 && v >= 0 && v <= 2048) { extern int gPortAudioMargin; gPortAudioMargin = v; }
        if (sscanf(line, "interp_dump_span=%d", &v) == 1 && v >= 1 && v <= 400) { extern int gPortInterpDumpSpan; gPortInterpDumpSpan = v; }
        if (sscanf(line, "interp_dump_at=%d", &v) == 1 && v >= 0) { extern int gPortInterpDumpAt; gPortInterpDumpAt = v; }
        if (sscanf(line, "fastswitch=%d", &v) == 1) { extern int gPortFastSwitch; gPortFastSwitch = v != 0; }
        if (sscanf(line, "replay_copy=%d", &v) == 1) { extern int gPortReplayCopy; gPortReplayCopy = v != 0; }
        if (sscanf(line, "replay_copy_check=%d", &v) == 1) { extern int gPortReplayCopyCheck; gPortReplayCopyCheck = v != 0; }
        if (sscanf(line, "audio_share=%d", &v) == 1) { extern int gPortAudioShare; gPortAudioShare = v; }
        if (sscanf(line, "audio_share_ab=%d", &v) == 1) { sAudioShareAB = v != 0; }
        if (sscanf(line, "raw_near=%d", &v) == 1) { extern int gPortRawNear; gPortRawNear = v; }
        if (sscanf(line, "raw_ratio=%d", &v) == 1 && v >= 10) { extern float gPortRawRatio; gPortRawRatio = v / 10.0f; }
        if (sscanf(line, "cstick=%d", &v) == 1) { sCstickCamera = v != 0; }
        if (sscanf(line, "flipdump=%d", &v) == 1) { extern int gPortFlipDump; gPortFlipDump = v != 0; }
        if (sscanf(line, "tjdump=%d", &v) == 1) { extern int gPortTjDumpFrame; gPortTjDumpFrame = v; }
        if (sscanf(line, "aa_ab=%d", &v) == 1) {
            sAaAB = v != 0;
        }
        if (sscanf(line, "aa=%d", &v) == 1) {
            extern int gPortAA;
            gPortAA = v != 0;
        }
        if (sscanf(line, "gpu_ab=%d", &v) == 1) { /* alternate the CPU / GPU vertex paths (hardware A/B) */
            sGpuAB = v != 0;
        }
        if (sscanf(line, "gpu_vtx=%d", &v) == 1) { /* GPU vertex path (gfx_citro3d.c), read before gfx init */
            extern int gPortGpuVtx;
            gPortGpuVtx = v != 0;
        }
        if (sscanf(line, "stereo_test=%d", &v) == 1) {
            gPortStereoTest = v != 0;
        }
        if (sscanf(line, "fps60=%d", &v) == 1) {
            sInterp = v;
        }
        if (sscanf(line, "frameskip=%d", &v) == 1) {
            sFrameSkip = v != 0;
        }
        if (sscanf(line, "split_ratio=%d", &v) == 1) { extern float gPortSplitRatio; gPortSplitRatio = v / 100.0f; sSplitSet = 1; }
        if (sscanf(line, "split_px=%d", &v) == 1) { extern float gPortSplitMinPx; gPortSplitMinPx = (float)v; sSplitSet = 1; }
        if (sscanf(line, "split_depth=%d", &v) == 1) { extern int gPortSplitDepth; gPortSplitDepth = v; sSplitSet = 1; }
        if (sscanf(line, "split_auto=%d", &v) == 1) { sSplitAuto = v != 0; }
        if (sscanf(line, "shade_split=%d", &v) == 1) {
            extern int gPortShadeSplit;
            gPortShadeSplit = v != 0;
        }
        if (sscanf(line, "shade_linear=%d", &v) == 1) {
            extern int gPortShadeLinear;
            gPortShadeLinear = v != 0;
        }
        if (sscanf(line, "perf_ab=%d", &v) == 1) {
            extern int gPortPerfAB;
            gPortPerfAB = v != 0;
        }
        if (sscanf(line, "perf_stages=%d", &v) == 1) {
            extern int gPortPerfStagesOn;
            gPortPerfStagesOn = v != 0;
        }
    }
    fclose(f);
}

/* PORT (2026-09-30): touch panel on the bottom screen, drawn straight into its RGB565
 * framebuffer (single-buffered by consoleInit; stdout/stderr are silenced once the panel is up).
 *   left:   VIEW (C-up: first person / Navi), rupees, small keys, SCREEN (4:3 / wide), OCARINA
 *   centre: hearts + magic, the minimap (software-drawn from gPortMinimap, port_minimap.h),
 *           tabs GEAR / MAP / ITEMS (START straight to that pause page)
 *   right:  C-left (Y), C-down (I; ZL on a New 3DS), C-right (X) with live item icons + ammo, BOOTS (cycles owned
 *           boots; ZR on a New 3DS)
 * Everything redraws only when it changes. Full description: docs/3ds-touch-panel.md. */
#include "port_minimap.h"
extern const unsigned char* Port_GetItemIcon(int itemId);
extern int gPortWidescreen;

static int sTouchUi, sTouchUiRedraw, sTouchUiN3ds;
static u16* sFb;
static int sFbDirty;

#define PRGB(r, g, b) (u16)((((r) >> 3) << 11) | (((g) >> 2) << 5) | ((b) >> 3))
#define COL_TEXT   PRGB(244, 244, 244)
#define COL_SHADOW PRGB(20, 20, 24)
#define COL_DIM    PRGB(196, 198, 206) /* "not available": still readable on the stone plates (130 was not, v42) */

/* item ids used by the panel (include/item.h) */
#define PANEL_ITEM_OCARINA_FAIRY   0x07
#define PANEL_ITEM_OCARINA_OF_TIME 0x08
#define PANEL_ITEM_BOOTS_KOKIRI    0x44

enum { P_VIEW, P_SCREEN, P_HUD, P_OCARINA, P_CLEFT, P_CDOWN, P_CRIGHT, P_BOOTS, P_GEAR, P_MAP, P_ITEMS, P_COUNT };
typedef struct {
    s16 x, y, w, h;
    const char* label;
    u16 btn;  /* N64 bits while held; 0 = tap action */
    s8 page;  /* pause page for the tabs (PAUSE_ITEM 0, MAP 1, EQUIP 3), -1 otherwise */
    u8 tr, tg, tb; /* tab face colour (0,0,0 = stone) */
} PanelPad;
static const PanelPad sPads[P_COUNT] = {
    { 4, 4, 56, 52, "VIEW", BTN_CUP_, -1, 0, 0, 0 },
    { 4, 112, 56, 34, "SCREEN", 0, -1, 0, 0, 0 },
    { 4, 148, 56, 32, "HUD", 0, -1, 0, 0, 0 },
    { 4, 184, 56, 52, "OCARINA", 0, -1, 0, 0, 0 },
    { 260, 4, 56, 56, "Y", BTN_CLEFT_, -1, 0, 0, 0 },
    { 260, 64, 56, 56, "I", BTN_CDOWN_, -1, 0, 0, 0 }, /* touch item slot I (+ ZL / D-pad down) */
    { 260, 124, 56, 56, "X", BTN_CRIGHT_, -1, 0, 0, 0 },
    { 260, 184, 56, 52, "BOOTS", 0, -1, 0, 0, 0 },
    { 66, 208, 60, 30, "GEAR", BTN_START_, 3, 52, 116, 60 },
    { 130, 208, 60, 30, "MAP", BTN_START_, 1, 140, 44, 48 },
    { 194, 208, 60, 30, "ITEMS", BTN_START_, 0, 48, 72, 150 },
};
/* centre column */
#define MAP_X 66
#define MAP_Y 34
#define MAP_W 188
#define MAP_H 170

static inline void Px(int x, int y, u16 c) {
    if ((unsigned)x < 320 && (unsigned)y < 240) sFb[x * 240 + (239 - y)] = c;
}
static inline u16 PxGet(int x, int y) {
    return ((unsigned)x < 320 && (unsigned)y < 240) ? sFb[x * 240 + (239 - y)] : 0;
}
static inline int Noise(int x, int y) { /* stable per-pixel grain for the stone look, -8..7 */
    unsigned h = (unsigned)(x * 374761393 + y * 668265263);
    h = (h ^ (h >> 13)) * 1274126177u;
    return (int)((h >> 24) & 15) - 8;
}
static inline int RoundDiv(int a, int b) { /* a / b to the nearest integer, b > 0 */
    return a >= 0 ? (a + b / 2) / b : -((-a + b / 2) / b);
}
static inline int Clamp8(int v) {
    return v < 0 ? 0 : (v > 255 ? 255 : v);
}
static void FillRect(int x, int y, int w, int h, u16 c) {
    int i, j;
    for (i = x; i < x + w; i++) {
        for (j = y; j < y + h; j++) Px(i, j, c);
    }
}
/* textured fill: base colour + grain */
static void FillStone(int x, int y, int w, int h, int r, int g, int b, int grain) {
    int i, j;
    for (i = x; i < x + w; i++) {
        for (j = y; j < y + h; j++) {
            int n = Noise(i, j) * grain / 8;
            Px(i, j, PRGB(Clamp8(r + n), Clamp8(g + n), Clamp8(b + n)));
        }
    }
}
static u16 Blend565(u16 bg, int r, int g, int b, int a) { /* a 0..255 */
    int br = (bg >> 11) << 3, bgc = ((bg >> 5) & 63) << 2, bb = (bg & 31) << 3;
    return PRGB(br + (r - br) * a / 255, bgc + (g - bgc) * a / 255, bb + (b - bb) * a / 255);
}
/* bevelled stone button: light stone rim, face inside */
static void DrawPlate(const PanelPad* p, int pressed) {
    int x = p->x, y = p->y, w = p->w, h = p->h;
    int rim = pressed ? 150 : 196;
    int tab = p->tr | p->tg | p->tb;
    FillStone(x, y, w, h, rim, rim - 6, rim - 16, 10);
    FillRect(x, y, w, 1, PRGB(236, 232, 220));
    FillRect(x, y + h - 1, w, 1, PRGB(70, 66, 60));
    FillRect(x + w - 1, y, 1, h, PRGB(96, 92, 84));
    if (tab) {
        FillStone(x + 4, y + 4, w - 8, h - 4, pressed ? p->tr / 2 : p->tr, pressed ? p->tg / 2 : p->tg,
                  pressed ? p->tb / 2 : p->tb, 6);
    } else {
        int f = pressed ? 96 : 150;
        FillStone(x + 3, y + 3, w - 6, h - 6, f, f - 4, f - 12, 12);
    }
    Px(x, y, 0), Px(x + w - 1, y, 0), Px(x, y + h - 1, 0), Px(x + w - 1, y + h - 1, 0);
}
/* 8x8 text with the console font, transparent, 1-pixel drop shadow; scale 1 or 2 */
static void DrawTextS(int x, int y, const char* s, u16 c, int scale) {
    PrintConsole* con = consoleGetDefault();
    int pass;
    for (pass = 0; pass < 2; pass++) {
        const char* t = s;
        int cx = x + (pass == 0 ? 1 : 0), cy = y + (pass == 0 ? 1 : 0);
        u16 col = pass == 0 ? COL_SHADOW : c;
        for (; *t; t++, cx += 8 * scale) {
            unsigned ch = (unsigned char)*t;
            const u8* g;
            int r, bit;
            if (ch < con->font.asciiOffset || ch >= con->font.asciiOffset + con->font.numChars) continue;
            g = con->font.gfx + (ch - con->font.asciiOffset) * 8;
            for (r = 0; r < 8 * scale; r++) {
                for (bit = 0; bit < 8 * scale; bit++) {
                    if (g[r / scale] & (0x80 >> (bit / scale))) Px(cx + bit, cy + r, col);
                }
            }
        }
    }
}
static void DrawText(int x, int y, const char* s, u16 c) {
    DrawTextS(x, y, s, c, 1);
}
/* the columns a console-font glyph actually draws (first, last; -1 for a blank) */
static void GlyphInk(unsigned char ch, int* first, int* last) {
    PrintConsole* con = consoleGetDefault();
    int r, col, bits = 0;
    *first = *last = -1;
    if (ch < con->font.asciiOffset || ch >= con->font.asciiOffset + con->font.numChars) return;
    for (r = 0; r < 8; r++) bits |= con->font.gfx[(ch - con->font.asciiOffset) * 8 + r];
    for (col = 0; col < 8; col++) {
        if (bits & (0x80 >> col)) {
            if (*first < 0) *first = col;
            *last = col;
        }
    }
}
/* centered on the pixels drawn, not on 8-pixel cells; a label wider than a 56-pixel pad's inside ("OCARINA":
 * 7 x 8 = 56 px spilled over the stone border, hardware v41) closes up to 7 pixels per letter (the console font
 * leaves one column blank) */
static void DrawTextCB(int cx, int y, const char* s, u16 c, int bold) {
    int n = (int)strlen(s), adv = n * 8 > 48 ? 7 : 8, i, lo = 1 << 20, hi = -1, f, l;
    char one[2] = { 0, 0 };
    for (i = 0; i < n; i++) {
        GlyphInk((unsigned char)s[i], &f, &l);
        if (f >= 0) {
            lo = i * adv + f < lo ? i * adv + f : lo;
            hi = i * adv + l > hi ? i * adv + l : hi;
        }
    }
    if (hi < 0) return;
    hi += bold; /* bold: drawn twice, one pixel apart */
    for (i = 0; i < n; i++) {
        one[0] = s[i];
        DrawText(cx - (hi - lo + 1) / 2 - lo + i * adv, y, one, c);
        if (bold) DrawText(cx - (hi - lo + 1) / 2 - lo + i * adv + 1, y, one, c);
    }
}
static void DrawTextC(int cx, int y, const char* s, u16 c) {
    DrawTextCB(cx, y, s, c, 0);
}
/* 32x32 RGBA32 item icon at size px (nearest), alpha-blended; dim = disabled button */
static void DrawIcon(int x, int y, const u8* rgba, int size, int dim) {
    int i, j, x0 = 32, x1 = -1, y0 = 32, y1 = -1;
    if (rgba == NULL) return;
    /* centered on the pixels the art covers, not on its 32x32 cell: item art sits up to 2 texels off-center
     * (Kokiri boots 2 right, bomb 1 right: visibly off on the plates, 2026-10-03) */
    for (j = 0; j < 32; j++) {
        for (i = 0; i < 32; i++) {
            if (rgba[(((j * 32 + i) * 4) + 3) ^ 7] != 0) {
                x0 = i < x0 ? i : x0, x1 = i > x1 ? i : x1;
                y0 = j < y0 ? j : y0, y1 = j > y1 ? j : y1;
            }
        }
    }
    if (x1 < 0) return;
    x += RoundDiv((32 - (x0 + x1 + 1)) * size, 64);
    y += RoundDiv((32 - (y0 + y1 + 1)) * size, 64);
    for (j = 0; j < size; j++) {
        for (i = 0; i < size; i++) {
            /* game RAM keeps logical byte k at address k ^ 7 on the 3DS (see gfx_src_swizzle) */
            int o = ((j * 32 / size) * 32 + (i * 32 / size)) * 4;
            int r = rgba[o ^ 7], g = rgba[(o + 1) ^ 7], b = rgba[(o + 2) ^ 7], a = rgba[(o + 3) ^ 7];
            if (a == 0) continue;
            if (dim) r = g = b = (r + g + b) / 6;
            Px(x + i, y + j, Blend565(PxGet(x + i, y + j), r, g, b, a));
        }
    }
}

/* ---- panel state (what is on screen) ---- */
static int sHeldPad = -1;
static PortHudInfo sShown;
static int sShownWide = -1, sShownIconsOk = -1, sShownNavi, sShownHud = -1;
static int sNaviPulse; /* the NAVI pad's phase while Navi calls (1: green) */
static const u8* sShownIconSeg;
static unsigned sLastHudSerial, sLastMapSerial;
static int sHudStale = 99, sMapStale = 99;

static int IconsOk(void) {
    return sHudStale < 4 && gPortHudIconSeg != NULL;
}

static void DrawPad(int i) {
    const PanelPad* p = &sPads[i];
    int cx = p->x + p->w / 2;
    char buf[8];
    DrawPlate(p, i == sHeldPad);
    if (i >= P_CLEFT && i <= P_CRIGHT) {
        int c = i - P_CLEFT;
        if (sShown.cItem[c] < 0x56 && IconsOk()) {
            DrawIcon(cx - 22, p->y + 5, gPortHudIconSeg + (c + 1) * 32 * 32 * 4, 44, sShown.cDisabled[c]);
            if (sShown.cAmmo[c] >= 0) {
                snprintf(buf, sizeof(buf), "%d", sShown.cAmmo[c]);
                DrawText(p->x + p->w - 5 - 8 * (int)strlen(buf), p->y + p->h - 13, buf,
                         sShown.cAmmo[c] == 0 ? PRGB(255, 90, 60) : PRGB(120, 250, 120));
            }
        }
        /* the 3DS button that also presses it; the middle slot is the touch slot "I" on every model (D-pad
         * down presses it too), and the New 3DS adds a "ZL" tag (2026-10-03: ZL alone meant nothing on an Old 3DS) */
        DrawText(p->x + 4, p->y + 4, p->label, PRGB(255, 230, 120));
        if (c == 1 && sTouchUiN3ds) DrawText(p->x + p->w - 4 - 16, p->y + 4, "ZL", PRGB(200, 204, 214));
    } else if (i == P_BOOTS) {
        if (sShown.boots >= 1 && sShown.boots <= 3) {
            DrawIcon(cx - 20, p->y + 3, Port_GetItemIcon(PANEL_ITEM_BOOTS_KOKIRI + sShown.boots - 1), 40, 0);
        }
        DrawTextC(cx, p->y + p->h - 12, "BOOTS", COL_TEXT);
        if (sTouchUiN3ds) DrawText(p->x + p->w - 4 - 16, p->y + 4, "ZR", PRGB(200, 204, 214)); /* ZR presses it */
    } else if (i == P_OCARINA) {
        int have = sShown.ocarina == PANEL_ITEM_OCARINA_FAIRY || sShown.ocarina == PANEL_ITEM_OCARINA_OF_TIME;
        if (have) DrawIcon(cx - 20, p->y + 3, Port_GetItemIcon(sShown.ocarina), 40, 0);
        DrawTextC(cx, p->y + p->h - 12, "OCARINA", have ? COL_TEXT : COL_DIM);
    } else if (i == P_VIEW && sShownNavi) {
        /* Navi wants to talk (the VIEW eye gives way to her): a glowing fairy, "NAVI". PORT (2026-10-05): it pulses
         * - blue, then green with a bright frame (she turns green when she has something to say) - so it is noticed
         * with the top-screen HUD off, where the "Navi" label on C-up is not shown */
        int fy = p->y + 20, dx, dy, green = sNaviPulse;
        int gr = green ? 120 : 120, gg = green ? 255 : 200, gb = green ? 150 : 255;
        if (green) {
            FillRect(p->x + 2, p->y + 2, p->w - 4, 2, PRGB(140, 255, 150));
            FillRect(p->x + 2, p->y + p->h - 4, p->w - 4, 2, PRGB(140, 255, 150));
            FillRect(p->x + 2, p->y + 2, 2, p->h - 4, PRGB(140, 255, 150));
            FillRect(p->x + p->w - 4, p->y + 2, 2, p->h - 4, PRGB(140, 255, 150));
        }
        for (dy = -16; dy <= 16; dy++) {
            for (dx = -24; dx <= 24; dx++) {
                int d2 = dx * dx + dy * dy;
                /* wings: two tilted ellipses on each side */
                int wx = dx < 0 ? -dx : dx, wyU = dy + 5, wyD = dy - 7;
                if ((wx > 5 && (wx - 13) * (wx - 13) * 9 + wyU * wyU * 49 <= 9 * 49 * 4) ||
                    (wx > 5 && (wx - 11) * (wx - 11) * 16 + wyD * wyD * 64 <= 16 * 64 * 2)) {
                    Px(cx + dx, fy + dy, Blend565(PxGet(cx + dx, fy + dy), 200, 235, 255, 150));
                }
                if (d2 <= 144) { /* glow */
                    int a = 255 - d2 * 255 / 144;
                    Px(cx + dx, fy + dy, Blend565(PxGet(cx + dx, fy + dy), gr, gg, gb, a));
                }
                if (d2 <= 16) Px(cx + dx, fy + dy, PRGB(250, 255, 255));
            }
        }
        DrawTextC(cx, p->y + p->h - 13, "NAVI", green ? PRGB(160, 255, 160) : PRGB(150, 220, 255));
    } else if (i == P_VIEW) {
        /* an eye: the VIEW button */
        int ey = p->y + 21, dx, dy;
        for (dy = -9; dy <= 9; dy++) {
            for (dx = -18; dx <= 18; dx++) {
                if (dx * dx * 81 + dy * dy * 324 <= 81 * 324) Px(cx + dx, ey + dy, PRGB(240, 240, 248));
                if (dx * dx + dy * dy <= 49) Px(cx + dx, ey + dy, PRGB(100, 80, 200));
                if (dx * dx + dy * dy <= 9) Px(cx + dx, ey + dy, PRGB(16, 16, 32));
            }
        }
        DrawTextC(cx, p->y + p->h - 13, "VIEW", COL_TEXT);
    } else if (i == P_SCREEN) {
        DrawTextC(cx, p->y + 6, "SCREEN", COL_TEXT);
        DrawTextC(cx, p->y + 19, gPortWidescreen ? "WIDE" : "4:3", PRGB(255, 230, 120));
    } else if (i == P_HUD) {
        DrawTextC(cx, p->y + 5, "HUD", COL_TEXT); /* the HUD on the top screen ("TOP HUD" did not fit) */
        DrawTextC(cx, p->y + 18, gPortHudTop ? "ON" : "OFF", PRGB(255, 230, 120));
    } else {
        /* tab: bold label, centered on its pixels between the tab's side rims (it sat 1-2 px right) */
        DrawTextCB(cx, p->y + 12, p->label, COL_TEXT, 1);
    }
    sFbDirty = 1;
}

static void DrawHeart(int x, int y, int fill16) { /* 9x8 heart, fill in 16ths (quarters shown) */
    static const char* const sShape[8] = { ".XX.XX..", "XXXXXXX.", "XXXXXXX.", "XXXXXXX.",
                                           ".XXXXX..", "..XXX...", "...X....", "........" };
    int r, c;
    for (r = 0; r < 7; r++) {
        for (c = 0; c < 7; c++) {
            if (sShape[r][c] != 'X') continue;
            /* quarters: fill from the bottom-left quadrant clockwise, approximated by columns */
            int filled = fill16 >= 16 || (fill16 > 0 && c < (fill16 * 7 + 15) / 16);
            Px(x + c, y + r, filled ? PRGB(230, 30, 40) : PRGB(70, 40, 44));
        }
    }
}

static void DrawStatus(void) {
    int i, hearts;
    FillStone(MAP_X, 0, MAP_W, MAP_Y - 2, 26, 22, 20, 4);
    if (sShown.valid) {
        hearts = sShown.healthCapacity / 16;
        for (i = 0; i < hearts && i < 20; i++) {
            int fill = sShown.health - i * 16;
            DrawHeart(MAP_X + 6 + (i % 10) * 10, 4 + (i / 10) * 9, fill < 0 ? 0 : (fill > 16 ? 16 : fill));
        }
        if (sShown.magicCapacity > 0) {
            int w = sShown.magicCapacity * (MAP_W - 12) / 96; /* 48 = single, 96 = double magic */
            int f = sShown.magic * (MAP_W - 12) / 96;
            FillRect(MAP_X + 5, 24, w + 2, 6, PRGB(230, 230, 230));
            FillRect(MAP_X + 6, 25, w, 4, PRGB(20, 20, 20));
            FillRect(MAP_X + 6, 25, f, 4, PRGB(40, 200, 60));
        }
    }
    sFbDirty = 1;
}

/* the pixels a console-font string draws: columns x0..x1 (8 per character) and rows y0..y1 */
static void TextInk(const char* s, int* x0, int* x1, int* y0, int* y1) {
    PrintConsole* con = consoleGetDefault();
    int i, r, f, l;
    *x0 = *y0 = 1 << 20, *x1 = *y1 = -1;
    for (i = 0; s[i]; i++) {
        unsigned ch = (unsigned char)s[i];
        GlyphInk((unsigned char)ch, &f, &l);
        if (f < 0) continue;
        *x0 = i * 8 + f < *x0 ? i * 8 + f : *x0;
        *x1 = i * 8 + l > *x1 ? i * 8 + l : *x1;
        for (r = 0; r < 8; r++) {
            if (con->font.gfx[(ch - con->font.asciiOffset) * 8 + r]) {
                *y0 = r < *y0 ? r : *y0;
                *y1 = r > *y1 ? r : *y1;
            }
        }
    }
}

/* rupees and small keys: icon + number centered in the left column (its pads' centre, x 32), the number centered on
 * the icon's height (it sat on the icon's top row, user report 2026-10-06) */
static void DrawCounters(void) {
    char buf[16];
    int x0, x1, y0, y1, left;
    FillStone(0, 60, 64, 52, 40, 42, 46, 6);
    if (sShown.rupees >= 0) {
        int dy;
        snprintf(buf, sizeof(buf), "%d", sShown.rupees);
        TextInk(buf, &x0, &x1, &y0, &y1);
        left = 32 - (10 + 4 + (x1 - x0 + 1)) / 2; /* the rupee is 10 pixels wide, then a 4-pixel gap */
        for (dy = 0; dy < 14; dy++) {            /* green rupee, rows 66..79 */
            int half = dy < 4 ? dy + 2 : (dy < 10 ? 5 : 14 - dy + 1);
            FillRect(left + 5 - half, 66 + dy, half * 2, 1, dy < 7 ? PRGB(90, 230, 110) : PRGB(40, 170, 60));
        }
        DrawTextS(left + 14 - x0, 66 + (14 - (y1 - y0 + 1)) / 2 - y0, buf, PRGB(150, 250, 150), 1);
    }
    if (sShown.keys >= 0) {
        snprintf(buf, sizeof(buf), "%d", sShown.keys);
        TextInk(buf, &x0, &x1, &y0, &y1);
        left = 32 - (8 + 4 + (x1 - x0 + 1)) / 2; /* small key: 8 pixels wide, rows 88..103 */
        FillRect(left, 88, 7, 7, PRGB(210, 210, 220));
        FillRect(left + 2, 90, 3, 3, PRGB(40, 42, 46));
        FillRect(left + 2, 95, 3, 9, PRGB(210, 210, 220));
        FillRect(left + 5, 99, 3, 2, PRGB(210, 210, 220));
        FillRect(left + 5, 102, 3, 2, PRGB(210, 210, 220));
        DrawText(left + 12 - x0, 88 + (16 - (y1 - y0 + 1)) / 2 - y0, buf, COL_TEXT);
    }
    sFbDirty = 1;
}

/* ---- minimap ---- */
static void MapTri(float cx, float cy, short yaw, float size, u16 col) {
    /* arrow along yaw: world +x is right and +z is down on the map */
    float a = yaw * (3.14159265f / 32768.0f);
    float fx = sinf(a), fy = cosf(a);
    float px[3] = { cx + fx * size, cx - fx * size * 0.6f + fy * size * 0.6f, cx - fx * size * 0.6f - fy * size * 0.6f };
    float py[3] = { cy + fy * size, cy - fy * size * 0.6f - fx * size * 0.6f, cy - fy * size * 0.6f + fx * size * 0.6f };
    int x0 = (int)floorf(fminf(px[0], fminf(px[1], px[2]))), x1 = (int)ceilf(fmaxf(px[0], fmaxf(px[1], px[2])));
    int y0 = (int)floorf(fminf(py[0], fminf(py[1], py[2]))), y1 = (int)ceilf(fmaxf(py[0], fmaxf(py[1], py[2])));
    int x, y, k;
    for (y = y0 - 1; y <= y1 + 1; y++) {
        for (x = x0 - 1; x <= x1 + 1; x++) {
            int pos = 0, neg = 0;
            for (k = 0; k < 3; k++) {
                int n = (k + 1) % 3;
                float e = (px[n] - px[k]) * (y + 0.5f - py[k]) - (py[n] - py[k]) * (x + 0.5f - px[k]);
                if (e >= 0) pos++; else neg++;
            }
            if ((pos == 3 || neg == 3) && x >= MAP_X && x < MAP_X + MAP_W && y >= MAP_Y && y < MAP_Y + MAP_H) {
                Px(x, y, col);
            }
        }
    }
}

static int MapTexel(const PortMinimap* m, int tx, int ty, int* lum) { /* alpha 0..255 */
    const u8* t = m->tex;
    int idx = ty * m->w + tx;
    int nib = (t[(idx >> 1) ^ 7] >> ((idx & 1) ? 0 : 4)) & 0xF; /* byte k at k ^ 7 */
    if (m->fmt == PORT_MINIMAP_I4) {
        *lum = 255;
        return nib * 17;
    }
    *lum = (nib >> 1) * 255 / 7;
    return (nib & 1) ? 255 : 0;
}

static void DrawMapInto(int have);

/* The bottom framebuffer is single-buffered and scanned out while we draw: repainting the parchment and
 * then the map in place showed the half-drawn state (the map flashed on hardware). Draw into a shadow
 * copy with the framebuffer's layout, then copy only the map columns over in one pass. */
extern void Port3ds_CacheFlush(const void* p, u32 size); /* gfx_3ds.c */
static int sFbMapDirty; /* only the map's columns changed: the flush covers just those (PORT PERF 2026-10-04) */
static void DrawMap(int have) {
    static u16 sShadow[320 * 240];
    u16* real = sFb;
    int x, dirty = sFbDirty;
    sFb = sShadow;
    DrawMapInto(have);
    sFb = real;
    sFbDirty = dirty; /* (the shadow copy is not scanned out) */
    for (x = MAP_X; x < MAP_X + MAP_W; x++) {
        /* column x holds rows 239..0; the map's rows MAP_Y..MAP_Y+MAP_H-1 are contiguous in it */
        int base = x * 240 + (239 - (MAP_Y + MAP_H - 1));
        memcpy(&real[base], &sShadow[base], MAP_H * sizeof(u16));
    }
    sFbMapDirty = 1;
}

/* PORT PERF (2026-10-01): the parchment (noise per pixel) and the resampled map texture only change
 * with the room, but were repainted on every redraw - every other frame while Link moves (hardware
 * profile: the panel took 2-7% of Old 3DS time). They are cached here, keyed by what they depend on;
 * a redraw copies them and paints only the markers. */
static u16 sMapBase[320 * 240];
static u32 sMapBaseKey;
static int sMapBaseValid;
static float sMapBaseS, sMapBaseOx, sMapBaseOy;

static void MapCopyColumns(u16* dst, const u16* src) {
    int x;
    for (x = MAP_X; x < MAP_X + MAP_W; x++) {
        int base = x * 240 + (239 - (MAP_Y + MAP_H - 1));
        memcpy(&dst[base], &src[base], MAP_H * sizeof(u16));
    }
}

static void DrawMapBase(int have, float* ps, float* pox, float* poy);

static void DrawMapInto(int have) {
    const PortMinimap* m = &gPortMinimap;
    float s, ox, oy;
    int x;
    u32 key = (u32)have * 0x9E3779B1u ^ (u32)(uintptr_t)m->tex * 31u ^ (u32)m->w * 7u ^ (u32)m->h * 13u ^
              (u32)m->r * 29u ^ (u32)m->g * 37u ^ (u32)m->b * 41u;
    if (have && m->tex != NULL && m->w > 0 && m->h > 0) { /* same buffer, new room: sample the texels */
        const u8* t = m->tex;
        int n = m->w * m->h / 2, k;
        for (k = 0; k < 64; k++) key = key * 33u + t[(k * (n / 64)) ^ 7];
    }
    if (!sMapBaseValid || key != sMapBaseKey) {
        DrawMapBase(have, &sMapBaseS, &sMapBaseOx, &sMapBaseOy);
        MapCopyColumns(sMapBase, sFb);
        sMapBaseKey = key;
        sMapBaseValid = 1;
    } else {
        MapCopyColumns(sFb, sMapBase);
    }
    sFbDirty = 1;
    if (!have || m->w <= 0 || m->h <= 0) return;
    s = sMapBaseS, ox = sMapBaseOx, oy = sMapBaseOy;
#define MX(v) (ox + ((v) - m->x) * s)
#define MY(v) (oy + ((v) - m->y) * s)
    for (x = 0; x < m->numIcons; x++) {
        int ix = (int)MX(m->iconX[x]), iy = (int)MY(m->iconY[x]);
        FillRect(ix, iy, 8, 8, PRGB(60, 30, 20));
        FillRect(ix + 1, iy + 1, 6, 6, PRGB(220, 70, 40));
    }
    for (x = 0; x < m->numMarks; x++) {
        int mx = (int)MX(m->markX[x]), my = (int)MY(m->markY[x]);
        if (m->markType[x] == 0 /* MAP_MARK_CHEST */) {
            FillRect(mx, my, 9, 7, PRGB(70, 36, 16));
            FillRect(mx + 1, my + 1, 7, 5, PRGB(210, 60, 40));
            FillRect(mx + 1, my + 3, 7, 1, PRGB(240, 200, 60));
        } else { /* boss */
            FillRect(mx, my, 9, 9, PRGB(30, 20, 20));
            FillRect(mx + 1, my + 1, 7, 7, PRGB(240, 240, 240));
            FillRect(mx + 2, my + 3, 2, 2, PRGB(200, 20, 20));
            FillRect(mx + 5, my + 3, 2, 2, PRGB(200, 20, 20));
        }
    }
    if (m->compass) {
        MapTri(MX(m->startX), MY(m->startY), m->startYaw, 7, PRGB(200, 0, 0));
        MapTri(MX(m->playerX), MY(m->playerY), m->playerYaw, 9, PRGB(40, 40, 20));
        MapTri(MX(m->playerX), MY(m->playerY), m->playerYaw, 7, PRGB(250, 240, 0));
    }
#undef MX
#undef MY
}

/* the parchment and the map texture, fitted to the frame (its scale/offset returned for the markers) */
static void DrawMapBase(int have, float* ps, float* pox, float* poy) {
    const PortMinimap* m = &gPortMinimap;
    float s, ox, oy, inv;
    int x, y, bx0, by0, bx1, by1, lum;
    *ps = 1.0f, *pox = 0.0f, *poy = 0.0f;
    /* parchment with a darker burnt edge */
    for (x = MAP_X; x < MAP_X + MAP_W; x++) {
        for (y = MAP_Y; y < MAP_Y + MAP_H; y++) {
            int e = x - MAP_X, d;
            if (MAP_X + MAP_W - 1 - x < e) e = MAP_X + MAP_W - 1 - x;
            if (y - MAP_Y < e) e = y - MAP_Y;
            if (MAP_Y + MAP_H - 1 - y < e) e = MAP_Y + MAP_H - 1 - y;
            d = e < 10 ? (10 - e) * 7 : 0;
            {
                int n = Noise(x >> 1, y >> 1) * 2;
                Px(x, y, PRGB(Clamp8(206 - d + n), Clamp8(176 - d * 5 / 4 + n), Clamp8(116 - d * 3 / 2 + n)));
            }
        }
    }
    sFbDirty = 1;
    if (!have || m->w <= 0 || m->h <= 0) return;
    /* fit the drawn part of the texture (plus the markers) to the frame: N64 maps sit in a corner of
     * their texture; shown large and centred */
    bx0 = m->w, by0 = m->h, bx1 = -1, by1 = -1;
    if (m->tex != NULL) {
        for (y = 0; y < m->h; y++) {
            for (x = 0; x < m->w; x++) {
                if (MapTexel(m, x, y, &lum) != 0) {
                    if (x < bx0) bx0 = x;
                    if (x > bx1) bx1 = x;
                    if (y < by0) by0 = y;
                    if (y > by1) by1 = y;
                }
            }
        }
    }
    if (bx1 < 0) bx0 = 0, by0 = 0, bx1 = m->w - 1, by1 = m->h - 1; /* nothing drawn: whole frame */
    bx1++, by1++;
    s = fminf((MAP_W - 16) / (float)(bx1 - bx0), (MAP_H - 16) / (float)(by1 - by0));
    if (s > 3.0f) s = 3.0f;
    ox = MAP_X + (MAP_W - (bx1 - bx0) * s) * 0.5f - bx0 * s;
    oy = MAP_Y + (MAP_H - (by1 - by0) * s) * 0.5f - by0 * s;
    inv = 1.0f / s;
    if (m->tex != NULL) {
        /* the game's colours are for a dark screen: darken them onto the parchment */
        int r = m->r * 2 / 5, g = m->g * 2 / 5, b = m->b * 2 / 5 + 70;
        for (y = MAP_Y + 2; y < MAP_Y + MAP_H - 2; y++) {
            int ty = (int)floorf((y + 0.5f - oy) * inv);
            if (ty < by0 || ty >= by1) continue;
            for (x = MAP_X + 2; x < MAP_X + MAP_W - 2; x++) {
                int tx = (int)floorf((x + 0.5f - ox) * inv), a;
                if (tx < bx0 || tx >= bx1) continue;
                a = MapTexel(m, tx, ty, &lum);
                if (a == 0) continue;
                Px(x, y, Blend565(PxGet(x, y), r * lum / 255, g * lum / 255, b * lum / 255, a));
            }
        }
    }
    *ps = s, *pox = ox, *poy = oy;
}

static void Port3ds_TouchUiInit(void) {
    bool n3ds = false;
    APT_CheckNew3DS(&n3ds);
    { extern void PortCompat_SilenceStderr(void); PortCompat_SilenceStderr(); }
    sTouchUi = 1;
    sTouchUiN3ds = n3ds;
    sTouchUiRedraw = 1; /* drawn on the first poll: renderer init (in the game loop) clears the screen */
}

/* PORT (2026-10-04): taps between updates. Input is read once per update; at low update rates (the Old 3DS, a heavy
 * scene) a quick tap on a touch pad could start and end between two reads and be lost (hardware v52 at ~11 updates
 * per second: the SCREEN pad "did nothing" on the title screen). The game thread also reads the buttons at each audio
 * pump (once per retrace, Port3ds_ScanBetweenUpdates); a touch or ZR press seen there is kept, with where the touch
 * landed, until the next update's touch panel poll. The game's own controller still reads the buttons once per
 * update, as on the N64. */
static u32 sTapDownAcc;      /* KEY_TOUCH / KEY_ZR presses since the last update's poll */
static touchPosition sTapPos; /* where that touch landed */
void Port3ds_ScanBetweenUpdates(void) {
    u32 d;
    if (Port3ds_OnRenderThread()) {
        return;
    }
    hidScanInput();
    d = hidKeysDown() & (KEY_TOUCH | KEY_ZR);
    if ((d & KEY_TOUCH) && !(sTapDownAcc & KEY_TOUCH)) {
        hidTouchRead(&sTapPos);
    }
    sTapDownAcc |= d;
}

/* held pads -> N64 bits; tap actions; redraws what changed */
static unsigned short Port3ds_TouchUiPoll(void) {
    static unsigned sPolls;
    int hit = -1, i, iconsOk;
    PortHudInfo now;
    if (!sTouchUi) return 0;
    sPolls++;
    sFb = (u16*)gfxGetFramebuffer(GFX_BOTTOM, GFX_LEFT, NULL, NULL);
    if (gPortTouchOcarina > 0) gPortTouchOcarina--; /* requests expire if the player could not act */
    if (gPortTouchBoots > 0) gPortTouchBoots--;

    /* freshness of the game-side data */
    sHudStale = (gPortHudSerial != sLastHudSerial) ? 0 : sHudStale + 1;
    sLastHudSerial = gPortHudSerial;
    sMapStale = (gPortMinimap.serial != sLastMapSerial) ? 0 : sMapStale + 1;
    sLastMapSerial = gPortMinimap.serial;

    Port_GetHudInfo(&now);
    if (sHudStale >= 4) now.keys = -1; /* the HUD is not being drawn (title, file select) */

    if (sTouchUiRedraw) {
        sTouchUiRedraw = 0;
        FillStone(0, 0, 320, 240, 40, 42, 46, 6);
        FillStone(MAP_X - 2, 0, MAP_W + 4, 240, 26, 22, 20, 4);
        sShown = now;
        sShownIconsOk = IconsOk();
        sShownIconSeg = gPortHudIconSeg;
        sShownWide = gPortWidescreen;
        for (i = 0; i < P_COUNT; i++) DrawPad(i);
        DrawStatus();
        DrawCounters();
        DrawMap(0);
    }

    {
        u32 down = hidKeysDown() | sTapDownAcc;
        if ((hidKeysHeld() & KEY_TOUCH) || (down & KEY_TOUCH)) {
            touchPosition tp;
            if (sTapDownAcc & KEY_TOUCH) {
                tp = sTapPos; /* the tap that started since the last poll (maybe already over) */
            } else {
                hidTouchRead(&tp);
            }
            for (i = 0; i < P_COUNT; i++) {
                const PanelPad* p = &sPads[i];
                if (tp.px >= p->x && tp.px < p->x + p->w && tp.py >= p->y && tp.py < p->y + p->h) hit = i;
            }
        }
    }
    if ((hidKeysDown() | sTapDownAcc) & KEY_TOUCH) {
        if (hit == P_SCREEN) {
            gPortWidescreen = !gPortWidescreen;
            Port3ds_SaveSettings();
        } else if (hit == P_HUD) {
            gPortHudTop = !gPortHudTop;
            Port3ds_SaveSettings();
        } else if (hit == P_OCARINA) {
            gPortTouchOcarina = 3;
        } else if (hit == P_BOOTS) {
            gPortTouchBoots = 3;
        } else if (hit >= 0 && sPads[hit].page >= 0) {
            gPortTouchPage = sPads[hit].page;
        }
    }
    /* New 3DS: ZR is the BOOTS pad (pressed look while held) */
    if ((hidKeysDown() | sTapDownAcc) & KEY_ZR) gPortTouchBoots = 3;
    sTapDownAcc = 0;
    if (hit < 0 && (hidKeysHeld() & KEY_ZR)) hit = P_BOOTS;
    if (hit != sHeldPad) {
        int old = sHeldPad;
        sHeldPad = hit;
        if (old >= 0) DrawPad(old);
        if (hit >= 0) DrawPad(hit);
    }

    iconsOk = IconsOk();
    /* C pads: item/ammo/disabled changes, and a few refreshes a second while icons may still be
     * arriving by async DMA after an equip */
    if (memcmp(now.cItem, sShown.cItem, 3) || memcmp(now.cDisabled, sShown.cDisabled, 3) ||
        memcmp(now.cAmmo, sShown.cAmmo, sizeof(now.cAmmo)) || iconsOk != sShownIconsOk ||
        (iconsOk && (sShownIconSeg != gPortHudIconSeg || (sPolls % 20) == 0))) {
        memcpy(sShown.cItem, now.cItem, 3);
        memcpy(sShown.cDisabled, now.cDisabled, 3);
        memcpy(sShown.cAmmo, now.cAmmo, sizeof(now.cAmmo));
        sShownIconsOk = iconsOk;
        sShownIconSeg = gPortHudIconSeg;
        for (i = 0; i < 3; i++) DrawPad(P_CLEFT + i);
    }
    if (now.boots != sShown.boots) sShown.boots = now.boots, DrawPad(P_BOOTS);
    if (now.ocarina != sShown.ocarina) sShown.ocarina = now.ocarina, DrawPad(P_OCARINA);
    if (gPortWidescreen != sShownWide) sShownWide = gPortWidescreen, DrawPad(P_SCREEN);
    if (gPortHudTop != sShownHud) sShownHud = gPortHudTop, DrawPad(P_HUD);
    {
        int navi = gPortHudNavi && sHudStale < 4;
        static int sNaviPulseT;
        if (navi != sShownNavi) {
            sShownNavi = navi;
            sNaviPulse = 1, sNaviPulseT = 0;
            DrawPad(P_VIEW);
        } else if (navi && ++sNaviPulseT >= 8) { /* ~0.4 s per phase */
            sNaviPulseT = 0;
            sNaviPulse ^= 1;
            DrawPad(P_VIEW);
        }
    }
    if (now.rupees != sShown.rupees || now.keys != sShown.keys) {
        sShown.rupees = now.rupees, sShown.keys = now.keys;
        DrawCounters();
    }
    if (now.valid != sShown.valid || now.health != sShown.health || now.healthCapacity != sShown.healthCapacity ||
        now.magic != sShown.magic || now.magicCapacity != sShown.magicCapacity) {
        sShown.valid = now.valid, sShown.health = now.health, sShown.healthCapacity = now.healthCapacity;
        sShown.magic = now.magic, sShown.magicCapacity = now.magicCapacity;
        DrawStatus();
    }

    { /* minimap: redrawn every other poll while the game feeds it, cleared once when it stops */
        static int sMapShown;
        static u32 sMapSig;
        if (sMapStale == 0 && (sPolls & 1)) {
            /* redraw only when something visible changed (arrows move by whole panel pixels) */
            const PortMinimap* m = &gPortMinimap;
            u32 sig = (u32)(uintptr_t)m->tex * 31u + (u32)m->w * 7u + (u32)m->h * 13u + m->compass * 17u +
                      (u32)(int)(m->playerX * 2.0f) * 101u + (u32)(int)(m->playerY * 2.0f) * 1009u +
                      (u32)(m->playerYaw >> 11) * 10007u + (u32)(int)m->startX * 3u + (u32)(int)m->startY * 5u +
                      m->numMarks * 19u + m->numIcons * 23u + (u32)m->r * 29u + (u32)m->g * 37u;
            if (m->tex != NULL && m->w > 0 && m->h > 0) { /* same buffer, new room: sample the texels */
                const u8* t = m->tex;
                int n = m->w * m->h / 2, k;
                for (k = 0; k < 64; k++) sig = sig * 33u + t[(k * (n / 64)) ^ 7];
            }
            if (!sMapShown || sig != sMapSig) {
                DrawMap(1);
                sMapSig = sig;
            }
            sMapShown = 1;
        } else if (sMapStale > 6 && sMapShown) {
            DrawMap(0);
            sMapShown = 0;
        }
    }

    if (sFbDirty) {
        sFbDirty = sFbMapDirty = 0;
        Port3ds_CacheFlush(sFb, 240 * 320 * 2);
    } else if (sFbMapDirty) { /* the minimap's redraws while Link moves: its columns only */
        sFbMapDirty = 0;
        Port3ds_CacheFlush(sFb + MAP_X * 240, MAP_W * 240 * 2);
    }

    { /* verification aid: with sdmc:/3ds/mm/capture_bottom present, dump the panel every 300 polls
       * (raw framebuffer + header, overwritten) so tools can check it without screenshots */
        if ((sPolls % 300) == 0) {
            FILE* flag = fopen("sdmc:/3ds/mm/capture_bottom", "rb");
            if (flag != NULL) {
                u16 w, h;
                u8* fb = gfxGetFramebuffer(GFX_BOTTOM, GFX_LEFT, &w, &h);
                FILE* out = fopen("sdmc:/3ds/mm/bottom_fb.bin", "wb");
                fclose(flag);
                if (out != NULL) {
                    u32 hdr[3] = { w, h, (u32)gfxGetScreenFormat(GFX_BOTTOM) };
                    fwrite(hdr, 4, 3, out);
                    fwrite(fb, 1, (size_t)w * h * gspGetBytesPerPixel(gfxGetScreenFormat(GFX_BOTTOM)), out);
                    fclose(out);
                }
            }
        }
    }
    /* the tabs press START only when opening a page from gameplay (or closing the pause menu) */
    return hit >= 0 ? sPads[hit].btn : 0;
}


s8 gPortCamX, gPortCamY;     /* C-stick camera input for this update (z_camera.c) */
int gPortMenuInput;          /* > 0 while a menu is up (set each update by the game: file select, pause) */
static void Port3ds_PollInput(void) {
    hidScanInput();
    u32 k = hidKeysHeld();
    /* PORT (2026-09-30): the button layout, complete on the Old 3DS (no ZL/ZR, no C-stick):
     *   A/B = A/B, L = Z-target, R = shield, Y/X = C-left/C-right, D-pad = C-up/C-down/C-left/C-right
     *   (the N64 D-pad is unused by OoT), SELECT = N64 L (minimap), START = START, touch panel =
     *   VIEW/C buttons/OCARINA/BOOTS/pause tabs/SCREEN + the minimap (docs/3ds-touch-panel.md).
     * New 3DS extras: C-stick = camera (settings cstick=0: the four C buttons), ZL = C-down (item slot I),
     * ZR = BOOTS (cycles the owned boots, like tapping the pad; 2026-10-03, it was a third C-up). */
    unsigned short b = 0;
    if (k & KEY_A)      b |= BTN_A_;
    if (k & KEY_B)      b |= BTN_B_;
    if (k & KEY_Y)      b |= BTN_CLEFT_;
    if (k & KEY_X)      b |= BTN_CRIGHT_;
    if (k & KEY_START)  b |= BTN_START_;
    if (k & KEY_SELECT) b |= BTN_L_;
    if (k & KEY_L)      b |= BTN_Z_;
    if (k & KEY_R)      b |= BTN_R_;
    if (k & KEY_ZL)     b |= BTN_CDOWN_;
    /* ZR: the BOOTS pad (Port3ds_TouchUiPoll); C-up stays on VIEW and D-pad up */
    /* PORT (2026-10-06, asked for): in menus (file select, the pause screens, the Bombers' Notebook:
     * gPortMenuInput, set by the game each update) the D-pad moves like the stick, as menus are navigated on the
     * N64; it presses no C button there. In gameplay, dialogue and ocarina playing it stays the C buttons. */
    int menu = gPortMenuInput > 0;
    if (gPortMenuInput > 0) gPortMenuInput--;
    if (!menu) {
        if (k & KEY_DUP)    b |= BTN_CUP_;
        if (k & KEY_DDOWN)  b |= BTN_CDOWN_;
        if (k & KEY_DLEFT)  b |= BTN_CLEFT_;
        if (k & KEY_DRIGHT) b |= BTN_CRIGHT_;
    }
    /* C-stick (New 3DS): turns the camera (z_camera.c Camera_Normal1) - PORT (2026-10-03), asked for on
     * hardware; settings cstick=0 makes it the four C buttons again (they are on Y/X/ZL/ZR/D-pad too) */
    gPortCamX = gPortCamY = 0;
    if (sCstickCamera) {
        static int sIrrst = -1;
        circlePosition cs;
        if (sIrrst < 0) sIrrst = R_SUCCEEDED(irrstInit());
        if (sIrrst) {
            irrstScanInput();
            irrstCstickRead(&cs);
            /* range about +-146; a dead zone, then -127..127 */
            gPortCamX = (s8)(abs(cs.dx) < 20 ? 0 : (cs.dx > 146 ? 127 : cs.dx < -146 ? -127 : cs.dx * 127 / 146));
            gPortCamY = (s8)(abs(cs.dy) < 20 ? 0 : (cs.dy > 146 ? 127 : cs.dy < -146 ? -127 : cs.dy * 127 / 146));
        }
    } else {
        if (k & KEY_CSTICK_UP)    b |= BTN_CUP_;
        if (k & KEY_CSTICK_DOWN)  b |= BTN_CDOWN_;
        if (k & KEY_CSTICK_LEFT)  b |= BTN_CLEFT_;
        if (k & KEY_CSTICK_RIGHT) b |= BTN_CRIGHT_;
    }
    b |= Port3ds_TouchUiPoll();

    circlePosition cp;
    hidCircleRead(&cp);
    /* circle pad range ~ +-156; scale to N64 +-80 */
    s3dsStickX = (signed char)(cp.dx * 80 / 156);
    s3dsStickY = (signed char)(cp.dy * 80 / 156);
    if (menu) { /* the D-pad as a full stick tilt (the circle pad still works) */
        if (k & KEY_DLEFT)  s3dsStickX = -80;
        if (k & KEY_DRIGHT) s3dsStickX = 80;
        if (k & KEY_DUP)    s3dsStickY = 80;
        if (k & KEY_DDOWN)  s3dsStickY = -80;
    }
    s3dsButtons = b;
}

/* render hooks — override sched_shim's weak PortGfx_RunTask (the 3DS analogue
 * of pc_gfx.c). Drives the citro3d backend via the gfx_pc interface. */
static int sGfxInited = 0;

void PortGfx_Init(void) {
    gfx_init(&gfx_3ds, &gfx_citro3d_api);
    sGfxInited = 1;
    { extern void PortDbg(const char*); PortDbg("[gfx] citro3d renderer initialized"); }
}

void PortGfx_FrameReady(void) {}

/* PORT (2026-09-24): N64-faithful pacing. OoT advances its logic once every R_UPDATE_RATE VI
 * retraces (3 -> 20 updates/s) and the N64 AudioMgr runs on EVERY retrace (60/s), independent of
 * the game. The port used to run one update per rendered frame (measured 26-29/s -> game ~1.4x too
 * fast, speed varying with scene load) and pumped audio once per update. Here: after presenting,
 * wait whole retraces until R_UPDATE_RATE retrace periods have passed since the previous update,
 * pumping audio once per retrace. A frame that renders late simply runs late (like N64 lag). */
static unsigned sPortAudioPumps = 0;
/* PORT PERF (2026-09-30): audio pumps also run DURING rendering. On the N64 the audio thread preempts the
 * game whenever a retrace arrives; here pumps only ran after the frame, so a slow frame (Old 3DS: 100 ms)
 * caught up with 5-6 back-to-back pumps, each waiting for the previous microcode task on the worker
 * core (measured 17-31 ms of main-thread waiting per frame, O3DS-sim). The display-list walker calls
 * Port3ds_MaybePumpAudio every 256 commands: one pump per elapsed retrace, so the worker's microcode
 * overlaps the rest of the frame. The display list is complete by then; audio only touches audio state. */
static double sLastPumpMs;  /* time of the last pump (any source) */
static int sMidFramePumps;  /* pumps since the previous Port3ds_PaceFrame */
#define PORT_RETRACE_MS (1000.0 / 59.83)
void Port3ds_MaybePumpAudio(void) {
    extern void Port3ds_PumpAudio(void);
    double now = (double)osGetTime();
    if (Port3ds_OnRenderThread()) {
        return; /* audio belongs to the game thread, which pumps it while it paces (render thread) */
    }
    if (sLastPumpMs != 0.0 && now - sLastPumpMs >= PORT_RETRACE_MS) {
        Port3ds_PumpAudio();
        sMidFramePumps++;
        sLastPumpMs += PORT_RETRACE_MS;
        if (now - sLastPumpMs > 4 * PORT_RETRACE_MS) sLastPumpMs = now; /* far behind: don't burst */
    }
}
/* PORT PERF (2026-10-02): pacing runs on svcGetSystemTick in milliseconds (osGetTime has whole-millisecond
 * steps: a frame finishing 50.4 ms after the update's start read as 50 < the 50.14 ms budget and waited one
 * more retrace - exposed by the present gate, whose last frame goes out just after the third vblank) */
static double Port3ds_NowMs(void) {
    return (double)svcGetSystemTick() / (SYSCLOCK_ARM11 / 1000.0);
}
static double sLast = 0.0; /* when the current update started (retrace-aligned with the present gate) */
int gPortInterpOn; /* this update shows in-between frames (z_actor.c draws actors a margin past the edges) */
/* PORT (2026-10-01): frame skip (PortGfx_RunTask). sBehindMs = how far the game is behind the N64's
 * schedule of one update every R_UPDATE_RATE retraces; kept only while frame skip is on (sSkipOn). */
static double sBehindMs;
static int sSkipOn, sSkippedLast;
static unsigned sSkipCount, sLateUpdates;
static int Port3ds_UpdateRate(void) {
    extern void* gRegEditor;
    int rate = 3;
    if (gRegEditor) rate = *(short*)((char*)gRegEditor + 0x14 + 126 * 2); /* R_UPDATE_RATE = SREG(30) */
    if (rate < 1) rate = 1;
    if (rate > 6) rate = 6;
    return rate;
}
extern void Port3ds_VBlankSeen(void); /* gfx_3ds.c: vblank clock anchor for the present gate */
/* PORT PERF (2026-10-03): with the flip presenter (gfx_3ds.c) every frame carries the vblank it is due at: the
 * update's frames are spread over its R_UPDATE_RATE vblanks starting R_UPDATE_RATE vblanks after the update's
 * start (one update of latency: the first frame has a whole update period to be drawn) */
static void Port3ds_SetSlotAt(double start, int j, int frames, int rate) {
    extern double Port3ds_VBlankAtOrBefore(double tMs);
    extern void Port3ds_SetFrameSlot(double slotMs);
    const double kRetraceMs = 1000.0 / 59.831;
    double base = Port3ds_VBlankAtOrBefore(start + 1.0);
    Port3ds_SetFrameSlot(start == 0.0 ? 0.0 : base + (rate + (rate * j + frames / 2) / frames) * kRetraceMs);
}
/* wait whole retraces (pumping audio on each) until `targetMs` is under one retrace away */
static void Port3ds_WaitUntil(double targetMs) {
    extern void Port3ds_PumpAudio(void);
    while (Port3ds_NowMs() + 2.0 < targetMs) {
        gspWaitForVBlank();
        Port3ds_VBlankSeen();
        Port3ds_PumpAudio();
        sLastPumpMs = (double)osGetTime();
        sMidFramePumps++;
    }
}
static void Port3ds_PaceFrame(void) {
    extern void* gRegEditor;
    extern void Port3ds_PumpAudio(void);
    const double kRetraceMs = 1000.0 / 59.83; /* 3DS LCD refresh */
    int rate = 3;
    if (gRegEditor) rate = *(short*)((char*)gRegEditor + 0x14 + 126 * 2); /* R_UPDATE_RATE = SREG(30) */
    if (rate < 1) rate = 1;
    if (rate > 6) rate = 6;
    int pumped = sMidFramePumps; /* already done during rendering */
    double now;
    /* with frame skip, time owed from late frames is made up by not waiting for it */
    const double budget = rate * kRetraceMs - (sSkipOn ? sBehindMs : 0.0);
    sMidFramePumps = 0;
    for (;;) {
        /* PORT (2026-09-30): check the budget BEFORE waiting. The loop used to wait for a retrace
         * first, so a frame that had already used its budget (hardware: ~44 ms of work vs 50 ms)
         * still waited up to one more retrace: measured pace 14.7 ms/frame, 17.1 updates/s on a New
         * 3DS instead of 20. A late frame now goes straight on (audio catches up below). */
        if (sLast != 0.0 && Port3ds_NowMs() - sLast >= budget) break;
        gspWaitForVBlank();
        Port3ds_VBlankSeen();
        Port3ds_PumpAudio(); /* build+dispatch one audio RSP task per retrace, as on N64 */
        sLastPumpMs = (double)osGetTime();
        pumped++;
        if (sLast == 0.0 || Port3ds_NowMs() - sLast + 2.0 >= budget) break;
    }
    now = Port3ds_NowMs();
    if (sLast != 0.0 && now - sLast > rate * kRetraceMs + kRetraceMs) {
        sLateUpdates++; /* took more than one retrace longer than the N64's schedule (loads, heavy frames) */
    }
    if (sSkipOn && sLast != 0.0) {
        /* capped: a long stall (scene load) is not made up with a burst of skipped frames */
        sBehindMs += now - sLast - rate * kRetraceMs;
        if (sBehindMs < 0.0) sBehindMs = 0.0;
        if (sBehindMs > 2.0 * rate * kRetraceMs) sBehindMs = 2.0 * rate * kRetraceMs;
    } else {
        sBehindMs = 0.0;
    }
    /* Audio runs on wall-clock retraces, not on how many waits happened: if rendering ate most of
     * the budget, catch up to one audio task per retrace actually elapsed (capped). */
    if (sLast != 0.0) {
        int due = (int)((now - sLast) / kRetraceMs + 0.5);
        if (due > 8) due = 8;
        while (pumped < due) { Port3ds_PumpAudio(); pumped++; }
        sLastPumpMs = (double)osGetTime();
    }
    sPortAudioPumps += (unsigned)pumped;
    {
        /* With the present gate, an update that ends just after a vblank (its last frame is released
         * there) starts on that vblank: its frames are shown on vblanks, so the next update's budget
         * counts from one too. Not for a long overrun (Old 3DS, loads): that would count up to a retrace
         * of lateness that did not happen (frame skip's sBehindMs). +1 ms: the clock's anchor is taken
         * after gspWaitForVBlank returns, slightly after the vblank itself. */
        extern int gPortPresentGate;
        extern double Port3ds_VBlankAtOrBefore(double tMs);
        double vb = Port3ds_VBlankAtOrBefore(now + 1.0);
        sLast = gPortPresentGate && now - vb < 3.0 ? vb : now;
    }
}

/* PORT PERF (2026-09-28): per-frame CPU breakdown in 268 MHz system ticks, logged every 300 frames as
 * average microseconds per frame: game = everything between two graph tasks (game logic + DL build),
 * dl = display-list interpretation (gfx_run), swap = frame end/GPU wait, pace = retrace waits + audio;
 * plus triangles and draw calls sent to the GPU per frame (gfx_pc.c counters). */
u32 gPortPerfTris, gPortPerfDraws, gPortPerfTrisIn;
u64 gPortPerfAudioMain;
static u64 sPerfGame, sPerfDl, sPerfSwap, sPerfPace, sPerfLastEnd;
static u64 sPerfDlWait; /* the part of sPerfDl spent waiting: the walk's C3D_FrameBegin (gPortPerfGpuWait also
                         * counts the in-between frames' waits, which fall in "swap") and its present gate */
/* PORT PERF (2026-10-04): memory probe, once per perf report (~1 ms). The Old 3DS profile spends far more on writing
 * the vertex and GPU command buffers (linear memory) than its clock explains (hardware v49 "gpu pack": 12-17% of the
 * time at Old 3DS speed, 1% at New 3DS speed). Logged: ticks per 32-byte line (ARM11 ticks, 268 MHz) to write 64 KB
 * with word stores and with 32-byte block copies, to read it, and to flush it, for linear memory and for the heap.
 * Meaningful on hardware only (Azahar models no caches). */
static void Port3ds_MemProbe(void) {
    extern void PortDbgX(const char*, unsigned);
    enum { N = 64 * 1024, LINES = N / 32 };
    static u32* sLin;
    static u32 sHeapBuf[N / 4] __attribute__((aligned(32))); /* (the newlib heap has almost nothing free) */
    u32* sHeap = sHeapBuf;
    static const u32 src[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
    u32* bufs[2];
    static const char* const names[2][4] = {
        { "perf mem probe linear: word stores ticks/line", "perf mem probe linear: block stores ticks/line",
          "perf mem probe linear: word loads ticks/line", "perf mem probe linear: flush ticks/line" },
        { "perf mem probe heap: word stores ticks/line", "perf mem probe heap: block stores ticks/line",
          "perf mem probe heap: word loads ticks/line", "perf mem probe heap: flush ticks/line" } };
    int b, i;
    if (sLin == NULL) sLin = (u32*)linearAlloc(N);
    if (sLin == NULL) return;
    bufs[0] = sLin, bufs[1] = sHeap;
    for (b = 0; b < 2; b++) {
        volatile u32* v = bufs[b];
        u64 t[5];
        u32 sum = 0;
        GSPGPU_FlushDataCache(bufs[1 - b], N); /* (touch the other buffer: this one's lines leave the L1) */
        for (i = 0; i < N / 4; i += 8) sum += ((volatile u32*)bufs[1 - b])[i];
        t[0] = svcGetSystemTick();
        for (i = 0; i < N / 4; i++) v[i] = (u32)i;
        t[1] = svcGetSystemTick();
        for (i = 0; i < N / 4; i += 8) memcpy(&bufs[b][i], src, 32);
        t[2] = svcGetSystemTick();
        for (i = 0; i < N / 4; i += 8) sum += v[i];
        t[3] = svcGetSystemTick();
        GSPGPU_FlushDataCache(bufs[b], N);
        t[4] = svcGetSystemTick();
        for (i = 0; i < 4; i++) PortDbgX(names[b][i], (unsigned)((t[i + 1] - t[i]) / LINES));
        if (sum == 0x12345678u) PortDbgX("perf mem probe (sum)", sum); /* (keeps the loads) */
    }
}

static void Port3ds_PerfReport(unsigned frames) {
    extern void PortDbgX(const char*, unsigned);
    const u64 div = (u64)frames * (SYSCLOCK_ARM11 / 1000000); /* ticks -> us per frame */
    PortDbgX("perf us/frame game", (unsigned)(sPerfGame / div));
    {
        extern u64 gPortPerfGpuWait;
        PortDbgX("perf us/frame dl (cpu)", (unsigned)((sPerfDl - sPerfDlWait) / div));
        PortDbgX("perf us/frame gpu wait", (unsigned)(gPortPerfGpuWait / div));
        gPortPerfGpuWait = 0;
    }
    {
        extern u64 gPortPerfTex, gPortPerfVtx, gPortPerfTri, gPortPerfFlush;
        extern u32 gPortPerfTexImports;
        PortDbgX("perf us/frame  dl.tex", (unsigned)(gPortPerfTex / div));
        PortDbgX("perf us/frame  dl.vtx", (unsigned)(gPortPerfVtx / div));
        PortDbgX("perf us/frame  dl.tri", (unsigned)(gPortPerfTri / div));
        PortDbgX("perf us/frame  dl.flush", (unsigned)(gPortPerfFlush / div));
        {
            extern u64 gPortPerfEmit, gPortPerfMtx;
            PortDbgX("perf us/frame  dl.tri.emit (clip+pack)", (unsigned)(gPortPerfEmit / div));
            PortDbgX("perf us/frame  dl.mtx", (unsigned)(gPortPerfMtx / div));
            gPortPerfEmit = gPortPerfMtx = 0;
        }
        PortDbgX("perf tex imports/frame", gPortPerfTexImports / frames);
        gPortPerfTex = gPortPerfVtx = gPortPerfTri = gPortPerfFlush = 0;
        gPortPerfTexImports = 0;
    }
    PortDbgX("perf us/frame swap", (unsigned)(sPerfSwap / div));
    PortDbgX("perf us/frame pace", (unsigned)(sPerfPace / div));
    {
        extern u64 gPortPerfAudioUcode, gPortPerfAudioWait, gPortPerfAudioMain;
        extern u32 gPortPerfAudioTasks;
        PortDbgX("perf us/frame  audio main (engine+wait)", (unsigned)(gPortPerfAudioMain / div));
        PortDbgX("perf us/frame  audio wait for ucode", (unsigned)(gPortPerfAudioWait / div));
        PortDbgX("perf us/task   audio ucode (worker)",
                 gPortPerfAudioTasks ? (unsigned)(gPortPerfAudioUcode / gPortPerfAudioTasks / (SYSCLOCK_ARM11 / 1000000)) : 0);
        gPortPerfAudioUcode = gPortPerfAudioWait = gPortPerfAudioMain = 0;
        { /* perf_stages: the 5 most expensive microcode ops, as op << 20 | us per frame */
            extern u64 gPortPerfAudOpTicks[32];
            extern int gPortPerfStagesOn;
            int k, i;
            for (k = 0; k < 5 && gPortPerfStagesOn; k++) {
                int best = 0;
                for (i = 1; i < 32; i++) if (gPortPerfAudOpTicks[i] > gPortPerfAudOpTicks[best]) best = i;
                if (gPortPerfAudOpTicks[best] == 0) break;
                PortDbgX("perf audio op<<20|us/frame", ((unsigned)best << 20) | (unsigned)(gPortPerfAudOpTicks[best] / div));
                gPortPerfAudOpTicks[best] = 0;
            }
            memset(gPortPerfAudOpTicks, 0, sizeof(gPortPerfAudOpTicks));
        }
        gPortPerfAudioTasks = 0;
    }
    PortDbgX("perf tris/frame", gPortPerfTris / frames);
    PortDbgX("perf tris in/frame (before clip+subdivision)", gPortPerfTrisIn / frames);
    { extern u32 gPortPerfSlowTris; PortDbgX("perf tris slow-path/frame", gPortPerfSlowTris / frames); gPortPerfSlowTris = 0; }
    gPortPerfTrisIn = 0;
    { /* memory budget (Old 3DS target: 96MB mode): linear free, regular heap in use (KB) */
        extern u32 linearSpaceFree(void);
        extern char* fake_heap_start;
        extern char* fake_heap_end;
        struct mallinfo mi = mallinfo();
        static u32 sMinLinearFree = 0xFFFFFFFF;
        u32 lf = linearSpaceFree();
        if (lf < sMinLinearFree) sMinLinearFree = lf;
        PortDbgX("mem linear free KB", lf / 1024);
        PortDbgX("mem linear free min KB", sMinLinearFree / 1024);
        PortDbgX("mem heap used KB", (unsigned)mi.uordblks / 1024);
        PortDbgX("mem heap size KB", (unsigned)(fake_heap_end - fake_heap_start) / 1024);
    }
    PortDbgX("perf draws/frame", gPortPerfDraws / frames);
    { extern u32 gPortPerfRawTris; extern int gPortRawVtx; PortDbgX("perf raw vertex path", (unsigned)gPortRawVtx);
      PortDbgX("perf raw tris/frame", gPortPerfRawTris / frames); gPortPerfRawTris = 0;
      { extern u32 gPortPerfRawMaterialized; PortDbgX("perf raw vertices lit on the CPU (mixed tris)/frame", gPortPerfRawMaterialized / frames);
        gPortPerfRawMaterialized = 0; }
      { extern int C3Df_SwitchWords(int t); /* c3d_fast.c: words per switch to GPU << 16 | to raw; 0 = plain citro3d */
        PortDbgX("perf raw switch words gpu<<16|raw", ((unsigned)C3Df_SwitchWords(0) << 16) | (unsigned)C3Df_SwitchWords(1)); }
      { extern u32 gPortPerfRawSwitches; PortDbgX("perf raw program switches/frame", gPortPerfRawSwitches / frames);
        gPortPerfRawSwitches = 0; } }
    { extern u32 gPortPerfRoomTris; PortDbgX("perf room tris in/frame", gPortPerfRoomTris / frames); gPortPerfRoomTris = 0; }
#ifdef PORT_ACTOR_PROF
    { /* the 10 actor types whose drawing took longest (us per update), with their draw calls per update */
        extern u64 gPortActorTicks[512];
        extern u32 gPortActorCalls[512];
        int n, i;
        for (n = 0; n < 10; n++) {
            int best = -1;
            for (i = 0; i < 512; i++) if (gPortActorTicks[i] && (best < 0 || gPortActorTicks[i] > gPortActorTicks[best])) best = i;
            if (best < 0) break;
            PortDbgX("perf actor id<<16|us per update", ((unsigned)best << 16) |
                     (unsigned)(gPortActorTicks[best] / (SYSCLOCK_ARM11 / 1000000) / frames));
            PortDbgX("perf actor id<<16|calls x10 per update", ((unsigned)best << 16) | (gPortActorCalls[best] * 10 / frames));
            gPortActorTicks[best] = 0;
        }
        memset(gPortActorTicks, 0, sizeof(gPortActorTicks));
        memset(gPortActorCalls, 0, sizeof(gPortActorCalls));
    }
#endif
    { extern u64 gPortPerfRoomTicks; PortDbgX("perf us/frame in room geometry", (unsigned)(gPortPerfRoomTicks / (SYSCLOCK_ARM11 / 1000000) / frames));
      gPortPerfRoomTicks = 0; }
    { extern u32 gPortPerfAudioUnderruns, gPortPerfAudioDrops; extern int gPortAudioMargin, gPortAudioCoreNow;
      PortDbgX("perf audio underruns (DSP ran dry)", gPortPerfAudioUnderruns);
      PortDbgX("perf audio buffers dropped (queue full)", gPortPerfAudioDrops);
      { extern int gPortRawRelax; PortDbgX("perf speed rules active (raw relax, no shade split)", (unsigned)gPortRawRelax); }
      PortDbgX("perf audio margin samples", (unsigned)(gPortAudioMargin >= 0 ? gPortAudioMargin : gPortAudioCoreNow == 1 ? 512 : 0));
      gPortPerfAudioUnderruns = gPortPerfAudioDrops = 0; }
#ifdef PORT_PERF_STAGES
    { extern u32 gPortRawWhy[8]; static const char* const why[8] = { "perf raw-off vtx near/frame", "perf raw-off vtx deep/frame",
          "perf raw-off vtx lights/frame", "perf raw-off vtx skinned/frame", "perf raw-off vtx texgen/frame",
          "perf raw-off vtx identity/frame", "perf raw-off vtx range/frame", "perf raw vtx/frame" };
      int i; for (i = 0; i < 8; i++) { PortDbgX(why[i], gPortRawWhy[i] / frames); gPortRawWhy[i] = 0; } }
#endif
    { extern u32 gPortPerfReplayCopies, gPortPerfReplayCopyWords; extern int gPortReplayCopy;
      PortDbgX("perf replay copy on", (unsigned)gPortReplayCopy);
      PortDbgX("perf replay frames copied", gPortPerfReplayCopies);
      PortDbgX("perf replay copy words/frame", gPortPerfReplayCopies ? gPortPerfReplayCopyWords / gPortPerfReplayCopies : 0);
      gPortPerfReplayCopies = gPortPerfReplayCopyWords = 0;
      { extern u32 gPortPerfCopyChecked, gPortPerfCopyBad, gPortCopyBadInfo[6];
        if (gPortPerfCopyChecked) {
            PortDbgX("perf replay copy checked frames", gPortPerfCopyChecked);
            PortDbgX("perf replay copy MISMATCH frames", gPortPerfCopyBad);
            if (gPortPerfCopyBad) {
                PortDbgX("perf replay copy first mismatch at word", gPortCopyBadInfo[0]);
                PortDbgX("perf replay copy first mismatch log word", gPortCopyBadInfo[1]);
                PortDbgX("perf replay copy first mismatch copy word", gPortCopyBadInfo[2]);
                PortDbgX("perf replay copy lengths/4 log<<16|copy", gPortCopyBadInfo[3]);
                PortDbgX("perf replay copy mismatch register<<16|param", gPortCopyBadInfo[4]);
                PortDbgX("perf replay copy mismatch last uniform index", gPortCopyBadInfo[5]);

            }
            gPortPerfCopyChecked = gPortPerfCopyBad = 0;
        } } }
    { extern u32 gPortGpuRoute[4]; PortDbgX("perf gpu tris tested", gPortGpuRoute[0] / frames); PortDbgX("perf gpu tris behind eye", gPortGpuRoute[1] / frames); PortDbgX("perf gpu tris near", gPortGpuRoute[2] / frames); PortDbgX("perf gpu tris to cpu", gPortGpuRoute[3] / frames); gPortGpuRoute[0] = gPortGpuRoute[1] = gPortGpuRoute[2] = gPortGpuRoute[3] = 0; }
    { /* citro3d state calls per frame (each marks state citro3d re-sends with the next draw) */
        extern u32 gPortC3dCalls[6];
        static const char* const n[6] = { "perf c3d shader loads/frame", "perf c3d const sets/frame",
                                          "perf c3d tex binds same/frame", "perf c3d tex binds new/frame",
                                          "perf c3d sampler sets/frame", "perf c3d tex uploads/frame" };
        int q;
        for (q = 0; q < 6; q++) { PortDbgX(n[q], gPortC3dCalls[q] / frames); gPortC3dCalls[q] = 0; }
    }
    {
        extern u32 gPortPerfMemQueries, gPortPerfMemHits, gPortPerfDlCmds, gPortPerfDlCalls;
        PortDbgX("perf mem queries/frame (svc)", gPortPerfMemQueries / frames);
        PortDbgX("perf mem query cache hits/frame", gPortPerfMemHits / frames);
        PortDbgX("perf dl commands/frame", gPortPerfDlCmds / frames);
        PortDbgX("perf dl G_DL calls/frame", gPortPerfDlCalls / frames);
        gPortPerfMemQueries = gPortPerfMemHits = gPortPerfDlCmds = gPortPerfDlCalls = 0;
        { /* the 8 most frequent opcodes: opcode << 16 | count per frame */
            extern u32 gPortPerfOpCounts[256];
            int k, i;
            for (k = 0; k < 8; k++) {
                int best = -1;
                for (i = 0; i < 256; i++) if (best < 0 || gPortPerfOpCounts[i] > gPortPerfOpCounts[best]) best = i;
                if (gPortPerfOpCounts[best] == 0) break;
                PortDbgX("perf dl top opcode<<16|count", ((unsigned)best << 16) | (gPortPerfOpCounts[best] / frames));
                gPortPerfOpCounts[best] = 0;
            }
            memset(gPortPerfOpCounts, 0, sizeof(gPortPerfOpCounts));
        }
        { /* perf_stages: the 8 most expensive opcodes, opcode << 20 | us per frame */
            extern u64 gPortPerfOpTicks[256];
            extern int gPortPerfStagesOn;
            int k, i;
            for (k = 0; k < 8 && gPortPerfStagesOn; k++) {
                int best = 0;
                for (i = 1; i < 256; i++) if (gPortPerfOpTicks[i] > gPortPerfOpTicks[best]) best = i;
                if (gPortPerfOpTicks[best] == 0) break;
                PortDbgX("perf dl op<<20|us/frame", ((unsigned)best << 20) | (unsigned)(gPortPerfOpTicks[best] / div));
                gPortPerfOpTicks[best] = 0;
            }
            memset(gPortPerfOpTicks, 0, sizeof(gPortPerfOpTicks));
        }
    }
    if (sProfOn) { /* sampling profiler: per-mille of samples per stage */
        static const char* const names[PROF_COUNT] = {
            "prof game", "prof dl walk", "prof vtx", "prof tri setup", "prof tri build", "prof tri emit",
            "prof tex", "prof rect", "prof mtx", "prof flush", "prof audio", "prof pace", "prof swap",
            "prof gpu wait", "prof replay", "prof input", "prof vtx.light", "prof submit", "prof tri split",
            "prof gpu palette", "prof gpu pack", "prof vtx.box", "prof vtx.raw", "prof raw emit",
            "prof c3d context", "prof c3d uniforms", "prof c3d draw" };
        u32 tot = 0;
        int i;
        for (i = 0; i < PROF_COUNT; i++) tot += sProfHist[i];
        for (i = 0; i < PROF_COUNT && tot; i++) {
            PortDbgX(names[i], (unsigned)((u64)sProfHist[i] * 1000 / tot));
            sProfHist[i] = 0;
        }
    }
    sPerfGame = sPerfDl = sPerfSwap = sPerfPace = sPerfDlWait = 0;
    gPortPerfTris = gPortPerfDraws = 0;
}

static double sWalkMs = 20.0;   /* measured: the recording walk + its shown frame, ms */
static double sReplayMs = 4.0;  /* measured: one replay frame, ms */
static int sReplayN;            /* in-between frames this logic frame */
static unsigned sReplayBrokenCnt;
/* per report: updates by in-between frames chosen (0/1/2), and updates without the choice (rate 1 menus,
 * a capture or readback pending, right after a skipped frame) - the averages alone mix scene loads with
 * steady play */
static unsigned sReplayHist[3], sReplayNoChoice;
static unsigned sReplayDropped; /* in-between frames dropped because the update's first frame was late */
static u64 sPerfInterp;             /* ticks in in-between passes */
static u32 sInterpFrames;           /* in-between frames drawn */
static double sInterpElapsedSum;    /* logic time before the passes, ms (summed per report) */
/* PORT PERF (2026-10-03): the render thread (settings render_thread=1). The N64 drew frame N on the RCP while its CPU
 * ran the logic of frame N+1, and OoT is built for it: display lists live in double-buffered pools and the game waits
 * for the previous task before reusing one. The port did both on one core, one after the other. With the thread, the
 * game thread hands each update's display list over (RenderJob) and goes on; the render thread (the New 3DS's core 2,
 * the Old 3DS's core 1) walks it and draws the in-between frames. One update in flight: the next hand-over waits until
 * the renderer is idle, so a pool is never rebuilt while it is read. Requests the game makes while building a frame
 * are latched into its job (Port3ds_TakeFrameRequests); texture-cache invalidations from the game thread are queued
 * (gfx_pc.c); the game reads depth only from copies the renderer queued (gfx_3ds.c); audio is pumped only by the game
 * thread; HOME / sleep events run on the game thread while the renderer is idle. */
typedef struct {
    Gfx* dl;
    int skip, interp, chose, replayN, rate, skipOn;
    double base;      /* sLast when the update started (its frames' vblank slots) */
    void* capture;    /* latched game requests */
    int flat;
    float focusW;
} RenderJob;
static int sRenderThreadOn;      /* the thread runs */
static int sRenderThreadUse;     /* jobs go to it (render_thread_ab alternates this) */
static int sRenderStarted;       /* Port3ds_RenderThreadStart ran */
static int sRenderCore = 2;      /* the drawing thread's core (New 3DS) */
static volatile int sRenderPhase; /* where the render thread is (the watchdog in Port3ds_RenderWaitIdle logs it) */
static Thread sRenderThread;
static LightEvent sRenderGo, sRenderIdle;
static RenderJob sRenderJob;
static volatile int sRenderBusy;
static u64 sPerfRenderWait;      /* game thread ticks waiting for the renderer */
static void Port3ds_SetSlotAt(double base, int j, int frames, int rate);

int Port3ds_OnRenderThread(void) {
    return sRenderThreadOn && threadGetCurrent() == sRenderThread;
}
void Port3ds_RenderPhase(int phase) {
    sRenderPhase = phase;
}
int Port3ds_RenderPhaseGet(void) {
    return sRenderPhase;
}

static void Port3ds_RenderWaitIdle(void) {
    u64 t0;
    if (!sRenderThreadOn || !sRenderBusy) {
        return;
    }
    t0 = svcGetSystemTick();
    while (sRenderBusy) {
        /* audio keeps its retrace pace while the game waits for a slow frame (Old 3DS: the drawing core is capped);
         * watchdog: a renderer that does not come back within 2 s is logged once with where it is (sRenderPhase,
         * gfx_3ds.c phases), so a freeze on hardware leaves its place in boot.log */
        if (LightEvent_WaitTimeout(&sRenderIdle, 4000000LL) != 0 && sRenderBusy) {
            static int sLogged;
            Port3ds_MaybePumpAudio();
            if (!sLogged && svcGetSystemTick() - t0 > 2ull * SYSCLOCK_ARM11) {
                extern void PortDbgX(const char* label, unsigned val);
                extern void Port3ds_LogFlush(void);
                sLogged = 1;
                PortDbgX("[render] STUCK: the renderer has not finished for 2 s, phase", (unsigned)sRenderPhase);
                Port3ds_LogFlush();
            }
        }
    }
    sPerfRenderWait += svcGetSystemTick() - t0;
}

/* one update's drawing: the walk (the logic frame, or its first in-between frame) and the in-between frames */
static void Port3ds_RenderJob(const RenderJob* j) {
    extern u64 gPortGateTicksTotal;
    extern u64 gPortPerfGpuWait;
    extern void Port3ds_ApplyFrameRequests(void* capture, int flat, float focusW);
    const u64 gateA = gPortGateTicksTotal, waitA = gPortPerfGpuWait;
    const double kRetraceMs = 1000.0 / 59.83;
    u64 tA = svcGetSystemTick(), tB, tC;
    int replayN = j->replayN, i;
    Port3ds_ApplyFrameRequests(j->capture, j->flat, j->focusW);
    sRenderPhase = 1; /* job start */
    if (!j->skip) {
        extern void gfx_interp_pass(float t, int record);
        if (!j->interp) { /* the tag bookkeeping still resets every frame (gfx_pc.c interp_group) */
            gfx_interp_pass(1.0f, 0);
        } else {
            extern void gfx_interp_begin_frame(void);
            extern void gfx_replay_begin(void);
            extern int gPortReplayFirstK;
            gfx_interp_begin_frame();
            gfx_interp_pass(1.0f, 1);
            if (j->chose) {
                if (replayN == 0 && j->skipOn) {
                    /* PORT PERF (2026-10-02): no in-between frame fits (always the case on an Old 3DS: 0 of 300
                     * updates in every Old-3DS-speed report, hardware v30). Recording the frame for replays (the
                     * draw log, the t = 1/3 and 2/3 matrix stacks, the tagged-matrix matching) would be pure cost:
                     * draw it directly. The next frame then has nothing to blend from, which only matters on a
                     * console that can afford in-between frames - frame skip is off there. */
                    gfx_interp_pass(1.0f, 0);
                } else {
                    gfx_replay_begin();
                    gPortReplayFirstK = 2 - replayN; /* 2 extra: 1/3 first; 1: 2/3 first; 0: the logic frame */
                }
            }
        }
        {
            extern int gPortFramesFollow;
            gPortFramesFollow = (j->interp && replayN > 0) ? replayN : 0; /* gfx_3ds.c overlap: frames after the walk */
        }
        Port3ds_SetSlotAt(j->base, 0, (j->interp && replayN > 0) ? replayN + 1 : 1, j->rate);
        sRenderPhase = 2; /* frame start (stereo switch, C3D_FrameBegin) */
        gfx_start_frame();
        sRenderPhase = 3; /* the walk */
        gfx_run(j->dl);
    }
    tB = svcGetSystemTick();
    {
        /* the walk's waits (its C3D_FrameBegin, its present gate): not display-list cost */
        const u64 dlWait = (gPortPerfGpuWait >= waitA ? gPortPerfGpuWait - waitA : 0) + (gPortGateTicksTotal - gateA);
        sPerfDl += tB - tA;
        sPerfDlWait += dlWait < tB - tA ? dlWait : tB - tA;
    }
    if (sBench && !j->skip) {
        extern int gPortLegacyVbo;
        sBenchDl[gPortLegacyVbo] += tB - tA;
        sBenchFrames[gPortLegacyVbo]++;
    }
    if (!j->skip) {
        gfx_end_frame();
    }
    tC = svcGetSystemTick();
    if (j->interp && !j->skip) {
        extern int gPortReplayBroken;
        extern void gfx_replay_frame(int k);
        int rate = j->rate;
        double ms = (double)(tC - tA - (gPortGateTicksTotal - gateA)) / (SYSCLOCK_ARM11 / 1000.0);
        sWalkMs += (ms - sWalkMs) * 0.25;
        if (j->chose && rate == 3 && !j->skipOn && !sRenderThreadUse) {
            /* the logic frame's deadline: done on the GPU 0.5 ms before the second vblank of the update */
            extern float gPortGpuDrawMsAvg;
            double done = Port3ds_NowMs() - j->base + gPortGpuDrawMsAvg;
            sDeadlineMarginMs += ((2.0 * kRetraceMs - 0.5 - done) - sDeadlineMarginMs) * 0.2;
            if (sDeadlineMarginMs < 1.5) {
                sCoarseAuto = 1;
            } else if (sDeadlineMarginMs > 6.0) { /* coarse saves ~3 ms: no flip-flopping */
                sCoarseAuto = 0;
            }
            sWalkDoneSum += done;
            sWalkDoneN++;
        }
        if (replayN > 0 && gPortReplayBroken) {
            replayN = 0; /* drawn directly (an off-screen render, a full buffer): no in-between frames */
            sReplayBrokenCnt++;
        }
        {
            /* PORT PERF (2026-10-03): the update's frames must end on its third vblank, or the next update starts
             * a retrace late and the game slows (hardware v41: 18.5-19.2 updates/s, the logic frame ~30-33 ms
             * after the update's start against a ~33 ms deadline). The frame just submitted is the update's
             * first; each retrace it reaches the screen past the second vblank costs an in-between frame - the
             * t = 2/3 one first (the logic frame, t = 1, always shows) - instead of a retrace of game time. */
            extern int gPortPresentGate;
            extern double Port3ds_PredictShownMs(void);
            extern int Port3ds_FlipActive(void);
            double shown = Port3ds_PredictShownMs();
            if (gPortPresentGate && !Port3ds_FlipActive() && replayN > 0 && shown > 0.0 && rate == 3) {
                int late = (int)((shown - (j->base + 2.0 * kRetraceMs) + 1.5) / kRetraceMs);
                if (late > 0) {
                    sReplayDropped += late < replayN ? late : replayN;
                    replayN = late < replayN ? replayN - late : 0;
                }
            }
        }
        for (i = 0; i < replayN; i++) {
            u64 t0;
            extern int Port3ds_FlipActive(void);
            PROF_SET(PROF_PACE);
            if (Port3ds_FlipActive()) {
                Port3ds_SetSlotAt(j->base, i + 1, replayN + 1, rate); /* drawn now, shown at its vblank */
            } else {
                Port3ds_WaitUntil(j->base + (rate * kRetraceMs) * (i + 1) / (replayN + 1));
            }
            u64 gateR = gPortGateTicksTotal;
            t0 = svcGetSystemTick();
            { extern int gPortFramesFollow; gPortFramesFollow = replayN - 1 - i; }
            sRenderPhase = 10 + i; /* in-between frame i */
            gfx_replay_frame(2 - replayN + 1 + i);
            gfx_end_frame();
            ms = (double)(svcGetSystemTick() - t0 - (gPortGateTicksTotal - gateR)) / (SYSCLOCK_ARM11 / 1000.0);
            sReplayMs += (ms - sReplayMs) * 0.25;
            sPerfInterp += svcGetSystemTick() - t0;
            sInterpFrames++;
        }
    }
    {
        extern void Port3ds_EndUpdateFrames(void);
        extern int gPortFramesFollow;
        gPortFramesFollow = 0;
        sRenderPhase = 20; /* closing the update */
        Port3ds_EndUpdateFrames(); /* gfx_3ds.c overlap: a citro3d frame still open (planned frames not drawn) ends */
    }
    sRenderPhase = 0;
    sPerfSwap += svcGetSystemTick() - tB;
}

static void Port3ds_RenderThreadMain(void* arg) {
    (void)arg;
    for (;;) {
        RenderJob job;
        LightEvent_Wait(&sRenderGo);
        if (!sRenderBusy) {
            continue;
        }
        __sync_synchronize();
        job = sRenderJob;
        Port3ds_RenderJob(&job);
        __sync_synchronize();
        sRenderBusy = 0;
        LightEvent_Signal(&sRenderIdle);
    }
}

static void Port3ds_RenderThreadStart(void) {
    extern void PortDbg(const char* s);
    extern void PortDbgX(const char* label, unsigned val);
    extern int gPortRenderThreaded, gPortEventsExternal;
    extern int Port3ds_FlipActive(void);
    s32 prio = 0x30;
    bool n3ds = false;
    if (!(sRenderThreadSetting || sRenderThreadAB) || sRenderStarted || !Port3ds_FlipActive()) {
        return; /* (the present gate's waits pump audio: the thread needs the flip presenter) */
    }
    sRenderStarted = 1;
    APT_CheckNew3DS(&n3ds);
    (void)n3ds;
    LightEvent_Init(&sRenderGo, RESET_ONESHOT);
    LightEvent_Init(&sRenderIdle, RESET_ONESHOT);
    svcGetThreadPriority(&prio, CUR_THREAD_HANDLE);
    /* the main thread's stack size (port/oot.rsf StackSize): the walk recurses through nested display lists */
    sRenderThread = threadCreate(Port3ds_RenderThreadMain, NULL, 256 * 1024, prio, sRenderCore, false);
    if (sRenderThread == NULL) {
        PortDbg("[render] thread: not created, drawing on the game thread");
        return;
    }
    sRenderThreadOn = 1;
    sRenderThreadUse = 1;
    gPortRenderThreaded = 1;
    gPortEventsExternal = 1;
    PortDbgX("[render] thread on core", (unsigned)sRenderCore);
}

void PortGfx_RunTask(OSTask* task) {
    u64 tA = svcGetSystemTick(), tC, tD;
#ifdef PORT_MM_NODRAW /* (bring-up: run the game without drawing, to isolate GPU faults) */
    { extern void PortDbgX(const char* label, unsigned val); static unsigned sN; if (sN++ < 3) PortDbgX("[nodraw] frame", sN); }
    return;
#endif
#ifdef PORT_LOG_SYNC
    { extern void PortDbgX(const char* label, unsigned val); static unsigned sF; if (sF < 40) PortDbgX("[gfx] frame", ++sF); }
#endif
    PROF_SET(PROF_INPUT);
    if (sPerfLastEnd != 0) sPerfGame += tA - sPerfLastEnd;
    Port3ds_RenderWaitIdle(); /* one update in flight (render thread) */
    if (!sGfxInited) {
        PortGfx_Init();
    }
    if ((sRenderThreadSetting || sRenderThreadAB) && !sRenderStarted && sGfxInited) {
        Port3ds_RenderThreadStart(); /* after the first frames: graphics and the flip presenter are up */
    }
    if (sRenderThreadUse) {
        extern void gfx_handle_events(void);
        gfx_handle_events(); /* HOME / sleep / exit: here, while the renderer is idle */
    }
    Port3ds_PollInput();
    { extern void Port3ds_PumpInput(void); Port3ds_PumpInput(); } /* live buttons -> game PadMgr */
    /* PORT MM TODO (audio bring-up): OoT needed Audio_PortEnsureNullChannels() here (an OoT game-side patch) */
    { extern void PortSram_Tick(void); PortSram_Tick(); } /* a save reaches the SD card 0.5 s after the game wrote it */
    /* PORT (2026-10-01): frame skip. On the N64 a frame that takes too long slows the game down (lag). The
     * Old 3DS spends about 80% of an update drawing it (hardware profile: ~98 ms per update, the N64 takes
     * 50 ms), so the game ran at about half speed. While the game is more than half an update behind
     * schedule, an update is not drawn: its logic and audio still run and the screen keeps the previous
     * frame. Never two in a row (at least 10 frames per second), never a frame the game reads back (pause
     * background, transitions), and no in-between frames right after a skip (the positions they blend from
     * would be two updates old). */
    int skip = 0, afterSkip = sSkippedLast;
    {
        extern int gPortO3dsSim;
        extern int Port3ds_InterpBlocked(void);
        static int sN3ds = -1;
        if (sN3ds < 0) {
            bool n3ds = false;
            APT_CheckNew3DS(&n3ds);
            sN3ds = n3ds;
        }
        sSkipOn = sFrameSkip > 0 || (sFrameSkip < 0 && (!sN3ds || gPortO3dsSim));
        {
            /* PORT PERF (2026-10-02): coarser N64-shading splits while CPU-bound (gfx_pc.c) */
            extern void gfx_split_thresholds(int coarse);
            static int sCoarse = -1;
            int coarse = sSkipOn || (sSplitAuto && sCoarseAuto);
            if (!sSplitSet && sCoarse != coarse) {
                gfx_split_thresholds(coarse);
                sCoarse = coarse;
            }
            sCoarseUpdates += !sSplitSet && coarse;
            { extern int gPortRawRelax; gPortRawRelax = sSpeedRules == 2 || (sSpeedRules == 1 && sSkipOn); } /* (gfx_pc.c) */
        }
        skip = sSkipOn && !sSkippedLast && sBehindMs > 0.5 * Port3ds_UpdateRate() * (1000.0 / 59.83) &&
               !Port3ds_InterpBlocked();
        sSkippedLast = skip;
        sSkipCount += skip;
    }
    if (sBench) {
        extern int gPortLegacyVbo;
        static unsigned sBenchTick;
        gPortLegacyVbo = (++sBenchTick) & 1;
    }
    RenderJob job;
    memset(&job, 0, sizeof(job));
    job.dl = (Gfx*)task->t.data_ptr;
    job.skip = skip;
    job.interp = sInterp;
    job.rate = Port3ds_UpdateRate();
    job.base = sLast;
    job.skipOn = sSkipOn;
    if (sInterp && !skip) {
        /* PORT (2026-09-30): 60 fps. The logic runs at 20/s (R_UPDATE_RATE retraces per update). The
         * display-list walk RECORDS the frame (gfx_pc.c "replay"), with every tagged matrix also evaluated
         * at t = 1/3 and 2/3 between the previous logic frame and this one (port_interp.h tags, the
         * Zelda64Recomp / Ship of Harkinian model); then each retrace shows a REPLAY - only the vertex
         * positions are recomputed and the recorded draws re-issued (a few ms instead of a whole walk).
         * As many in-between frames as fit (measured costs), so a slow scene/console drops to fewer and
         * the logic keeps its N64 speed. The world is shown one logic frame (50 ms) late, as in SoH. */
        extern int Port3ds_InterpBlocked(void);
        const double kRetraceMs = 1000.0 / 59.83;
        int rate = job.rate;
        sReplayN = 0;
        sChose = 0;
        if (!(sLast != 0.0 && rate > 1 && !sBench && !Port3ds_InterpBlocked() && !afterSkip)) {
            sReplayNoChoice++;
        } else {
            /* with the render thread the logic runs beside the drawing: the whole update is the drawing's budget */
            double budget = rate * kRetraceMs, elapsed = sRenderThreadUse ? 2.0 : Port3ds_NowMs() - sLast;
            sChose = 1;
            for (sReplayN = rate - 1; sReplayN > 0 && sInterp < 2; sReplayN--) {
                if (elapsed + sWalkMs + sReplayN * sReplayMs <= budget) break;
            }
            if (sReplayN > 2) sReplayN = 2; /* t = 1/3, 2/3 */
            sReplayHist[sReplayN]++;
            sInterpElapsedSum += elapsed;
        }
        job.chose = sChose;
        job.replayN = sReplayN;
    }
    {
        extern void Port3ds_TakeFrameRequests(void** capture, int* flat, float* focusW);
        Port3ds_TakeFrameRequests(&job.capture, &job.flat, &job.focusW);
    }
    gPortInterpOn = sInterp && !skip && job.replayN > 0; /* for the NEXT update's actor drawing (z_actor.c) */
    if (sRenderThreadUse) {
        /* frames the game reads back (pause background, statediff) are drawn before the game goes on */
        extern int Port3ds_DrawIdActive(void);
        int wait = job.capture != NULL || Port3ds_DrawIdActive();
        sRenderJob = job;
        __sync_synchronize();
        sRenderBusy = 1;
        LightEvent_Signal(&sRenderGo);
        if (wait) {
            Port3ds_RenderWaitIdle();
        }
    } else {
        Port3ds_RenderJob(&job);
    }
    tC = svcGetSystemTick();
    PROF_SET(PROF_PACE);
    Port3ds_PaceFrame();
    PROF_SET(PROF_GAME);
    tD = svcGetSystemTick();
    sPerfPace += tD - tC;
    sPerfLastEnd = tD;
    /* Frame-rate log: game updates per second measured on the wall clock (x10), every 300 frames.
     * OoT's logic is designed for 20/s (R_UPDATE_RATE=3 VI retraces per update at 60 Hz). */
    { static u64 t0 = 0; static unsigned n = 0;
      if (t0 == 0) t0 = osGetTime();
      if (++n == 300) { u64 t1 = osGetTime();
          Port3ds_RenderWaitIdle(); /* the switches below may touch the renderer */
          extern void PortDbgX(const char*, unsigned); extern void* gRegEditor;
          /* PORT (2026-10-01): the report goes to boot.log only while a measurement switch is on (prof,
           * perf_ab, gpu_ab, aa_ab): ~50 lines every 15 s, each flushed to the SD card, otherwise */
          { extern int gPortPerfAB; sLogMute = !(sProfOn || gPortPerfAB || sGpuAB || sAaAB || sCmdflushAB || sPresentAB || sOverlapAB ||
                                                     sRawAB || sRenderThreadAB || sAudioShareAB || sReplayCopyAB || sSpeedRulesAB); }
          PortDbgX("perf updates/s x10", (unsigned)(3000000ull / (t1 - t0 ? t1 - t0 : 1)));
          PortDbgX("perf frames shown/s x10 (60fps interp)",
                   (unsigned)((300ull - sSkipCount + sInterpFrames) * 10000ull / (t1 - t0 ? t1 - t0 : 1)));
          { /* memory over a long session (leak check): linear heap (GPU buffers, textures) and the app heap */
              extern u32 linearSpaceFree(void);
              struct mallinfo mi = mallinfo();
              PortDbgX("perf mem linear free KB", linearSpaceFree() / 1024);
              PortDbgX("perf mem heap used KB", (unsigned)mi.uordblks / 1024);
              PortDbgX("perf mem heap free KB", (unsigned)mi.fordblks / 1024);
          }
          {
              extern u64 gPortPerfFrameEnd;
              extern float gPortPerfGpuProcMs, gPortPerfGpuDrawMs;
              extern unsigned gPortPerfGpuFrames;
              unsigned nf = gPortPerfGpuFrames ? gPortPerfGpuFrames : 1;
              PortDbgX("perf us/GPU frame C3D_FrameEnd (cpu)", (unsigned)(gPortPerfFrameEnd / nf / (SYSCLOCK_ARM11 / 1000000)));
              PortDbgX("perf us/GPU frame cpu FrameBegin..End", (unsigned)(gPortPerfGpuProcMs * 1000.0f / nf));
              PortDbgX("perf us/GPU frame gpu drawing (hw only)", (unsigned)(gPortPerfGpuDrawMs * 1000.0f / nf));
              PortDbgX("perf GPU frames (of 300 updates)", gPortPerfGpuFrames);
              gPortPerfFrameEnd = 0;
              gPortPerfGpuProcMs = gPortPerfGpuDrawMs = 0.0f;
              gPortPerfGpuFrames = 0;
          }
          { extern int gPortCmdlistFlush; PortDbgX("perf cmdlist_flush", (unsigned)gPortCmdlistFlush); }
          { /* gfx_3ds.c presentation model: frames that reached the screen (est.) and the gate's waiting */
              extern int gPortPresentGate;
              extern unsigned gPortPerfReplaced, gPortPerfPresented;
              extern u64 gPortPerfGateWait;
              unsigned shown = gPortPerfPresented - (gPortPerfReplaced < gPortPerfPresented ? gPortPerfReplaced : 0);
              PortDbgX("perf present gate", (unsigned)gPortPresentGate);
              { /* the 3D slider doubles draws and GPU work: say which mode a report measured */
                  extern unsigned gPortPerfStereoFrames;
                  PortDbgX("perf frames drawn in 3D (of presented)", gPortPerfStereoFrames);
                  PortDbgX("perf frames presented", gPortPerfPresented);
                  gPortPerfStereoFrames = 0;
              }
              PortDbgX("perf us logic frame done on GPU after update start", sWalkDoneN ? (unsigned)(sWalkDoneSum * 1000.0 / sWalkDoneN) : 0);
              PortDbgX("perf updates with coarse split (of 300)", sCoarseUpdates);
              sWalkDoneSum = 0.0;
              sWalkDoneN = sCoarseUpdates = 0;
              PortDbgX("perf frames displayed/s x10 (est)", (unsigned)((u64)shown * 10000ull / (t1 - t0 ? t1 - t0 : 1)));
              PortDbgX("perf frames replaced before shown", gPortPerfReplaced);
              PortDbgX("perf us/update present gate wait", (unsigned)(gPortPerfGateWait / 300 / (SYSCLOCK_ARM11 / 1000000)));
              gPortPerfReplaced = gPortPerfPresented = 0;
              gPortPerfGateWait = 0;
          }
          PortDbgX("perf frames skipped (of 300 updates)", sSkipCount);
          PortDbgX("perf updates late >1 retrace (of 300)", sLateUpdates);
          PortDbgX("perf updates with 2 in-between frames", sReplayHist[2]);
          PortDbgX("perf updates with 1 in-between frame", sReplayHist[1]);
          PortDbgX("perf updates with 0 in-between (budget)", sReplayHist[0]);
          PortDbgX("perf updates without interp choice", sReplayNoChoice);
          PortDbgX("perf in-between frames dropped (first frame late)", sReplayDropped);
          { /* gfx_3ds.c flip presenter: exact counts */
              extern unsigned gPortPerfFlipShown, gPortPerfFlipSkipped, gPortPerfFlipRepeats;
              extern int Port3ds_FlipActive(void);
              PortDbgX("perf flip presenter", (unsigned)Port3ds_FlipActive());
              { extern void Port3ds_FlipDump(void); Port3ds_FlipDump(); }
              PortDbgX("perf flip frames shown/s x10", (unsigned)((u64)gPortPerfFlipShown * 10000ull / (t1 - t0 ? t1 - t0 : 1)));
              PortDbgX("perf flip frames skipped (late)", gPortPerfFlipSkipped);
              PortDbgX("perf flip refreshes repeating a frame", gPortPerfFlipRepeats);
              gPortPerfFlipShown = gPortPerfFlipSkipped = gPortPerfFlipRepeats = 0;
          }
          { /* gfx_3ds.c CPU/GPU overlap: frames that handed the GPU over without ending the citro3d frame */
              extern int gPortOverlap;
              extern u32 gPortPerfOverlapFrames;
              extern float gPortPerfCmdBufMax;
              PortDbgX("perf render thread", (unsigned)sRenderThreadUse);
              PortDbgX("perf us/update game waits for the renderer", (unsigned)(sPerfRenderWait / 300 / (SYSCLOCK_ARM11 / 1000000)));
              sPerfRenderWait = 0;
              PortDbgX("perf overlap on", (unsigned)gPortOverlap);
              PortDbgX("perf overlap frames (of 300 updates)", gPortPerfOverlapFrames);
              PortDbgX("perf cmdbuf peak use % (1 MB)", (unsigned)(gPortPerfCmdBufMax * 100.0f));
              {
                  extern u64 gPortPerfDepthWait;
                  extern u32 gPortPerfDepthAsync;
                  PortDbgX("perf queued depth copies read", gPortPerfDepthAsync);
                  PortDbgX("perf us/read waiting for a queued depth copy",
                           gPortPerfDepthAsync ? (unsigned)(gPortPerfDepthWait / gPortPerfDepthAsync / (SYSCLOCK_ARM11 / 1000000)) : 0);
                  gPortPerfDepthWait = 0;
                  gPortPerfDepthAsync = 0;
              }
              gPortPerfOverlapFrames = 0;
              gPortPerfCmdBufMax = 0.0f;
          }
          sReplayDropped = 0;
          sLateUpdates = sReplayNoChoice = 0;
          sReplayHist[0] = sReplayHist[1] = sReplayHist[2] = 0;
          PortDbgX("perf frame skip on", (unsigned)sSkipOn);
          { extern int gPortShadeLinear; PortDbgX("perf shade linear (GPU path)", (unsigned)gPortShadeLinear); }
          sSkipCount = 0;
          { extern uint32_t gPortInterpMatched, gPortInterpMissed;
            PortDbgX("perf interp mtx matched/frame", sInterpFrames ? gPortInterpMatched / sInterpFrames : 0);
            PortDbgX("perf interp mtx missed/frame", sInterpFrames ? gPortInterpMissed / sInterpFrames : 0);
            PortDbgX("perf us/replay frame", sInterpFrames ? (unsigned)(sPerfInterp / sInterpFrames / (SYSCLOCK_ARM11 / 1000000)) : 0);
            PortDbgX("perf us/walk+first frame (est)", (unsigned)(sWalkMs * 1000.0));
            PortDbgX("perf us/replay frame (est)", (unsigned)(sReplayMs * 1000.0));
            PortDbgX("perf replay fallbacks (direct frames)", sReplayBrokenCnt);
            sReplayBrokenCnt = 0;
            PortDbgX("perf us/frame logic before passes", (unsigned)(sInterpElapsedSum * 1000.0 / n));
            sInterpElapsedSum = 0;
            { extern uint32_t gPortInterpSame, gPortInterpJump;
              PortDbgX("perf interp mtx identical/frame", sInterpFrames ? gPortInterpSame / sInterpFrames : 0);
              PortDbgX("perf interp mtx jump (not blended)/frame", sInterpFrames ? gPortInterpJump / sInterpFrames : 0);
              gPortInterpSame = gPortInterpJump = 0; }
            { extern uint32_t gPortInterpVtx, gPortInterpVtxMiss;
              PortDbgX("perf interp skin vtx blended/frame", sInterpFrames ? gPortInterpVtx / sInterpFrames : 0);
              PortDbgX("perf interp skin vtx loads unmatched/frame", sInterpFrames ? gPortInterpVtxMiss / sInterpFrames : 0);
              gPortInterpVtx = gPortInterpVtxMiss = 0; }
            gPortInterpMatched = gPortInterpMissed = 0; }
          sInterpFrames = 0; sPerfInterp = 0;
          if (gRegEditor) PortDbgX("perf R_UPDATE_RATE", (unsigned)*(short*)((char*)gRegEditor + 0x14 + 126 * 2));
          PortDbgX("perf audio pumps/s x10", (unsigned)((u64)sPortAudioPumps * 10000ull / (t1 - t0 ? t1 - t0 : 1)));
          { extern u32 gPortPerfCacheFallbacks; PortDbgX("perf cache ops through the GPU service (fallbacks)", gPortPerfCacheFallbacks); gPortPerfCacheFallbacks = 0; }
          { extern unsigned gPortAudioCore1Limit; extern int gPortAudioCoreNow;
            PortDbgX("perf audio mixer system-core share %", gPortAudioCore1Limit);
            PortDbgX("perf audio mixer core", (unsigned)gPortAudioCoreNow); }
          sPortAudioPumps = 0;
          Port3ds_PerfReport(n);
          if (!sLogMute) Port3ds_MemProbe();
          if (sBench && sBenchFrames[0] && sBenchFrames[1]) {
              PortDbgX("bench us/frame INDEXED", (unsigned)(sBenchDl[0] / sBenchFrames[0] / (SYSCLOCK_ARM11 / 1000000)));
              PortDbgX("bench us/frame ARRAY", (unsigned)(sBenchDl[1] / sBenchFrames[1] / (SYSCLOCK_ARM11 / 1000000)));
              sBenchDl[0] = sBenchDl[1] = 0;
              sBenchFrames[0] = sBenchFrames[1] = 0;
          }
          { /* perf_ab=1: alternate New 3DS speed / Old 3DS approximation every 4 reports (~1 min) so one
             * play session measures both (docs/3ds-60fps-plan.md P0) */
              extern int gPortPerfAB, gPortO3dsSim;
              static unsigned sReports;
              bool n3ds = false;
              APT_CheckNew3DS(&n3ds);
              PortDbgX(gPortO3dsSim ? "perf mode O3DS-sim (268MHz no L2)" : "perf mode N3DS (804MHz L2)", 1);
              { extern int gPortGpuVtx; PortDbgX(gPortGpuVtx ? "perf vertex path GPU" : "perf vertex path CPU", 1); }
              { extern int gPortAA; PortDbgX("perf anti-aliasing", (unsigned)gPortAA); }
              static unsigned sPgReports;
              /* every 4 reports: with cmdflush_ab (every 2) one session covers all four combinations */
              if (sPresentAB && (++sPgReports % 4) == 0) { /* applied from the next frame */
                  extern int gPortPresentGate;
                  gPortPresentGate = !gPortPresentGate;
              }
              static unsigned sRtReports;
              if (sRenderThreadAB && sRenderThreadOn && (++sRtReports % 2) == 0) { /* the renderer is idle here */
                  extern int gPortRenderThreaded, gPortEventsExternal;
                  sRenderThreadUse = !sRenderThreadUse;
                  gPortRenderThreaded = gPortEventsExternal = sRenderThreadUse;
              }
              static unsigned sOvReports;
              if (sOverlapAB && (++sOvReports % 2) == 0) { /* from the next update (gfx_3ds.c reads it per frame) */
                  extern int gPortOverlap;
                  gPortOverlap = !gPortOverlap;
              }
              static unsigned sCfReports;
              if (sCmdflushAB && (++sCfReports % 2) == 0) { /* applied from the next C3D_FrameEnd */
                  extern int gPortCmdlistFlush;
                  gPortCmdlistFlush = !gPortCmdlistFlush;
              }
              static unsigned sAaReports;
              if (sAaAB && (++sAaReports % 2) == 0) { /* applied before the next frame (gfx_3ds.c) */
                  extern int gPortAA;
                  gPortAA = !gPortAA;
              }
              static unsigned sShareReports;
              if (sAudioShareAB && (++sShareReports % 2) == 0) { /* (only while the mixer is on core 1) */
                  extern void Port3ds_AudioSetShare(int percent);
                  extern unsigned gPortAudioCore1Limit;
                  Port3ds_AudioSetShare(gPortAudioCore1Limit == 30 ? 55 : gPortAudioCore1Limit == 55 ? 80 : 30);
              }
              static unsigned sRawReports;
              if (sRawAB && (++sRawReports % 2) == 0) { /* gfx_start_frame latches it for the next frame */
                  extern int gPortRawVtxWant;
                  gPortRawVtxWant = !gPortRawVtxWant;
              }
              static unsigned sSpeedReports;
              if (sSpeedRulesAB && (++sSpeedReports % 2) == 0) { /* latched per update (gPortRawRelax) */
                  sSpeedRules = sSpeedRules ? 0 : 1;
              }
              static unsigned sCopyReports;
              if (sReplayCopyAB && (++sCopyReports % 2) == 0) { /* read when a recording ends: next update */
                  extern int gPortReplayCopy;
                  gPortReplayCopy = !gPortReplayCopy;
              }
              static unsigned sGpuReports;
              if (sGpuAB && (++sGpuReports % 2) == 0) { /* between frames: takes effect from the next one */
                  extern int gPortGpuVtx;
                  extern void gfx_citro3d_set_gpu_mode(int gpu);
                  gfx_citro3d_set_gpu_mode(!gPortGpuVtx);
              }
              if (gPortPerfAB && n3ds && (++sReports % 4) == 0) {
                  gPortO3dsSim = !gPortO3dsSim;
                  osSetSpeedupEnable(!gPortO3dsSim);
                  if (sRenderThreadOn && !sRenderThreadAB) {
                      /* PORT (2026-10-04): the Old 3DS has no core 2 - its phases draw on the game thread, as an
                       * Old 3DS does (the renderer is idle here) */
                      extern int gPortRenderThreaded, gPortEventsExternal;
                      sRenderThreadUse = !gPortO3dsSim;
                      gPortRenderThreaded = gPortEventsExternal = sRenderThreadUse;
                  }
              }
          }
          sLogMute = 0;
          n = 0; t0 = t1; } }
}

#define ROM_PATH "sdmc:/3ds/mm/baserom-decompressed.z64"
#define LOG_PATH "sdmc:/3ds/mm/boot.log"
int gPortMmAudio; /* 1 when built with PORT_MM_AUDIO: the audio pump (ultra_shims.c) runs */

static void boot_flush(void) {
    gfxFlushBuffers();
    gfxSwapBuffers();
    gspWaitForVBlank();
}

/* PORT PERF (2026-09-18): logging must be CHEAP — it is called from engine paths that run
 * every frame. The old Log() did gspWaitForVBlank() (a ~16ms hardware wait) AND fopen/fclose
 * on the SD on EVERY call, so per-frame logging stalled the game to ~1fps (the roadmap's
 * "never ship per-frame logging" trap). Now: keep one file handle open, fwrite+fflush the
 * line (crash still leaves the last line on the card), and print to the console — no vblank
 * wait, no reopen. The bottom-screen console updates on the graph loop's own swap. */
static FILE* sLogFile = NULL;
/* PORT PERF (2026-10-03): once the game runs, log lines are written by a low-priority thread. Each line used
 * to be flushed to the SD card on the spot; the perf report (~60 lines every 300 updates while a measurement
 * switch is on) stalled the game for up to half a second each time (hardware v42: one update a retrace late in
 * every report, felt as a pause "out of nowhere"). Boot messages stay synchronous: a crash during startup still
 * leaves its last line in boot.log. */
#define LOG_BUF 16384
static char sLogBuf[2][LOG_BUF];
static int sLogLen, sLogCur;
static LightLock sLogLock;
static LightEvent sLogEvent;
static Thread sLogThread;
static volatile int sLogAsync;

static void Log_WriterThread(void* arg) {
    (void)arg;
    for (;;) {
        char* buf;
        int len;
        LightEvent_Wait(&sLogEvent);
        LightLock_Lock(&sLogLock);
        buf = sLogBuf[sLogCur];
        len = sLogLen;
        sLogCur ^= 1; /* the game appends to the other buffer meanwhile */
        sLogLen = 0;
        LightLock_Unlock(&sLogLock);
        if (len > 0) {
            if (!sLogFile) sLogFile = fopen(LOG_PATH, "a");
            if (sLogFile) {
                fwrite(buf, 1, (size_t)len, sLogFile);
                fflush(sLogFile);
            }
        }
    }
}

static void Log_StartAsync(void) {
    s32 prio = 0x30;
    LightLock_Init(&sLogLock);
    LightEvent_Init(&sLogEvent, RESET_ONESHOT);
    svcGetThreadPriority(&prio, CUR_THREAD_HANDLE);
    sLogThread = threadCreate(Log_WriterThread, NULL, 8 * 1024, 0x3F, -2, true); /* lowest priority, same core */
    sLogAsync = sLogThread != NULL;
    (void)prio;
}

/* append one line for the writer thread (sLogAsync) */
static void Log_Append(const char* s) {
    int n = (int)strlen(s);
    LightLock_Lock(&sLogLock);
    if (sLogLen + n + 1 <= LOG_BUF) { /* full (the writer far behind): the line is dropped */
        memcpy(sLogBuf[sLogCur] + sLogLen, s, (size_t)n);
        sLogBuf[sLogCur][sLogLen + n] = '\n';
        sLogLen += n + 1;
    }
    LightLock_Unlock(&sLogLock);
    LightEvent_Signal(&sLogEvent);
}

/* before exit(): what the writer has not written yet (gfx_3ds.c HOME -> Close) */
void Port3ds_LogFlush(void) {
    if (!sLogAsync) return;
    LightLock_Lock(&sLogLock);
    if (sLogLen > 0) {
        if (!sLogFile) sLogFile = fopen(LOG_PATH, "a");
        if (sLogFile) {
            fwrite(sLogBuf[sLogCur], 1, (size_t)sLogLen, sLogFile);
            fflush(sLogFile);
        }
        sLogLen = 0;
    }
    LightLock_Unlock(&sLogLock);
}

static void Log(const char* s) {
    /* PORT (2026-09-20): an engine path calls the logger with an empty string ~15x/frame,
     * which flooded boot.log with bare '\n' (millions of lines / ~18 MB per session — real
     * SD-write load on hardware, the roadmap's "never ship per-frame logging" trap). Empty
     * lines carry no information, so drop them here at the chokepoint. */
    if (s == NULL || s[0] == '\0' || sLogMute) return;
    if (!sTouchUi) printf("%s\n", s); /* the bottom screen is the touch panel once it is up */
    if (sLogAsync) {
        Log_Append(s);
        return;
    }
    if (!sLogFile) sLogFile = fopen(LOG_PATH, "a");
    if (sLogFile) { fputs(s, sLogFile); fputc('\n', sLogFile); fflush(sLogFile); }
}

/* checkpoint logger called from engine init (main.c) */
void PortDbg(const char* s) { Log(s); }

/* hex value logger for diagnostics (e.g. scene-data dump) */
void PortDbgX(const char* label, unsigned val) {
    /* PORT (2026-09-21): format hex MANUALLY — sprintf("%s=%08x") is broken in the port's
     * libc (produces an empty buffer, which Log()'s empty-string guard then drops), so every
     * PortDbgX diagnostic was silently invisible. Manual formatting fixes all value-logging. */
    char buf[128]; int n = 0; int i;
    static const char hx[] = "0123456789abcdef";
    if (label) { while (label[n] != '\0' && n < 100) { buf[n] = label[n]; n++; } }
    buf[n++] = '='; buf[n++] = '0'; buf[n++] = 'x';
    for (i = 28; i >= 0; i -= 4) buf[n++] = hx[(val >> i) & 0xF];
    buf[n] = '\0';
    Log(buf);
}

/* Fast file-only loggers for high-volume renderer tracing (no console print). */
void PortLogFast(const char* s) {
    if (s == NULL || s[0] == '\0') return;
    if (sLogAsync) { /* the writer thread owns the file now */
        Log_Append(s);
        return;
    }
    if (!sLogFile) sLogFile = fopen(LOG_PATH, "a");
    if (sLogFile) { fputs(s, sLogFile); fputc('\n', sLogFile); }
}
void PortLogFastX(const char* label, unsigned val) {
    /* hex by hand like PortDbgX: the port's sprintf("%s=%08x") gives an empty string, so these lines were dropped */
    char buf[96];
    int n = 0, i;
    static const char hx[] = "0123456789abcdef";
    if (label) { while (label[n] != '\0' && n < 80) { buf[n] = label[n]; n++; } }
    buf[n++] = '='; buf[n++] = '0'; buf[n++] = 'x';
    for (i = 28; i >= 0; i -= 4) buf[n++] = hx[(val >> i) & 0xF];
    buf[n] = '\0';
    PortLogFast(buf);
}

/* Return the end address (base+size) of the mapped, readable memory block that
 * contains `addr`, or 0 if `addr` is unmapped/unreadable. The display-list
 * interpreter uses this to bound its reads: an un-terminated or garbage DL that
 * would otherwise walk into unmapped memory and data-abort is stopped cleanly at
 * the edge of its mapped block. One svcQueryMemory per 4KB page walked = cheap. */
/* PORT PERF (2026-09-30): the display-list walker and texture imports check every new memory block
 * with svcQueryMemory (a kernel call). The app's memory map is fixed after boot (heaps are reserved up
 * front), so readable ranges already confirmed are remembered: 8 most recent, round-robin. */
u32 gPortPerfMemQueries, gPortPerfMemHits;
u64 gPortPerfAudOpTicks[32]; /* audio_microcode.c: ticks per microcode op (perf_stages) */
unsigned PortMem_ReadableEnd(unsigned addr) {
    /* per thread (renderer, audio worker, game): the entries are written without a lock (2026-10-03) */
    static __thread unsigned sBase[8], sEnd[8];
    static __thread int sNext;
    MemInfo mi;
    PageInfo pi;
    int i;
    for (i = 0; i < 8; i++) {
        if (addr - sBase[i] < sEnd[i] - sBase[i]) {
            gPortPerfMemHits++;
            return sEnd[i];
        }
    }
    gPortPerfMemQueries++;
    if (R_FAILED(svcQueryMemory(&mi, &pi, addr))) return 0;
    if (mi.state == MEMSTATE_FREE || mi.state == MEMSTATE_RESERVED) return 0;
    if (!(mi.perm & MEMPERM_READ)) return 0;
    sBase[sNext] = mi.base_addr;
    sEnd[sNext] = mi.base_addr + mi.size;
    sNext = (sNext + 1) & 7;
    return mi.base_addr + mi.size;
}

/* Keep the bottom-screen console up so the message is readable instead of
 * silently bouncing back to the HOME menu. */
static void boot_halt(const char* msg) {
    Log(msg);
    printf("Press START to exit.\n");
    boot_flush();
    while (aptMainLoop()) {
        hidScanInput();
        if (hidKeysDown() & KEY_START) break;
        gspWaitForVBlank();
    }
}

/* ===== MINIMAL BOOT TEST (disabled) =====
 * Isolation build used to prove the CIA packaging. Kept for future debugging.
 * Renamed out of the way; the real entry point is main() below. */
int main_minimal(int argc, char** argv) {
    (void)argc; (void)argv;
    gfxInitDefault();
    consoleInit(GFX_BOTTOM, NULL);

    int n = 0;
    while (aptMainLoop()) {
        hidScanInput();
        if (hidKeysDown() & KEY_START) break;
        printf("\x1b[2;2HOoT port MINIMAL boot test -- frame %d   ", n++);
        printf("\x1b[4;2HIf you can read this, packaging is OK.");
        printf("\x1b[6;2HPress START to exit.");
        gfxFlushBuffers();
        gfxSwapBuffers();
        gspWaitForVBlank();
    }
    gfxExit();
    return 0;
}

/* Delete any existing Luma crash dumps at boot. Luma writes crash_dump_%08u.dmp
 * using the lowest free index, so with the folder emptied each launch, this
 * session's crash (if any) always lands as crash_dump_00000000.dmp — one file,
 * same name, which makes the user's Mac auto-transfer trivial. */
static void WipeCrashDumps(void) {
    const char* dir = "sdmc:/luma/dumps/arm11";
    DIR* d = opendir(dir);
    if (d == NULL) return;
    struct dirent* ent;
    char path[300];
    while ((ent = readdir(d)) != NULL) {
        if (ent->d_name[0] == '.') continue;
        snprintf(path, sizeof(path), "%s/%s", dir, ent->d_name);
        remove(path);
    }
    closedir(d);
}

/* full boot path — the real entry point */
/* PORT DEBUG: svcOutputDebugString markers show up in Azahar's log with timestamps —
 * reliable boot-progress tracing before the SD boot.log is even open. */
#define DBG(s) svcOutputDebugString((s), sizeof(s) - 1)

/* PORT (2026-09-29): first thing in main, before any engine code: prove the process started and record
 * how it was launched and what memory it got (a hardware CIA launch showed nothing at all). */
static void Port_EarlyBootMarker(void) {
    FILE* f = fopen("sdmc:/3ds/mm/boot_early.log", "a"); /* own file: boot.log is recreated later */
    if (f != NULL) {
        bool isNew = false;
        u64 programId = 0;
        APT_CheckNew3DS(&isNew);
        APT_GetProgramID(&programId);
        fprintf(f, "=== main() reached: program %016llx, %s launch, %s 3DS, app mem %lu KB (free %lu KB), linear free %lu KB ===\n",
                (unsigned long long)programId, envIsHomebrew() ? "3dsx" : "CIA/3ds", isNew ? "New" : "Old",
                (unsigned long)(osGetMemRegionSize(MEMREGION_APPLICATION) / 1024),
                (unsigned long)(osGetMemRegionFree(MEMREGION_APPLICATION) / 1024),
                (unsigned long)(linearSpaceFree() / 1024));
        fclose(f);
    }
}

/* PORT (2026-10-04): the system re-applies the title's CPU settings when the game comes back from the HOME menu or
 * sleep: the system-core share granted to the audio mixer (Old 3DS, audio_3ds.c) and the New 3DS speed mode are lost
 * (found by the gdx-3ds port on hardware). Without the share the Old 3DS's mixer would stall the game after HOME. */
static aptHookCookie sCpuStateHook;
static void Port3ds_CpuStateHook(APT_HookType type, void* param) {
    (void)param;
    if (type == APTHOOK_ONRESTORE || type == APTHOOK_ONWAKEUP) {
        extern unsigned gPortAudioCore1Limit;
        extern int gPortO3dsSim;
        bool n3ds = false;
        APT_CheckNew3DS(&n3ds);
        if (n3ds) {
            osSetSpeedupEnable(!gPortO3dsSim);
        }
        if (gPortAudioCore1Limit != 0) {
            APT_SetAppCpuTimeLimit(gPortAudioCore1Limit);
        }
    }
}

int main(int argc, char** argv) {
    (void)argc; (void)argv;
    Port_EarlyBootMarker();
    { extern void PortOverlayStatics_Init(void); PortOverlayStatics_Init(); } /* before any game code */
    DBG("PORT: main() entered");
    /* PORT PERF (2026-09-28): New 3DS: run the app core at 804 MHz with the L2 cache (default is the
     * original 268 MHz mode, ~3x slower). No effect on an original 3DS. */
    osSetSpeedupEnable(true);
    aptHook(&sCpuStateHook, Port3ds_CpuStateHook, NULL);
    gfxInitDefault();
    DBG("PORT: gfxInitDefault done");
    consoleInit(GFX_BOTTOM, NULL);
    DBG("PORT: consoleInit done");
    { extern void PortCompat_InitStreams(void); PortCompat_InitStreams(); }
    WipeCrashDumps(); /* keep only this run's crash dump, named crash_dump_00000000.dmp */

    /* Truncate the log file at the start of every boot (PORT 2026-10-04: the previous one is kept as boot_prev.log,
     * so a second session - or a restart after a crash - does not erase it). */
    {
        FILE* in = fopen(LOG_PATH, "rb");
        if (in != NULL) {
            FILE* out = fopen("sdmc:/3ds/mm/boot_prev.log", "wb");
            static char buf[16 * 1024];
            size_t n;
            while (out != NULL && (n = fread(buf, 1, sizeof(buf), in)) > 0) {
                fwrite(buf, 1, n, out);
            }
            if (out != NULL) fclose(out);
            fclose(in);
        }
    }
    { FILE* f = fopen(LOG_PATH, "w");
      if (f) { fputs("=== OoT 3DS boot log ===\n", f); fclose(f); } }

    { extern const char gPortVersion[]; /* Makefile.3ds: git describe, or port/VERSION */
      char line[160] = "The Legend of Zelda: Ocarina of Time N64 3DS Port ";
      strncat(line, gPortVersion, 60);
      strcat(line, " booting...");
      Log(line); }
    /* ndsp plumbing AFTER the log is set up so its init status is visible (was before the
     * truncation above, which wiped its logs). Tries HLE even without dspfirm.cdc. */
    { extern void Port3ds_AudioInit(void); Port3ds_AudioInit(); }

    /* PORT DEBUG: the port assumes the linear heap is at 0x08000000 (segment-8 collision
     * handling). Log where libctru's linear heap actually lands on this Azahar/firmware. */
    { void* _lp = linearAlloc(0x1000);
      void* _lp2 = linearAlloc(0x100000);
      char _b[128];
      snprintf(_b, sizeof(_b), "MEM: linear base=%08x  +1MB=%08x  (24MB set)",
               (unsigned)(uintptr_t)_lp, (unsigned)(uintptr_t)_lp2);
      Log(_b);
      if (_lp) linearFree(_lp); if (_lp2) linearFree(_lp2); }

    FILE* rf = fopen(ROM_PATH, "rb");
    if (rf == NULL) { boot_halt("ROM not found at " ROM_PATH); gfxExit(); return 0; }
    fclose(rf);
    Log("ROM found.");

    PortDma_Init(ROM_PATH);
    {
        FILE* fb;
        hidScanInput();
        if ((hidKeysHeld() & KEY_L) && (fb = fopen("sdmc:/3ds/mm/settings_b.txt", "r")) != NULL) {
            fclose(fb);
            sSettingsPath = "sdmc:/3ds/mm/settings_b.txt";
            Log("settings: settings_b.txt (L held at start)");
        }
    }
    Port3ds_LoadSettings();
    Port3ds_ProfStart();
    {
        extern int gPortO3dsSim;
        if (gPortO3dsSim) {
            osSetSpeedupEnable(false);
            DBG("PORT: o3ds_sim: New 3DS speedup OFF (268 MHz, no L2) - Old 3DS approximation");
        }
    }

    Log("DMA init OK.");
    { /* PORT MM (bring-up): physical addresses of the main regions, to place GPU address errors */
      extern char port_seg_gameplay_keep[], __end__[]; extern int gLoBuffer;
      void* h = malloc(16); void* l = linearAlloc(16);
      PortDbgX("phys .data asset (gameplay_keep)", (unsigned)osConvertVirtToPhys(port_seg_gameplay_keep));
      PortDbgX("  virt", (unsigned)(uintptr_t)port_seg_gameplay_keep);
      PortDbgX("phys .bss (gLoBuffer)", (unsigned)osConvertVirtToPhys(&gLoBuffer));
      PortDbgX("phys image end", (unsigned)osConvertVirtToPhys(__end__));
      PortDbgX("phys app heap", (unsigned)osConvertVirtToPhys(h)); PortDbgX("  virt", (unsigned)(uintptr_t)h);
      PortDbgX("phys linear", (unsigned)osConvertVirtToPhys(l)); PortDbgX("  virt", (unsigned)(uintptr_t)l);
      { extern u32 __ctru_linear_heap, __ctru_linear_heap_size;
        PortDbgX("linear heap virt", (unsigned)__ctru_linear_heap); PortDbgX("linear heap size", (unsigned)__ctru_linear_heap_size);
        PortDbgX("linear space free", (unsigned)linearSpaceFree()); }
      free(h); linearFree(l); }
    /* PORT (2026-09-24): bootproc() normally calls Locale_Init (cart header -> gCurrentRegion,
     * which SaveContext_Init turns into the save language). The port enters Main() directly,
     * so region stayed 0 and the US ROM showed Japanese text. Run it here, after the ROM opens. */
    /* PORT MM TODO: OoT ran Locale_Init() here for the save language; check MM's region/language setup */

    gViConfigModeType = 0;

#ifdef PORT_LOG_SYNC
    PortDbgX("fb top", (unsigned)(uintptr_t)gfxGetFramebuffer(GFX_TOP, GFX_LEFT, NULL, NULL));
    PortDbgX("fb bottom", (unsigned)(uintptr_t)gfxGetFramebuffer(GFX_BOTTOM, GFX_LEFT, NULL, NULL));
#endif
    Log("calling Main() (engine init)...");
    Main(0);
    Log("Main() returned; entering graph loop.");
#ifdef PORT_LOG_SYNC
    PortDbgX("after Main: fb top", (unsigned)(uintptr_t)gfxGetFramebuffer(GFX_TOP, GFX_LEFT, NULL, NULL));
    PortDbgX("after Main: fb bottom", (unsigned)(uintptr_t)gfxGetFramebuffer(GFX_BOTTOM, GFX_LEFT, NULL, NULL));
    PortDbgX("after Main: linear free", (unsigned)linearSpaceFree());
#endif

    /* Audio isn't initialized on 3DS (audio thread never runs), so the SFX bank
     * link-lists are garbage and any Audio_StopSfxById/etc. walk spins forever.
     * Audio_ResetSfx() is CPU-side only (resets gSfxBanks to empty) and makes
     * all the SFX functions safe until real audio lands. */
    { extern void AudioSfx_Reset(void); AudioSfx_Reset(); Log("AudioSfx_Reset (sfx banks) done"); } /* (MM name) */
    /* Point gAudioCtx table pointers at the native compiled tables so direct game
     * reads (e.g. fanfare -> AudioLoad_GetFontsForSequence) can't NULL-deref. */
    /* PORT MM TODO (audio bring-up): OoT ran Audio_PortInitTables() here (an OoT game-side patch) */
    /* Real audio bring-up: Audio_Init (AudioLoad_Init) sets up the audio heap and
     * loads the spec, which sets audioBufferParameters.specUnk4 (nonzero). Without
     * it AudioThread_Update's task path is gated off (specUnk4==0) so no synthesis
     * task is ever built. DMA handler defaults to osEPiStartDma (port-routed). */
#ifdef PORT_MM_AUDIO
    { extern void Audio_Init(void); extern void Audio_InitSound(void);
      Audio_Init(); Log("Audio_Init done");
      Audio_InitSound(); Log("Audio_InitSound done"); }
    gPortMmAudio = 1;
#else
    /* PORT MM TODO (audio bring-up): audio is off until MM's synthesis is ported - in Azahar it reads and writes
     * unmapped memory (fatal on hardware). Build with PORT_EXTRA=-DPORT_MM_AUDIO to try it. */
    Log("audio off (PORT_MM_AUDIO not set)");
#endif

    Port3ds_TouchUiInit(); /* boot finished: the bottom screen becomes the control panel */
#ifndef PORT_LOG_SYNC /* (debug: PORT_EXTRA=-DPORT_LOG_SYNC keeps every line written at once, for crash hunting) */
    Log_StartAsync();      /* from here on the SD card is written by a background thread */
#endif
    {
        /* PORT PERF (2026-10-04): the Old 3DS layout. Its second core (1) is shared with the system, which gives an
         * application only the share granted by APT_SetAppCpuTimeLimit. Measured on hardware at Old 3DS speed: the game
         * loop there (v50) ran 41 instead of 60 updates per second with crackling audio; the drawing there (v51) took
         * 72-128 ms per frame instead of 25-40 on core 0, and the game, waiting for each frame, fell to 7-13 updates
         * per second. (Both ran with 30% of the core, not the 55% asked for: the exheader's MaxCpu refused it. v54 raises
         * the limit to 89% and asks for 80% - worth measuring again.) So an Old 3DS draws on core 0 with the game (no render thread), and only the audio mixer (~22% of a
         * core) uses core 1. o3ds_layout=1 reproduces this on a New 3DS (mixer on core 1, no render thread) for faithful
         * Old 3DS tests with o3ds_sim=1. */
        bool n3ds = false;
        APT_CheckNew3DS(&n3ds);
        if (!n3ds || sO3dsLayout) {
            extern int gPortAudioCore;
            gPortAudioCore = 1;
            sRenderThreadSetting = sRenderThreadAB = 0;
            Log("[render] Old 3DS layout: one core for the game and the drawing, audio mixer on core 1");
        }
    }
    Graph_ThreadEntry(0);

    boot_halt("graph loop exited");
    gfxExit();
    return 0;
}

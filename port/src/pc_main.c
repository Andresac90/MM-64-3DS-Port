/*
 * pc_main.c — native entry point for the OoT port (Phase 3 bring-up).
 * Boot chain: replicate the few globals idle.c sets, run the game's real
 * Main() for engine init (shims defuse its blocking points), then drive the
 * gamestate machine directly — Graph_ThreadEntry IS the game loop.
 */
#include "ultra64.h"

extern int fprintf();
extern void* stderr;
extern void PortDma_Init();
extern void PortRdram_Init();

extern u8 gViConfigModeType;
extern OSViMode gViConfigMode;
extern OSViMode osViModeNtscLan1;
extern u32 gViConfigFeatures;
extern f32 gViConfigXScale;
extern f32 gViConfigYScale;

extern void Main(void* arg);
extern void Graph_ThreadEntry(void* arg);
extern void Audio_InitSound(void);

int main(int argc, char** argv) {
    const char* rom = (argc > 1) ? argv[1]
                                 : "/home/aceve/oot/baseroms/ntsc-1.0/baserom-decompressed.z64";
    fprintf(stderr, "oot-port: pc bring-up\n");
    PortRdram_Init();
    PortDma_Init(rom);

    /* what Idle_ThreadEntry sets up before spawning the Main thread */
    gViConfigFeatures = OS_VI_GAMMA_OFF | OS_VI_DITHER_FILTER_ON;
    gViConfigXScale = 1.0f;
    gViConfigYScale = 1.0f;
    gViConfigModeType = OS_VI_NTSC_LAN1;
    gViConfigMode = osViModeNtscLan1;

    Main(0);
    fprintf(stderr, "oot-port: engine init done\n");
    /* game-side audio state init — normally run by the audio boot path;
       builds the sfx/sequence lists so per-frame walkers are coherent */
    Audio_InitSound();
    fprintf(stderr, "oot-port: audio state initialized, entering graph loop\n");
    Graph_ThreadEntry(0);
    fprintf(stderr, "oot-port: graph loop exited\n");
    return 0;
}

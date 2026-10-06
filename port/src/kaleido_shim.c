/*
 * kaleido_shim.c — replaces src/code/z_kaleido_manager.c.
 * Pause-screen "overlays" are statically linked; loading is identity and the
 * vram→ram address translation is a passthrough. The original's bzero over
 * the loaded region would wipe live native code — gone.
 */
#include "kaleido_manager.h"
#include "segment_symbols.h"
#include "global.h" /* PORT: was OoT play_state.h */

#define KALEIDO_OVERLAY(name, nameString) \
    { NULL, ROM_FILE(ovl_##name), _ovl_##name##SegmentStart, _ovl_##name##SegmentEnd, 0, nameString, }

KaleidoMgrOverlay gKaleidoMgrOverlayTable[] = {
    KALEIDO_OVERLAY(kaleido_scope, "kaleido_scope"),
    KALEIDO_OVERLAY(player_actor, "player_actor"),
};

KaleidoMgrOverlay* gKaleidoMgrCurOvl = NULL;
u8 gBossMarkState = 0;

void KaleidoManager_LoadOvl(KaleidoMgrOverlay* ovl) {
    ovl->loadedRamAddr = ovl->vramStart; /* identity: offset stays 0 */
    ovl->offset = 0;
    gKaleidoMgrCurOvl = ovl;
}

void KaleidoManager_ClearOvl(KaleidoMgrOverlay* ovl) {
    if (ovl->loadedRamAddr != NULL) {
        ovl->offset = 0;
        ovl->loadedRamAddr = NULL;
        gKaleidoMgrCurOvl = NULL;
    }
}

void KaleidoManager_Init(PlayState* play) {
    (void)play; /* no shared load area needed: nothing is copied */
}

void KaleidoManager_Destroy(void) {
    gKaleidoMgrCurOvl = NULL;
}

void* KaleidoManager_GetRamAddr(void* vram) {
    return vram; /* native pointers are already correct */
}

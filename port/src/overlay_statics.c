/* overlay_statics.c - give natively linked overlays the N64's "fresh load" semantics for static data.
 *
 * On the N64, Overlay_Load copies an overlay's .data from ROM and zeroes its .bss every time the game
 * (re)loads it: an actor overlay after its last instance was deleted, kaleido on every pause, effect
 * and gamestate overlays likewise. Game code relies on that, e.g. ObjectKankyo's `sIsSpawned` is set
 * and never cleared, so on the port the forest fairy sparkles vanished from every scene after the
 * first (found by tools/statediff tour: Lost Woods / Sacred Forest Meadow).
 *
 * Makefile.3ds renames each overlay object's .data/.bss to ovld_<ovl>/ovlb_<ovl>; the linker provides
 * __start_/__stop_ bounds (table: tools/gen_overlay_statics.py). At boot the initial .data of every
 * overlay is snapshotted; PortOverlayStatics_Reset (from Overlay_Load) restores it and zeroes .bss. */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    uint32_t vromStart;
    char* dataStart;
    char* dataStop;
    char* bssStart;
    char* bssStop;
} OverlayStatics;

#include "../src_gen/overlay_statics_table.inc"

#define NUM_OVERLAYS (sizeof(sOverlayStatics) / sizeof(sOverlayStatics[0]))
static char* sInitialData[NUM_OVERLAYS];

void PortOverlayStatics_Init(void) {
    size_t i;
    for (i = 0; i < NUM_OVERLAYS; i++) {
        const OverlayStatics* o = &sOverlayStatics[i];
        if (o->dataStart != NULL && o->dataStop > o->dataStart) {
            sInitialData[i] = malloc(o->dataStop - o->dataStart);
            if (sInitialData[i] != NULL) {
                memcpy(sInitialData[i], o->dataStart, o->dataStop - o->dataStart);
            }
        }
    }
}

void PortOverlayStatics_Reset(uintptr_t vromStart) {
    size_t i;
    for (i = 0; i < NUM_OVERLAYS; i++) {
        const OverlayStatics* o = &sOverlayStatics[i];
        if (o->vromStart == vromStart) {
            if (sInitialData[i] != NULL) {
                memcpy(o->dataStart, sInitialData[i], o->dataStop - o->dataStart);
                /* PORT MM TODO (cutscenes): the OoT port re-normalized BE-packed cutscene scripts here
                 * (Cutscene_ForgetNormalizedRange, an OoT z_demo.c patch) */
            }
            if (o->bssStart != NULL && o->bssStop > o->bssStart) {
                memset(o->bssStart, 0, o->bssStop - o->bssStart);
            }
            return;
        }
    }
}

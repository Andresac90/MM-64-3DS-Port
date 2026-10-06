/*
 * loadfragment_shim.c — replaces src/libu64/loadfragment2_n64.c.
 * All overlay code is statically linked native. Returning vramStart as the
 * "loaded" address makes every relocation offset in the engine compute to
 * zero, so the tables' native function pointers pass through unchanged.
 */
#include "ultra64.h"

void Overlay_Relocate(void* allocatedRamAddr, void* ovlRelocs, void* vramStart) {
    (void)allocatedRamAddr; (void)ovlRelocs; (void)vramStart;
}

extern void PortOverlayStatics_Reset(uintptr_t vromStart);

/* A (re)load restarts the overlay's static variables, like copying .data/.bss fresh from ROM on N64
 * (overlay_statics.c). */
u32 Overlay_Load(uintptr_t vromStart, uintptr_t vromEnd, void* vramStart, void* vramEnd,
                 void* allocatedRamAddr) {
    (void)vromEnd; (void)allocatedRamAddr;
    PortOverlayStatics_Reset(vromStart);
    return (u32)((uintptr_t)vramEnd - (uintptr_t)vramStart);
}

void* Overlay_AllocateAndLoad(uintptr_t vromStart, uintptr_t vromEnd, void* vramStart, void* vramEnd) {
    (void)vromEnd; (void)vramEnd;
    PortOverlayStatics_Reset(vromStart);
    return vramStart;
}

#ifndef SEGMENTED_ADDRESS_H
#define SEGMENTED_ADDRESS_H

/* PORT OVERRIDE of include/segmented_address.h.
 * Natively-linked asset data stores real pointers where the N64 stored
 * segment offsets; both flow through here. Real pointers (>= 0x10000000)
 * pass through untouched; true segment references resolve via gSegments,
 * adding K0BASE only when the base is an emulated-RDRAM physical address.
 */

#include "ultra64.h"
#include "stdint.h"

extern uintptr_t gSegments[NUM_SEGMENTS];
#ifdef __3DS__
extern char __end__[]; /* top of the loaded binary image (3dsx linker symbol) */
#endif

static inline void* PortSegmentedToVirtual(uintptr_t addr) {
    uintptr_t v;

#ifdef __3DS__
    /* 3DS: host RAM is at physical-equivalent addresses below 0x80000000; there
     * is no emulated RDRAM at 0x80000000. KSEG pointers strip to host physical;
     * native pointers pass through; segment refs resolve via gSegments (host). */
    if (addr >= 0x80000000u) {
        return (void*)(addr & 0x1FFFFFFFu); /* KSEG0/KSEG1 -> host physical */
    }
    if (addr >= 0x10000000u) {
        return (void*)addr; /* native host pointer */
    }
    /* D7 fix (2026-07-31): the 3DS app heap (newlib malloc, where DMA'd assets + the game
     * arena live) is at 0x08000000-0x0FFFFFFF, colliding with OoT segment numbers 8-0xF.
     * A native heap pointer (arena allocations start ~0x0824xxxx) has large low-24-bits
     * (>= 0x100000); a genuine seg-8..F reference has a small offset (< 1MB) into its
     * asset. Treat the large-offset case as native BEFORE gSegments translation. Mirror of
     * gfx_pc.c seg_addr. Fixes the file-select (seg 8 UI assets) reading garbage. */
    if (addr >= 0x08000000u && (addr & 0x00FFFFFFu) >= 0x00100000u) {
        return (void*)addr;
    }
    /* A pointer INTO the loaded binary image [0x00100000, __end__) is a relocated
     * native asset pointer, not a raw segment offset — native-compiled asset data
     * stores real pointers, never N64 segment addresses. It must pass through even
     * when its high nibble happens to match a SET segment (e.g. a scene-data list
     * pointer relocated to 0x0200xxxx, whose nibble 2 collides with the loaded
     * scene segment): translating it would corrupt the pointer. This is the crux
     * that broke Scene_CommandPlayerEntryList once the binary grew past 0x02000000.
     * Must be checked BEFORE the gSegments logic. */
    if (addr >= 0x00100000u && addr < (uintptr_t)__end__) {
        return (void*)addr;
    }
    /* If this "segment" slot is unset, `addr` is NOT a segment offset — it is a
     * real pointer baked into the native asset that just happens to land below
     * 0x10000000 (3DS .data lives there). Pass it through untouched. Mangling it
     * as a segment offset (dropping the high nibble) was corrupting real asset
     * pointers like 0x01f71a80 -> 0x00f71a80. */
    if (gSegments[SEGMENT_NUMBER(addr)] == 0) {
        return (void*)addr;
    }
    v = gSegments[SEGMENT_NUMBER(addr)] + SEGMENT_OFFSET(addr);
    if (v >= 0x80000000u) {
        v &= 0x1FFFFFFFu; /* segment base was a KSEG0 address */
    }
    return (void*)v;
#else
    if (addr >= 0x10000000u) {
        return (void*)addr; /* already a native (or KSEG) pointer */
    }
    v = gSegments[SEGMENT_NUMBER(addr)] + SEGMENT_OFFSET(addr);
    if (v < 0x10000000u) {
        v += K0BASE; /* physical emulated-RDRAM base: restore KSEG0 view */
    }
    return (void*)v;
#endif
}

/* The N64 macro evaluates `addr` twice (SEGMENT_NUMBER + SEGMENT_OFFSET), and the game depends on it:
 * e.g. SEGMENTED_TO_VIRTUAL(Rand_ZeroOne() < 0.5f ? A : B) in the bubble effects draws two random
 * numbers, and a single evaluation desynced every later random value (found by tools/statediff
 * rngtrace, Lake Hylia). Evaluate it twice too; the result uses the second value, as on N64 for
 * the same-segment addresses the game passes. */
#define SEGMENTED_TO_VIRTUAL(addr) ((void)(addr), PortSegmentedToVirtual((uintptr_t)(addr)))
/* MM's name for it (include/segmented_address.h) */
#define SEGMENTED_TO_K0(addr) SEGMENTED_TO_VIRTUAL(addr)

#endif

/*
 * boot_globals.c — globals the N64 boot ROM / linker provided.
 * Types must match include/ultra64/os_system.h exactly.
 */
#include "ultra64.h"

s32 osRomType = 0;       /* cartridge */
void* osRomBase = 0;
s32 osTvType = 1;        /* NTSC */
s32 osResetType = 0;     /* cold boot */
s32 osCicId = 6105;
s32 osVersion = 2;
u32 osMemSize = 8 * 1024 * 1024;
s32 osAppNMIBuffer[0x10];

/* VI mode tables — zeroed; the platform layer ignores VI mode contents. */
OSViMode osViModeNtscLan1;
OSViMode osViModeMpalLan1;
OSViMode osViModePalLan1;
OSViMode osViModeFpalLan1;

/* Build stamp (replaces src/boot/build.c, which is generated). */
const char gBuildCreator[] = "oot-3ds-port";
const char gBuildDate[] = "2026-06-11";
const char gBuildMakeOption[] = "port";

/* HW interrupt routine registration — no interrupts on the host. */
void __osSetHWIntrRoutine(OSHWIntr intr, s32 (*callback)(void), void* sp) {
    (void)intr; (void)callback; (void)sp;
}
void __osGetHWIntrRoutine(OSHWIntr intr, s32 (**callbackOut)(void), void** spOut) {
    (void)intr;
    if (callbackOut != 0) { *callbackOut = 0; }
    if (spOut != 0) { *spOut = 0; }
}

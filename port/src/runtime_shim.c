/*
 * runtime_shim.c — replaces src/libu64/runtime.c (IDO C++ runtime support).
 * Runtime_Init would seed gSystemArena with an N64 address range; the port's
 * SystemArena is host malloc, so init is a no-op and New/Delete route there.
 */
#include "ultra64.h"

extern void* SystemArena_Malloc(u32 size);
extern void SystemArena_Free(void* ptr);

/* opaque storage for &gSystemArena takers; never used as a real arena */
u8 gSystemArena[0x80];

typedef void (*BlockFunc)(void*);
typedef void (*BlockFunc2)(void*, u32);
typedef void (*BlockFunc9)(void*, u32, u32, u32, u32, u32, u32, u32, u32);

void* Runtime_New(u32 size) { return SystemArena_Malloc(size); }
void Runtime_Delete(void* ptr) { SystemArena_Free(ptr); }

void func_800FC868(void* blk, u32 nBlk, u32 blkSize, BlockFunc arg3) {
    u32 i;
    for (i = 0; i < nBlk; i++) {
        arg3((void*)((u8*)blk + i * blkSize));
    }
}
void func_800FC8D8(void* blk, u32 nBlk, s32 blkSize, BlockFunc2 arg3) {
    u32 i;
    for (i = 0; i < nBlk; i++) {
        arg3((void*)((u8*)blk + i * blkSize), 0);
    }
}
void* func_800FC948(void* blk, u32 nBlk, u32 blkSize, BlockFunc9 arg3) {
    u32 i;
    for (i = 0; i < nBlk; i++) {
        arg3((void*)((u8*)blk + i * blkSize), 0, 0, 0, 0, 0, 0, 0, 0);
    }
    return blk;
}
void func_800FCA18(void* blk, u32 nBlk, u32 blkSize, BlockFunc2 arg3, s32 arg4) {
    u32 i;
    (void)arg4;
    for (i = 0; i < nBlk; i++) {
        arg3((void*)((u8*)blk + i * blkSize), 0);
    }
}

void Runtime_ExecuteGlobalCtors(void) {}
void Runtime_Init(void* start, u32 size) { (void)start; (void)size; }

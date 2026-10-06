/* mm_stubs.c - Majora's Mask symbols the 3DS port provides itself (GAME world, MM's prototypes).
 *
 * - libc64 sleep (src/boot/libc64/sleep.c is not compiled: it busy-waits on the N64 count register)
 * - libc64 malloc API (src/boot/libc64/malloc.c is not compiled: its malloc/free would replace the 3DS C library's,
 *   which the platform layer needs). malloc/free/realloc/calloc are the C library's; the game's system heap
 *   (SystemHeap_Init -> MallocInit) keeps MM's own arena allocator over the region the game gives it.
 * - the Voice Recognition Unit (the N64 microphone accessory): not connected
 * - RSP microcode symbols: no RSP; the port renders display lists itself
 */
#include "ultra64.h"
#include "libc64/sleep.h"
#include "libc64/malloc.h"
#include "libc64/fixed_point.h"
#include "PR/os_voice.h"
#include "PR/gs2dex.h"
#include "variables.h"


/* usleep: the C library's (declared by libc64/sleep.h with MM's prototype) */

void csleep(OSTime time) { usleep((u32)OS_CYCLES_TO_USEC(time)); }
void nsleep(u32 nsec) { usleep(nsec / 1000); }
void msleep(u32 msec) { usleep(msec * 1000); }

Arena malloc_arena;
static s32 sMallocInit;
void MallocInit(void* start, size_t size) {
    __osMallocInit(&malloc_arena, start, size);
    sMallocInit = true;
}
void MallocCleanup(void) {
    __osMallocCleanup(&malloc_arena);
    sMallocInit = false;
}
s32 MallocIsInitialized(void) { return sMallocInit; }
void* malloc_r(size_t size) { return __osMallocR(&malloc_arena, size); }
void GetFreeArena(size_t* maxFreeBlock, size_t* bytesFree, size_t* bytesAllocated) {
    __osGetSizes(&malloc_arena, maxFreeBlock, bytesFree, bytesAllocated);
}
s32 CheckArena(void) { return __osCheckArena(&malloc_arena); }

/* the game's malloc/free (port/include/libc64/malloc.h maps the names): MM's arena, as on the N64 */
void* PortGame_Malloc(size_t size) { return __osMalloc(&malloc_arena, size); }
void PortGame_Free(void* ptr) { __osFree(&malloc_arena, ptr); }
void* PortGame_Realloc(void* oldPtr, size_t newSize) { return __osRealloc(&malloc_arena, oldPtr, newSize); }
void* PortGame_Calloc(size_t num, size_t size) {
    void* p = __osMalloc(&malloc_arena, num * size);
    if (p != NULL) {
        bzero(p, num * size);
    }
    return p;
}
/* the system heap's memory: the N64's size, from the 3DS linear heap (like the OoT port's game arena) */
void* PortGame_SystemHeapRegion(size_t size) {
    extern void* linearAlloc(size_t size);
    return linearAlloc(size);
}

f32 qNaN0x10000 = 0.0f; /* set to the quiet NaN 0x7F810000 below (libc64/fp.s) */
__attribute__((constructor)) static void PortMm_InitQNaN(void) {
    union { u32 u; f32 f; } v = { 0x7F810000 };
    qNaN0x10000 = v.f;
}

void __osCleanupThread(void) {}
s32 osEPiLinkHandle(OSPiHandle* handle) { (void)handle; return 0; }

#define VOICE_NOT_CONNECTED 1 /* (CONT_ERR_NO_CONTROLLER) */
s32 osVoiceInit(OSMesgQueue* mq, OSVoiceHandle* hd, int channel) { (void)mq; (void)hd; (void)channel; return VOICE_NOT_CONNECTED; }
s32 osVoiceSetWord(OSVoiceHandle* hd, u8* word) { (void)hd; (void)word; return VOICE_NOT_CONNECTED; }
s32 osVoiceCheckWord(u8* word) { (void)word; return VOICE_NOT_CONNECTED; }
s32 osVoiceStartReadData(OSVoiceHandle* hd) { (void)hd; return VOICE_NOT_CONNECTED; }
s32 osVoiceStopReadData(OSVoiceHandle* hd) { (void)hd; return VOICE_NOT_CONNECTED; }
s32 osVoiceGetReadData(OSVoiceHandle* hd, OSVoiceData* result) { (void)hd; (void)result; return VOICE_NOT_CONNECTED; }
s32 osVoiceClearDictionary(OSVoiceHandle* hd, u8 numWords) { (void)hd; (void)numWords; return VOICE_NOT_CONNECTED; }
s32 osVoiceMaskDictionary(OSVoiceHandle* hd, u8* maskPattern, int size) { (void)hd; (void)maskPattern; (void)size; return VOICE_NOT_CONNECTED; }
s32 osVoiceControlGain(OSVoiceHandle* hd, s32 analog, s32 digital) { (void)hd; (void)analog; (void)digital; return VOICE_NOT_CONNECTED; }

STACK(aspMainStack, 0x400);
static u64 sAudioSPData[0x10];
u64* gAudioSPDataPtr = sAudioSPData;
u64 gspS2DEX2_fifoTextStart[1], gspS2DEX2_fifoTextEnd[1];
u64 gspS2DEX2_fifoDataStart[1], gspS2DEX2_fifoDataEnd[1];

unsigned gPortDbgRealloc[4]; /* (bring-up: game.c GameState_Realloc failure numbers) */

#ifndef PORT_LIBC64_MALLOC_H
#define PORT_LIBC64_MALLOC_H
/* PORT OVERRIDE: MM's malloc/free/realloc/calloc are its own allocator over the system heap (libc64/malloc.c on
 * the N64: __osMalloc on malloc_arena). The 3DS build keeps the C library's malloc for the platform layer, so in the
 * game's code these names map to MM's arena (port/src/mm_stubs.c) - the game gets exactly its N64 heap behaviour
 * (GameState_Realloc sizes its arena from GetFreeArena, as on the N64). */
#include_next "libc64/malloc.h"

void* PortGame_Malloc(size_t size);
void PortGame_Free(void* ptr);
void* PortGame_Realloc(void* oldPtr, size_t newSize);
void* PortGame_Calloc(size_t num, size_t size);
#define malloc PortGame_Malloc
#define free PortGame_Free
#define realloc PortGame_Realloc
#define calloc PortGame_Calloc
/* the system heap's memory (N64: from the end of the buffers segment to the framebuffers) */
void* PortGame_SystemHeapRegion(size_t size);
#endif

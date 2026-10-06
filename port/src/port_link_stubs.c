/* PORT (2026-09-17): n64dd is disabled in this port (idle.c D_80121211 == 0), but the
 * game references these n64dd entry points. Provide inert stubs so the image links;
 * none run because the n64dd path is gated off. Signatures are loose on purpose (C
 * links by name); real bodies live in src/n64dd/ if n64dd is ever brought up. */
void func_801C6EA0() {}
void func_801C70FC() {}
void func_801C7268() {}
void func_801C7658() {}
void func_801C7818() {}
void func_801C7C1C() {}
void func_801C7E78() {}
void n64dd_SetDiskVersion() {}

/* _Printf is libultra's printf formatter (sprintf/osSyncPrintf backend). The port logs
 * via PortDbg, so a no-op formatter is enough to link and boot silently. Revisit if any
 * gameplay path needs real sprintf output. */
int _Printf() { return 0; }

/* PORT (2026-09-18): heap sizing. The APP heap (newlib malloc, at 0x08000000) holds the
 * game's DMA'd scene/object assets — Hyrule Field fills it well past 8MB, so a too-small
 * app heap makes those writes land on unmapped memory (the 831K-write flood at ~0x08B2xxxx
 * that tanked emulation to 1% and crashed Azahar). Give it real room. The LINEAR heap
 * (linearAlloc, ~0x14000000) is only gfx (VBO 2MB + framebuffers + ~8MB textures).
 * Total resident: 42MB .data + these must fit the New-3DS 124MB grant. */
/* PORT (2026-09-23): the game arena moved to the LINEAR heap (arena_shim.c) so its
 * pointers can't collide with OoT segments 8-0xF. The app heap now only serves newlib/
 * citro3d/libctru host allocations; linear holds gfx (VBO/framebuffers/textures ~12MB)
 * plus the whole game arena (scenes, objects, display-list pools, UI segments). */
/* PORT (2026-09-28): the regular heap needs little (measured ~0.5MB: newlib, JPEG decode buffers); the
 * linear heap (game arena, GPU buffers, textures; measured ~11MB peak in play) takes all the rest, so
 * the same binary fits the New 3DS 124MB mode (~66MB linear) and the Old 3DS 96MB mode (~39MB linear).
 * The image itself is ~51MB (assets compiled in). Was a fixed 16MB + 52MB, which needs 124MB. */
unsigned __ctru_heap_size = 6u * 1024 * 1024;
unsigned __ctru_linear_heap_size = 0; /* 0 = everything left (libctru __system_allocateHeaps) */

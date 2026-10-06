/* statedump.c - write raw game state to the SD card for tools/statediff (N64-vs-3DS state diff).
 * Enabled only in builds with GAME_EXTRA=-DPORT_STATEDUMP=<gameplayFrames>; the hook in
 * Play_Update calls these once, at the start of that frame. Files (all raw, native byte order):
 *   sdmc:/3ds/oot/sd_play.bin    PlayState
 *   sdmc:/3ds/oot/sd_save.bin    SaveContext
 *   sdmc:/3ds/oot/sd_actors.bin  per actor: u32 category, u32 size, then the whole instance
 *   sdmc:/3ds/oot/sd_rng.bin     RNG trace (see PortRngTrace below)
 *   sdmc:/3ds/oot/sd_depth.bin   u32 width, u32 height, then the previous frame's D24S8 depth (linear)
 * tools/statediff/statediff.py reads the same structs from ares (N64) and diffs field by field. */
#include <stdio.h>
#include <stdint.h>
#include <sys/stat.h>

static FILE* sActors;

static void write_file(const char* path, const void* data, unsigned size) {
    FILE* f = fopen(path, "wb");
    if (f != NULL) {
        fwrite(data, 1, size, f);
        fclose(f);
    }
}

/* tools/make_navi_icon.py (GAME_EXTRA=-DPORT_NAVIGEN): the last finished frame's color, read back in RGBA8
 * (header width, height; the render target's portrait layout, as the statediff color dumps) */
void PortStateDump_WriteColor(const char* path) {
    extern const uint32_t* Port3ds_GetColor(int back, int* width, int* height);
    int w, h;
    const uint32_t* color = Port3ds_GetColor(0, &w, &h);
    FILE* f;
    if (color == NULL || (f = fopen(path, "wb")) == NULL) {
        return;
    }
    {
        uint32_t hdr[2] = { (uint32_t)w, (uint32_t)h };
        fwrite(hdr, 1, sizeof(hdr), f);
        fwrite(color, 4, (size_t)w * h, f);
    }
    fclose(f);
}

/* tools/make_link_banner.py (GAME_EXTRA=-DPORT_ICONGEN): Link's pause preview texture */
void PortStateDump_WriteFile(const char* path, const void* data, unsigned size) {
    write_file(path, data, size);
}

static int sDumpIndex;

static const char* dump_path(const char* name) {
    static char path[64];
    if (sDumpIndex < 0) {
        snprintf(path, sizeof(path), "sdmc:/3ds/oot/sd_%s.bin", name);
    } else {
        snprintf(path, sizeof(path), "sdmc:/3ds/oot/tour/sd_%s_%d.bin", name, sDumpIndex);
    }
    return path;
}

void PortStateDump_Begin(int index, const void* play, unsigned playSize, const void* save, unsigned saveSize) {
    extern const uint32_t* Port3ds_GetDepth(int* width, int* height);
    int w, h;
    const uint32_t* depth = Port3ds_GetDepth(&w, &h);

    sDumpIndex = index;
    if (index >= 0) {
        mkdir("sdmc:/3ds/oot/tour", 0777);
    }
    write_file(dump_path("play"), play, playSize);
    write_file(dump_path("save"), save, saveSize);
    { /* last two finished frames' color buffers (RGBA8, linear): renderer comparison vs ares */
        extern const uint32_t* Port3ds_GetColor(int back, int* width, int* height);
        int back;
        for (back = 0; back < 2; back++) {
            int cw, ch;
            const uint32_t* color = Port3ds_GetColor(back, &cw, &ch);
            if (color != NULL) {
                extern void PortGfx_WriteDrawLog(int back, const char* path);
                extern const uint32_t* Port3ds_GetDepthSlot(int back, int* width, int* height);
                int dw, dh;
                const uint32_t* dslot = Port3ds_GetDepthSlot(back, &dw, &dh);
                FILE* f;
                PortGfx_WriteDrawLog(back, dump_path(back ? "draws1" : "draws0")); /* .bin name, text content */
                if (dslot != NULL && (f = fopen(dump_path(back ? "depthst1" : "depthst0"), "wb")) != NULL) {
                    uint32_t dh2[2] = { (uint32_t)dw, (uint32_t)dh };
                    fwrite(dh2, 1, sizeof(dh2), f);
                    fwrite(dslot, 4, (size_t)dw * dh, f);
                    fclose(f);
                }
                f = fopen(dump_path(back ? "color1" : "color0"), "wb");
                if (f != NULL) {
                    uint32_t hdr[2] = { (uint32_t)cw, (uint32_t)ch };
                    fwrite(hdr, 1, sizeof(hdr), f);
                    fwrite(color, 4, (size_t)cw * ch, f);
                    fclose(f);
                }
            }
        }
    }
    if (depth != NULL) { /* previous frame's depth buffer, linear, as read back by gfx_3ds.c */
        FILE* f = fopen(dump_path("depth"), "wb");
        if (f != NULL) {
            uint32_t hdr[2] = { (uint32_t)w, (uint32_t)h };
            fwrite(hdr, 1, sizeof(hdr), f);
            fwrite(depth, 4, (size_t)w * h, f);
            fclose(f);
        }
    }
    sActors = fopen(dump_path("actors"), "wb");
}

/* game globals: concatenated raw values, sizes as given (see statediff.py GLOBALS) */
void PortStateDump_Globals(const void* const* ptrs, const uint16_t* sizes, int count) {
    FILE* f = fopen(dump_path("glob"), "wb");
    int i;
    if (f != NULL) {
        for (i = 0; i < count; i++) {
            fwrite(ptrs[i], 1, sizes[i], f);
        }
        fclose(f);
    }
}

void PortStateDump_Actor(unsigned category, const void* actor, unsigned size) {
    uint32_t hdr[2] = { category, size };
    if (sActors != NULL) {
        fwrite(hdr, 1, sizeof(hdr), sActors);
        fwrite(actor, 1, size, sActors);
    }
}

/* RNG trace: records of (kind, value). kind 0 = frame marker (value = frame, then a record
 * (0xFF, rng state)); kind 1 Rand_Next, 2 Rand_ZeroOne, 3 Rand_ZeroFloat, 4 Rand_CenteredFloat,
 * 5 Rand_S16Offset, 6 Rand_S16OffsetStride (value = caller return address). */
#define RNG_TRACE_MAX (256 * 1024)
static uint32_t sRngTrace[RNG_TRACE_MAX][2];
static uint32_t sRngCount;
static int sRngFrames;

void PortRngTrace(uint32_t kind, const void* caller) {
    if (sRngFrames && sRngCount < RNG_TRACE_MAX) {
        sRngTrace[sRngCount][0] = kind;
        sRngTrace[sRngCount][1] = (uint32_t)(uintptr_t)caller;
        sRngCount++;
    }
}

void PortRngTrace_Frame(uint32_t frame, uint32_t state) {
    extern void Port3ds_RequestDepth(void);
    extern void Port3ds_RequestColor(void);
    Port3ds_RequestDepth(); /* keep the depth readback running so sd_depth.bin can be written */
    Port3ds_RequestColor(); /* and the color readback for sd_color*.bin (renderer ground truth) */
    sRngFrames = 1;
    if (sRngCount + 2 <= RNG_TRACE_MAX) {
        sRngTrace[sRngCount][0] = 0;
        sRngTrace[sRngCount][1] = frame;
        sRngTrace[sRngCount + 1][0] = 0xFF;
        sRngTrace[sRngCount + 1][1] = state;
        sRngCount += 2;
    }
}

/* extra named blob for the current capture (sd_<name>_<index>.bin) */
void PortStateDump_Blob(const char* name, const void* data, unsigned size) {
    write_file(dump_path(name), data, size);
}

void PortStateDump_End(void) {
    extern void PortDbg(const char* str);
    if (sActors != NULL) {
        fclose(sActors);
        sActors = NULL;
    }
    if (sDumpIndex >= 0) {
        char msg[48];
        snprintf(msg, sizeof(msg), "statedump: tour capture %d", sDumpIndex);
        PortDbg(msg);
        return;
    }
    write_file("sdmc:/3ds/oot/sd_rng.bin", sRngTrace, sRngCount * 8);
    PortDbg("statedump: wrote sd_play/sd_save/sd_actors.bin");
}

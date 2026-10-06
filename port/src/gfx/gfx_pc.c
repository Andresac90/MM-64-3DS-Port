#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <assert.h>

#ifndef _LANGUAGE_C
#define _LANGUAGE_C
#endif
#include <PR/gbi.h>
#include "ultra64/gs2dex.h" /* uObjBg for S2DEX background rectangles */

#include "config.h"

#include "gfx_pc.h"
#include "port_prof.h"
#include "gfx_cc.h"
#include "gfx_window_manager_api.h"
#include "gfx_rendering_api.h"
#include "gfx_screen_config.h"

uintptr_t gfx_port_segments[16];
unsigned int gfx_port_tri_count;
unsigned int gfx_port_frame_index;
extern uintptr_t gSegments[];

#ifdef __3DS__
/* Log helpers (file-only, no vsync) used by the DL-walk crash guard below. */
extern void PortLogFast(const char* s);
extern void PortLogFastX(const char* label, unsigned val);
/* Top of the loaded binary image (3dsx linker symbol); bounds the native
 * static-image pointer range in seg_addr. */
extern char __end__[];
#endif

/* PORT: warn-once instead of abort so renderer limits don't crash */
#define SUPPORT_CHECK(x) do { static int _w=0; if(!(x)&&!_w){_w=1; fprintf(stderr, "[gfx] unsupported: %s\n", #x);} } while(0)

/* PORT: texel/TLUT byte access, endian-corrected.
 * Texture and palette data are stored as u64[] blobs. On the N64 (big-endian)
 * the 8 bytes of each u64 are in memory order; our native little-endian build
 * reverses them within each u64, so the byte at logical offset k physically
 * lives at k^7 (base is u64-aligned). Every raw texel/palette read below indexes
 * through this to recover N64 byte order. Geometry (Vtx/Gfx) is unaffected — it's
 * typed struct fields the compiler already lays out correctly for LE. */
#ifdef __3DS__
/* PORT (2026-09-24): texel reads (TEXB) now come from the logical-order staging buffer built by
 * gfx_gather_texture(); palette reads (TEXP) still read memory in place, so they keep the
 * per-8-byte swizzle of natively-compiled (u64-array) assets. */
#define TEXB(base, k) ((base)[(k)])
#define TEXP(base, k) ((base)[(uintptr_t)(k) ^ 7u])
#define TEX_SRC_BYTE(p) (*(const uint8_t*)((uintptr_t)(p) ^ 7u)) /* absolute-address unswizzle */
#define TEX_SRC_BYTE_X(p, x) (*(const uint8_t*)((uintptr_t)(p) ^ (x)))
#else
#define TEX_SRC_BYTE_X(p, x) (*(const uint8_t*)(p))
#define TEXB(base, k) ((base)[(k)])
#define TEXP(base, k) ((base)[(k)])
#define TEX_SRC_BYTE(p) (*(const uint8_t*)(p))
#endif

// SCALE_M_N: upscale/downscale M-bit integer to N-bit
#define SCALE_5_8(VAL_) (((VAL_) * 0xFF) / 0x1F)
#define SCALE_8_5(VAL_) ((((VAL_) + 4) * 0x1F) / 0xFF)
#define SCALE_4_8(VAL_) ((VAL_) * 0x11)
#define SCALE_8_4(VAL_) ((VAL_) / 0x11)
#define SCALE_3_8(VAL_) ((VAL_) * 0x24)
#define SCALE_8_3(VAL_) ((VAL_) / 0x24)

#define HALF_SCREEN_WIDTH (SCREEN_WIDTH / 2)
#define HALF_SCREEN_HEIGHT (SCREEN_HEIGHT / 2)

#define RATIO_X (gfx_current_dimensions.width / (2.0f * HALF_SCREEN_WIDTH))
#define RATIO_Y (gfx_current_dimensions.height / (2.0f * HALF_SCREEN_HEIGHT))

/* 4:3 content is centered in the target: N64 x -> x * RATIO_Y + GFX_PILLAR_X */
#define GFX_PILLAR_X ((gfx_current_dimensions.width - SCREEN_WIDTH * RATIO_Y) / 2.0f)

#define MAX_BUFFERED 256
/* PORT (2026-09-25): F3DEX2 supports 7 directional lights + ambient (G_MAX_LIGHTS); sm64 used 2. OoT
 * sends up to 7 (e.g. torch-lit rooms): with 2, G_MW_NUMLIGHT made the lighting loop write
 * current_lights_coeffs past the array, which corrupted current_num_lights and then everything after
 * rsp (buf_vbo_len / buf_vbo_num_tris -> out-of-bounds vertex reads, found by the tools/statediff
 * scene tour in the Treasure Chest Shop), and lights 3..7 were dropped. */
#define MAX_LIGHTS 7
#define MAX_VERTICES 64

struct RGBA {
    uint8_t r, g, b, a;
};

struct XYWidthHeight {
    uint16_t x, y, width, height;
};

struct LoadedVertex {
    float x, y, z, w;
    float u, v;
    struct RGBA color;
    uint8_t clip_rej;
};

struct TextureHashmapNode {
    struct TextureHashmapNode *next;
    
    const uint8_t *texture_addr;
    uint8_t fmt, siz;
    uint32_t size_bytes; /* PORT: part of the key -- tile loads can share a start address */
    uint32_t pal_hash;   /* PORT: CI textures: hash of the palette colors they use (0 otherwise) */
    uint32_t line_size_bytes; /* PORT: the tile's row pitch: same bytes, different width = different texture */
    uint16_t width, height;   /* PORT: dimensions uploaded (texture coordinates are normalized to them) */
    
    uint32_t texture_id;
    uint8_t cms, cmt;
    bool linear_filter;
};
static struct {
    struct TextureHashmapNode *hashmap[1024];
    struct TextureHashmapNode pool[512];
    uint32_t pool_pos;
} gfx_texture_cache;


static struct RSP {
    float modelview_matrix_stack[11][4][4];
    uint8_t modelview_matrix_stack_size;
    
    float MP_matrix[4][4];
    float P_matrix[4][4];
    
    Light_t current_lights[MAX_LIGHTS + 1];
    float current_lights_coeffs[MAX_LIGHTS][3];
    float current_lookat_coeffs[2][3]; // lookat_x, lookat_y
    Light_t current_lookat[2]; /* the game's gSPLookAt (G_MV_LIGHT offsets 0 and 24) */
    uint8_t current_num_lights; // includes ambient light
    bool lights_changed;
    
    uint32_t geometry_mode;
    int16_t fog_mul, fog_offset;
    uint8_t clip_ratio; /* G_MW_CLIP guard band (FRUSTRATIO_n) */
    float half_px_x, half_px_y; /* half an N64 pixel of the current viewport, in NDC */
    
    struct {
        // U0.16
        uint16_t s, t;
    } texture_scaling_factor;
    
    struct LoadedVertex loaded_vertices[MAX_VERTICES + 4];
} rsp;

static struct RDP {
    const uint8_t *palette;
    struct {
        const uint8_t *addr;
        uint8_t siz;
        uint32_t width; /* texture image width in texels (G_SETTIMG), needed by LOADTILE */
    } texture_to_load;
    struct {
        const uint8_t *addr;
        uint32_t size_bytes;
        uint32_t line_size_bytes;            /* bytes loaded per row */
        uint32_t full_image_line_size_bytes; /* source row stride (== line for LOADBLOCK) */
    } loaded_texture[2];
    /* PORT (2026-09-25): the 8 RDP tile descriptors (was: only the render tile). Texture unit i draws
     * with tile first_tile + i (G_TEXTURE's tile), whose TMEM slot is tmem != 0 (libultraship's model:
     * one texture at TMEM 0, another anywhere else). TEXEL1 had no tile of its own before. */
    struct TileDesc {
        uint8_t fmt;
        uint8_t siz;
        uint8_t palette; /* CI4 palette bank (G_SETTILE palette field) */
        uint8_t cms, cmt;
        uint8_t shifts, shiftt;
        uint16_t uls, ult, lrs, lrt; // U10.2
        uint32_t line_size_bytes;
        uint16_t tmem;
    } tiles[8];
    uint8_t first_tile;
    bool drawing_rect; /* texture rectangles take no bilinear half-texel offset */
    bool textures_changed[2];
    /* PORT (2026-09-25): TLUT memory like the RDP's upper TMEM: 256 colors (logical big-endian RGBA5551),
     * loaded by G_LOADTLUT at (load tile tmem - 256); CI4 reads bank tile.palette*16, CI8 all 256.
     * Before, "the last TLUT's address" was used for every CI texture (bank ignored) - tools/statediff
     * fbdiff attributed most wrong pixels in Kokiri Forest / Fire Temple to CI draws. */
    uint16_t tlut[256];
    
    uint32_t other_mode_l, other_mode_h;
    uint64_t combine_mode; /* raw G_SETCOMBINE: (w0 & 0xFFFFFF) << 32 | w1, decoded per draw (gfx_cc_key) */
    uint8_t prim_lod_frac;

    struct RGBA env_color, prim_color, fog_color, fill_color;
    struct XYWidthHeight viewport, scissor;
    bool viewport_or_scissor_changed;
    void *z_buf_address;
    void *color_image_address;
    uint32_t color_image_width;
    float vp_raw[4];          /* N64 viewport: x, y (bottom origin), width, height, in N64 pixels */
    uint32_t scissor_raw[4];  /* G_SETSCISSOR ulx, uly, lrx, lry (10.2) */
} rdp;

static struct RenderingState {
    bool depth_test;
    bool depth_mask;
    bool decal_mode;
    bool alpha_blend;
    struct XYWidthHeight viewport, scissor;
    struct ShaderProgram *shader_program;
    uint64_t cc_id0; /* combiner key of shader_program (gfx_cc.h) */
    uint32_t cc_id1;
    struct GfxCombineConsts consts; /* last constants handed to the backend */
    struct TextureHashmapNode *textures[2];
} rendering_state;

struct GfxDimensions gfx_current_dimensions;

static bool dropped_frame;

static float buf_vbo[MAX_BUFFERED * (26 * 3)]; // 3 vertices in a triangle and 26 floats per vtx
static size_t buf_vbo_len;
static size_t buf_vbo_num_tris;
#ifdef __3DS__
/* bench=1 (3ds_main.c): the array path on alternate frames, to A/B the indexed batches on hardware */
int gPortLegacyVbo;
/* indexed batches (see pack_vertex) */
static const uint8_t* sCurVidx;          /* loaded-vertex slots of the triangle emitted unsplit, or NULL */
static uint32_t sBatchId = 1;
static uint16_t sBatchVtx[MAX_VERTICES + 4];
static uint32_t sBatchStamp[MAX_VERTICES + 4];
#endif

static struct GfxWindowManagerAPI *gfx_wapi;
static struct GfxRenderingAPI *gfx_rapi;

/* PORT PERF (2026-10-03): the sm64 PC port's get_time() timed every flush and texture import and threw the result
 * away; clock_gettime does 64-bit divisions in software on the ARM11 (no divide instruction): ~6% of the Old 3DS CPU
 * in Azahar PC samples (tools/pcprof.py). Removed. */

/* PORT PERF (2026-09-28): stage timers (opt-in, see PORT_PERF_STAGES) for the frame profiler (3ds_main.c Port3ds_PerfReport), in
 * 268 MHz ticks: texture import, vertex transform, triangle setup (excluding flushes), backend draws. */
#ifdef __3DS__
#include <3ds/svc.h>
#include <3ds/synchronization.h>
u64 gPortPerfTex, gPortPerfVtx, gPortPerfTri, gPortPerfFlush, gPortPerfEmit, gPortPerfMtx;
u32 gPortPerfTexImports;
/* opt-in: compile-time PORT_PERF_STAGES, or at run time with perf_stages=1 in settings.txt (one tick
 * read per triangle while on) */
int gPortPerfStagesOn;
u32 gPortPerfDlCmds, gPortPerfDlCalls; /* display-list commands interpreted / G_DL calls, per report */
u32 gPortPerfOpCounts[256];              /* per opcode, per report (top ones logged) */
u64 gPortPerfOpTicks[256];
u32 gPortPerfSlowTris; /* triangles that re-evaluated the render state (not the fast path) */               /* perf_stages: ticks per opcode (G_DL sub-lists excluded) */
int gPortO3dsSim;
int gPortPerfAB; /* settings perf_ab=1: 3ds_main.c alternates N3DS / O3DS-sim speed for measurements */
#ifdef PORT_PERF_STAGES
#define PERF_T() svcGetSystemTick()
#else
#define PERF_T() (gPortPerfStagesOn ? svcGetSystemTick() : 0)
#endif
#else
#define PERF_T() 0
static uint64_t gPortPerfTex, gPortPerfVtx, gPortPerfTri, gPortPerfFlush, gPortPerfEmit, gPortPerfMtx;
static uint32_t gPortPerfTexImports;
#endif
static void gfx_flush_impl(void);
static void import_texture_impl(int unit, int tile_index);
static float gfx_adjust_x_for_aspect_ratio(float x);
/* GPU vertex path (gpu_vtx=1): the batch's matrix palette, flushed with the draw (see "GPU vertex path") */
extern int gPortGpuVtx;
extern void gfx_citro3d_draw_indexed(void);
#ifdef __3DS__
static int sInRoomDl;
u64 gPortPerfRoomTicks;
#ifdef PORT_ACTOR_PROF
u64 gPortActorTicks[512];
u32 gPortActorCalls[512];
#endif /* time inside room geometry display lists (z_room.c tags), per report */  /* walking the room's geometry (z_room.c G_NOOP tags 0x3D5E5200 / 0x3D5E5201) */
u32 gPortPerfRoomTris; /* perf report */
#endif
#define GPU_PAL 18
static uint16_t sGpuPal[GPU_PAL];
static int sGpuPalN, sGpuPalSent; /* entries in use / entries already uploaded to the GPU */
static uint32_t sGpuPalEpoch = 1; /* bumps when the palette is emptied (full, new frame) */
static void gfx_flush(void) {
    uint64_t t0 = PERF_T();
    PROF_PUSH(PROF_FLUSH);
    gfx_flush_impl();
    PROF_POP();
    gPortPerfFlush += PERF_T() - t0;
}
static void import_texture(int unit, int tile_index) {
    uint64_t t0 = PERF_T();
    PROF_PUSH(PROF_TEX);
    import_texture_impl(unit, tile_index);
    PROF_POP();
    gPortPerfTex += PERF_T() - t0;
    gPortPerfTexImports++;
}

#ifdef __3DS__
/* PORT (2026-09-24): minimal render-target routing. The citro3d backend has ONE target (the
 * screen), but OoT renders into other color images too: the pause menu's Link preview
 * (Player_DrawPause -> its own color/depth buffers), PreRender framebuffer save/copy/filter
 * passes, transition tiles. Painting those onto the screen covered the pause menu with
 * their full-rect clears. Only draws whose color image is a real framebuffer (SysCfb) reach
 * the screen; other targets are dropped until real render-to-texture exists (roadmap G10). */
extern uintptr_t sSysCfbFbPtr[2];
static inline int gfx_cimg_is_screen(void) {
    uintptr_t a = (uintptr_t)rdp.color_image_address;
    /* a framebuffer address with another width is a different image (Player_DrawPause's 64-wide one) */
    return a == 0 || ((a == sSysCfbFbPtr[0] || a == sSysCfbFbPtr[1]) &&
                      (rdp.color_image_width == 0 || rdp.color_image_width == SCREEN_WIDTH));
}
#else
static inline int gfx_cimg_is_screen(void) { return 1; }
#endif

#ifdef __3DS__
/* Per-draw attribution for tools/statediff fbdiff (renderer vs ares): while the color readback runs,
 * every flush (one render state) gets an id 1..254 that the backend writes into the stencil buffer,
 * and its state is logged here. A differing pixel then names the draw - and the GBI state - that
 * produced it. Logs are kept per finished frame, aligned with gfx_3ds.c's color/depth slots. */
typedef struct {
    uint16_t id, tris;
    uint64_t cc; /* combiner key (gfx_cc.h) */
    uint32_t omh, oml, geo, prim, env;
    const void* tex0;
    const void* tex1;
    uint8_t fmt, siz, blend, cimg_screen, pal;
    uint16_t tw, th;
    const void* tlutsrc;
    uint16_t tlut0, tlut1; /* first two colors of the bank the draw uses (CI) */
    uint32_t fogcol;
    int16_t fogmul, fogoff;
    float v0[10]; /* first packed vertex: pos(4) [uv(2)] then fog / color floats */
    const void* cmd; /* DL command (address) that produced the last geometry of the batch */
    uint16_t nsub, nbehind; /* source triangles subdivided / clipped for a vertex behind the eye */
} PortDrawRec;
static uint16_t sBatchSub, sBatchBehind;
#define PORT_DRAWLOG_MAX 2048
static PortDrawRec sDrawLogCur[PORT_DRAWLOG_MAX], sDrawLog[2][PORT_DRAWLOG_MAX];
static const Gfx* sPortCurCmd;   /* command being interpreted */
static int sDrawLogCurN, sDrawLogN[2], sDrawSeq;
extern int Port3ds_DrawIdActive(void);
extern int Port3ds_ColorLatestSlot(void);
extern void gfx_citro3d_set_draw_id(int id);

static PortDrawRec sPendingRec; /* state of the batch being built, captured at its first triangle */

/* called for every triangle while buffering: the first triangle of a batch snapshots the state the
 * whole batch is drawn with (rdp can change before the batch is flushed, so flush time is too late) */
static void port_draw_snapshot(void) {
    PortDrawRec* r = &sPendingRec;
    if (buf_vbo_num_tris != 0 || !Port3ds_DrawIdActive()) {
        return;
    }
    r->cc = rendering_state.cc_id0;
    r->omh = rdp.other_mode_h;
    r->oml = rdp.other_mode_l;
    r->geo = rsp.geometry_mode;
    r->prim = (rdp.prim_color.r << 24) | (rdp.prim_color.g << 16) | (rdp.prim_color.b << 8) | rdp.prim_color.a;
    r->env = (rdp.env_color.r << 24) | (rdp.env_color.g << 16) | (rdp.env_color.b << 8) | rdp.env_color.a;
    r->tex0 = rdp.loaded_texture[0].addr;
    r->tex1 = rdp.loaded_texture[1].addr;
    const struct TileDesc* rt = &rdp.tiles[rdp.first_tile];
    r->fmt = rt->fmt;
    r->siz = rt->siz;
    r->tw = (rt->lrs - rt->uls + 4) / 4;
    r->th = (rt->lrt - rt->ult + 4) / 4;
    r->blend = rendering_state.alpha_blend;
    r->cimg_screen = gfx_cimg_is_screen();
    r->pal = rt->palette;
    r->tlutsrc = rdp.palette;
    r->tlut0 = rdp.tlut[((rt->siz == G_IM_SIZ_4b) ? (rt->palette & 15) * 16 : 0)];
    r->tlut1 = rdp.tlut[((rt->siz == G_IM_SIZ_4b) ? (rt->palette & 15) * 16 : 0) + 1];
    r->fogcol = (rdp.fog_color.r << 24) | (rdp.fog_color.g << 16) | (rdp.fog_color.b << 8) | rdp.fog_color.a;
    r->fogmul = rsp.fog_mul;
    r->fogoff = rsp.fog_offset;
    r->cmd = sPortCurCmd;
}

static void port_draw_id_begin(void) {
    PortDrawRec* r;
    if (!Port3ds_DrawIdActive()) {
        gfx_citro3d_set_draw_id(0);
        return;
    }
    r = (sDrawLogCurN < PORT_DRAWLOG_MAX) ? &sDrawLogCur[sDrawLogCurN++] : NULL;
    sDrawSeq++;
    gfx_citro3d_set_draw_id((sDrawSeq - 1) % 254 + 1);
    if (r != NULL) {
        *r = sPendingRec;
        r->id = (sDrawSeq - 1) % 254 + 1;
        r->tris = buf_vbo_num_tris;
        r->nsub = sBatchSub;
        r->nbehind = sBatchBehind;
        memcpy(r->v0, buf_vbo, sizeof(r->v0));
    }
}

static void port_draw_log_commit(void) { /* after the frame's readbacks: log -> that frame's slot */
    int slot = Port3ds_ColorLatestSlot();
    if (slot >= 0 && Port3ds_DrawIdActive()) {
        memcpy(sDrawLog[slot], sDrawLogCur, sizeof(PortDrawRec) * sDrawLogCurN);
        sDrawLogN[slot] = sDrawLogCurN;
    }
    sDrawLogCurN = 0;
    sDrawSeq = 0;
}

/* back = 0: most recently finished frame (same slot convention as Port3ds_GetColor) */
void PortGfx_WriteDrawLog(int back, const char* path) {
    int slot = Port3ds_ColorLatestSlot(), i;
    FILE* f;
    if (slot < 0 || (f = fopen(path, "w")) == NULL) {
        return;
    }
    if (back) {
        slot ^= 1;
    }
    fprintf(f, "# id tris cc omh oml geo prim env tex0 tex1 fmt siz tw th blend screen pal tlutsrc tlut0 tlut1 fogcol fogmul fogoff v0[0..9]\n");
    for (i = 0; i < sDrawLogN[slot]; i++) {
        const PortDrawRec* r = &sDrawLog[slot][i];
        fprintf(f, "%u %u %016llx %08lx %08lx %08lx %08lx %08lx %p %p %u %u %u %u %u %u %u %p %04x %04x\n", r->id,
                r->tris, (unsigned long long)r->cc, (unsigned long)r->omh, (unsigned long)r->oml, (unsigned long)r->geo,
                (unsigned long)r->prim, (unsigned long)r->env, r->tex0, r->tex1, r->fmt, r->siz, r->tw, r->th,
                r->blend, r->cimg_screen, r->pal, r->tlutsrc, r->tlut0, r->tlut1);
        fseek(f, -1, SEEK_CUR); /* append the fog fields to the same line */
        fprintf(f, " %08lx %d %d %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n", (unsigned long)r->fogcol, r->fogmul,
                r->fogoff, r->v0[0], r->v0[1], r->v0[2], r->v0[3], r->v0[4], r->v0[5], r->v0[6], r->v0[7], r->v0[8],
                r->v0[9]);
        fseek(f, -1, SEEK_CUR);
        fprintf(f, " cmd=%p sub=%u behind=%u\n", r->cmd, r->nsub, r->nbehind);
    }
    fclose(f);
}
#endif

static void gfx_flush_impl(void) {
    if (buf_vbo_len > 0) {
#ifdef __3DS__
        port_draw_id_begin();
#endif
#ifdef __3DS__
        if (gPortGpuVtx) {
            extern void gfx_citro3d_gpu_palette(const uint16_t* slots, int start, int n);
            if (sGpuPalN > sGpuPalSent) {
                gfx_citro3d_gpu_palette(sGpuPal + sGpuPalSent, sGpuPalSent, sGpuPalN - sGpuPalSent);
                sGpuPalSent = sGpuPalN;
            }
            gfx_citro3d_draw_indexed();
        } else if (gPortLegacyVbo) {
            gfx_rapi->draw_triangles(buf_vbo, buf_vbo_len, buf_vbo_num_tris); /* array path (A/B bench) */
        } else {
            gfx_citro3d_draw_indexed();
        }
        sBatchId++; /* vertex reuse is per batch */
        sBatchSub = sBatchBehind = 0;
#else
        gfx_rapi->draw_triangles(buf_vbo, buf_vbo_len, buf_vbo_num_tris);
#endif
        buf_vbo_len = 0;
        buf_vbo_num_tris = 0;
    }
}

/* PORT (2026-09-25): combiner key (gfx_cc.h), decoded from the raw G_SETCOMBINE words per the RDP
 * mux tables (A,B: 4 bits, C: 5 bits, D: 3 bits for RGB; 3 bits each for alpha, where C has its own
 * table). Replaces the sm64 3-bit encoding (no ONE / COMBINED / alpha broadcasts, cycle 1 dropped). */
static uint8_t cc_rgb_src(uint32_t v, int slot) {
    switch (v) {
        case G_CCMUX_COMBINED: return CCS_COMB;
        case G_CCMUX_TEXEL0: return CCS_TEX0;
        case G_CCMUX_TEXEL1: return CCS_TEX1;
        case G_CCMUX_PRIMITIVE: return CCS_PRIM;
        case G_CCMUX_SHADE: return CCS_SHADE;
        case G_CCMUX_ENVIRONMENT: return CCS_ENV;
        case 6: return (slot == 0 || slot == 3) ? CCS_1 : CCS_0; /* B: key center, C: key scale */
        case 7: return slot == 2 ? CCS_COMBA : CCS_0;          /* A: noise, B: K4, D: zero */
    }
    if (slot == 2) {
        switch (v) {
            case G_CCMUX_TEXEL0_ALPHA: return CCS_TEX0A;
            case G_CCMUX_TEXEL1_ALPHA: return CCS_TEX1A;
            case G_CCMUX_PRIMITIVE_ALPHA: return CCS_PRIMA;
            case G_CCMUX_SHADE_ALPHA: return CCS_SHADEA;
            case G_CCMUX_ENV_ALPHA: return CCS_ENVA;
            case G_CCMUX_LOD_FRACTION: return CCS_LODF;
            case G_CCMUX_PRIM_LOD_FRAC: return CCS_PRIMLODF;
        }
    }
    return CCS_0;
}

static uint8_t cc_alpha_src(uint32_t v, int slot) {
    switch (v) {
        case 0: return slot == 2 ? CCS_LODF : CCS_COMB;
        case G_ACMUX_TEXEL0: return CCS_TEX0;
        case G_ACMUX_TEXEL1: return CCS_TEX1;
        case G_ACMUX_PRIMITIVE: return CCS_PRIM;
        case G_ACMUX_SHADE: return CCS_SHADE;
        case G_ACMUX_ENVIRONMENT: return CCS_ENV;
        case 6: return slot == 2 ? CCS_PRIMLODF : CCS_1;
    }
    return CCS_0;
}

static bool cc_refs(const uint8_t s[4], uint8_t a, uint8_t b) {
    for (int i = 0; i < 4; i++) {
        if (s[i] == a || s[i] == b) {
            return true;
        }
    }
    return false;
}

static uint64_t gfx_cc_key(bool two_cycle, bool use_alpha) {
    uint32_t w0 = (uint32_t)(rdp.combine_mode >> 32), w1 = (uint32_t)rdp.combine_mode;
    const uint32_t m[2][2][4] = {
        { { (w0 >> 20) & 0xF, (w1 >> 28) & 0xF, (w0 >> 15) & 0x1F, (w1 >> 15) & 7 },
          { (w0 >> 12) & 7, (w1 >> 12) & 7, (w0 >> 9) & 7, (w1 >> 9) & 7 } },
        { { (w0 >> 5) & 0xF, (w1 >> 24) & 0xF, w0 & 0x1F, (w1 >> 6) & 7 },
          { (w1 >> 21) & 7, (w1 >> 3) & 7, (w1 >> 18) & 7, w1 & 7 } },
    };
    uint8_t s[2][2][4];
    for (int c = 0; c < 2; c++) {
        for (int ch = 0; ch < 2; ch++) {
            uint8_t* t = s[c][ch];
            for (int j = 0; j < 4; j++) {
                t[j] = ch ? cc_alpha_src(m[c][ch][j], j) : cc_rgb_src(m[c][ch][j], j);
                if (!two_cycle) { /* 1-cycle: TEXEL1 reads the same texel as TEXEL0 (libultraship) */
                    t[j] = t[j] == CCS_TEX1 ? CCS_TEX0 : t[j] == CCS_TEX1A ? CCS_TEX0A : t[j];
                } else if (c == 1) { /* the 2nd cycle sees the next tile: TEXEL0 <-> TEXEL1 */
                    static const uint8_t sw[16] = { 0, 1, CCS_TEX1, CCS_TEX0, CCS_TEX1A, CCS_TEX0A, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };
                    t[j] = sw[t[j]];
                } else if (t[j] == CCS_COMB || t[j] == CCS_COMBA) {
                    t[j] = CCS_0; /* cycle 0 COMBINED: previous pixel's result, unused by OoT */
                }
            }
            if (t[0] == t[1] || t[2] == CCS_0) { /* (A-A)*C+D, (A-B)*0+D -> D */
                t[0] = t[1] = t[2] = CCS_0;
            }
        }
    }
    if (!two_cycle) {
        for (int ch = 0; ch < 2; ch++) {
            s[1][ch][0] = s[1][ch][1] = s[1][ch][2] = CCS_0;
            s[1][ch][3] = CCS_COMB;
        }
    } else {
        if (!cc_refs(s[1][0], CCS_COMB, CCS_COMB)) { /* cycle-0 RGB unused */
            s[0][0][0] = s[0][0][1] = s[0][0][2] = s[0][0][3] = CCS_0;
        }
        if (!cc_refs(s[1][0], CCS_COMBA, CCS_COMBA) && !cc_refs(s[1][1], CCS_COMB, CCS_COMB)) { /* cycle-0 alpha */
            s[0][1][0] = s[0][1][1] = s[0][1][2] = s[0][1][3] = CCS_0;
        }
    }
    if (!use_alpha) { /* alpha unused (no blending, no alpha test): constant 1 */
        s[0][1][0] = s[0][1][1] = s[0][1][2] = CCS_0;
        s[0][1][3] = CCS_1;
        s[1][1][0] = s[1][1][1] = s[1][1][2] = CCS_0;
        s[1][1][3] = CCS_COMB;
    }
    uint64_t key = 0;
    for (int c = 0; c < 2; c++) {
        for (int ch = 0; ch < 2; ch++) {
            for (int j = 0; j < 4; j++) {
                key |= (uint64_t)s[c][ch][j] << (c * 32 + ch * 16 + j * 4);
            }
        }
    }
    return key;
}

/* which inputs a key reads: SHADER_OPT_TEX0/TEX1, and bits 8.. = 1 << CCS_* for the constants */
static uint32_t gfx_cc_usage(uint64_t key) {
    uint32_t u = 0;
    for (int i = 0; i < 16; i++) {
        uint8_t v = (key >> (i * 4)) & 0xF;
        if (v == CCS_TEX0 || v == CCS_TEX0A) {
            u |= SHADER_OPT_TEX0;
        } else if (v == CCS_TEX1 || v == CCS_TEX1A) {
            u |= SHADER_OPT_TEX1;
        } else {
            u |= 1u << (8 + v);
        }
    }
    return u;
}

/* PORT (2026-09-24): textures are cached by source address, which assumes texture memory never
 * changes. DMA'd buffers are reused in place (e.g. the A/B/START action labels in doActionSegment):
 * after a new label was DMA'd to the same address the cache kept returning the old one ("Attack"
 * instead of "Decide" on the pause screen). dma_shim.c calls this for every DMA destination; any
 * cached texture starting inside the written range is dropped (sentinel address never matches). */
#ifdef __3DS__
/* PORT (2026-10-03): with the render thread (3ds_main.c), the game thread loads data for the next frame while the
 * renderer draws the current one, which may still use the old contents (as the N64's RCP did). Invalidations from
 * other threads are queued and applied when the renderer starts its next frame (gfx_start_frame). */
extern int gPortRenderThreaded;
extern int Port3ds_OnRenderThread(void);
static struct { const void* start; uint32_t size; } sInvQ[64];
static int sInvN, sInvOverflow;
static LightLock sInvLock = 1; /* libctru: 1 = unlocked */
static void gfx_texture_cache_invalidate_now(const void* start, uint32_t size);
static void gfx_texture_cache_apply_queued(void) {
    int i, n;
    if (sInvN == 0 && !sInvOverflow) {
        return;
    }
    LightLock_Lock(&sInvLock);
    n = sInvN;
    for (i = 0; i < n; i++) {
        gfx_texture_cache_invalidate_now(sInvQ[i].start, sInvQ[i].size);
    }
    if (sInvOverflow) { /* more than the queue holds before one frame: drop every cached texture */
        gfx_texture_cache_invalidate_now((const void*)0, 0xFFFFFFFFu);
    }
    sInvN = 0;
    sInvOverflow = 0;
    LightLock_Unlock(&sInvLock);
}
void gfx_texture_cache_invalidate_range(const void* start, uint32_t size) {
    if (gPortRenderThreaded && !Port3ds_OnRenderThread()) {
        LightLock_Lock(&sInvLock);
        if (sInvN < (int)(sizeof(sInvQ) / sizeof(sInvQ[0]))) {
            sInvQ[sInvN].start = start, sInvQ[sInvN].size = size, sInvN++;
        } else {
            sInvOverflow = 1;
        }
        LightLock_Unlock(&sInvLock);
        return;
    }
    gfx_texture_cache_invalidate_now(start, size);
}
static void gfx_texture_cache_invalidate_now(const void* start, uint32_t size) {
#else
void gfx_texture_cache_invalidate_range(const void* start, uint32_t size) {
#endif
    const uint8_t* s = (const uint8_t*)start;
    const uint8_t* e = s + size;
    size_t i;
    if (e < s) e = (const uint8_t*)~(uintptr_t)0; /* the whole address space */
    for (i = 0; i < gfx_texture_cache.pool_pos; i++) {
        struct TextureHashmapNode* n = &gfx_texture_cache.pool[i];
        if (n->texture_addr >= s && n->texture_addr < e) {
            n->texture_addr = (const uint8_t*)1;
        }
    }
}

static uint32_t gfx_ci_palette_hash(uint32_t fmt, uint32_t siz);
static uint32_t sTlutHash16[16], sTlutHash256;
static int sTlutHashValid;
static const struct TileDesc* sImpTile; /* tile of the texture being imported (import_texture) */
static struct TextureHashmapNode* sImpNode; /* its cache node */

static bool gfx_texture_cache_lookup(int tile, struct TextureHashmapNode **n, const uint8_t *orig_addr, uint32_t fmt, uint32_t siz, uint32_t size_bytes) {
    uint32_t pal_hash = gfx_ci_palette_hash(fmt, siz);
    size_t hash = (uintptr_t)orig_addr;
    hash = (hash >> 5) & 0x3ff;
    struct TextureHashmapNode **node = &gfx_texture_cache.hashmap[hash];
    while (*node != NULL && *node - gfx_texture_cache.pool < gfx_texture_cache.pool_pos) {
        if ((*node)->texture_addr == orig_addr && (*node)->fmt == fmt && (*node)->siz == siz && (*node)->size_bytes == size_bytes &&
            (*node)->pal_hash == pal_hash && (*node)->line_size_bytes == sImpTile->line_size_bytes) {
            gfx_rapi->select_texture(tile, (*node)->texture_id);
            *n = *node;
            return true;
        }
        node = &(*node)->next;
    }
    if (gfx_texture_cache.pool_pos == sizeof(gfx_texture_cache.pool) / sizeof(struct TextureHashmapNode)) {
        // Pool is full. We just invalidate everything and start over.
        gfx_texture_cache.pool_pos = 0;
#ifdef __3DS__
        {
            static int sWraps;
            extern void PortDbgX(const char* label, unsigned val);
            if (sWraps++ < 16) {
                PortDbgX("TEXCACHE wrap at frame", (unsigned)gfx_port_frame_index);
            }
        }
#endif
        node = &gfx_texture_cache.hashmap[hash];
        //puts("Clearing texture cache");
    }
    *node = &gfx_texture_cache.pool[gfx_texture_cache.pool_pos++];
    if ((*node)->texture_addr == NULL) {
        (*node)->texture_id = gfx_rapi->new_texture();
    }
    gfx_rapi->select_texture(tile, (*node)->texture_id);
    gfx_rapi->set_sampler_parameters(tile, false, 0, 0);
    (*node)->cms = 0;
    (*node)->cmt = 0;
    (*node)->linear_filter = false;
    (*node)->next = NULL;
    (*node)->texture_addr = orig_addr;
    (*node)->pal_hash = pal_hash;
    (*node)->line_size_bytes = sImpTile->line_size_bytes;
    (*node)->width = (*node)->height = 0;
    (*node)->fmt = fmt;
    (*node)->siz = siz;
    (*node)->size_bytes = size_bytes;
    *n = *node;
    return false;
}

//do not allocate this buffer on the stack, as some platforms
//may have limited stack space
static uint8_t rgba32_buf[65536] __attribute__((aligned(32)));

#ifdef PORT_ICONGEN
/* tools/make_link_banner.py --model: Link's pause-menu preview as a 3D model for the HOME Menu banner. While the
 * preview's colour image is the target, every vertex is processed on the CPU (the N64's own lighting and texture
 * coordinates) and every triangle recorded with its world-space positions and material; each texture is decoded again
 * when a triangle first uses it (the tile state is that draw's). sdmc:/3ds/oot/link_mesh.bin is written once, after the
 * preview's 10th frame (it is drawn while the menu is open). */
extern u16 gPortIconGenBuf[];
typedef struct {
    uint64_t combine;
    uint32_t prim, env, tex, flags; /* flags: 1 texture used, 2 two-cycle, 4 cutout, 8 blend, cms << 8, cmt << 12 */
    float v[3][5];                  /* x y z u v */
    uint32_t c[3];                  /* r g b a */
} MeshTri;
#define MESH_MAX 1500 /* Link's preview: ~750 */
static MeshTri sMeshTris[MESH_MAX];
static int sMeshN, sMeshFrameHasTris, sMeshDbgVtx, sMeshDbgTri, sMeshDbgTri1;
static float sMeshPos[MAX_VERTICES + 4][3];
typedef struct {
    uint8_t* rgba;
    uint16_t w, h;
} MeshTex;
static MeshTex sMeshTex[4096];
static int mesh_capturing(void) {
    return rdp.color_image_address == (void*)gPortIconGenBuf;
}
static uint16_t sMeshTexOrder[4096]; /* ids by upload, oldest first: freed first when the heap runs out */
static int sMeshTexOrderN;
static void mesh_keep_texture(uint32_t id, const uint8_t* buf, uint32_t w, uint32_t h) {
    if (id < 4096 && w * h <= 16384) {
        int i, k;
        free(sMeshTex[id].rgba);
        sMeshTex[id].rgba = NULL;
        for (i = k = 0; i < sMeshTexOrderN; i++) { /* (re-uploaded: moves to the end) */
            if (sMeshTexOrder[i] != id) sMeshTexOrder[k++] = sMeshTexOrder[i];
        }
        sMeshTexOrderN = k;
        while ((sMeshTex[id].rgba = malloc(w * h * 4)) == NULL && sMeshTexOrderN > 0) {
            uint16_t old = sMeshTexOrder[0];
            free(sMeshTex[old].rgba);
            sMeshTex[old].rgba = NULL;
            memmove(sMeshTexOrder, sMeshTexOrder + 1, sizeof(uint16_t) * (size_t)--sMeshTexOrderN);
        }
        if (sMeshTex[id].rgba != NULL) {
            memcpy(sMeshTex[id].rgba, buf, w * h * 4);
            sMeshTex[id].w = (uint16_t)w, sMeshTex[id].h = (uint16_t)h;
            sMeshTexOrder[sMeshTexOrderN++] = (uint16_t)id;
        }
    }
}
static void mesh_write(void) {
    FILE* f = fopen("sdmc:/3ds/oot/link_mesh.tmp", "wb");
    uint8_t used[4096];
    uint32_t i, nt = 0;
    if (f == NULL) {
        return;
    }
    memset(used, 0, sizeof(used));
    fwrite("OOTM", 1, 4, f);
    fwrite(&sMeshN, 4, 1, f);
    fwrite(sMeshTris, sizeof(MeshTri), (size_t)sMeshN, f);
    for (i = 0; i < (uint32_t)sMeshN; i++) {
        if (sMeshTris[i].tex < 4096 && sMeshTex[sMeshTris[i].tex].rgba != NULL && !used[sMeshTris[i].tex]) {
            used[sMeshTris[i].tex] = 1, nt++;
        }
    }
    fwrite(&nt, 4, 1, f);
    for (i = 0; i < 4096; i++) {
        if (used[i]) {
            uint32_t hdr[3] = { i, sMeshTex[i].w, sMeshTex[i].h };
            fwrite(hdr, 4, 3, f);
            fwrite(sMeshTex[i].rgba, 4, (size_t)sMeshTex[i].w * sMeshTex[i].h, f);
        }
    }
    fclose(f);
    rename("sdmc:/3ds/oot/link_mesh.tmp", "sdmc:/3ds/oot/link_mesh.bin"); /* complete when it appears */
}
#endif

#ifdef PORT_ICONGEN
static int sMeshRedecode; /* import_texture_impl decodes for the capture only: no cache lookup, no upload */
#endif
static void gfx_upload_texture(uint8_t* buf, uint32_t width, uint32_t height) {
#ifdef PORT_ICONGEN
    if (sMeshRedecode) {
        mesh_keep_texture(sImpNode->texture_id, buf, width, height);
        return;
    }
#endif
    gfx_rapi->upload_texture(buf, width, height);
    sImpNode->width = width;
    sImpNode->height = height;
}

static void import_texture_rgba16(int tile) {
    for (uint32_t i = 0; i < rdp.loaded_texture[tile].size_bytes / 2; i++) {
        uint16_t col16 = (TEXB(rdp.loaded_texture[tile].addr, 2 * i) << 8) | TEXB(rdp.loaded_texture[tile].addr, 2 * i + 1);
        uint8_t a = col16 & 1;
        uint8_t r = col16 >> 11;
        uint8_t g = (col16 >> 6) & 0x1f;
        uint8_t b = (col16 >> 1) & 0x1f;
        rgba32_buf[4*i + 0] = SCALE_5_8(r);
        rgba32_buf[4*i + 1] = SCALE_5_8(g);
        rgba32_buf[4*i + 2] = SCALE_5_8(b);
        rgba32_buf[4*i + 3] = a ? 255 : 0;
    }
    
    uint32_t width = sImpTile->line_size_bytes / 2;
    uint32_t height = rdp.loaded_texture[tile].size_bytes / sImpTile->line_size_bytes;
    
    gfx_upload_texture(rgba32_buf, width, height);
}

static void import_texture_ia4(int tile) {
    for (uint32_t i = 0; i < rdp.loaded_texture[tile].size_bytes * 2; i++) {
        uint8_t byte = TEXB(rdp.loaded_texture[tile].addr, i / 2);
        uint8_t part = (byte >> (4 - (i % 2) * 4)) & 0xf;
        uint8_t intensity = part >> 1;
        uint8_t alpha = part & 1;
        uint8_t r = intensity;
        uint8_t g = intensity;
        uint8_t b = intensity;
        rgba32_buf[4*i + 0] = SCALE_3_8(r);
        rgba32_buf[4*i + 1] = SCALE_3_8(g);
        rgba32_buf[4*i + 2] = SCALE_3_8(b);
        rgba32_buf[4*i + 3] = alpha ? 255 : 0;
    }
    
    uint32_t width = sImpTile->line_size_bytes * 2;
    uint32_t height = rdp.loaded_texture[tile].size_bytes / sImpTile->line_size_bytes;

    gfx_upload_texture(rgba32_buf, width, height);
}

static void import_texture_ia8(int tile) {
    for (uint32_t i = 0; i < rdp.loaded_texture[tile].size_bytes; i++) {
        uint8_t intensity = TEXB(rdp.loaded_texture[tile].addr, i) >> 4;
        uint8_t alpha = TEXB(rdp.loaded_texture[tile].addr, i) & 0xf;
        uint8_t r = intensity;
        uint8_t g = intensity;
        uint8_t b = intensity;
        rgba32_buf[4*i + 0] = SCALE_4_8(r);
        rgba32_buf[4*i + 1] = SCALE_4_8(g);
        rgba32_buf[4*i + 2] = SCALE_4_8(b);
        rgba32_buf[4*i + 3] = SCALE_4_8(alpha);
    }
    
    uint32_t width = sImpTile->line_size_bytes;
    uint32_t height = rdp.loaded_texture[tile].size_bytes / sImpTile->line_size_bytes;
    
    gfx_upload_texture(rgba32_buf, width, height);
}

static void import_texture_ia16(int tile) {
    for (uint32_t i = 0; i < rdp.loaded_texture[tile].size_bytes / 2; i++) {
        uint8_t intensity = TEXB(rdp.loaded_texture[tile].addr, 2 * i);
        uint8_t alpha = TEXB(rdp.loaded_texture[tile].addr, 2 * i + 1);
        uint8_t r = intensity;
        uint8_t g = intensity;
        uint8_t b = intensity;
        rgba32_buf[4*i + 0] = r;
        rgba32_buf[4*i + 1] = g;
        rgba32_buf[4*i + 2] = b;
        rgba32_buf[4*i + 3] = alpha;
    }
    
    uint32_t width = sImpTile->line_size_bytes / 2;
    uint32_t height = rdp.loaded_texture[tile].size_bytes / sImpTile->line_size_bytes;
    
    gfx_upload_texture(rgba32_buf, width, height);
}

static void import_texture_ci4(int tile) {
    for (uint32_t i = 0; i < rdp.loaded_texture[tile].size_bytes * 2; i++) {
        uint8_t byte = TEXB(rdp.loaded_texture[tile].addr, i / 2);
        uint8_t idx = (byte >> (4 - (i % 2) * 4)) & 0xf;
        uint16_t col16 = rdp.tlut[((sImpTile->palette & 15) * 16 + idx) & 0xFF]; /* bank + index */
        uint8_t a = col16 & 1;
        uint8_t r = col16 >> 11;
        uint8_t g = (col16 >> 6) & 0x1f;
        uint8_t b = (col16 >> 1) & 0x1f;
        rgba32_buf[4*i + 0] = SCALE_5_8(r);
        rgba32_buf[4*i + 1] = SCALE_5_8(g);
        rgba32_buf[4*i + 2] = SCALE_5_8(b);
        rgba32_buf[4*i + 3] = a ? 255 : 0;
    }
    
    uint32_t width = sImpTile->line_size_bytes * 2;
    uint32_t height = rdp.loaded_texture[tile].size_bytes / sImpTile->line_size_bytes;
    
    gfx_upload_texture(rgba32_buf, width, height);
}

static void import_texture_ci8(int tile) {
    for (uint32_t i = 0; i < rdp.loaded_texture[tile].size_bytes; i++) {
        uint8_t idx = TEXB(rdp.loaded_texture[tile].addr, i);
        uint16_t col16 = rdp.tlut[idx];
        uint8_t a = col16 & 1;
        uint8_t r = col16 >> 11;
        uint8_t g = (col16 >> 6) & 0x1f;
        uint8_t b = (col16 >> 1) & 0x1f;
        rgba32_buf[4*i + 0] = SCALE_5_8(r);
        rgba32_buf[4*i + 1] = SCALE_5_8(g);
        rgba32_buf[4*i + 2] = SCALE_5_8(b);
        rgba32_buf[4*i + 3] = a ? 255 : 0;
    }
    
    uint32_t width = sImpTile->line_size_bytes;
    uint32_t height = rdp.loaded_texture[tile].size_bytes / sImpTile->line_size_bytes;
    
    gfx_upload_texture(rgba32_buf, width, height);
}

static void import_texture_i4(int tile) {
    /* uses module-static rgba32_buf (3DS stack is tiny; no 32KB stack local) */
    for (uint32_t i = 0; i < rdp.loaded_texture[tile].size_bytes * 2; i++) {
        uint8_t byte = TEXB(rdp.loaded_texture[tile].addr, i / 2);
        uint8_t part = (byte >> (4 - (i % 2) * 4)) & 0xf;
        uint8_t intensity = part << 4 | part;
        rgba32_buf[4*i + 0] = intensity;
        rgba32_buf[4*i + 1] = intensity;
        rgba32_buf[4*i + 2] = intensity;
        rgba32_buf[4*i + 3] = intensity;
    }
    uint32_t width = sImpTile->line_size_bytes * 2;
    uint32_t height = rdp.loaded_texture[tile].size_bytes / sImpTile->line_size_bytes;
    gfx_upload_texture(rgba32_buf, width, height);
}

static void import_texture_i8(int tile) {
    /* uses module-static rgba32_buf (3DS stack is tiny; no 64KB stack local) */
    for (uint32_t i = 0; i < rdp.loaded_texture[tile].size_bytes; i++) {
        uint8_t intensity = TEXB(rdp.loaded_texture[tile].addr, i);
        rgba32_buf[4*i + 0] = intensity;
        rgba32_buf[4*i + 1] = intensity;
        rgba32_buf[4*i + 2] = intensity;
        rgba32_buf[4*i + 3] = intensity;
    }
    uint32_t width = sImpTile->line_size_bytes;
    uint32_t height = rdp.loaded_texture[tile].size_bytes / sImpTile->line_size_bytes;
    gfx_upload_texture(rgba32_buf, width, height);
}

static void import_texture_rgba32(int tile) {
    /* Reorder bytes (TEXB) into the staging buffer rather than uploading the u64
     * blob directly, so the LE 8-byte reversal is corrected like the other formats. */
    for (uint32_t i = 0; i < rdp.loaded_texture[tile].size_bytes; i++) {
        rgba32_buf[i] = TEXB(rdp.loaded_texture[tile].addr, i);
    }
    uint32_t width = sImpTile->line_size_bytes / 2;
    uint32_t height = (rdp.loaded_texture[tile].size_bytes / 2) / sImpTile->line_size_bytes;
    gfx_upload_texture(rgba32_buf, width, height);
}

/* PORT (2026-09-29): the unswizzle for a texture/TLUT source. u64[] asset arrays and game-written RAM keep
 * logical byte k at address ^ 7; the few asset arrays the decomp emits as u32[] (Kokiri, other townsfolk:
 * textures only 4-byte aligned in the ROM) keep it at address ^ 3. Reading those with ^ 7 scrambled
 * their palettes and texels (striped hair, camouflage skin). Table: port/src_gen/u32_asset_ranges.c. */
#ifdef __3DS__
#include "port_u32_assets.h"
static uintptr_t gfx_src_swizzle(const void* p) {
    static const PortU32Asset* sSorted[256];
    static int sN = -1;
    uintptr_t a = (uintptr_t)p;
    int lo, hi;
    if (sN < 0) { /* sort the linked arrays by address once */
        int i, j;
        sN = 0;
        for (i = 0; i < gPortU32AssetCount && sN < 256; i++) {
            if (gPortU32Assets[i].addr != NULL) {
                const PortU32Asset* e = &gPortU32Assets[i];
                for (j = sN++; j > 0 && (uintptr_t)sSorted[j - 1]->addr > (uintptr_t)e->addr; j--) {
                    sSorted[j] = sSorted[j - 1];
                }
                sSorted[j] = e;
            }
        }
    }
    lo = 0;
    hi = sN - 1;
    while (lo <= hi) {
        int mid = (lo + hi) / 2;
        uintptr_t b = (uintptr_t)sSorted[mid]->addr;
        if (a < b) {
            hi = mid - 1;
        } else if (a >= b + sSorted[mid]->size) {
            lo = mid + 1;
        } else {
            return 3u;
        }
    }
    return 7u;
}
#else
#define gfx_src_swizzle(p) 0u
#endif

/* PORT (2026-09-24): copy the loaded texture into a staging buffer laid out like TMEM (each loaded
 * row at the render tile's line stride), in logical byte order. Handles LOADTILE sub-rects (source
 * stride = full image width) and unswizzles natively-compiled assets by ABSOLUTE address, which is
 * byte-identical to the old base-relative ^7 for 8-aligned bases. Returns staged size, 0 on overflow. */
static uint8_t sTexStage[65536] __attribute__((aligned(32)));
static uint32_t gfx_gather_texture(int tile) {
    const uint8_t* src = rdp.loaded_texture[tile].addr;
    uint32_t size = rdp.loaded_texture[tile].size_bytes;
    uint32_t row = rdp.loaded_texture[tile].line_size_bytes;
    uint32_t full = rdp.loaded_texture[tile].full_image_line_size_bytes;
    uint32_t dst_stride, rows, x, y;
    uintptr_t sw = gfx_src_swizzle(src);
    if (row == 0 || full == 0 || row == full) { /* contiguous (LOADBLOCK) */
        if (size > sizeof(sTexStage)) return 0;
        for (x = 0; x < size; x++) sTexStage[x] = TEX_SRC_BYTE_X(src + x, sw);
        return size;
    }
    rows = size / row;
    dst_stride = sImpTile->line_size_bytes;
    if (dst_stride < row) dst_stride = row;
    if (rows * dst_stride > sizeof(sTexStage)) return 0;
    for (y = 0; y < rows; y++) {
        for (x = 0; x < row; x++) sTexStage[y * dst_stride + x] = TEX_SRC_BYTE_X(src + y * full + x, sw);
        for (; x < dst_stride; x++) sTexStage[y * dst_stride + x] = 0;
    }
    return rows * dst_stride;
}

#ifdef PORT_TEXDUMP
/* DEBUG TOOL (opt-in, build with PORT_EXTRA=-DPORT_TEXDUMP): write every uniquely-addressed
 * texture the port decodes (source address, N64 fmt/siz, decoded RGBA8) to
 * sdmc:/3ds/oot/texdump.bin, for diffing against the decomp's golden PNGs
 * (tools/texdiff.py). Record: "TEXD" u32 addr, u8 fmt, u8 siz, u16 0, u32 w, u32 h, w*h*4 RGBA. */
static void texdump_record(const uint8_t* addr, uint8_t fmt, uint8_t siz, uint32_t size_bytes, uint32_t line_bytes) {
    static FILE* f = NULL;
    static const uint8_t* seen[512];
    static int nseen = 0, dead = 0;
    uint32_t bits, w, h, hdr[4];
    int i;
    if (dead || line_bytes == 0) return;
    for (i = 0; i < nseen; i++) if (seen[i] == addr) return;
    if (nseen >= 512) { if (f) { fclose(f); f = NULL; } dead = 1; return; }
    seen[nseen++] = addr;
    bits = (siz == G_IM_SIZ_4b) ? 4 : (siz == G_IM_SIZ_8b) ? 8 : (siz == G_IM_SIZ_16b) ? 16 : 32;
    w = (siz == G_IM_SIZ_32b) ? line_bytes / 2 : line_bytes * 8 / bits; /* 32b: TMEM line counts 2 bytes/texel */
    h = (siz == G_IM_SIZ_32b) ? (size_bytes / 2) / line_bytes : size_bytes / line_bytes;
    if (w == 0 || h == 0 || w * h * 4 > sizeof(rgba32_buf)) return;
    if (f == NULL) { f = fopen("sdmc:/3ds/oot/texdump.bin", "wb"); if (f == NULL) { dead = 1; return; } }
    hdr[0] = 0x44584554u; /* "TEXD" */
    hdr[1] = (uint32_t)(uintptr_t)addr;
    hdr[2] = (uint32_t)fmt | ((uint32_t)siz << 8);
    hdr[3] = w;
    fwrite(hdr, 4, 4, f);
    fwrite(&h, 4, 1, f);
    fwrite(rgba32_buf, 1, w * h * 4, f);
    fflush(f);
}
#endif

static void import_texture_impl(int unit, int tile_index) {
    sImpTile = &rdp.tiles[tile_index];
    int tile = sImpTile->tmem != 0; /* TMEM slot: the importers below index loaded_texture[] with it */
    uint8_t fmt = sImpTile->fmt;
    uint8_t siz = sImpTile->siz;

#ifdef PORT_ICONGEN
    if (!sMeshRedecode)
#endif
    if (gfx_texture_cache_lookup(unit, &rendering_state.textures[unit], rdp.loaded_texture[tile].addr, fmt, siz, rdp.loaded_texture[tile].size_bytes)) {
        return;
    }
    sImpNode = rendering_state.textures[unit];
#ifdef __3DS__
    /* Crash-proof the texel read: a bad segment/asset pointer can leave a texture
     * address in unmapped memory (the gap above the loaded image). Reading it in
     * the import loop would data-abort. Verify the whole texture is inside a mapped
     * block; if not, skip the upload (the previously-bound texture stays) rather
     * than crash — same philosophy as the DL-walk guard. */
    { extern unsigned PortMem_ReadableEnd(unsigned addr);
      uintptr_t a = (uintptr_t)rdp.loaded_texture[tile].addr;
      uint32_t _row = rdp.loaded_texture[tile].line_size_bytes, _full = rdp.loaded_texture[tile].full_image_line_size_bytes;
      uint32_t _extent = (_row && _full && _row != _full)
          ? (rdp.loaded_texture[tile].size_bytes / _row - 1) * _full + _row
          : rdp.loaded_texture[tile].size_bytes;
      unsigned e = PortMem_ReadableEnd((unsigned)a);
      if (e == 0 || a + _extent > (uintptr_t)e) {
          PortLogFastX("[TEX] skip unmapped tex", (unsigned)a);
          return;
      } }
#endif

    const uint8_t* orig_addr = rdp.loaded_texture[tile].addr;
    uint32_t orig_size = rdp.loaded_texture[tile].size_bytes;
    uint32_t staged = gfx_gather_texture(tile);
    if (staged == 0) return;
    rdp.loaded_texture[tile].addr = sTexStage; /* decoders read the logical-order staging copy */
    rdp.loaded_texture[tile].size_bytes = staged;
    if (fmt == G_IM_FMT_RGBA) {
        if (siz == G_IM_SIZ_16b) {
            import_texture_rgba16(tile);
        } else if (siz == G_IM_SIZ_32b) {
            import_texture_rgba32(tile);
        } else {
            goto unsupported;
        }
    } else if (fmt == G_IM_FMT_IA) {
        if (siz == G_IM_SIZ_4b) {
            import_texture_ia4(tile);
        } else if (siz == G_IM_SIZ_8b) {
            import_texture_ia8(tile);
        } else if (siz == G_IM_SIZ_16b) {
            import_texture_ia16(tile);
        } else {
            goto unsupported;
        }
    } else if (fmt == G_IM_FMT_CI) {
        if (siz == G_IM_SIZ_4b) {
            import_texture_ci4(tile);
        } else if (siz == G_IM_SIZ_8b) {
            import_texture_ci8(tile);
        } else {
            goto unsupported;
        }
    } else if (fmt == G_IM_FMT_I) {
        if (siz == G_IM_SIZ_4b) {
            import_texture_i4(tile);
        } else if (siz == G_IM_SIZ_8b) {
            import_texture_i8(tile);
        } else {
            goto unsupported;
        }
    } else {
    unsupported:
        fprintf(stderr, "[gfx] unsupported texture fmt %d siz %d (skipped)\n", fmt, siz);
    }
    rdp.loaded_texture[tile].addr = orig_addr;
    rdp.loaded_texture[tile].size_bytes = orig_size;
#ifdef PORT_TEXDUMP
    texdump_record(rdp.loaded_texture[tile].addr, fmt, siz, rdp.loaded_texture[tile].size_bytes,
                   sImpTile->line_size_bytes);
#endif
}

static void gfx_normalize_vector(float v[3]) {
    float s = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    v[0] /= s;
    v[1] /= s;
    v[2] /= s;
}

static void gfx_transposed_matrix_mul(float res[3], const float a[3], const float b[4][4]) {
    res[0] = a[0] * b[0][0] + a[1] * b[0][1] + a[2] * b[0][2];
    res[1] = a[0] * b[1][0] + a[1] * b[1][1] + a[2] * b[1][2];
    res[2] = a[0] * b[2][0] + a[1] * b[2][1] + a[2] * b[2][2];
}

static void calculate_normal_dir(const Light_t *light, float coeffs[3]) {
    float light_dir[3] = {
        light->dir[0] / 127.0f,
        light->dir[1] / 127.0f,
        light->dir[2] / 127.0f
    };
    gfx_transposed_matrix_mul(coeffs, light_dir, rsp.modelview_matrix_stack[rsp.modelview_matrix_stack_size - 1]);
    gfx_normalize_vector(coeffs);
}

static void gfx_matrix_mul(float res[4][4], const float a[4][4], const float b[4][4]) {
    float tmp[4][4];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            tmp[i][j] = a[i][0] * b[0][j] +
                        a[i][1] * b[1][j] +
                        a[i][2] * b[2][j] +
                        a[i][3] * b[3][j];
        }
    }
    memcpy(res, tmp, sizeof(tmp));
}

/* ---- PORT (2026-09-30): frame interpolation for 60 fps (docs/3ds-60fps-plan.md) ----
 * OoT's logic runs at 20 Hz. The port draws each logic frame's display list again for in-between frames
 * (t = 1/3, 2/3), with matrices (and skinned vertices) interpolated from the previous logic frame's to
 * this one's. Same model as Zelda64Recomp's RT64 matrix groups (gEXMatrixGroup) and Ship of Harkinian's
 * labelled recording: identity comes from TAGS the game code emits (port_interp.h: actor, limb, skin
 * limb, camera, skybox), never from guessing. A matrix is keyed by its group path (nested tag ids) and
 * its index inside the group; untagged matrices are drawn as they are. Rigid transforms are
 * interpolated decomposed (translation, per-axis scale, re-orthonormalised rotation) like RT64's
 * G_EX_INTERPOLATE_DECOMPOSE, so a turning limb does not shrink. The exact pass (t = 1) records. */
#include "port_interp.h"
extern int gPortReplayRec;
#define INTERP_MAX 4096
#define INTERP_HASH 8192
typedef struct {
    uint32_t key;
    float m[4][4];
} InterpMtx;
static InterpMtx sInterpTab[2][INTERP_MAX]; /* [sInterpCurTab] is being recorded, the other is the previous */
static int sInterpN[2];
static int sInterpCurTab;
static int16_t sInterpHash[INTERP_HASH]; /* previous table: key -> index + 1 */
/* skinned vertices (Skin system: Epona, ...): positions per keyed vertex load */
#define INTERP_VTX_MAX 12288
#define INTERP_VLOADS 512
typedef struct {
    uint32_t key;
    uint16_t n, first;
} InterpVLoad;
static int16_t sInterpVPos[2][INTERP_VTX_MAX][3];
static InterpVLoad sInterpVLoad[2][INTERP_VLOADS];
static int sInterpVN[2], sInterpVLN[2];
static int sInterpRecord = 0;
/* group stack (G_NOOP tags) */
#define INTERP_DEPTH 16
static uint32_t sGrpId[INTERP_DEPTH], sGrpMtx[INTERP_DEPTH], sGrpVtx[INTERP_DEPTH];
static uint8_t sGrpFlags[INTERP_DEPTH];
static int sGrpDepth;
/* occurrences of the same group path within one pass (a model drawn twice under one tag) */
static uint32_t sOccKey[2048], sOccCnt[2048], sOccStamp[2048], sInterpStamp;
uint32_t gPortInterpMatched, gPortInterpMissed, gPortInterpSame, gPortInterpJump, gPortInterpVtx, gPortInterpVtxMiss;

static uint32_t interp_mix(uint32_t a, uint32_t b) {
    uint32_t h = a * 0x9E3779B1u ^ (b + 0x7F4A7C15u + (a << 6) + (a >> 2));
    h ^= h >> 15;
    h *= 0x85EBCA6Bu;
    h ^= h >> 13;
    return h;
}

static void interp_group(uint32_t w0, uint32_t w1) {
    int op = (w0 >> 8) & 0xFF;
    if (op == PORT_INTERP_OP_PUSH) {
        if (sGrpDepth < INTERP_DEPTH) {
            uint32_t parent = sGrpDepth > 0 ? sGrpId[sGrpDepth - 1] : 0x51ED5EEDu;
            uint32_t id = interp_mix(parent, w1), h = (id ^ (id >> 11)) & 2047;
            int probes;
            /* the same path pushed again in this pass (e.g. a model drawn twice): number it. Bounded:
             * with the table full (no per-frame reset) an unbounded probe never ended - fps60=0 hung */
            for (probes = 0; probes < 2048; probes++) {
                if (sOccStamp[h] != sInterpStamp) {
                    sOccStamp[h] = sInterpStamp;
                    sOccKey[h] = id;
                    sOccCnt[h] = 0;
                    break;
                }
                if (sOccKey[h] == id) {
                    break;
                }
                h = (h + 1) & 2047;
            }
            id = interp_mix(id, probes < 2048 ? sOccCnt[h]++ : 0);
            sGrpId[sGrpDepth] = id;
            sGrpFlags[sGrpDepth] = (uint8_t)(w0 & 0xFF) | (sGrpDepth > 0 ? (sGrpFlags[sGrpDepth - 1] & PORT_INTERP_SKIP) : 0);
            sGrpMtx[sGrpDepth] = 0;
            sGrpVtx[sGrpDepth] = 0;
        }
        sGrpDepth++;
    } else if (op == PORT_INTERP_OP_POP) {
        if (sGrpDepth > 0) {
            sGrpDepth--;
        }
    }
}

static int interp_find(uint32_t key) {
    uint32_t h = (key * 2654435761u) & (INTERP_HASH - 1);
    while (sInterpHash[h] != 0) {
        int i = sInterpHash[h] - 1;
        if (i < 0x4000 ? sInterpTab[sInterpCurTab ^ 1][i].key == key : 0) {
            return i;
        }
        h = (h + 1) & (INTERP_HASH - 1);
    }
    return -1;
}

/* decomposed blend of rigid transforms (row-vector N64 layout: rows 0-2 = scaled axes, row 3 = translation) */
static int interp_decomposed(float out[4][4], const float a[4][4], const float b[4][4], float t) {
    float ax[3][3], bx[3][3], sa[3], sb[3];
    int i, k;
    for (i = 0; i < 3; i++) {
        sa[i] = sqrtf(a[i][0] * a[i][0] + a[i][1] * a[i][1] + a[i][2] * a[i][2]);
        sb[i] = sqrtf(b[i][0] * b[i][0] + b[i][1] * b[i][1] + b[i][2] * b[i][2]);
        if (sa[i] < 1e-6f || sb[i] < 1e-6f) {
            return 0;
        }
        for (k = 0; k < 3; k++) {
            ax[i][k] = a[i][k] / sa[i];
            bx[i][k] = b[i][k] / sb[i];
        }
        /* more than ~90 degrees in one logic frame: a snap, not motion (Ship of Harkinian's angle rule) */
        if (ax[i][0] * bx[i][0] + ax[i][1] * bx[i][1] + ax[i][2] * bx[i][2] < 0.0f) {
            return 0;
        }
    }
    /* nlerp the axes, then Gram-Schmidt (keeps the rotation rigid) */
    {
        float r[3][3], l;
        for (i = 0; i < 3; i++) {
            for (k = 0; k < 3; k++) {
                r[i][k] = ax[i][k] + (bx[i][k] - ax[i][k]) * t;
            }
        }
        l = sqrtf(r[0][0] * r[0][0] + r[0][1] * r[0][1] + r[0][2] * r[0][2]);
        for (k = 0; k < 3; k++) r[0][k] /= l;
        {
            float d = r[1][0] * r[0][0] + r[1][1] * r[0][1] + r[1][2] * r[0][2];
            for (k = 0; k < 3; k++) r[1][k] -= d * r[0][k];
        }
        l = sqrtf(r[1][0] * r[1][0] + r[1][1] * r[1][1] + r[1][2] * r[1][2]);
        if (l < 1e-6f) {
            return 0;
        }
        for (k = 0; k < 3; k++) r[1][k] /= l;
        {
            /* third axis: the cross product, with the handedness of the source (mirrored models) */
            float c0 = r[0][1] * r[1][2] - r[0][2] * r[1][1];
            float c1 = r[0][2] * r[1][0] - r[0][0] * r[1][2];
            float c2 = r[0][0] * r[1][1] - r[0][1] * r[1][0];
            float sgn = (c0 * bx[2][0] + c1 * bx[2][1] + c2 * bx[2][2]) < 0.0f ? -1.0f : 1.0f;
            r[2][0] = c0 * sgn, r[2][1] = c1 * sgn, r[2][2] = c2 * sgn;
        }
        for (i = 0; i < 3; i++) {
            float s = sa[i] + (sb[i] - sa[i]) * t;
            for (k = 0; k < 3; k++) {
                out[i][k] = r[i][k] * s;
            }
            out[i][3] = 0.0f;
        }
        for (k = 0; k < 3; k++) {
            out[3][k] = a[3][k] + (b[3][k] - a[3][k]) * t;
        }
        out[3][3] = 1.0f;
    }
    return 1;
}

/* blend `m` (this logic frame's matrix) from the previous frame's into out[k] for t = (k + 1) / 3 (copies
 * of m when there is nothing to blend); records m on the exact pass. Returns 1 when out[] was written. */
static int interp_matrix3(const float m[4][4], uint8_t parameters, float out[2][4][4]) {
    uint32_t key;
    int idx, k;
    const float(*p)[4];
    if (sGrpDepth <= 0 || sGrpDepth > INTERP_DEPTH) {
        return 0; /* untagged: drawn as it is */
    }
    key = interp_mix(sGrpId[sGrpDepth - 1], sGrpMtx[sGrpDepth - 1]++ * 4 + (parameters & G_MTX_PROJECTION ? 1 : 0)) | 1;
    if (sInterpRecord) {
        int n = sInterpN[sInterpCurTab];
        if (n < INTERP_MAX) {
            sInterpTab[sInterpCurTab][n].key = key;
            memcpy(sInterpTab[sInterpCurTab][n].m, m, sizeof(float) * 16);
            sInterpN[sInterpCurTab] = n + 1;
        }
    }
    if (!gPortReplayRec || (sGrpFlags[sGrpDepth - 1] & PORT_INTERP_SKIP)) {
        return 0;
    }
    idx = interp_find(key);
    if (idx < 0) {
        gPortInterpMissed++;
        return 0;
    }
    gPortInterpMatched++;
    p = sInterpTab[sInterpCurTab ^ 1][idx].m;
    if (memcmp(p, m, sizeof(float) * 16) == 0) {
        gPortInterpSame++;
        return 0;
    }
    for (k = 0; k < 2; k++) {
        float t = (k + 1) * (1.0f / 3.0f);
        if (p[0][3] == 0.0f && p[1][3] == 0.0f && p[2][3] == 0.0f && p[3][3] == 1.0f && m[0][3] == 0.0f &&
            m[1][3] == 0.0f && m[2][3] == 0.0f && m[3][3] == 1.0f) {
            /* rigid (modelview, camera view): decomposed. Teleports are the game side's call (the group's
             * SKIP flag: actor moved too far, camera cut), as in Zelda64Recomp - a translation threshold here
             * cannot work, limb matrices are in unscaled model units (x100). */
            if (!interp_decomposed(out[k], p, m, t)) {
                gPortInterpJump++;
                return 0;
            }
        } else {
            int i, j;
            for (i = 0; i < 4; i++) { /* perspective projections: per element */
                for (j = 0; j < 4; j++) {
                    out[k][i][j] = p[i][j] + (m[i][j] - p[i][j]) * t;
                }
            }
        }
    }
    return 1;
}

/* skinned vertex loads (groups tagged PORT_INTERP_VERTS): records this load's positions; returns the
 * previous frame's positions for the same load (NULL: none / not a skinned load) */
static const int16_t (*interp_vtx_prev(const Vtx* v, size_t n))[3] {
    uint32_t key;
    int g = sGrpDepth - 1;
    size_t i;
    if (g < 0 || g >= INTERP_DEPTH || !(sGrpFlags[g] & PORT_INTERP_VERTS) || n == 0 || n > 64) {
        return NULL;
    }
    key = interp_mix(sGrpId[g] ^ 0x5EB1u, sGrpVtx[g]++) | 1;
    if (sInterpRecord) {
        int c = sInterpVLN[sInterpCurTab], f = sInterpVN[sInterpCurTab];
        if (c < INTERP_VLOADS && f + (int)n <= INTERP_VTX_MAX) {
            sInterpVLoad[sInterpCurTab][c].key = key;
            sInterpVLoad[sInterpCurTab][c].n = (uint16_t)n;
            sInterpVLoad[sInterpCurTab][c].first = (uint16_t)f;
            for (i = 0; i < n; i++) {
                sInterpVPos[sInterpCurTab][f + i][0] = v[i].v.ob[0];
                sInterpVPos[sInterpCurTab][f + i][1] = v[i].v.ob[1];
                sInterpVPos[sInterpCurTab][f + i][2] = v[i].v.ob[2];
            }
            sInterpVLN[sInterpCurTab] = c + 1;
            sInterpVN[sInterpCurTab] = f + (int)n;
        }
    }
    if (!gPortReplayRec || (sGrpFlags[g] & PORT_INTERP_SKIP)) {
        return NULL;
    }
    {
        const int pt = sInterpCurTab ^ 1;
        int c;
        for (c = 0; c < sInterpVLN[pt]; c++) { /* few skinned loads per frame: a linear scan is enough */
            const InterpVLoad* L = &sInterpVLoad[pt][c];
            if (L->key == key && L->n == n) {
                gPortInterpVtx += (uint32_t)n;
                return (const int16_t(*)[3])sInterpVPos[pt][L->first];
            }
        }
        gPortInterpVtxMiss++;
    }
    return NULL;
}

/* 3ds_main.c: once per logic frame, before its passes: the tables the previous exact pass recorded
 * become "previous" and are indexed; the exact pass of this frame records into the other ones */
void gfx_interp_begin_frame(void) {
    int i;
    sInterpCurTab ^= 1;
    sInterpN[sInterpCurTab] = 0;
    sInterpVN[sInterpCurTab] = 0;
    sInterpVLN[sInterpCurTab] = 0;
    memset(sInterpHash, 0, sizeof(sInterpHash));
    for (i = 0; i < sInterpN[sInterpCurTab ^ 1]; i++) {
        uint32_t h = (sInterpTab[sInterpCurTab ^ 1][i].key * 2654435761u) & (INTERP_HASH - 1);
        while (sInterpHash[h] != 0) h = (h + 1) & (INTERP_HASH - 1);
        sInterpHash[h] = (int16_t)(i + 1);
    }
}

/* before the display-list walk: `record` = keep the tables for the next frame's interpolation */
void gfx_interp_pass(float t, int record) {
    (void)t;
    sInterpStamp++;
    sGrpDepth = 0;
    sInterpRecord = record;
}

/* ---- PORT (2026-09-30): replay (60 fps without walking the display list again) ----
 * In-between frames used to re-run the whole display list (as costly as the frame itself: 20-35 ms on
 * a New 3DS). Now the logic frame's walk RECORDS: the backend logs its state calls and draws instead of
 * issuing them (gfx_citro3d.c), and here every vertex written to the VBO gets a recipe - the object-space
 * position(s) and matrix slot it came from, or, for vertices made by splitting/clipping, the weights of
 * the original triangle's three vertices (clip-space affine, so they stay valid). The matrix stacks are
 * evaluated at t = 1/3 and 2/3 alongside the real one. Each shown frame then only rewrites the VBO
 * positions (t = 1/3, 2/3, or the recorded t = 1) and re-issues the log: colours, lighting, texture
 * coordinates and everything else stay as recorded for the logic frame. */
extern float* gfx_citro3d_vbo_info(int** pos, u32* cap, float scale[4]);
int gPortReplayRec;   /* 1 while the walk records for replay */
extern int gPortReplayFirstK;
int gPortReplayBroken; /* this frame cannot be replayed (an off-screen target, a buffer overflow) */
#define REC_SLOTS 1024
#define REC_SRC 8192
#define REC_VTX 20000
#define REC_NONE 0xFFFF
static float sAltMV[2][11][4][4], sAltP[2][4][4];
static float (*sRecMP)[2][4][4]; /* per matrix slot: MP at t = 1/3, 2/3 */
static int sRecSlotN, sRecSlotCur = -1;
typedef struct {
    float ob[2][3]; /* object-space position at t = 1/3, 2/3 (skinned vertices move) */
    float hx, hy, A; /* clip adjustments at load: half-pixel offsets, aspect squeeze */
    uint16_t slot;
} RecSrc;
static RecSrc* sRecSrc;
static int sRecSrcN;
static uint16_t sLoadSrc[MAX_VERTICES + 4]; /* per loaded vertex: its source, REC_NONE = none */
typedef struct {
    uint16_t src[3];
    float wgt[3];
    float orig[5]; /* the recorded (t = 1) x y z w and stereo offset, PICA layout */
    uint32_t stamp;
} RecVtx;
static RecVtx* sRecVtx;
static uint32_t sRecStamp = 1;
static int sRecVtxMax;
static float (*sRecClip)[4]; /* per source, per replay */
/* the triangle being emitted (gfx_sp_tri1_impl): its loaded vertices' sources and clip positions */
static int sRecTriOk;
static uint16_t sRecTriSrc[3];
static float sRecTriPos[3][3]; /* x y w */

static void gfx_replay_alloc(void) {
    if (sRecVtx == NULL) {
        sRecMP = malloc(sizeof(*sRecMP) * REC_SLOTS);
        sRecSrc = malloc(sizeof(RecSrc) * REC_SRC);
        sRecVtx = malloc(sizeof(RecVtx) * REC_VTX);
        sRecClip = malloc(sizeof(*sRecClip) * REC_SRC);
    }
}

/* 3ds_main.c, before the walk */
void gfx_replay_begin(void) {
    gfx_replay_alloc();
    gPortReplayRec = sRecVtx != NULL && sRecMP != NULL && sRecSrc != NULL && sRecClip != NULL;
    gPortReplayBroken = !gPortReplayRec;
    sRecSlotN = sRecSrcN = 0;
    sRecSlotCur = -1;
    sRecVtxMax = 0;
    sRecStamp++;
    memset(sLoadSrc, 0xFF, sizeof(sLoadSrc));
}

static void rec_matrix_changed(void) {
    sRecSlotCur = -1;
}

/* the current MP slot (made on first use after a matrix change) */
static int rec_slot(void) {
    if (sRecSlotCur < 0) {
        int k, top = rsp.modelview_matrix_stack_size - 1;
        if (sRecSlotN >= REC_SLOTS || top < 0) {
            gPortReplayBroken = 1;
            return -1;
        }
        for (k = 0; k < 2; k++) {
            gfx_matrix_mul(sRecMP[sRecSlotN][k], sAltMV[k][top], sAltP[k]);
        }
        sRecSlotCur = sRecSlotN++;
    }
    return sRecSlotCur;
}

/* a loaded vertex: its source record */
static void rec_load_vertex(int dest, const Vtx_t* v, const int16_t* prev) {
    RecSrc* s;
    int slot, k;
    if (sRecSrcN >= REC_SRC || (slot = rec_slot()) < 0) {
        sLoadSrc[dest] = REC_NONE;
        gPortReplayBroken = 1;
        return;
    }
    s = &sRecSrc[sRecSrcN];
    for (k = 0; k < 2; k++) {
        float t = (k + 1) * (1.0f / 3.0f);
        int c;
        for (c = 0; c < 3; c++) {
            s->ob[k][c] = prev != NULL ? prev[c] + (v->ob[c] - prev[c]) * t : (float)v->ob[c];
        }
    }
    s->hx = rsp.half_px_x;
    s->hy = rsp.half_px_y;
    s->A = (4.0f / 3.0f) / ((float)gfx_current_dimensions.width / (float)gfx_current_dimensions.height);
    s->slot = (uint16_t)slot;
    sLoadSrc[dest] = (uint16_t)sRecSrcN++;
}

/* a vertex written to the VBO at `first`: its recipe */
static void rec_vbo_vertex(int first, const float* d, float px, float py, float pw, int slot) {
    RecVtx* r;
    if (first >= REC_VTX) {
        gPortReplayBroken = 1;
        return;
    }
    r = &sRecVtx[first];
    r->orig[0] = d[0], r->orig[1] = d[1], r->orig[2] = d[2], r->orig[3] = d[3], r->orig[4] = d[12];
    r->stamp = sRecStamp;
    r->src[0] = r->src[1] = r->src[2] = REC_NONE;
    if (first + 1 > sRecVtxMax) {
        sRecVtxMax = first + 1;
    }
    if (slot >= 0 && sLoadSrc[slot] != REC_NONE) {
        r->src[0] = sLoadSrc[slot];
        r->wgt[0] = 1.0f;
        return;
    }
    if (sRecTriOk) {
        /* a new vertex (split/clipped): solve p = a A + b B + c C in clip (x, y, w) */
        const float(*T)[3] = sRecTriPos;
        float det = T[0][0] * (T[1][1] * T[2][2] - T[2][1] * T[1][2]) - T[1][0] * (T[0][1] * T[2][2] - T[2][1] * T[0][2]) +
                    T[2][0] * (T[0][1] * T[1][2] - T[1][1] * T[0][2]);
        if (fabsf(det) > 1e-9f) {
            float inv = 1.0f / det, P[3] = { px, py, pw };
            float a = (P[0] * (T[1][1] * T[2][2] - T[2][1] * T[1][2]) - T[1][0] * (P[1] * T[2][2] - T[2][1] * P[2]) +
                       T[2][0] * (P[1] * T[1][2] - T[1][1] * P[2])) * inv;
            float b = (T[0][0] * (P[1] * T[2][2] - T[2][1] * P[2]) - P[0] * (T[0][1] * T[2][2] - T[2][1] * T[0][2]) +
                       T[2][0] * (T[0][1] * P[2] - P[1] * T[0][2])) * inv;
            float c = 1.0f - a - b;
            (void)c;
            c = (T[0][0] * (T[1][1] * P[2] - P[1] * T[1][2]) - T[1][0] * (T[0][1] * P[2] - P[1] * T[0][2]) +
                 P[0] * (T[0][1] * T[1][2] - T[1][1] * T[0][2])) * inv;
            r->src[0] = sRecTriSrc[0], r->src[1] = sRecTriSrc[1], r->src[2] = sRecTriSrc[2];
            r->wgt[0] = a, r->wgt[1] = b, r->wgt[2] = c;
        }
    }
}

/* ---- PORT (2026-10-01): GPU vertex path (gpu_vtx=1; gfx_citro3d.c, shader_gpu.v.pica) ----
 * G_VTX keeps vertices in model space and tags each with the current matrix SLOT: the MP matrix (and,
 * while recording for 60 fps, its t = 1/3 and 2/3 versions from the parallel stacks) turned into the 4
 * output rows the vertex shader multiplies by. A batch's distinct slots form its palette (GPU_PAL). */
#define GPU_SLOTS 4096
#define GPU_SLOT_IDENTITY 0xFFFF /* rectangles: their vertices are already in clip space */
typedef struct {
    float rows[3][4][4]; /* [replay k][output row][x y z 1 coefficient] */
    uint8_t camLit;      /* the modelview scales uniformly: camera-space raw lighting is the N64's (raw_light_set) */
} GpuSlot;
static GpuSlot* sGpuSlot;
static int sGpuSlotN, sGpuSlotCur = -1;
static uint16_t sLoadSlot[MAX_VERTICES + 4];
static float sLoadDpos[MAX_VERTICES + 4][3];
static uint8_t sProbeStale[MAX_VERTICES + 4]; /* 3D border probes: clip position not computed since load */
static uint8_t sZWStale[MAX_VERTICES + 4];    /* the clip z/w below not computed since load */
static float sZW[MAX_VERTICES + 4][4]; /* out z, w, and (screen-linear mode) out0 = y, out1 = -x */
static uint32_t* sSlotPalBatch;
static uint8_t* sSlotPalIdx;
static uint32_t sIdentPalBatch;
static uint8_t sIdentPalIdx;

/* MP (row-vector: clip_j = sum_i ob_i * MP[i][j]) -> output rows (y', -x', -(z + w) / 2, w) with the
 * half-pixel offset and aspect squeeze, exactly as gfx_sp_vertex + pack_vertex compute them */
static void gpu_rows_from_mp(float rows[4][4], const float mp[4][4], float hx, float hy, float A) {
    int i;
    for (i = 0; i < 4; i++) {
        float cx = mp[i][0], cy = mp[i][1], cz = mp[i][2], cw = mp[i][3];
        rows[0][i] = cy - hy * cw;
        rows[1][i] = -(A * (cx + hx * cw));
        rows[2][i] = -0.5f * (cz + cw);
        rows[3][i] = cw;
    }
}

/* PORT PERF (2026-10-05): MP = MV x P only when vertices need it - a skeleton loads and multiplies several matrices per
 * limb before its vertices (Old 3DS hardware v58: matrices 8% of the drawing; a 4x4 product is 64 VFP multiply-adds) */
static bool sMPDirty = true;
static inline void mp_update(void) {
    if (sMPDirty) {
        sMPDirty = false;
        gfx_matrix_mul(rsp.MP_matrix, rsp.modelview_matrix_stack[rsp.modelview_matrix_stack_size - 1], rsp.P_matrix);
    }
}

static int gpu_slot_get(void) {
    if (sGpuSlotCur < 0) {
        int k, top = rsp.modelview_matrix_stack_size - 1;
        float A = gfx_adjust_x_for_aspect_ratio(1.0f);
        GpuSlot* g;
        if (sGpuSlot == NULL) {
            sGpuSlot = malloc(sizeof(GpuSlot) * GPU_SLOTS);
            sSlotPalBatch = calloc(GPU_SLOTS, sizeof(uint32_t));
            sSlotPalIdx = calloc(GPU_SLOTS, 1);
            if (sGpuSlot == NULL || sSlotPalBatch == NULL || sSlotPalIdx == NULL) {
                return GPU_SLOT_IDENTITY;
            }
        }
        if (sGpuSlotN >= GPU_SLOTS) {
            return sGpuSlotN - 1; /* table full (never seen): reuse the last slot */
        }
        g = &sGpuSlot[sGpuSlotN];
        mp_update();
        {
            /* the N64 lights in model space, normalising the transformed light direction: in camera space that is
             * n . MV / |MV row| only for a uniform scale (rows of equal length; a skew would show here too) */
            const float(*mv)[4] = rsp.modelview_matrix_stack[top];
            float l0 = mv[0][0] * mv[0][0] + mv[0][1] * mv[0][1] + mv[0][2] * mv[0][2];
            float l1 = mv[1][0] * mv[1][0] + mv[1][1] * mv[1][1] + mv[1][2] * mv[1][2];
            float l2 = mv[2][0] * mv[2][0] + mv[2][1] * mv[2][1] + mv[2][2] * mv[2][2];
            float d01 = mv[0][0] * mv[1][0] + mv[0][1] * mv[1][1] + mv[0][2] * mv[1][2];
            float d02 = mv[0][0] * mv[2][0] + mv[0][1] * mv[2][1] + mv[0][2] * mv[2][2];
            /* lengths within 2% (squares within 4%), rows within ~1 degree of square */
            g->camLit = l0 > 0.0f && fabsf(l1 - l0) < 0.04f * l0 && fabsf(l2 - l0) < 0.04f * l0 &&
                        fabsf(d01) < 0.02f * l0 && fabsf(d02) < 0.02f * l0;
        }
        gpu_rows_from_mp(g->rows[2], rsp.MP_matrix, rsp.half_px_x, rsp.half_px_y, A);
        for (k = 0; k < 2; k++) {
            if (gPortReplayRec && top >= 0) {
                float mp[4][4];
                gfx_matrix_mul(mp, sAltMV[k][top], sAltP[k]);
                gpu_rows_from_mp(g->rows[k], mp, rsp.half_px_x, rsp.half_px_y, A);
            } else {
                memcpy(g->rows[k], g->rows[2], sizeof(g->rows[2]));
            }
        }
        sSlotPalBatch[sGpuSlotN] = 0;
        sGpuSlotCur = sGpuSlotN++;
    }
    return sGpuSlotCur;
}

/* the logic frame's (replay k = 2) output rows of a slot, without copying */
static inline const float (*gpu_rows2(uint16_t slot))[4] {
    static const float ident[4][4] = { { 0, 1, 0, 0 }, { -1, 0, 0, 0 }, { 0, 0, -0.5f, -0.5f }, { 0, 0, 0, 1 } };
    if (slot == GPU_SLOT_IDENTITY || sGpuSlot == NULL || slot >= sGpuSlotN) {
        return ident;
    }
    return (const float(*)[4])sGpuSlot[slot].rows[2];
}

/* gfx_citro3d.c: the rows of a palette entry for replay k */
void gfx_gpu_slot_rows(uint16_t slot, int k, float rows[4][4]) {
    if (slot == GPU_SLOT_IDENTITY || sGpuSlot == NULL || slot >= sGpuSlotN) {
        static const float ident[4][4] = { { 0, 1, 0, 0 }, { -1, 0, 0, 0 }, { 0, 0, -0.5f, -0.5f }, { 0, 0, 0, 1 } };
        memcpy(rows, ident, sizeof(ident));
        return;
    }
    memcpy(rows, sGpuSlot[slot].rows[k < 0 || k > 2 ? 2 : k], sizeof(float) * 16);
}

/* the same rows as one array of 16 floats, without a copy (gfx_citro3d.c replay by copy) */
const float* gfx_gpu_slot_row_ptr(uint16_t slot, int k) {
    static const float ident[16] = { 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, -0.5f, -0.5f, 0, 0, 0, 1 };
    if (slot == GPU_SLOT_IDENTITY || sGpuSlot == NULL || slot >= sGpuSlotN) {
        return ident;
    }
    return &sGpuSlot[slot].rows[k < 0 || k > 2 ? 2 : k][0][0];
}

/* rewrite the VBO positions for replay k (0: t = 1/3, 1: t = 2/3, 2: the recorded t = 1) */
void gfx_replay_positions(int k, bool z_is_from_0_to_1) {
    int pos_dummy;
    int* posp = &pos_dummy;
    u32 cap;
    float scale[4];
    float* vbo = gfx_citro3d_vbo_info(&posp, &cap, scale);
    int i, j;
    if (vbo == NULL) {
        return;
    }
    if (k < 2) {
        for (i = 0; i < sRecSrcN; i++) {
            const RecSrc* s = &sRecSrc[i];
            const float(*m)[4] = sRecMP[s->slot][k];
            const float* o = s->ob[k];
            float x = o[0] * m[0][0] + o[1] * m[1][0] + o[2] * m[2][0] + m[3][0];
            float y = o[0] * m[0][1] + o[1] * m[1][1] + o[2] * m[2][1] + m[3][1];
            float z = o[0] * m[0][2] + o[1] * m[1][2] + o[2] * m[2][2] + m[3][2];
            float w = o[0] * m[0][3] + o[1] * m[1][3] + o[2] * m[2][3] + m[3][3];
            x += s->hx * w;
            y -= s->hy * w;
            sRecClip[i][0] = x * s->A, sRecClip[i][1] = y, sRecClip[i][2] = z, sRecClip[i][3] = w;
        }
    }
    for (i = 0; i < sRecVtxMax; i++) {
        const RecVtx* r = &sRecVtx[i];
        float* d = vbo + i * 13;
        if (r->stamp != sRecStamp) {
            continue; /* not ours (written by another path): left as it is */
        }
        if (k == 2 || r->src[0] == REC_NONE) {
            d[0] = r->orig[0], d[1] = r->orig[1], d[2] = r->orig[2], d[3] = r->orig[3], d[12] = r->orig[4];
            continue;
        }
        {
            float c[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
            for (j = 0; j < 3 && r->src[j] != REC_NONE; j++) {
                const float* s = sRecClip[r->src[j]];
                c[0] += r->wgt[j] * s[0], c[1] += r->wgt[j] * s[1], c[2] += r->wgt[j] * s[2], c[3] += r->wgt[j] * s[3];
            }
            d[0] = c[1];
            d[1] = -c[0];
            d[2] = -(z_is_from_0_to_1 ? (c[2] + c[3]) * 0.5f : c[2]);
            d[3] = c[3];
            /* stereo offset s = w - convergence (stereo_offset), 0 for flat layers */
            d[12] = r->orig[4] == 0.0f ? 0.0f : r->orig[4] + (c[3] - r->orig[3]);
        }
    }
}

static void gfx_sp_matrix(uint8_t parameters, const int32_t *addr) {
    float matrix[4][4];
#ifndef GBI_FLOATS
    /* PORT: OoT (Matrix_MtxFToMtx) stores the Mtx as 16 int16 integer parts
       (halfwords 0..15) followed by 16 uint16 fractional parts (halfwords
       16..31). Read each 16-bit half individually so host endianness is
       irrelevant (reading them as int32 pairs flips the halves on LE). */
    {
        const int16_t *ip = (const int16_t *)addr;
        const uint16_t *fp = (const uint16_t *)addr + 16;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                int k = i * 4 + j;
                /* guMtxF2L packs 2 elements per 32-bit word; on a little-endian
                   host the two 16-bit halves land swapped, so element k lives at
                   halfword (k ^ 1). */
                int sk = k ^ 1;
                int32_t fixed = ((int32_t)ip[sk] << 16) | fp[sk];
                matrix[i][j] = fixed / 65536.0f;
            }
        }
    }
#else
    memcpy(matrix, addr, sizeof(matrix));
#endif
    float alt[2][4][4]; /* replay: this matrix at t = 1/3, 2/3 */
    int haveAlt = sInterpRecord ? interp_matrix3(matrix, parameters, alt) : 0;
    if (gPortReplayRec && !haveAlt) {
        memcpy(alt[0], matrix, sizeof(matrix));
        memcpy(alt[1], matrix, sizeof(matrix));
    }
    
    { /* PORT rawmtx */
        extern unsigned int gfx_port_frame_index;
        static int rt = -2;
        if (rt == -2) { const char* e = getenv("PORT_RAWMTX"); rt = e ? atoi(e) : -1; }
        if (rt >= 0 && (int)gfx_port_frame_index == rt && (parameters & 4)) {
            const uint16_t* a = (const uint16_t*)addr;
            fprintf(stderr, "[RAW params=%02x] int:", parameters);
            for (int z = 0; z < 16; z++) fprintf(stderr, " %04x", a[z]);
            fprintf(stderr, " | frac:");
            for (int z = 16; z < 32; z++) fprintf(stderr, " %04x", a[z]);
            fprintf(stderr, "\n");
        }
    }
    { /* PORT loaded-mtx dump */
        extern unsigned int gfx_port_frame_index;
        static int tgt = -2;
        if (tgt == -2) { const char* e = getenv("PORT_MTXLOG"); tgt = e ? atoi(e) : -1; }
        if (tgt >= 0 && (int)gfx_port_frame_index == tgt) {
            fprintf(stderr, "[LOADMTX %s params=%02x]\n",
                    (parameters & G_MTX_PROJECTION) ? "PROJ" : "MV", parameters);
            for (int r = 0; r < 4; r++)
                fprintf(stderr, "   %9.4f %9.4f %9.4f %9.4f\n",
                        matrix[r][0], matrix[r][1], matrix[r][2], matrix[r][3]);
        }
    }
    if (gPortReplayRec) { /* the same stack operation on the t = 1/3 and 2/3 stacks */
        int k, top = rsp.modelview_matrix_stack_size - 1;
        for (k = 0; k < 2; k++) {
            if (parameters & G_MTX_PROJECTION) {
                if (parameters & G_MTX_LOAD) {
                    memcpy(sAltP[k], alt[k], sizeof(matrix));
                } else {
                    gfx_matrix_mul(sAltP[k], alt[k], sAltP[k]);
                }
            } else {
                int t = top;
                if ((parameters & G_MTX_PUSH) && rsp.modelview_matrix_stack_size < 11) {
                    t = top + 1;
                    memcpy(sAltMV[k][t], sAltMV[k][top], sizeof(matrix));
                }
                if (parameters & G_MTX_LOAD) {
                    memcpy(sAltMV[k][t], alt[k], sizeof(matrix));
                } else {
                    gfx_matrix_mul(sAltMV[k][t], alt[k], sAltMV[k][t]);
                }
            }
        }
        rec_matrix_changed();
    }
    if (parameters & G_MTX_PROJECTION) {
        if (parameters & G_MTX_LOAD) {
            memcpy(rsp.P_matrix, matrix, sizeof(matrix));
        } else {
            gfx_matrix_mul(rsp.P_matrix, matrix, rsp.P_matrix);
        }
    } else { // G_MTX_MODELVIEW
        if ((parameters & G_MTX_PUSH) && rsp.modelview_matrix_stack_size < 11) {
            ++rsp.modelview_matrix_stack_size;
            memcpy(rsp.modelview_matrix_stack[rsp.modelview_matrix_stack_size - 1], rsp.modelview_matrix_stack[rsp.modelview_matrix_stack_size - 2], sizeof(matrix));
        }
        if (parameters & G_MTX_LOAD) {
            memcpy(rsp.modelview_matrix_stack[rsp.modelview_matrix_stack_size - 1], matrix, sizeof(matrix));
        } else {
            gfx_matrix_mul(rsp.modelview_matrix_stack[rsp.modelview_matrix_stack_size - 1], matrix, rsp.modelview_matrix_stack[rsp.modelview_matrix_stack_size - 1]);
        }
        rsp.lights_changed = 1;
    }
    sMPDirty = true;
    sGpuSlotCur = -1;
}

static void gfx_sp_pop_matrix(uint32_t count) {
    rec_matrix_changed();
    sGpuSlotCur = -1;
    while (count--) {
        if (rsp.modelview_matrix_stack_size > 0) {
            --rsp.modelview_matrix_stack_size;
            if (rsp.modelview_matrix_stack_size > 0) {
                sMPDirty = true;
            }
        }
    }
}

static float gfx_adjust_x_for_aspect_ratio(float x) {
    return x * (4.0f / 3.0f) / ((float)gfx_current_dimensions.width / (float)gfx_current_dimensions.height);
}

static uint32_t sPVStamp[MAX_VERTICES + 4]; /* see sPVCache */
static const int16_t (*sVtxPrev)[3]; /* replay: the previous frame's positions of a skinned load */
static void gfx_sp_vertex_impl(size_t n_vertices, size_t dest_index, const Vtx *vertices);
static void gfx_sp_vertex(size_t n_vertices, size_t dest_index, const Vtx *vertices) {
    uint64_t t0 = PERF_T();
    sVtxPrev = sGrpDepth > 0 && sInterpRecord ? interp_vtx_prev(vertices, n_vertices) : NULL;
    PROF_SET(PROF_VTX);
    gfx_sp_vertex_impl(n_vertices, dest_index, vertices);
    PROF_SET(PROF_DL);
    gPortPerfVtx += PERF_T() - t0;
}

/* the lights' and LookAt's directions in the current model space (after a matrix or light change) */
static void lights_refresh(void) {
    for (int i = 0; i < rsp.current_num_lights - 1; i++) {
        calculate_normal_dir(&rsp.current_lights[i], rsp.current_lights_coeffs[i]);
    }
    /* PORT (2026-09-30): the game's LookAt (camera direction for G_TEXTURE_GEN environment maps
     * and hilites), transformed like the lights - libultraship GfxSpVertex. It was always a fixed
     * +x/+y (sm64 PC port): dull swords/metal, the flat N64 boot logo. */
    /* No LookAt loaded yet this frame: a fixed +x/+y (sm64 PC port). Measured with tools/statediff
     * (Chamber of the Sages pedestal, the one surface env-mapped before any LookAt): this 263
     * wrong flat px, a ZERO LookAt 449, the previous frame's last one 446. On the N64 the LookAt
     * lives outside the 0x420-byte F3DZEX2 data image reloaded per task (that image is zero
     * there), so it keeps what an earlier task left - not reproducible exactly. */
    static const Light_t lookat_def[2] = { {{0, 0, 0}, 0, {0, 0, 0}, 0, {127, 0, 0}, 0},
                                           {{0, 0, 0}, 0, {0, 0, 0}, 0, {0, 127, 0}, 0} };
    for (int k = 0; k < 2; k++) {
        const Light_t* la = &rsp.current_lookat[k];
        if (la->dir[0] == 0 && la->dir[1] == 0 && la->dir[2] == 0) {
            la = &lookat_def[k];
        }
        calculate_normal_dir(la, rsp.current_lookat_coeffs[k]);
    }
    rsp.lights_changed = false;
}

/* shade colour (lighting, or the vertex colour) and texture coordinates (texgen, or the vertex's), shared
 * by the CPU and GPU vertex paths; alpha and position are each path's */
static inline __attribute__((always_inline)) void vtx_shade(const Vtx_t* v, const Vtx_tn* vn, struct LoadedVertex* d,
                                                           float inv127) {
    short U = v->tc[0] * rsp.texture_scaling_factor.s >> 16;
    short V = v->tc[1] * rsp.texture_scaling_factor.t >> 16;

    if (rsp.geometry_mode & G_LIGHTING) {
        PROF_SET(PROF_VTX_LIGHT);
        if (rsp.lights_changed) {
            lights_refresh();
        }

        int r = rsp.current_lights[rsp.current_num_lights - 1].col[0];
        int g = rsp.current_lights[rsp.current_num_lights - 1].col[1];
        int b = rsp.current_lights[rsp.current_num_lights - 1].col[2];

        for (int i = 0; i < rsp.current_num_lights - 1; i++) {
            float intensity = 0;
            intensity += vn->n[0] * rsp.current_lights_coeffs[i][0];
            intensity += vn->n[1] * rsp.current_lights_coeffs[i][1];
            intensity += vn->n[2] * rsp.current_lights_coeffs[i][2];
            intensity *= inv127;
            if (intensity > 0.0f) {
                r += intensity * rsp.current_lights[i].col[0];
                g += intensity * rsp.current_lights[i].col[1];
                b += intensity * rsp.current_lights[i].col[2];
            }
        }

        d->color.r = r > 255 ? 255 : r;
        d->color.g = g > 255 ? 255 : g;
        d->color.b = b > 255 ? 255 : b;

        if (rsp.geometry_mode & G_TEXTURE_GEN) {
            float dotx = 0, doty = 0;
            dotx += vn->n[0] * rsp.current_lookat_coeffs[0][0];
            dotx += vn->n[1] * rsp.current_lookat_coeffs[0][1];
            dotx += vn->n[2] * rsp.current_lookat_coeffs[0][2];
            doty += vn->n[0] * rsp.current_lookat_coeffs[1][0];
            doty += vn->n[1] * rsp.current_lookat_coeffs[1][1];
            doty += vn->n[2] * rsp.current_lookat_coeffs[1][2];

            /* libultraship: clamp, and G_TEXTURE_GEN_LINEAR maps through acos.
             * PORT (2026-10-05): over the same range as the plain mapping, 0..1/2 (GlideN64: acos(-x) * 325.94931 =
             * acos(-x) / pi * 1024, against (x + 1) * 512). libultraship's acos(-x) / 4 spans 0..pi/4, so every linearly
             * mapped metal or crystal had its environment map squeezed 1.57x: the blue warp's crystal came out streaked
             * white instead of blue (Chamber of the Sages, user report + tools/statediff fbdiff). */
            dotx *= inv127;
            doty *= inv127;
            dotx = dotx < -1.0f ? -1.0f : (dotx > 1.0f ? 1.0f : dotx);
            doty = doty < -1.0f ? -1.0f : (doty > 1.0f ? 1.0f : doty);
            if (rsp.geometry_mode & G_TEXTURE_GEN_LINEAR) {
                dotx = acosf(-dotx) * (0.5f / 3.14159265f);
                doty = acosf(-doty) * (0.5f / 3.14159265f);
            } else {
                dotx = (dotx + 1.0f) / 4.0f;
                doty = (doty + 1.0f) / 4.0f;
            }
            U = (int32_t)(dotx * rsp.texture_scaling_factor.s);
            V = (int32_t)(doty * rsp.texture_scaling_factor.t);
        }
        PROF_SET(PROF_VTX);
    } else {
        d->color.r = v->cn[0];
        d->color.g = v->cn[1];
        d->color.b = v->cn[2];
    }

    d->u = U;
    d->v = V;
}

#ifdef __3DS__
/* PORT PERF (2026-10-04): raw vertex path (settings raw_vtx, 3ds_main.c; shader_raw.v.pica). A load that needs no
 * CPU vertex work - no texture coordinate generation, no skinned 60 fps delta, at most 4 directional lights - only
 * records where the game's vertices are, their matrix slot and (lit) a snapshot of the lights in model space, as the
 * RSP evaluated them at load time. Its triangles copy the 16-byte vertices into the raw vertex buffer as they are
 * (gpu_emit_tri_raw); the shader does the rest. No CPU split for the N64's screen-linear shading there: the GPU's
 * perspective-correct interpolation (as PC ports draw). The Super Mario 64 3DS port's design (Emu64): the per-vertex
 * and per-triangle CPU work was ~45% of an Old 3DS frame here (tools/pcprof.py). */
int gPortRawVtx;          /* this frame (latched in gfx_start_frame) */
int gPortRawVtxWant = 1;  /* settings raw_vtx=0/1 (default on), raw_vtx_ab=1 alternates it (3ds_main.c) */
#ifdef PORT_PERF_STAGES
u32 gPortRawWhy[8]; /* vertices kept off the raw path: near, deep, lights, skinned, texgen, identity, range; raw */
#endif
int gPortRawNear = 1; /* (measurement) */
float gPortRawRatio = 3.0f; /* settings raw_ratio x10: loads deeper than this stay processed (see gfx_sp_vertex_gpu) */
typedef struct {
    float amb[3], dir[4][3], col[4][3];
    float lit[4]; /* shader_raw.v.pica lit: 1, camera space (dir in camera space), -1 / (aspect P00), 1 / P11 */
} RawLights;
#define RAW_LIGHT_SETS 64
static const Vtx* sLoadRaw[MAX_VERTICES + 4];   /* the game's vertex behind a loaded slot (raw path), NULL: processed */
static uint8_t sLoadLit[MAX_VERTICES + 4];       /* its light set + 1, 0 = unlit */
/* PORT PERF (2026-10-05): the Old 3DS speed rules (settings speed_rules, 3ds_main.c). Every load stays raw and every
 * triangle goes to the GPU: no N64 NoN depth clamp (the PICA clips at the near plane itself, as 3DS games do: shaders'
 * remap.z, gfx_citro3d.c setRemap), no CPU clipping, no splits for the RDP's screen-linear shading. The per-triangle CPU
 * work these rules save was a third of Old 3DS drawing (hardware v58). The cost: shading gradients like a PC port's and
 * geometry within the near plane distance of the camera cut instead of flattened. A first version kept the depth clamp
 * with fewer CPU triangles: the clamp tilted near-camera ground, which flickered and cracked (hardware v59). */
int gPortRawRelax;
static RawLights sRawLights[RAW_LIGHT_SETS];
static int sRawLightsN;                          /* sets this frame */
static int sRawDraw;                             /* the backend draws with the raw program */
static int sRawLitCur = -1;                      /* light set (+1) of the raw batch being built */
static uint32_t sRawParamBatch;                  /* batch whose raw parameters were sent */
extern void gfx_citro3d_raw_mode(int raw);
extern int gfx_citro3d_raw_ready(void);
extern void gfx_citro3d_raw_params(const float uvc0[4], const float uvc1[4], const float lit[4], const float lamb[3],
                                   const float ldir[4][3], const float lcol[4][3]);
extern void* gfx_citro3d_raw_vbo(int** pos, u32* cap);

/* the current lights as a raw light set (+1); 0 when they do not fit (the load stays on the processed path).
 * cam: in camera space (PORT PERF 2026-10-05) - the same set for every limb of a model, so its draws and limb seams stay
 * on the GPU; else in the load's model space as the N64 transforms them (non-uniformly scaled matrices). */
static int raw_light_set(int cam) {
    RawLights L;
    int i, k, n = rsp.current_num_lights - 1;
    if (n > 4 || n < 0) {
        return 0;
    }
    if (!cam && rsp.lights_changed) {
        lights_refresh();
    }
    memset(&L, 0, sizeof(L));
    for (k = 0; k < 3; k++) {
        L.amb[k] = rsp.current_lights[n].col[k] * (1.0f / 255.0f);
    }
    /* OoT's modelview ends in WORLD space (the camera's view matrix is in the projection, View_ApplyPerspective) and
     * its lights are world directions. The projection P = view x perspective: its first, second and fourth columns
     * are the camera's x, y and -z axes in world space, scaled by the perspective's x and y factors and 1. */
    float ax[3][3], sx = 0.0f, sy = 0.0f, sw = 0.0f;
    if (cam) {
        const float(*P)[4] = rsp.P_matrix;
        for (k = 0; k < 3; k++) {
            ax[0][k] = P[k][0], ax[1][k] = P[k][1], ax[2][k] = -P[k][3];
            sx += P[k][0] * P[k][0], sy += P[k][1] * P[k][1], sw += P[k][3] * P[k][3];
        }
        if (sw < 0.81f || sw > 1.21f || sx <= 0.0f || sy <= 0.0f) {
            cam = 0; /* not a perspective with w = -z (orthographic menus, unusual matrices): model space */
            if (rsp.lights_changed) {
                lights_refresh();
            }
        } else {
            sx = sqrtf(sx), sy = sqrtf(sy), sw = sqrtf(sw);
            for (k = 0; k < 3; k++) {
                ax[0][k] /= sx, ax[1][k] /= sy, ax[2][k] /= sw;
            }
        }
    }
    for (i = 0; i < n; i++) {
        if (cam) { /* the game's world direction, normalised as calculate_normal_dir does, turned into camera space */
            float d[3] = { rsp.current_lights[i].dir[0], rsp.current_lights[i].dir[1], rsp.current_lights[i].dir[2] };
            gfx_normalize_vector(d);
            for (k = 0; k < 3; k++) {
                L.dir[i][k] = (d[0] * ax[k][0] + d[1] * ax[k][1] + d[2] * ax[k][2]) * (1.0f / 127.0f);
            }
        }
        for (k = 0; k < 3; k++) {
            if (!cam) {
                L.dir[i][k] = rsp.current_lights_coeffs[i][k] * (1.0f / 127.0f);
            }
            L.col[i][k] = rsp.current_lights[i].col[k] * (1.0f / 255.0f);
        }
    }
    L.lit[0] = 1.0f;
    if (cam) {
        /* the shader reads x, y and z of the camera from the palette rows (y', -x' = -aspect * x, w = -z): x and y
         * scaled back by the perspective's factors; |w row| is the modelview's scale times sw, folded into x and y */
        L.lit[1] = 1.0f;
        L.lit[2] = -sw / (gfx_adjust_x_for_aspect_ratio(1.0f) * sx);
        L.lit[3] = sw / sy;
    }
    if (sRawLightsN > 0 && memcmp(&L, &sRawLights[sRawLightsN - 1], sizeof(L)) == 0) {
        return sRawLightsN;
    }
    if (sRawLightsN >= RAW_LIGHT_SETS) {
        return 0;
    }
    sRawLights[sRawLightsN++] = L;
    return sRawLightsN;
}

/* GPU vertex path: model space + matrix slot; the shader transforms, clips and fogs. Skinned vertices
 * carry the delta to the previous frame's position for the 60 fps in-between frames. */
static void gfx_sp_vertex_gpu(size_t n_vertices, size_t dest_index, const Vtx* vertices) {
    const float inv127 = 1.0f / 127.0f;
    const uint16_t slot = (uint16_t)gpu_slot_get(); /* one matrix state for the whole load */
    uint8_t rej = 0;
    int nearEye = 0; /* the raw path: the load reaches the eye's neighbourhood */
    PROF_SET(PROF_VTX_BOX);
    {
        /* PORT PERF (2026-10-01): off-screen rejection for the whole load. The GPU path doesn't transform
         * vertices on the CPU, so it could not drop off-screen triangles, and a material entirely off
         * screen became a draw (367 draws/frame vs 144). The load's model-space bounding box is
         * transformed instead (8 corners): every vertex is outside a clip plane if all corners are (the
         * plane tests are linear), so its triangles are skipped exactly as clip_rej skips them. */
        /* the box in integers, converted once (per-component float compares stall the ARM11 VFP: ~25% of this
         * function in the Old 3DS profile) */
        int imn[3] = { 32767, 32767, 32767 }, imx[3] = { -32768, -32768, -32768 };
        float mn[3], mx[3];
        const float aspect = gfx_adjust_x_for_aspect_ratio(1.0f);
        const float(*m)[4] = rsp.MP_matrix;
        int c, k;
        for (size_t i = 0; i < n_vertices; i++) {
            const short* ob = vertices[i].v.ob;
            /* PORT PERF (2026-10-04): this loop is the frame's first read of the game's vertices; on an Old 3DS a cache
             * miss costs 50-190 cycles per 32-byte line (hardware v55 memory probe), so the line two ahead is requested
             * early (PLD: a hint, never faults) */
            __builtin_prefetch(&vertices[i + 4]);
            for (k = 0; k < 3; k++) {
                int o = ob[k];
                imn[k] = o < imn[k] ? o : imn[k];
                imx[k] = o > imx[k] ? o : imx[k];
            }
        }
        for (k = 0; k < 3; k++) {
            mn[k] = (float)imn[k], mx[k] = (float)imx[k];
        }
        /* PORT (2026-10-03): the slot's output rows (y', -x', -(z + w) / 2, w: half-pixel offset and aspect
         * included) for the logic frame and, when the walk records 60 fps in-between frames, for the t = 1/3 and
         * 2/3 cameras those frames show: rejected only if off screen for all three. Judged by the logic frame's
         * camera alone, objects at the trailing edge of a fast camera turn vanished from the in-between frames
         * (whole sub-display lists too, through G_CULLDL; hardware v42). */
        int k0 = gPortReplayRec ? 0 : 2, t;
        rej = (slot == GPU_SLOT_IDENTITY) ? 0 : (1 | 2 | 4 | 8 | 32);
        (void)m;
        (void)aspect;
        for (t = k0; t <= 2 && rej != 0; t++) {
            const float(*r)[4] = (const float(*)[4])sGpuSlot[slot].rows[t];
            for (c = 0; c < 8 && rej != 0; c++) {
                float ox = (c & 1) ? mx[0] : mn[0], oy = (c & 2) ? mx[1] : mn[1], oz = (c & 4) ? mx[2] : mn[2];
                float o0 = r[0][0] * ox + r[0][1] * oy + r[0][2] * oz + r[0][3]; /* y' */
                float o1 = r[1][0] * ox + r[1][1] * oy + r[1][2] * oz + r[1][3]; /* -x' */
                float o2 = r[2][0] * ox + r[2][1] * oy + r[2][2] * oz + r[2][3]; /* -(z + w) / 2 */
                float w = r[3][0] * ox + r[3][1] * oy + r[3][2] * oz + r[3][3];
                uint8_t cr = 0;
                if (o1 > w) cr |= 1;   /* x' < -w */
                if (o1 < -w) cr |= 2;  /* x' > w */
                if (o0 < -w) cr |= 4;
                if (o0 > w) cr |= 8;
                if (o2 < -w) cr |= 32; /* z > w */
                rej &= cr;
            }
        }
        if (gPortRawVtx && rej == 0 && slot != GPU_SLOT_IDENTITY && !gPortRawRelax) {
            /* raw path: near the eye - a box corner behind it or in front of the near plane, in any of the frames drawn
             * from this load. Its triangles need the CPU's N64 handling (the NoN clamp, clipping behind the eye; the
             * screen-linear split matters most there too: the floor under the camera), so the load is processed. */
            for (t = k0; t <= 2 && !nearEye; t++) {
                const float(*r)[4] = (const float(*)[4])sGpuSlot[slot].rows[t];
                /* w and -(z + w) / 2 are linear over the box: their extremes over the 8 corners are per-axis choices
                 * of the box's ends (PORT PERF 2026-10-04: 12 products instead of 8 corners x 2 dot products) */
                float wlo = r[3][3], whi = r[3][3], o2hi = r[2][3];
                for (k = 0; k < 3; k++) {
                    float a = r[3][k] * mn[k], b = r[3][k] * mx[k], e = r[2][k] * mn[k], f = r[2][k] * mx[k];
                    wlo += a < b ? a : b;
                    whi += a < b ? b : a;
                    o2hi += e < f ? f : e;
                }
                if ((wlo <= 1.0f || o2hi > 0.0f) && gPortRawNear) {
                    nearEye = 1; /* a corner behind the eye or in front of the near plane */
                }
#ifdef PORT_PERF_STAGES
                if (nearEye) gPortRawWhy[0] += n_vertices;
#endif
                /* a deep load (its far side gPortRawRatio times farther than its near side): its shade and fog would
                 * be interpolated with perspective over a long gradient (House of Skulltula's floor: 7.4 -> 13.6 mean
                 * error against the N64) - the processed path splits it N64-style */
                if (!nearEye && whi > gPortRawRatio * wlo) {
                    nearEye = 1;
#ifdef PORT_PERF_STAGES
                    gPortRawWhy[1] += n_vertices;
#endif
                }
            }
        }
    }
    {
#ifdef PORT_ICONGEN
        if (mesh_capturing()) {
            sMeshDbgVtx++;
            const float(*mv)[4] = rsp.modelview_matrix_stack[rsp.modelview_matrix_stack_size - 1];
            for (size_t i = 0; i < n_vertices && dest_index + i < MAX_VERTICES + 4; i++) {
                const short* ob = vertices[i].v.ob;
                for (int k = 0; k < 3; k++) {
                    sMeshPos[dest_index + i][k] = ob[0] * mv[0][k] + ob[1] * mv[1][k] + ob[2] * mv[2][k] + mv[3][k];
                }
            }
        }
#endif
        int lit = 0, raw = gPortRawVtx && sVtxPrev == NULL && !(rsp.geometry_mode & G_TEXTURE_GEN) &&
                           dest_index + n_vertices <= MAX_VERTICES && slot != GPU_SLOT_IDENTITY;
        if (nearEye) {
            raw = 0; /* (see nearEye) */
        }
#ifdef PORT_ICONGEN
        if (mesh_capturing()) {
            raw = 0; /* (the mesh capture reads the CPU's colours and texture coordinates) */
        }
#endif
        if (raw && (rsp.geometry_mode & G_LIGHTING)) {
            lit = raw_light_set(sGpuSlot[slot].camLit);
            raw = lit != 0;
#ifdef PORT_PERF_STAGES
            if (!raw) gPortRawWhy[2] += n_vertices;
#endif
        }
#ifdef PORT_PERF_STAGES
        if (gPortRawVtx && rej == 0) {
            if (sVtxPrev != NULL) gPortRawWhy[3] += n_vertices;
            else if (rsp.geometry_mode & G_TEXTURE_GEN) gPortRawWhy[4] += n_vertices;
            else if (slot == GPU_SLOT_IDENTITY) gPortRawWhy[5] += n_vertices;
            else if (dest_index + n_vertices > MAX_VERTICES) gPortRawWhy[6] += n_vertices;
            if (raw) gPortRawWhy[7] += n_vertices;
        }
#endif
        if (raw) {
            PROF_SET(PROF_VTX_RAW);
            for (size_t i = 0; i < n_vertices; i++, dest_index++) {
                const Vtx_t* v = &vertices[i].v;
                struct LoadedVertex* d = &rsp.loaded_vertices[dest_index];
                sPVStamp[dest_index] = 0;
                sBatchStamp[dest_index] = 0;
                sLoadSlot[dest_index] = slot;
                sProbeStale[dest_index] = 1;
                sZWStale[dest_index] = 1;
                sLoadRaw[dest_index] = &vertices[i];
                sLoadLit[dest_index] = (uint8_t)lit;
                d->x = v->ob[0], d->y = v->ob[1], d->z = v->ob[2], d->w = 1.0f; /* (G_BRANCH_Z, rectangles' neighbours) */
                d->clip_rej = rej;
            }
            return;
        }
        for (size_t i = 0; i < n_vertices && dest_index + i < MAX_VERTICES + 4; i++) {
            sLoadRaw[dest_index + i] = NULL;
        }
    }
    PROF_SET(PROF_VTX);
    if (sVtxPrev == NULL && dest_index + n_vertices <= MAX_VERTICES + 4) {
        memset(sLoadDpos[dest_index], 0, n_vertices * sizeof(sLoadDpos[0])); /* not skinned: no motion */
    }
    for (size_t i = 0; i < n_vertices; i++, dest_index++) {
        const Vtx_t* v = &vertices[i].v;
        const Vtx_tn* vn = &vertices[i].n;
        struct LoadedVertex* d = &rsp.loaded_vertices[dest_index];
        sPVStamp[dest_index] = 0;
        sBatchStamp[dest_index] = 0;
        sLoadSlot[dest_index] = slot;
        sProbeStale[dest_index] = 1;
        sZWStale[dest_index] = 1;
        d->x = v->ob[0], d->y = v->ob[1], d->z = v->ob[2], d->w = 1.0f;
        d->clip_rej = rej;
        if (sVtxPrev != NULL) {
            sLoadDpos[dest_index][0] = sVtxPrev[i][0] - v->ob[0];
            sLoadDpos[dest_index][1] = sVtxPrev[i][1] - v->ob[1];
            sLoadDpos[dest_index][2] = sVtxPrev[i][2] - v->ob[2];
        }
        vtx_shade(v, vn, d, inv127);
        d->color.a = v->cn[3]; /* with G_FOG the shader replaces it by the fog factor */
    }
}
#endif

static void gfx_sp_vertex_impl(size_t n_vertices, size_t dest_index, const Vtx *vertices) {
    mp_update(); /* (lazy, see gpu_slot_get) */
    /* PORT PERF (2026-10-01): loop invariants hoisted - the aspect squeeze was a float division per
     * vertex (gfx_adjust_x_for_aspect_ratio), the light intensities divided by 127 per light per vertex */
    const float aspect = gfx_adjust_x_for_aspect_ratio(1.0f);
    const float inv127 = 1.0f / 127.0f;
#ifdef __3DS__
    if (gPortGpuVtx) {
        gfx_sp_vertex_gpu(n_vertices, dest_index, vertices);
        return;
    }
#endif
    for (size_t i = 0; i < n_vertices; i++, dest_index++) {
        const Vtx_t *v = &vertices[i].v;
        const Vtx_tn *vn = &vertices[i].n;
        struct LoadedVertex *d = &rsp.loaded_vertices[dest_index];
        sPVStamp[dest_index] = 0; /* its packed form (gfx_sp_tri1_impl) is stale */
#ifdef __3DS__
        sBatchStamp[dest_index] = 0; /* and so is its VBO copy in the current batch */
        if (gPortReplayRec) {
            rec_load_vertex((int)dest_index, v, sVtxPrev != NULL ? sVtxPrev[i] : NULL);
        }
#endif
        
        float x = v->ob[0] * rsp.MP_matrix[0][0] + v->ob[1] * rsp.MP_matrix[1][0] + v->ob[2] * rsp.MP_matrix[2][0] + rsp.MP_matrix[3][0];
        float y = v->ob[0] * rsp.MP_matrix[0][1] + v->ob[1] * rsp.MP_matrix[1][1] + v->ob[2] * rsp.MP_matrix[2][1] + rsp.MP_matrix[3][1];
        float z = v->ob[0] * rsp.MP_matrix[0][2] + v->ob[1] * rsp.MP_matrix[1][2] + v->ob[2] * rsp.MP_matrix[2][2] + rsp.MP_matrix[3][2];
        float w = v->ob[0] * rsp.MP_matrix[0][3] + v->ob[1] * rsp.MP_matrix[1][3] + v->ob[2] * rsp.MP_matrix[2][3] + rsp.MP_matrix[3][3];
        
        /* PORT (2026-09-27): the RDP rasterizes triangles half a pixel down-right of texture rectangles at
         * the same coordinates; drawing both with one convention put all 3D geometry half an N64 pixel
         * up-left of the N64's (measured with tools/statediff/fbshift.py: minimum at exactly one 2x2
         * supersample in both axes in 3D scenes, none for 2D backgrounds). */
        x += rsp.half_px_x * w;
        y -= rsp.half_px_y * w;
        x *= aspect;
        
        vtx_shade(v, vn, d, inv127);
        
        // trivial clip rejection
        d->clip_rej = 0;
        if (x < -w) d->clip_rej |= 1;
        if (x > w) d->clip_rej |= 2;
        if (y < -w) d->clip_rej |= 4;
        if (y > w) d->clip_rej |= 8;
        /* no near-plane rejection: OoT's F3DZEX2 is the NoN (no near clipping) microcode */
        if (z > w) d->clip_rej |= 32;
        
        d->x = x;
        d->y = y;
        d->z = z;
        d->w = w;
        
        if (rsp.geometry_mode & G_FOG) {
            if (fabsf(w) < 0.001f) {
                // To avoid division by zero
                w = 0.001f;
            }
            
            float winv = 1.0f / w;
            if (winv < 0.0f) {
                winv = 32767.0f;
            }
            
            float fog_z = z * winv * rsp.fog_mul + rsp.fog_offset;
            if (fog_z < 0) fog_z = 0;
            if (fog_z > 255) fog_z = 255;
            d->color.a = fog_z; // Use alpha variable to store fog factor
        } else {
            d->color.a = v->cn[3];
        }
    }
}

/* PORT (2026-09-25): N64 triangle setup. The RSP clips against the near plane in clip space
 * (attributes linear in clip space) and the RDP then interpolates shade - and fog, which rides in shade
 * alpha - LINEARLY IN SCREEN SPACE, while texture coordinates and depth are perspective-correct. The
 * PICA interpolates everything perspective-correctly, which on large near-camera triangles (floors,
 * terrain) visibly moves shading and fog (tools/statediff fbdiff: Fire Temple floor ~25% too dark,
 * top error in Kokiri Forest). Triangles whose w varies a lot are clipped like the RSP does and split
 * at screen-space edge midpoints: new vertices take screen-linear shade and perspective-correct
 * position/uv, so inside each piece both interpolations agree. Small or flat triangles pass through. */
typedef struct {
    float x, y, z, w; /* clip space (N64 z) */
    float uv[2][2];   /* per texture unit, normalized to the uploaded texture */
    float c[4];       /* shade rgba 0..1 */
    float s;          /* stereo: clip-space horizontal offset at full shift (stereo_offset); interpolated
                       * exactly like x/y so split edges stay on their neighbours' edges (no cracks) */
} PVtx;

/* PORT (2026-09-30): stereoscopic depth per vertex, in clip units for a shift of 1 (the shader scales it
 * by the eye's +-shift, so the disparity is shift * s / w). Piecewise AFFINE in w around the convergence
 * c (Link's distance): behind him s = w - c (true stereo), in front of him s = STEREO_NEAR_SLOPE * (w - c)
 * (the near floor comes forward gently instead of popping out). Triangles crossing the plane w = c are
 * split on it (gfx_emit_tri), so inside every piece s is exactly affine and the RDP-style interpolation
 * of split/clipped vertices is exact: no cracks, no flattening. (v12 clamped a curve per vertex in the
 * shader: bent split edges = white crack dots, and a flat near floor; a smooth curve interpolated
 * across big ground triangles took the far vertex's depth.) 2D/ortho draws (w exactly 1) stay at the
 * screen; 0 when 3D is off. Vertices behind the camera (w <= 0, floors under it) follow the near slope. */
/* per loaded vertex: packed form cached for the triangles sharing it (gfx_sp_tri1_impl) */
static PVtx sPVCache[MAX_VERTICES + 4];
static uint32_t sPVState = 1;

#define STEREO_NEAR_SLOPE 1.0f /* 1 = true stereo everywhere (v4); the split below is skipped */

/* PORT (2026-09-30, stereo v4): automatic convergence against stereo-window violations. Geometry in front
 * of the screen plane that is cut by the screen border (the floor at the bottom edge) is what hurts in
 * stereo (research: stereo window violations; 3DS/cinema guidelines keep crossed parallax off the frame
 * edges). Probe points along the bottom and side borders get the depth (w) of the nearest 3D surface
 * covering them, from every drawn triangle (homogeneous barycentrics: exact even with vertices behind
 * the camera, like floors under it). gfx_3ds.c converges at min(Link, the 3rd-nearest probe), so border
 * geometry sits at the screen and everything recedes behind it. (Edge crossings missed a single huge
 * ground triangle covering the whole bottom border: the title-screen field popped out 20 px.) */
#define STEREO_PROBES 13
static const float sStereoProbe[STEREO_PROBES][2] = {
    { -0.9f, -0.97f }, { -0.6f, -0.97f }, { -0.3f, -0.97f }, { 0.0f, -0.97f }, { 0.3f, -0.97f }, { 0.6f, -0.97f },
    { 0.9f, -0.97f },  { -0.97f, -0.5f }, { -0.97f, 0.0f },  { -0.97f, 0.5f }, { 0.97f, -0.5f }, { 0.97f, 0.0f },
    { 0.97f, 0.5f },
};
float gPortStereoProbeW[STEREO_PROBES]; /* nearest w per probe this frame (0 = nothing), gfx_3ds.c resets */
static int sStereoDrawMode; /* G_NOOP stereo tag: only mode 0 (3D by distance) is measured */
static void stereo_probe_tri(const PVtx* t[3]) {
    for (int k = 0; k < STEREO_PROBES; k++) {
        float X = sStereoProbe[k][0], Y = sStereoProbe[k][1];
        float u0 = t[0]->x - X * t[0]->w, u1 = t[1]->x - X * t[1]->w, u2 = t[2]->x - X * t[2]->w;
        float v0 = t[0]->y - Y * t[0]->w, v1 = t[1]->y - Y * t[1]->w, v2 = t[2]->y - Y * t[2]->w;
        float b0 = u1 * v2 - u2 * v1, b1 = u2 * v0 - u0 * v2, b2 = u0 * v1 - u1 * v0;
        float d = b0 + b1 + b2;
        if (d == 0.0f) continue;
        if (d > 0.0f ? (b0 < 0.0f || b1 < 0.0f || b2 < 0.0f) : (b0 > 0.0f || b1 > 0.0f || b2 > 0.0f)) continue;
        float w = (b0 * t[0]->w + b1 * t[1]->w + b2 * t[2]->w) / d;
        if (w > 1.0f && (gPortStereoProbeW[k] == 0.0f || w < gPortStereoProbeW[k])) {
            gPortStereoProbeW[k] = w;
        }
    }
}
static inline float stereo_conv(void) {
#ifdef __3DS__
    extern float gPortStereoSep, gPortStereoConv;
    return gPortStereoSep == 0.0f ? 0.0f : gPortStereoConv;
#else
    return 0.0f;
#endif
}
static inline float stereo_offset(float w) {
    float c = stereo_conv();
    if (c == 0.0f || w == 1.0f) { /* 3D off, or ortho/2D (w exactly 1): screen depth */
        return 0.0f;
    }
    return w >= c ? w - c : STEREO_NEAR_SLOPE * (w - c);
}

/* split thresholds (settings split_ratio x100, split_px, split_depth: measurement knobs for the cost/accuracy
 * trade-off). PORT PERF (2026-10-02): swept against the N64 (tools/statediff, the 7 most shading-sensitive
 * scenes + perfbench, GPU path): 1.15/10/6 -> 1.5/24/3 cut the split share 6.5% -> 1.6% of the frame and
 * triangles/frame 5285 -> 3969, at +0.5 average error in Jabu-Jabu and +0.1 in the Grottos, the rest equal.
 * 2.0/32/2 (Jabu-Jabu +2.8, all scenes worse) is past the knee. Since the crack-free splitting (gfx_subdiv_tri)
 * the depth is per edge: 3 gave the same 7-scene errors as 6 and the old cost (perfbench title demo: display list
 * 18.8 ms, 5055 triangles; 6 per edge was 20.2 ms, 5930). */
float gPortSplitRatio = 1.15f;
float gPortSplitMinPx = 10.0f;
int gPortSplitDepth = 3; /* per edge (crack-free splitting, gfx_subdiv_tri): 3 matched 6 on the 7-scene comparison */
/* full tour (101 scenes): 1.5/24/3 raised the average error 5.917 -> 5.986 (Castle Courtyard +1.8): the
 * coarse thresholds are used only while frame skip is on (an Old 3DS: CPU-bound), 3ds_main.c */
void gfx_split_thresholds(int coarse) {
    gPortSplitRatio = coarse ? 1.5f : 1.15f;
    gPortSplitMinPx = coarse ? 24.0f : 10.0f;
    gPortSplitDepth = 3; /* halvings per edge; the old per-triangle limit was 6 (fine) / 3 (coarse) */
}
#define SUBDIV_MAX_RATIO gPortSplitRatio
#define SUBDIV_MIN_PIXELS gPortSplitMinPx
#define SUBDIV_MAX_DEPTH gPortSplitDepth
#define SUBDIV_FLAT_RANGE (2.5f / 255.0f) /* shade/fog spread below which splitting changes nothing visible */
/* settings.txt shade_split=0: no splitting for the N64's screen-linear shade/fog interpolation (the GPU
 * interpolates perspective-correctly, like PC ports). Near-plane and guard-band handling stay. */
int gPortShadeSplit = 1;

#ifdef __3DS__
/* PORT PERF (2026-09-30): indexed batches (gfx_citro3d.c). Each vertex is written once per batch straight
 * into the VBO; a loaded N64 vertex (sCurVidx: the slots of the triangle being emitted unsplit) is reused
 * by index while the batch lasts. sBatchId bumps on every flush / packed-state change / frame; G_VTX
 * clears the stamp of each slot it rewrites. Split or clipped pieces (new vertices) are never reused. */
extern float* gfx_citro3d_vtx_reserve(u32 n, u32* first);
extern void gfx_citro3d_vtx_write(float* dst, const float* src);
extern int gfx_citro3d_idx_push(u32 a, u32 b, u32 c);
extern void gfx_citro3d_draw_indexed(void);

extern float* gfx_citro3d_vbo_info(int** pos, u32* cap, float scale[4]);
static float* sVbo;           /* VBO base (gfx_citro3d.c) */
static int* sVboPos;          /* its running vertex index */
static u32 sVboCap;
static float sVboScale[4];    /* texcoord scales of the bound texture units, per batch */
static uint32_t sVboInfoBatch; /* batch the above were fetched for */

static int pack_vertex(const PVtx* p, bool z_is_from_0_to_1, int slot) {
    u32 first;
    float* d;
    if (slot >= 0 && sBatchStamp[slot] == sBatchId) {
        return sBatchVtx[slot];
    }
    if (sVboInfoBatch != sBatchId) { /* textures are bound per batch: fetch once */
        sVbo = gfx_citro3d_vbo_info(&sVboPos, &sVboCap, sVboScale);
        sVboInfoBatch = sBatchId;
    }
    if (sVbo == NULL || (u32)*sVboPos >= sVboCap) {
        return -1;
    }
    first = (*sVboPos)++;
    d = sVbo + first * 13;
    /* the PICA layout (gfx_citro3d.c writeVertex): portrait (y, -x), z negated, scaled texcoords */
    d[0] = p->y;
    d[1] = -p->x;
    d[2] = -(z_is_from_0_to_1 ? (p->z + p->w) * 0.5f : p->z);
    d[3] = p->w;
    d[4] = p->uv[0][0] * sVboScale[0];
    d[5] = 1 - p->uv[0][1] * sVboScale[1];
    d[6] = p->uv[1][0] * sVboScale[2];
    d[7] = 1 - p->uv[1][1] * sVboScale[3];
    d[8] = p->c[0];
    d[9] = p->c[1];
    d[10] = p->c[2];
    d[11] = p->c[3];
    d[12] = p->s;
    if (gPortReplayRec) {
        rec_vbo_vertex((int)first, d, p->x, p->y, p->w, slot);
    }
    if (buf_vbo_len == 0) { /* the batch's first vertex, for the draw log (port_draw_snapshot) */
        buf_vbo[0] = p->x, buf_vbo[1] = p->y, buf_vbo[2] = z_is_from_0_to_1 ? (p->z + p->w) * 0.5f : p->z;
        buf_vbo[3] = p->w, buf_vbo[4] = p->uv[0][0], buf_vbo[5] = p->uv[0][1], buf_vbo[6] = p->uv[1][0];
        buf_vbo[7] = p->uv[1][1], buf_vbo[8] = p->c[0], buf_vbo[9] = p->c[1];
        buf_vbo_len = 1;
    }
    if (slot >= 0) {
        sBatchVtx[slot] = (uint16_t)first;
        sBatchStamp[slot] = sBatchId;
    }
    return (int)first;
}
#endif

#ifdef __3DS__
static void gpu_pack_clip_tri(const PVtx* t[3]);
/* PORT DEBUG (2026-10-02): settings tjdump=<frame>: every triangle drawn to the screen in that display-list
 * walk, in screen space (x/w, y/w), to sdmc:/3ds/oot/tjdump.bin for tools/tjunctions.py, which counts
 * T-junctions - a vertex inside another triangle's edge, where the PICA's rasterizer can leave pixel cracks
 * (the shading split adds vertices on edges). Off unless the setting is present. */
static void* sDrawTarget; /* NULL = screen (gfx_select_target) */
int gPortTjDumpFrame = -1;
static float (*sTj)[8];
static int sTjN, sTjOn;
#define TJ_MAX 60000
static void tj_add(float x0, float y0, float x1, float y1, float x2, float y2, int kind) {
    if (sTj == NULL || sTjN >= TJ_MAX || sDrawTarget != NULL) {
        return;
    }
    sTj[sTjN][0] = x0, sTj[sTjN][1] = y0, sTj[sTjN][2] = x1, sTj[sTjN][3] = y1;
    sTj[sTjN][4] = x2, sTj[sTjN][5] = y2;
    sTj[sTjN][6] = (float)gfx_port_tri_count; /* the source triangle (its pieces share it) */
    sTj[sTjN][7] = (float)kind;               /* 0 drawn whole, 1 piece of a split/clipped triangle */
    sTjN++;
}
#endif
static void gfx_pack_tri(const PVtx* t[3], bool z_is_from_0_to_1) {
    if (buf_vbo_num_tris == MAX_BUFFERED) {
        gfx_flush();
    }
#ifdef __3DS__
    if (sTjOn) {
        tj_add(t[0]->x / t[0]->w, t[0]->y / t[0]->w, t[1]->x / t[1]->w, t[1]->y / t[1]->w, t[2]->x / t[2]->w,
               t[2]->y / t[2]->w, sCurVidx == NULL);
    }
#endif
#ifdef __3DS__
    if (gPortGpuVtx) { /* GPU path: a piece of a triangle split/clipped on the CPU, in clip space */
        gpu_pack_clip_tri(t);
        return;
    }
    if (!gPortLegacyVbo) {
        int ix[3], i;
        for (i = 0; i < 3; i++) {
            ix[i] = pack_vertex(t[i], z_is_from_0_to_1, sCurVidx != NULL ? sCurVidx[i] : -1);
            if (ix[i] < 0) {
                return; /* this frame's vertex buffer is full */
            }
        }
        if (!gfx_citro3d_idx_push(ix[0], ix[1], ix[2])) {
            return;
        }
        buf_vbo_len = buf_vbo_len ? buf_vbo_len : 1;
        if (++buf_vbo_num_tris == MAX_BUFFERED) {
            gfx_flush();
        }
        return;
    }
#endif
    for (int i = 0; i < 3; i++) {
        const PVtx* p = t[i];
        float z = p->z, w = p->w;
        if (z_is_from_0_to_1) {
            z = (z + w) / 2.0f;
        }
        buf_vbo[buf_vbo_len++] = p->x;
        buf_vbo[buf_vbo_len++] = p->y;
        buf_vbo[buf_vbo_len++] = z;
        buf_vbo[buf_vbo_len++] = w;
        buf_vbo[buf_vbo_len++] = p->uv[0][0];
        buf_vbo[buf_vbo_len++] = p->uv[0][1];
        buf_vbo[buf_vbo_len++] = p->uv[1][0];
        buf_vbo[buf_vbo_len++] = p->uv[1][1];
        buf_vbo[buf_vbo_len++] = p->c[0];
        buf_vbo[buf_vbo_len++] = p->c[1];
        buf_vbo[buf_vbo_len++] = p->c[2];
        buf_vbo[buf_vbo_len++] = p->c[3];
        buf_vbo[buf_vbo_len++] = p->s;
    }
    if (++buf_vbo_num_tris == MAX_BUFFERED) {
        gfx_flush();
    }
}

/* screen-space midpoint of a-b: perspective-correct position/uv, screen-linear shade */
static void pvtx_mid_screen(const PVtx* a, const PVtx* b, PVtx* m) {
    float qa = 1.0f / a->w, qb = 1.0f / b->w, q = 0.5f * (qa + qb), w = 1.0f / q;
    m->x = 0.5f * (a->x * qa + b->x * qb) * w;
    m->y = 0.5f * (a->y * qa + b->y * qb) * w;
    m->z = 0.5f * (a->z * qa + b->z * qb) * w;
    m->w = w;
    for (int i = 0; i < 4; i++) {
        m->uv[i >> 1][i & 1] = 0.5f * (a->uv[i >> 1][i & 1] * qa + b->uv[i >> 1][i & 1] * qb) * w;
    }
    for (int i = 0; i < 4; i++) {
        m->c[i] = 0.5f * (a->c[i] + b->c[i]);
    }
    m->s = 0.5f * (a->s * qa + b->s * qb) * w; /* like x/y: the split point stays on the shifted edge */
}

/* how much an edge needs splitting: screen length in N64 pixels, if its w ratio is significant */
static float pvtx_edge_score(const PVtx* a, const PVtx* b) {
    float hi = a->w > b->w ? a->w : b->w, lo = a->w > b->w ? b->w : a->w;
    if (hi < SUBDIV_MAX_RATIO * lo) { /* common case: no division */
        return 0.0f;
    }
    float r = hi / lo;
    float dx = (a->x / a->w - b->x / b->w) * (SCREEN_WIDTH / 2), dy = (a->y / a->w - b->y / b->w) * (SCREEN_HEIGHT / 2);
    float len = sqrtf(dx * dx + dy * dy);
    return len < SUBDIV_MIN_PIXELS ? 0.0f : len * (r - 1.0f);
}

/* PORT (2026-10-02): crack-free splitting. The N64 draws whole triangles; the port cuts big near ones (the RDP's
 * screen-linear shade/fog, near plane, guard band). A cut edge gains vertices that the neighbouring triangle
 * sharing that edge must also have, or the edge becomes a T-junction and the 3DS rasterizer leaves pixel cracks
 * along it (hardware v39: thin white dots on the title screen's ground, worse in 3D without anti-aliasing;
 * tools/tjunctions.py counted ~80 such junctions in one title frame). So an edge is split only by a rule that
 * depends on that edge alone - its two end vertices (bit-identical in both triangles: the midpoint formula is
 * symmetric) and its level, the number of halvings that made it - never on the triangle around it: no shared
 * recursion depth, no triangle-wide flatness test. Recursion goes on until no edge qualifies, so each edge ends
 * up with exactly the points its own recursion gives, whatever order a triangle splits its edges in. */
static bool sub_edge_splits(const PVtx* a, const PVtx* b, int level) {
    return level < SUBDIV_MAX_DEPTH && gPortShadeSplit && pvtx_edge_score(a, b) > 0.0f;
}

static void gfx_subdiv_tri(const PVtx* a, const PVtx* b, const PVtx* c, const uint8_t lv[3], int guard, bool zf) {
    const PVtx* t[3] = { a, b, c };
    int best = -1;
    float bestScore = 0.0f;
    if (guard < 4 * 8) { /* never reached in practice (edge levels bound the recursion) */
        for (int e = 0; e < 3; e++) {
            if (lv[e] < SUBDIV_MAX_DEPTH && gPortShadeSplit) {
                float sc = pvtx_edge_score(t[e], t[(e + 1) % 3]);
                if (sc > bestScore) {
                    bestScore = sc;
                    best = e;
                }
            }
        }
    }
    if (best < 0) {
        gfx_pack_tri(t, zf);
        return;
    }
    const PVtx *e0 = t[best], *e1 = t[(best + 1) % 3], *opp = t[(best + 2) % 3];
    const uint8_t half = (uint8_t)(lv[best] + 1);
    /* children (e0, m, opp) and (m, e1, opp): the two halves, their shared new edge m-opp (same level in
     * both), and the parent's other two edges unchanged */
    const uint8_t la[3] = { half, half, lv[(best + 2) % 3] }, lb[3] = { half, lv[(best + 1) % 3], half };
    PVtx m;
    pvtx_mid_screen(e0, e1, &m);
    gfx_subdiv_tri(e0, &m, opp, la, guard + 1, zf); /* same winding as (e0, e1, opp) */
    gfx_subdiv_tri(&m, e1, opp, lb, guard + 1, zf);
}

/* the points the edge a-b gains (in order from a), by the same rule as gfx_subdiv_tri */
static int sub_edge_points(const PVtx* a, const PVtx* b, int level, PVtx* out, int n, int cap) {
    PVtx m;
    if (n >= cap || !sub_edge_splits(a, b, level)) {
        return n;
    }
    pvtx_mid_screen(a, b, &m);
    n = sub_edge_points(a, &m, level + 1, out, n, cap);
    if (n < cap) {
        out[n++] = m;
    }
    return sub_edge_points(&m, b, level + 1, out, n, cap);
}

/* a triangle whose shade and fog are flat needs no inner splits, but its edges must still gain the points its
 * neighbours give them: its outline with those points, as a fan around its centroid */
static void gfx_subdiv_outline(const PVtx* a, const PVtx* b, const PVtx* c, bool zf) {
    static PVtx poly[3 * 64 + 3];
    const PVtx* t[3] = { a, b, c };
    const int cap = (int)(sizeof(poly) / sizeof(poly[0]));
    PVtx g;
    int n = 0, i;
    for (int e = 0; e < 3; e++) {
        poly[n++] = *t[e];
        n = sub_edge_points(t[e], t[(e + 1) % 3], 0, poly, n, cap);
    }
    if (n == 3) {
        gfx_pack_tri(t, zf);
        return;
    }
    /* the centroid in clip space: a point of the triangle, attributes as the GPU interpolates them */
    g.x = (a->x + b->x + c->x) * (1.0f / 3.0f), g.y = (a->y + b->y + c->y) * (1.0f / 3.0f);
    g.z = (a->z + b->z + c->z) * (1.0f / 3.0f), g.w = (a->w + b->w + c->w) * (1.0f / 3.0f);
    for (i = 0; i < 4; i++) {
        g.uv[i >> 1][i & 1] = (a->uv[i >> 1][i & 1] + b->uv[i >> 1][i & 1] + c->uv[i >> 1][i & 1]) * (1.0f / 3.0f);
        g.c[i] = (a->c[i] + b->c[i] + c->c[i]) * (1.0f / 3.0f);
    }
    g.s = (a->s + b->s + c->s) * (1.0f / 3.0f);
    for (i = 0; i < n; i++) {
        const PVtx* f[3] = { &g, &poly[i], &poly[(i + 1) % n] };
        gfx_pack_tri(f, zf);
    }
}

/* the two ends of an edge in a fixed order (both triangles sharing it compute its cut points identically) */
static bool pvtx_before(const PVtx* p, const PVtx* q) {
    if (p->x != q->x) return p->x < q->x;
    if (p->y != q->y) return p->y < q->y;
    if (p->z != q->z) return p->z < q->z;
    return p->w < q->w;
}

static void pvtx_lerp_clip(const PVtx* a, const PVtx* b, float t, PVtx* o) {
    o->x = a->x + (b->x - a->x) * t;
    o->y = a->y + (b->y - a->y) * t;
    o->z = a->z + (b->z - a->z) * t;
    o->w = a->w + (b->w - a->w) * t;
    for (int i = 0; i < 4; i++) {
        o->uv[i >> 1][i & 1] = a->uv[i >> 1][i & 1] + (b->uv[i >> 1][i & 1] - a->uv[i >> 1][i & 1]) * t;
    }
    for (int i = 0; i < 4; i++) {
        o->c[i] = a->c[i] + (b->c[i] - a->c[i]) * t;
    }
    o->s = a->s + (b->s - a->s) * t;
}

/* NoN microcode: nothing is clipped or rejected at the near plane; depth in front of it clamps */
static void pvtx_clamp_near(PVtx* p) {
    if (p->z < -p->w) {
        p->z = -p->w;
    }
}

static void gfx_emit_tri_one(const PVtx* a, const PVtx* b, const PVtx* c, bool zf);

/* stereo: split triangles that cross the convergence plane w = c (see stereo_offset) */
static void gfx_emit_tri(const PVtx* a, const PVtx* b, const PVtx* c, bool zf) {
    float cv = stereo_conv();
    const PVtx* t[3] = { a, b, c };
    int nearCnt = 0;
    if (cv != 0.0f && sStereoDrawMode == 0 && a->w != 1.0f) {
        stereo_probe_tri(t);
    }
    if (cv != 0.0f && STEREO_NEAR_SLOPE != 1.0f) {
        for (int i = 0; i < 3; i++) nearCnt += (t[i]->w != 1.0f && t[i]->w < cv);
    }
    if (nearCnt == 0 || nearCnt == 3) {
        gfx_emit_tri_one(a, b, c, zf);
        return;
    }
#ifdef __3DS__
    sCurVidx = NULL; /* pieces split at the convergence plane are new vertices */
#endif
    PVtx side[2][4];
    int n[2] = { 0, 0 };
    for (int i = 0; i < 3; i++) {
        const PVtx *p = t[i], *q = t[(i + 1) % 3];
        int sp = p->w < cv, sq = q->w < cv;
        side[sp][n[sp]++] = *p;
        if (sp != sq) {
            PVtx m;
            if (pvtx_before(q, p)) { /* the same cut point as the neighbour sharing this edge */
                pvtx_lerp_clip(q, p, (cv - q->w) / (p->w - q->w), &m);
            } else {
                pvtx_lerp_clip(p, q, (cv - p->w) / (q->w - p->w), &m);
            }
            m.w = cv; /* exactly on the plane: s = 0 from both sides */
            m.s = 0.0f;
            side[0][n[0]++] = m;
            side[1][n[1]++] = m;
        }
    }
    for (int k = 0; k < 2; k++) {
        for (int i = 1; i + 1 < n[k]; i++) {
            gfx_emit_tri_one(&side[k][0], &side[k][i], &side[k][i + 1], zf);
        }
    }
}

static void gfx_emit_tri_one(const PVtx* a, const PVtx* b, const PVtx* c, bool zf) {
    const PVtx* t[3] = { a, b, c };
    PVtx poly[9], tmp[9];
    int n = 0;
    /* The RSP clips a triangle when a vertex lies outside the guard band (x, y = +-ratio * w; rooms
     * use ratio 1 = the screen edges): the new edge vertices get attributes interpolated in clip
     * space, and only between those does the RDP interpolate linearly. */
    float r = rsp.clip_ratio ? (float)rsp.clip_ratio : 2.0f;
    bool outside = false;
    for (int i = 0; i < 3; i++) {
        const PVtx* p = t[i];
        outside |= p->w <= 0.0f || p->x < -r * p->w || p->x > r * p->w || p->y < -r * p->w || p->y > r * p->w;
    }
    if (!outside) {
        bool big = false, near = false;
        /* PORT PERF (2026-10-01): the split only exists because the RDP interpolates shade/fog screen-
         * linearly and the PICA perspective-correctly; when the three vertices' shade and fog are (almost)
         * equal both interpolations give the same values (the difference is bounded by their range), so
         * there is nothing to split. Splitting was the bulk of the triangle cost (GPU-path profile). */
        float crange = 0.0f;
        for (int k = 0; k < 4; k++) {
            float lo = t[0]->c[k], hi = lo;
            lo = t[1]->c[k] < lo ? t[1]->c[k] : lo, hi = t[1]->c[k] > hi ? t[1]->c[k] : hi;
            lo = t[2]->c[k] < lo ? t[2]->c[k] : lo, hi = t[2]->c[k] > hi ? t[2]->c[k] : hi;
            crange = hi - lo > crange ? hi - lo : crange;
        }
        for (int e = 0; e < 3; e++) {
            big |= sub_edge_splits(t[e], t[(e + 1) % 3], 0); /* per edge: see gfx_subdiv_tri */
            near |= t[e]->z < -t[e]->w;
        }
        if (!big && !near) {
            gfx_pack_tri(t, zf); /* unsplit: may reuse the loaded vertices (sCurVidx) */
            return;
        }
        PROF_SET(PROF_SPLIT);
#ifdef __3DS__
        sCurVidx = NULL; /* split pieces are new vertices */
#endif
        for (int i = 0; i < 3; i++) {
            poly[i] = *t[i];
            pvtx_clamp_near(&poly[i]);
        }
#ifdef __3DS__
        sBatchSub++;
#endif
        if (crange <= SUBDIV_FLAT_RANGE) {
            gfx_subdiv_outline(&poly[0], &poly[1], &poly[2], zf); /* flat: the inside needs no splits */
        } else {
            static const uint8_t lv0[3] = { 0, 0, 0 };
            gfx_subdiv_tri(&poly[0], &poly[1], &poly[2], lv0, 0, zf);
        }
        return;
    }
    PROF_SET(PROF_SPLIT);
#ifdef __3DS__
    sBatchBehind++; /* counts guard-band clips */
    sCurVidx = NULL; /* clipped pieces are new vertices */
#endif
    for (int i = 0; i < 3; i++) {
        poly[n++] = *t[i];
    }
    for (int plane = 0; plane < 5 && n >= 3; plane++) {
        int m = 0;
        for (int i = 0; i < n; i++) {
            const PVtx *p = &poly[i], *q = &poly[(i + 1) % n];
            float dp, dq;
            switch (plane) {
                case 0: dp = r * p->w + p->x; dq = r * q->w + q->x; break;
                case 1: dp = r * p->w - p->x; dq = r * q->w - q->x; break;
                case 2: dp = r * p->w + p->y; dq = r * q->w + q->y; break;
                case 3: dp = r * p->w - p->y; dq = r * q->w - q->y; break;
                default: dp = p->w - 1e-3f; dq = q->w - 1e-3f; break; /* the eye itself */
            }
            if (dp >= 0.0f) {
                tmp[m++] = *p;
            }
            if ((dp >= 0.0f) != (dq >= 0.0f) && m < 9) {
                if (pvtx_before(q, p)) { /* the neighbour sharing this edge cuts it at the same point */
                    pvtx_lerp_clip(q, p, dq / (dq - dp), &tmp[m++]);
                } else {
                    pvtx_lerp_clip(p, q, dp / (dp - dq), &tmp[m++]);
                }
            }
        }
        memcpy(poly, tmp, sizeof(PVtx) * m);
        n = m;
    }
    for (int i = 0; i < n; i++) {
        pvtx_clamp_near(&poly[i]);
    }
    for (int i = 1; i + 1 < n; i++) {
        static const uint8_t lv0[3] = { 0, 0, 0 };
        gfx_subdiv_tri(&poly[0], &poly[i], &poly[i + 1], lv0, 0, zf);
    }
}

/* PORT (2026-09-28): route draws to the color image's target: the screen, or an off-screen render target
 * read back into the game's buffer after the frame (gfx_3ds.c Port3ds_SetDrawTarget). Off-screen targets
 * are a 1:1 320x240 N64-pixel space: no pillarbox, no aspect squeeze, no supersampling. */
static void* sDrawTarget; /* NULL = screen */
/* PORT PERF (2026-10-03): the target only changes with the color image (or the frame boundary): checked per triangle
 * and rectangle, it re-ran gfx_cimg_is_screen each time (~1.6% of the Old 3DS CPU, tools/pcprof.py) */
static bool sTargetDirty = true;
static void gfx_apply_viewport(void);
static void gfx_apply_scissor(void);
static void gfx_select_target_slow(void);
static inline void gfx_select_target(void) {
    if (sTargetDirty) {
        gfx_select_target_slow();
    }
}
static void gfx_select_target_slow(void) {
#ifdef __3DS__
    extern void Port3ds_SetDrawTarget(void* addr, int width, int height);
    void* want = gfx_cimg_is_screen() ? NULL : rdp.color_image_address;
    sTargetDirty = false;
    if (want == sDrawTarget) {
        return;
    }
    gfx_flush();
    if (want != NULL && gPortReplayRec) {
        /* an off-screen render (read back into game memory): this frame is drawn straight away instead */
        extern void gfx_citro3d_rec_abort(void);
        gfx_citro3d_rec_abort();
        gPortReplayRec = 0;
        gPortReplayBroken = 1;
    }
    sDrawTarget = want;
    sGpuSlotCur = -1; /* the aspect squeeze differs on off-screen targets */
    if (want != NULL) {
        gfx_current_dimensions.width = SCREEN_WIDTH;
        gfx_current_dimensions.height = SCREEN_HEIGHT;
    } else {
        gfx_wapi->get_dimensions(&gfx_current_dimensions.width, &gfx_current_dimensions.height);
    }
    gfx_current_dimensions.aspect_ratio = (float)gfx_current_dimensions.width / (float)gfx_current_dimensions.height;
    Port3ds_SetDrawTarget(want, (int)rdp.color_image_width, (int)((rdp.scissor_raw[3] + 3) / 4));
    gfx_apply_viewport();
    gfx_apply_scissor();
    memset(&rendering_state.viewport, 0xFF, sizeof(rendering_state.viewport)); /* force re-send */
    memset(&rendering_state.scissor, 0xFF, sizeof(rendering_state.scissor));
#endif
}

/* PORT PERF (2026-09-30): per-triangle render-state setup (target, depth, viewport/scissor, combiner and
 * shader, constants, blending, textures and samplers) only changes when a display-list command changes
 * state. gfx_run_dl clears sTriStateOk on every command except vertex/triangle/matrix/DL-flow ones; while
 * it is set, triangles reuse the values derived for the previous one (sTC) and skip straight to
 * building their vertices. */
static bool sTriStateOk;
static struct {
    bool used_textures[2];
    const struct TileDesc* utile[2];
    float inv_tex_w[2], inv_tex_h[2];
    float u_scale[2], v_scale[2], u_off[2], v_off[2];
    bool linear_filter;
    bool z_is_from_0_to_1;
} sTC;

#ifdef __3DS__
/* ---- GPU vertex path: triangles (see the slot table above) ---- */
typedef struct {
    float pos[4];  /* model space, w = 1 (rectangles, split pieces: clip space, identity slot) */
    float uv0[2], uv1[2];
    float dpos[4]; /* skinned: previous frame's position - this one; w = 1: shade alpha is already fog */
    uint8_t c[4];  /* shade (alpha replaced by fog in the shader when G_FOG) */
    uint8_t idx;   /* palette index */
    uint8_t pad[3];
} GpuVtx;          /* 56 bytes = GPU_STRIDE in gfx_citro3d.c */
extern void* gfx_citro3d_gpu_vbo(int** pos, u32* cap, float scale[4]);
extern void gfx_citro3d_gpu_params(float fogMul, float fogOff, float fogOn, float conv, float lin);
extern void gfx_citro3d_gpu_cull(int mode);
static GpuVtx* sGpuVbo;
static int* sGpuVboPos;
static u32 sGpuVboCap;
static float sGpuScale[4];
static uint32_t sGpuVboBatch;
/* PORT PERF (2026-10-03): per batch, each texture coordinate is one multiply-add of the loaded vertex's u or v:
 * uv0.x = u * sGpuUvA[0] + sGpuUvB[0], uv0.y = 1 - (v * sGpuUvA[1] + sGpuUvB[1]), the same for uv1 with [2], [3]
 * (scale, offset, filter half-texel, 1 / size and the VBO's texture scale folded together) */
static float sGpuUvA[4], sGpuUvB[4];
static void gpu_uv_coeffs(void) {
    int t;
    for (t = 0; t < 2; t++) {
        if (sTC.used_textures[t]) {
            float half = sTC.linear_filter ? 0.5f : 0.0f;
            sGpuUvA[2 * t] = sTC.u_scale[t] * sTC.inv_tex_w[t] * sGpuScale[2 * t];
            sGpuUvB[2 * t] = (half - sTC.u_off[t]) * sTC.inv_tex_w[t] * sGpuScale[2 * t];
            sGpuUvA[2 * t + 1] = sTC.v_scale[t] * sTC.inv_tex_h[t] * sGpuScale[2 * t + 1];
            sGpuUvB[2 * t + 1] = (half - sTC.v_off[t]) * sTC.inv_tex_h[t] * sGpuScale[2 * t + 1];
        } else {
            sGpuUvA[2 * t] = sGpuUvB[2 * t] = sGpuUvA[2 * t + 1] = sGpuUvB[2 * t + 1] = 0.0f;
        }
    }
}
static int sGpuCull = -1, sGpuFogOn = -1;
static int16_t sGpuFogMul, sGpuFogOff;
static float sGpuConvSet = -1.0f;
/* PORT (2026-10-01): screen-linear shading (shader_gpu.v.pica stp.z): the GPU interpolates shade and fog
 * linearly on the screen like the RDP, so the CPU split is only needed for draws that use texture unit 1
 * (no projective mode there). Matches the split in Azahar (101-scene tour 5.92 vs 5.92), but OFF by default:
 * on a New 3DS (hardware v29) textures smeared and streaked with it - the projective texture coordinates
 * (s = u / w, q = 1 / w) lose precision in the real PICA's interpolators. settings.txt shade_linear=1. */
int gPortShadeLinear = 0;
static int sGpuLin = -1;
static void gpu_send_params(void) {
    /* N64 fog = z / w * mul + off; the shader sees z' = -(z + w) / 2: z / w = -2 z'/w - 1 */
    gfx_citro3d_gpu_params(-2.0f * rsp.fog_mul, (float)rsp.fog_offset - rsp.fog_mul, (float)sGpuFogOn, sGpuConvSet,
                           sGpuLin > 0 ? 1.0f : 0.0f);
}

/* per-triangle draw state the GPU path owns: cull mode, fog, stereo convergence (rectangles change the
 * geometry mode directly, so this is checked per triangle, not only after state commands) */
static int gpu_tri_state(void) {
    int cull = (rsp.geometry_mode & G_CULL_BOTH) >> 9; /* 1 front, 2 back, 3 both */
    int fogOn = (rsp.geometry_mode & G_FOG) != 0;
    float conv = stereo_conv();
    if (cull == 3) {
        return 0; /* the CPU path drew nothing either */
    }
    if (cull != sGpuCull) {
        gfx_flush();
        gfx_citro3d_gpu_cull(cull);
        sGpuCull = cull;
    }
    if (fogOn != sGpuFogOn || conv != sGpuConvSet || (fogOn && (rsp.fog_mul != sGpuFogMul || rsp.fog_offset != sGpuFogOff))) {
        gfx_flush();
        sGpuFogOn = fogOn, sGpuConvSet = conv, sGpuFogMul = rsp.fog_mul, sGpuFogOff = rsp.fog_offset;
        gpu_send_params();
    }
    return 1;
}

/* PORT PERF (2026-10-01): the palette PERSISTS across draws - an entry stays in its uniforms until the
 * palette is full (then it is emptied after a flush) or the frame ends; each draw uploads only the
 * entries added since the previous one. (Uploading the whole palette per draw and drawing whenever it
 * was full gave 367 draws/frame instead of 144 and tripled the draw-submission cost.) */
static int gpu_pal_index(uint16_t slot) {
    if (slot == GPU_SLOT_IDENTITY) {
        return sIdentPalBatch == sGpuPalEpoch ? sIdentPalIdx : -1;
    }
    return sSlotPalBatch[slot] == sGpuPalEpoch ? sSlotPalIdx[slot] : -1;
}

static int gpu_pal_add(uint16_t slot) {
    int i = gpu_pal_index(slot);
    if (i >= 0) {
        return i;
    }
    i = sGpuPalN++;
    sGpuPal[i] = slot;
    if (slot == GPU_SLOT_IDENTITY) {
        sIdentPalBatch = sGpuPalEpoch, sIdentPalIdx = (uint8_t)i;
    } else {
        sSlotPalBatch[slot] = sGpuPalEpoch, sSlotPalIdx[slot] = (uint8_t)i;
    }
    return i;
}

/* texture coordinates of a loaded vertex under the current render state (libultraship per-tile math) */
static inline void gpu_uv(const struct LoadedVertex* lv, float uv[2][2]) {
    int t;
    for (t = 0; t < 2; t++) {
        uv[t][0] = uv[t][1] = 0.0f;
        if (sTC.used_textures[t]) {
            float u = lv->u * sTC.u_scale[t] - sTC.u_off[t], v = lv->v * sTC.v_scale[t] - sTC.v_off[t];
            if (sTC.linear_filter) {
                u += 0.5f;
                v += 0.5f;
            }
            uv[t][0] = u * sTC.inv_tex_w[t];
            uv[t][1] = v * sTC.inv_tex_h[t];
        }
    }
}

/* a piece of a triangle split/clipped on the CPU (gfx_emit_tri), drawn in clip space via the identity
 * palette entry with its shade/fog precomputed (dpos.w = 1) */
/* PORT (2026-10-01): the source triangle of the pieces gpu_emit_cpu is cutting (set while it runs). Every
 * piece vertex (split midpoints, guard-band and stereo-plane intersections) is an affine combination of the
 * three source vertices in clip space, hence the same combination in model space: written in model space with
 * the source's palette entry, a piece moves with the camera in the 60 fps in-between frames like the rest of
 * the triangle. (In clip space with the identity entry, pieces stayed where the logic frame put them while
 * their neighbours moved: floor edges showed whenever Link walked - hardware v29/v30.) */
static int sPieceSrcOk;
static uint16_t sPieceSlot;
static float sPieceClip[3][3];  /* x, y, w of the source vertices (the PVtx space) */
static float sPieceModel[3][3]; /* their model-space positions */
static float sPieceDpos[3][3];  /* their skinned deltas toward the previous frame */

static void gpu_pack_clip_tri(const PVtx* t[3]) {
    int i, ix[3], pal = gpu_pal_index(GPU_SLOT_IDENTITY), spal = -1;
    float wgt[3][3];
    if (pal < 0) {
        pal = gpu_pal_add(GPU_SLOT_IDENTITY); /* the caller made room */
    }
    if (sPieceSrcOk) {
        /* weights of each piece vertex: solve p = a A + b B + c C in (x, y, w) (Cramer's rule) */
        const float(*m)[3] = sPieceClip;
        float det = m[0][0] * (m[1][1] * m[2][2] - m[2][1] * m[1][2]) - m[1][0] * (m[0][1] * m[2][2] - m[2][1] * m[0][2]) +
                    m[2][0] * (m[0][1] * m[1][2] - m[1][1] * m[0][2]);
        spal = gpu_pal_index(sPieceSlot);
        if (fabsf(det) < 1e-9f || spal < 0) {
            spal = -1;
        } else {
            float inv = 1.0f / det;
            for (i = 0; i < 3; i++) {
                float px = t[i]->x, py = t[i]->y, pw = t[i]->w;
                /* columns: A = m[0], B = m[1], C = m[2] (each x, y, w) */
                wgt[i][0] = (px * (m[1][1] * m[2][2] - m[2][1] * m[1][2]) - m[1][0] * (py * m[2][2] - m[2][1] * pw) +
                             m[2][0] * (py * m[1][2] - m[1][1] * pw)) * inv;
                wgt[i][1] = (m[0][0] * (py * m[2][2] - m[2][1] * pw) - px * (m[0][1] * m[2][2] - m[2][1] * m[0][2]) +
                             m[2][0] * (m[0][1] * pw - py * m[0][2])) * inv;
                wgt[i][2] = (m[0][0] * (m[1][1] * pw - py * m[1][2]) - m[1][0] * (m[0][1] * pw - py * m[0][2]) +
                             px * (m[0][1] * m[1][2] - m[1][1] * m[0][2])) * inv;
            }
        }
    }
    if (sGpuVboBatch != sBatchId) {
        sGpuVbo = (GpuVtx*)gfx_citro3d_gpu_vbo(&sGpuVboPos, &sGpuVboCap, sGpuScale);
        sGpuVboBatch = sBatchId;
        gpu_uv_coeffs(); /* gpu_emit_tri's per-batch coefficients */
    }
    for (i = 0; i < 3; i++) {
        const PVtx* p = t[i];
        GpuVtx tv, *d = &tv; /* built here, then one block store into the vertex buffer (see gpu_emit_tri) */
        int k;
        if (sGpuVbo == NULL || (u32)*sGpuVboPos >= sGpuVboCap) {
            return;
        }
        ix[i] = (*sGpuVboPos)++;
        if (spal >= 0) { /* model space with the source's palette entry (z clamps at the near plane in the shader) */
            for (k = 0; k < 3; k++) {
                d->pos[k] = wgt[i][0] * sPieceModel[0][k] + wgt[i][1] * sPieceModel[1][k] + wgt[i][2] * sPieceModel[2][k];
                d->dpos[k] = wgt[i][0] * sPieceDpos[0][k] + wgt[i][1] * sPieceDpos[1][k] + wgt[i][2] * sPieceDpos[2][k];
            }
            d->pos[3] = 1.0f;
            d->dpos[3] = 1.0f; /* shade alpha already holds the N64 fog factor */
            d->idx = (uint8_t)spal;
        } else {
            d->pos[0] = p->x, d->pos[1] = p->y, d->pos[2] = p->z, d->pos[3] = p->w;
            d->dpos[0] = d->dpos[1] = d->dpos[2] = 0.0f;
            d->dpos[3] = 1.0f;
            d->idx = (uint8_t)pal;
        }
        d->uv0[0] = p->uv[0][0] * sGpuScale[0];
        d->uv0[1] = 1 - p->uv[0][1] * sGpuScale[1];
        d->uv1[0] = p->uv[1][0] * sGpuScale[2];
        d->uv1[1] = 1 - p->uv[1][1] * sGpuScale[3];
        for (k = 0; k < 4; k++) {
            float c = p->c[k] * 255.0f + 0.5f;
            d->c[k] = (uint8_t)(c < 0.0f ? 0.0f : c > 255.0f ? 255.0f : c);
        }
        tv.pad[0] = tv.pad[1] = tv.pad[2] = 0;
        memcpy(&sGpuVbo[ix[i]], &tv, sizeof(tv));
    }
    if (!gfx_citro3d_idx_push(ix[0], ix[1], ix[2])) {
        return;
    }
    buf_vbo_len = buf_vbo_len ? buf_vbo_len : 1;
    if (++buf_vbo_num_tris == MAX_BUFFERED) {
        gfx_flush();
    }
}

/* clip z and w of a loaded vertex (two dot products, cached until the slot is reloaded) */
static inline const float* gpu_vtx_zw(int slot, uint16_t sl) {
    if (sZWStale[slot]) {
        const struct LoadedVertex* lv = &rsp.loaded_vertices[slot];
        const float(*rows)[4] = gpu_rows2(sl);
        float o2 = rows[2][0] * lv->x + rows[2][1] * lv->y + rows[2][2] * lv->z + rows[2][3];
        float w = rows[3][0] * lv->x + rows[3][1] * lv->y + rows[3][2] * lv->z + rows[3][3];
        sZW[slot][0] = -2.0f * o2 - w; /* out.z = -(z + w) / 2 */
        sZW[slot][1] = w;
        sZW[slot][2] = rows[0][0] * lv->x + rows[0][1] * lv->y + rows[0][2] * lv->z + rows[0][3];
        sZW[slot][3] = rows[1][0] * lv->x + rows[1][1] * lv->y + rows[1][2] * lv->z + rows[1][3];
        sZWStale[slot] = 0;
    }
    return sZW[slot];
}

/* the CPU path's N64 triangle setup for one triangle (big near triangles: screen-linear shade/fog by
 * splitting, near-plane clamp, guard-band clip): build its clip-space vertices and hand it over */
static void gpu_emit_cpu(const uint8_t vidx[3], const uint16_t sl[3]) {
    PVtx pv[3];
    int i;
    for (i = 0; i < 3; i++) {
        const struct LoadedVertex* lv = &rsp.loaded_vertices[vidx[i]];
        const float* zw = gpu_vtx_zw(vidx[i], sl[i]); /* the routing test's numbers, bit for bit */
        PVtx* p = &pv[i];
        p->x = -zw[3], p->y = zw[2], p->w = zw[1], p->z = zw[0];
        gpu_uv(lv, p->uv);
        p->c[0] = lv->color.r * (1.0f / 255.0f);
        p->c[1] = lv->color.g * (1.0f / 255.0f);
        p->c[2] = lv->color.b * (1.0f / 255.0f);
        if (rsp.geometry_mode & G_FOG) { /* as gfx_sp_vertex on the CPU path */
            float w = fabsf(p->w) < 0.001f ? 0.001f : p->w, winv = 1.0f / w;
            float fz;
            if (winv < 0.0f) winv = 32767.0f;
            fz = p->z * winv * rsp.fog_mul + rsp.fog_offset;
            fz = fz < 0.0f ? 0.0f : fz > 255.0f ? 255.0f : fz;
            p->c[3] = fz * (1.0f / 255.0f);
        } else {
            p->c[3] = lv->color.a * (1.0f / 255.0f);
        }
        p->s = stereo_offset(p->w);
    }
    {
        /* what the CPU path does before splitting: trivial off-screen rejection (all three outside the
         * same plane) and back/front-face culling (without them the split code also cut up triangles
         * the CPU path never drew: 156 vs 68 per-mille of frame time in "emit") */
        uint8_t rej = gPortReplayRec ? 0 : 0x2F; /* recording: the in-between cameras may still see it */
        for (i = 0; i < 3 && rej != 0; i++) {
            const PVtx* p = &pv[i];
            uint8_t r = 0;
            if (p->x < -p->w) r |= 1;
            if (p->x > p->w) r |= 2;
            if (p->y < -p->w) r |= 4;
            if (p->y > p->w) r |= 8;
            if (p->z > p->w) r |= 32;
            rej &= r;
        }
        if (rej) {
            return;
        }
        if ((rsp.geometry_mode & G_CULL_BOTH) != 0) {
            float cross = -(pv[0].x * (pv[1].y * pv[2].w - pv[2].y * pv[1].w) - pv[1].x * (pv[0].y * pv[2].w - pv[2].y * pv[0].w) +
                            pv[2].x * (pv[0].y * pv[1].w - pv[1].y * pv[0].w));
            if ((rsp.geometry_mode & G_CULL_BOTH) == G_CULL_FRONT ? cross <= 0 : cross >= 0) {
                return;
            }
        }
    }
    sPieceSrcOk = sl[0] == sl[1] && sl[1] == sl[2] && sl[0] != GPU_SLOT_IDENTITY;
    if (sPieceSrcOk) {
        for (i = 0; i < 3; i++) {
            const struct LoadedVertex* lv = &rsp.loaded_vertices[vidx[i]];
            sPieceClip[i][0] = pv[i].x, sPieceClip[i][1] = pv[i].y, sPieceClip[i][2] = pv[i].w;
            sPieceModel[i][0] = lv->x, sPieceModel[i][1] = lv->y, sPieceModel[i][2] = lv->z;
            sPieceDpos[i][0] = sLoadDpos[vidx[i]][0], sPieceDpos[i][1] = sLoadDpos[vidx[i]][1];
            sPieceDpos[i][2] = sLoadDpos[vidx[i]][2];
        }
        sPieceSlot = sl[0];
    }
    {
        int need = (gpu_pal_index(GPU_SLOT_IDENTITY) < 0) + (sPieceSrcOk && gpu_pal_index(sl[0]) < 0);
        if (sGpuPalN + need > GPU_PAL) {
            gfx_flush();
            sGpuPalEpoch++;
            sGpuPalN = sGpuPalSent = 0;
        }
    }
    gpu_pal_add(GPU_SLOT_IDENTITY);
    if (sPieceSrcOk) {
        gpu_pal_add(sl[0]);
    }
    sCurVidx = NULL;
    gfx_emit_tri(&pv[0], &pv[1], &pv[2], sTC.z_is_from_0_to_1);
    sPieceSrcOk = 0;
}

u32 gPortGpuRoute[4]; /* perf report: GPU-path triangles tested / behind the eye / near plane / split on the CPU */
u32 gPortPerfRawTris;  /* perf report: triangles drawn by the raw vertex path */

/* raw vertex path (gfx_sp_vertex_gpu): the three vertices are the game's own, copied as they are */
/* this frame's raw vertex buffer (gfx_citro3d_raw_vbo): base, running index, capacity */
static uint8_t* sRawVboP;
static int* sRawPosP;
static u32 sRawCapN;
static uint32_t sRawVboFrame = ~0u;

static void gpu_emit_tri_raw(const uint8_t vidx[3]) {
    uint16_t sl[3];
    int i, j, need = 0, ix[3];
    const int lit = sLoadLit[vidx[0]]; /* (gpu_emit_tri: all three raw, one light set) */
    PROF_SET(PROF_RAW_EMIT);
    for (i = 0; i < 3; i++) {
        sl[i] = sLoadSlot[vidx[i]];
    }
    if (!sRawDraw) { /* the raw program and vertex format */
        gfx_flush();
        gfx_citro3d_raw_mode(1);
        sRawDraw = 1;
        sRawParamBatch = 0;
    }
    if (lit != sRawLitCur && buf_vbo_num_tris > 0) {
        gfx_flush(); /* other lights: a draw of its own */
    }
    for (i = 0; i < 3; i++) {
        int dup = 0;
        for (j = 0; j < i; j++) dup |= sl[j] == sl[i];
        if (!dup && gpu_pal_index(sl[i]) < 0) need++;
    }
    if (sGpuPalN + need > GPU_PAL) { /* palette full: draw what uses it, then start an empty one */
        gfx_flush();
        sGpuPalEpoch++;
        sGpuPalN = sGpuPalSent = 0;
    } else if (buf_vbo_num_tris == MAX_BUFFERED) {
        gfx_flush();
    }
    if (sRawParamBatch != sBatchId || lit != sRawLitCur) {
        /* texture coordinates: U = tc * scaling >> 16 then the per-tile math of gpu_emit_tri, folded into one
         * multiply-add per axis; t flipped (1 - t) as the GPU path writes it */
        float uvc[2][4], scale[4];
        int t;
        int* dummyPos;
        u32 dummyCap;
        (void)gfx_citro3d_gpu_vbo(&dummyPos, &dummyCap, scale); /* the bound units' texture scales */
        for (t = 0; t < 2; t++) {
            if (sTC.used_textures[t]) {
                float half = sTC.linear_filter ? 0.5f : 0.0f;
                float ks = rsp.texture_scaling_factor.s * (1.0f / 65536.0f);
                float kt = rsp.texture_scaling_factor.t * (1.0f / 65536.0f);
                uvc[t][0] = ks * sTC.u_scale[t] * sTC.inv_tex_w[t] * scale[2 * t];
                uvc[t][1] = -(kt * sTC.v_scale[t] * sTC.inv_tex_h[t] * scale[2 * t + 1]);
                uvc[t][2] = (half - sTC.u_off[t]) * sTC.inv_tex_w[t] * scale[2 * t];
                uvc[t][3] = 1.0f - (half - sTC.v_off[t]) * sTC.inv_tex_h[t] * scale[2 * t + 1];
            } else {
                uvc[t][0] = uvc[t][1] = uvc[t][2] = uvc[t][3] = 0.0f;
            }
        }
        if (lit > 0) {
            const RawLights* L = &sRawLights[lit - 1];
            gfx_citro3d_raw_params(uvc[0], uvc[1], L->lit, L->amb, L->dir, L->col);
        } else {
            static const float unlit[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
            gfx_citro3d_raw_params(uvc[0], uvc[1], unlit, NULL, NULL, NULL);
        }
        sRawParamBatch = sBatchId;
        sRawLitCur = lit;
    }
    if (sRawVboFrame != gfx_port_frame_index || sRawVboP == NULL) {
        sRawVboP = (uint8_t*)gfx_citro3d_raw_vbo(&sRawPosP, &sRawCapN);
        sRawVboFrame = gfx_port_frame_index;
    }
    for (i = 0; i < 3; i++) {
        int slot = vidx[i];
        uint8_t* d;
        if (sBatchStamp[slot] == sBatchId) {
            ix[i] = sBatchVtx[slot];
            continue;
        }
        if (sRawVboP == NULL || (u32)*sRawPosP >= sRawCapN) {
            return; /* this frame's raw vertex buffer is full */
        }
        ix[i] = (*sRawPosP)++;
        d = sRawVboP + (size_t)ix[i] * 16;
        {
            /* one 16-byte block store: the buffer is linear memory, where scattered small stores cost the most on
             * an Old 3DS (no L2). The N64 flag field (bytes 6-7, the high half of word 1) carries the palette index. */
            const uint32_t pal = (uint32_t)gpu_pal_add(sl[i]);
            const uint32_t* w = (const uint32_t*)(const void*)sLoadRaw[slot];
            uint32_t* o = (uint32_t*)(void*)d; /* 16-byte aligned (the buffer and the stride) */
            uint32_t w0 = w[0], w1 = w[1], w2 = w[2], w3 = w[3];
            o[0] = w0, o[1] = (w1 & 0xFFFFu) | (pal << 16), o[2] = w2, o[3] = w3;
        }
        sBatchVtx[slot] = (uint16_t)ix[i];
        sBatchStamp[slot] = sBatchId;
    }
    if (!gfx_citro3d_idx_push(ix[0], ix[1], ix[2])) {
        return;
    }
    gPortPerfRawTris++;
    buf_vbo_len = buf_vbo_len ? buf_vbo_len : 1;
    if (++buf_vbo_num_tris == MAX_BUFFERED) {
        gfx_flush();
    }
}

/* PORT (2026-10-04): a raw vertex turned into a processed one (gpu_emit_tri), lit on the CPU with its own load's light
 * set exactly as shader_raw.v.pica does: ambient + sum max(0, n . L) * C, clamped */
u32 gPortPerfRawMaterialized;
static void raw_materialize(int slot) {
    const Vtx* vtx = sLoadRaw[slot];
    const Vtx_t* v = &vtx->v;
    struct LoadedVertex* d = &rsp.loaded_vertices[slot];
    const int lit = sLoadLit[slot];
    d->u = (short)(v->tc[0] * rsp.texture_scaling_factor.s >> 16);
    d->v = (short)(v->tc[1] * rsp.texture_scaling_factor.t >> 16);
    if (lit > 0) {
        const RawLights* L = &sRawLights[lit - 1];
        const signed char* n = vtx->n.n;
        float c[3], nv[3] = { n[0], n[1], n[2] };
        int k, i;
        if (L->lit[1] != 0.0f && sLoadSlot[slot] < sGpuSlotN) {
            /* a camera-space set: the normal to camera space as shader_raw.v.pica does (the logic frame's rows) */
            const float(*r)[4] = (const float(*)[4])sGpuSlot[sLoadSlot[slot]].rows[2];
            float s = sqrtf(r[3][0] * r[3][0] + r[3][1] * r[3][1] + r[3][2] * r[3][2]);
            float is = s > 0.0f ? 1.0f / s : 0.0f;
            nv[0] = (r[1][0] * n[0] + r[1][1] * n[1] + r[1][2] * n[2]) * L->lit[2] * is;
            nv[1] = (r[0][0] * n[0] + r[0][1] * n[1] + r[0][2] * n[2]) * L->lit[3] * is;
            nv[2] = -(r[3][0] * n[0] + r[3][1] * n[1] + r[3][2] * n[2]) * is;
        }
        for (k = 0; k < 3; k++) {
            c[k] = L->amb[k];
        }
        for (i = 0; i < 4; i++) {
            float in = nv[0] * L->dir[i][0] + nv[1] * L->dir[i][1] + nv[2] * L->dir[i][2];
            if (in > 0.0f) {
                for (k = 0; k < 3; k++) {
                    c[k] += in * L->col[i][k];
                }
            }
        }
        d->color.r = (uint8_t)(c[0] >= 1.0f ? 255 : (int)(c[0] * 255.0f + 0.5f));
        d->color.g = (uint8_t)(c[1] >= 1.0f ? 255 : (int)(c[1] * 255.0f + 0.5f));
        d->color.b = (uint8_t)(c[2] >= 1.0f ? 255 : (int)(c[2] * 255.0f + 0.5f));
    } else {
        d->color.r = v->cn[0], d->color.g = v->cn[1], d->color.b = v->cn[2];
    }
    d->color.a = v->cn[3];
    sLoadDpos[slot][0] = sLoadDpos[slot][1] = sLoadDpos[slot][2] = 0.0f; /* (raw loads skip that reset) */
    sLoadRaw[slot] = NULL;
    sBatchStamp[slot] = 0; /* not in this batch's GPU-path vertices yet */
    gPortPerfRawMaterialized++;
}

/* PORT PERF (2026-10-05): the raw path's common case - a triangle in a run of raw triangles under the same state -
 * without the general triangle path (target, culling, state and mode checks, palette insertion, the parameters):
 * all three vertices raw with the batch's light set, nothing changed since the last accepted triangle, and every
 * vertex new to the batch with its matrix already in the palette. Anything else (or a buffer about to fill) returns 0
 * and takes gfx_sp_tri1, which does the same work in full. */
static int raw_tri_fast(int a, int b, int c) {
    const int v[3] = { a, b, c };
    int i, ix[3];
    if (!sTriStateOk || !sRawDraw || sRawParamBatch != sBatchId || a >= MAX_VERTICES || b >= MAX_VERTICES ||
        c >= MAX_VERTICES || sLoadRaw[a] == NULL || sLoadRaw[b] == NULL || sLoadRaw[c] == NULL ||
        sLoadLit[a] != sRawLitCur || sLoadLit[b] != sRawLitCur || sLoadLit[c] != sRawLitCur ||
        buf_vbo_num_tris + 1 >= MAX_BUFFERED || sRawVboFrame != gfx_port_frame_index || sRawVboP == NULL) {
        return 0;
    }
    if (rsp.loaded_vertices[a].clip_rej & rsp.loaded_vertices[b].clip_rej & rsp.loaded_vertices[c].clip_rej) {
        gfx_port_tri_count++;
        { extern u32 gPortPerfTrisIn; gPortPerfTrisIn++; gPortPerfRoomTris += sInRoomDl != 0; }
        return 1; /* entirely outside one clip plane: as gfx_sp_tri1_impl */
    }
    for (i = 0; i < 3; i++) {
        int slot = v[i];
        if (sBatchStamp[slot] == sBatchId) {
            ix[i] = sBatchVtx[slot];
        } else {
            int pal = gpu_pal_index(sLoadSlot[slot]);
            const uint32_t* w;
            uint32_t* o;
            if (pal < 0 || (u32)*sRawPosP >= sRawCapN) {
                return 0; /* (vertices placed so far stay valid in the batch) */
            }
            ix[i] = (*sRawPosP)++;
            w = (const uint32_t*)(const void*)sLoadRaw[slot];
            o = (uint32_t*)(void*)(sRawVboP + (size_t)ix[i] * 16);
            o[0] = w[0], o[1] = (w[1] & 0xFFFFu) | ((uint32_t)pal << 16), o[2] = w[2], o[3] = w[3];
            sBatchVtx[slot] = (uint16_t)ix[i];
            sBatchStamp[slot] = sBatchId;
        }
    }
    gfx_port_tri_count++;
    { extern u32 gPortPerfTrisIn; gPortPerfTrisIn++; gPortPerfRoomTris += sInRoomDl != 0; }
    if (!gfx_citro3d_idx_push(ix[0], ix[1], ix[2])) {
        return 1; /* index buffer full: dropped, as the general path does */
    }
    gPortPerfRawTris++;
    buf_vbo_len = buf_vbo_len ? buf_vbo_len : 1;
    ++buf_vbo_num_tris;
    return 1;
}

static void gpu_emit_tri(const uint8_t vidx[3]) {
    uint16_t sl[3];
    int i, j, need = 0, ix[3];
    {
        /* raw only when all three vertices are raw and share one light set: the shader lights a draw with one set, in
         * one model space. A triangle across two limbs (Link's hat and head) with the first limb's lights showed its
         * facets (hardware v54); a triangle across a raw and a processed load was dropped or got garbage colours. */
        int r0 = vidx[0] < MAX_VERTICES && sLoadRaw[vidx[0]] != NULL;
        int r1 = vidx[1] < MAX_VERTICES && sLoadRaw[vidx[1]] != NULL;
        int r2 = vidx[2] < MAX_VERTICES && sLoadRaw[vidx[2]] != NULL;
        if (r0 && r1 && r2 && sLoadLit[vidx[0]] == sLoadLit[vidx[1]] && sLoadLit[vidx[0]] == sLoadLit[vidx[2]]) {
            gpu_emit_tri_raw(vidx);
            return;
        }
        if (r0) raw_materialize(vidx[0]);
        if (r1) raw_materialize(vidx[1]);
        if (r2) raw_materialize(vidx[2]);
    }
    if (sRawDraw) { /* back to the GPU path's program and vertex format */
        gfx_flush();
        gfx_citro3d_raw_mode(0);
        sRawDraw = 0;
    }
    {
        int lin = gPortShadeLinear && gPortShadeSplit && !gPortRawRelax && !sTC.used_textures[1];
        if (lin != sGpuLin) {
            gfx_flush();
            sGpuLin = lin;
            gpu_send_params();
        }
    }
    for (i = 0; i < 3; i++) {
        sl[i] = vidx[i] >= MAX_VERTICES ? GPU_SLOT_IDENTITY : sLoadSlot[vidx[i]];
    }
    if (vidx[0] < MAX_VERTICES && vidx[1] < MAX_VERTICES && vidx[2] < MAX_VERTICES) {
        /* N64-exact cases stay on the CPU (gfx_emit_tri_one): big depth range (the RDP shades and fogs
         * screen-linearly; Fire Temple floors were ~25% too dark without the split), a vertex behind
         * the eye or in front of the near plane (the NoN microcode clamps there; the PICA would clip) */
        const float* a = gpu_vtx_zw(vidx[0], sl[0]);
        const float* b = gpu_vtx_zw(vidx[1], sl[1]);
        const float* c = gpu_vtx_zw(vidx[2], sl[2]);
        float hi = a[1] > b[1] ? a[1] : b[1], lo = a[1] < b[1] ? a[1] : b[1];
        hi = c[1] > hi ? c[1] : hi;
        lo = c[1] < lo ? c[1] : lo;
        int cpu = !gPortRawRelax && (lo <= 0.0f || a[0] < -a[1] || b[0] < -b[1] || c[0] < -c[1]); /* (speed rules: the GPU clips) */
        { extern u32 gPortGpuRoute[4]; gPortGpuRoute[0]++; if (lo <= 0.0f) gPortGpuRoute[1]++; else if (cpu) gPortGpuRoute[2]++; }
        if (!cpu && sGpuLin > 0) {
            /* screen-linear mode: the RSP clips triangles that leave the guard band (x, y beyond +-ratio * w)
             * and interpolates the new edge vertices in clip space - the CPU clipper does exactly that */
            float r = rsp.clip_ratio ? (float)rsp.clip_ratio : 2.0f;
            const float* q[3] = { a, b, c };
            for (j = 0; j < 3 && !cpu; j++) {
                float rw = r * q[j][1];
                cpu = q[j][2] > rw || q[j][2] < -rw || q[j][3] > rw || q[j][3] < -rw;
            }
        }
        /* (speed rules: no split for depth alone - triangles clipped near the camera still split, which keeps their
         * per-vertex depth clamp small) */
        if (!cpu && gPortShadeSplit && !gPortRawRelax && !sGpuLin && hi >= SUBDIV_MAX_RATIO * lo) {
            /* PORT (2026-10-02): any edge the CPU splitter would cut sends the triangle there - even when its
             * shade and fog are flat (it then gets its outline only, gfx_subdiv_outline): a neighbour that cuts
             * the shared edge would otherwise leave a T-junction, a crack on hardware. Same clip positions as
             * gpu_emit_cpu (sZW), so both triangles of an edge decide alike. */
            PVtx q[3];
            int e, k;
            for (k = 0; k < 3; k++) {
                const float* zw = k == 0 ? a : k == 1 ? b : c;
                q[k].x = -zw[3], q[k].y = zw[2], q[k].z = zw[0], q[k].w = zw[1];
            }
            for (e = 0; e < 3 && !cpu; e++) {
                cpu = sub_edge_splits(&q[e], &q[(e + 1) % 3], 0);
            }
        }
    gpu_no_split:
        if (cpu) {
            { extern u32 gPortGpuRoute[4]; gPortGpuRoute[3]++; }
            PROF_SET(PROF_SPLIT);
            gpu_emit_cpu(vidx, sl);
            return;
        }
        if (sTjOn) {
            tj_add(-a[3] / a[1], a[2] / a[1], -b[3] / b[1], b[2] / b[1], -c[3] / c[1], c[2] / c[1], 0);
        }
    }
    PROF_SET(PROF_GPU_PAL);
    for (i = 0; i < 3; i++) {
        int dup = 0;
        for (j = 0; j < i; j++) dup |= sl[j] == sl[i];
        if (!dup && gpu_pal_index(sl[i]) < 0) need++;
    }
    if (sGpuPalN + need > GPU_PAL) { /* palette full: draw what uses it, then start an empty one */
        gfx_flush();
        sGpuPalEpoch++;
        sGpuPalN = sGpuPalSent = 0;
    } else if (buf_vbo_num_tris == MAX_BUFFERED) {
        gfx_flush();
    }
    PROF_SET(PROF_GPU_PACK);
    if (sGpuVboBatch != sBatchId) { /* textures are bound per batch: fetch once */
        sGpuVbo = (GpuVtx*)gfx_citro3d_gpu_vbo(&sGpuVboPos, &sGpuVboCap, sGpuScale);
        sGpuVboBatch = sBatchId;
        gpu_uv_coeffs();
    }
    for (i = 0; i < 3; i++) {
        int slot = vidx[i];
        const struct LoadedVertex* lv = &rsp.loaded_vertices[slot];
        GpuVtx* d;
        if (sBatchStamp[slot] == sBatchId) {
            ix[i] = sBatchVtx[slot];
            continue;
        }
        if (sGpuVbo == NULL || (u32)*sGpuVboPos >= sGpuVboCap) {
            return; /* this frame's vertex buffer is full */
        }
        ix[i] = (*sGpuVboPos)++;
        d = &sGpuVbo[ix[i]];
        {
            /* built in registers / the stack, then one sequential block store into the vertex buffer (linear memory:
             * the scattered float and byte stores were the Old 3DS's "gpu pack" stage, 12-17% of its time on
             * hardware v49 against 1% at New 3DS speed) */
            GpuVtx t;
            float u = (float)lv->u, v = (float)lv->v;
            t.pos[0] = lv->x, t.pos[1] = lv->y, t.pos[2] = lv->z, t.pos[3] = 1.0f;
            /* texture coordinates: as gfx_sp_tri1_impl's packed vertices (libultraship per-tile math), per batch
             * coefficients (gpu_uv_coeffs) */
            t.uv0[0] = u * sGpuUvA[0] + sGpuUvB[0];
            t.uv0[1] = 1 - (v * sGpuUvA[1] + sGpuUvB[1]);
            t.uv1[0] = u * sGpuUvA[2] + sGpuUvB[2];
            t.uv1[1] = 1 - (v * sGpuUvA[3] + sGpuUvB[3]);
            if (slot < MAX_VERTICES) {
                t.dpos[0] = sLoadDpos[slot][0], t.dpos[1] = sLoadDpos[slot][1], t.dpos[2] = sLoadDpos[slot][2];
            } else {
                t.dpos[0] = t.dpos[1] = t.dpos[2] = 0.0f;
            }
            t.dpos[3] = 0.0f;
            t.c[0] = lv->color.r, t.c[1] = lv->color.g, t.c[2] = lv->color.b, t.c[3] = lv->color.a;
            t.idx = (uint8_t)gpu_pal_add(sl[i]);
            t.pad[0] = t.pad[1] = t.pad[2] = 0;
            memcpy(d, &t, sizeof(t));
        }
        sBatchVtx[slot] = (uint16_t)ix[i];
        sBatchStamp[slot] = sBatchId;
    }
    if (stereo_conv() != 0.0f && sStereoDrawMode == 0) {
        /* 3D on: feed the border probes of the automatic convergence (gfx_3ds.c) as the CPU path does;
         * clip x/y/w per vertex from its slot rows (out0 = y, out1 = -x, out3 = w), once per frame */
        static PVtx sProbeV[MAX_VERTICES + 4];
        const PVtx* t[3];
        for (i = 0; i < 3; i++) {
            int slot = vidx[i];
            if (slot >= MAX_VERTICES || sProbeStale[slot]) { /* rectangles: rewritten without a load */
                const struct LoadedVertex* lv = &rsp.loaded_vertices[slot];
                const float(*rows)[4] = gpu_rows2(sl[i]);
                sProbeV[slot].y = rows[0][0] * lv->x + rows[0][1] * lv->y + rows[0][2] * lv->z + rows[0][3];
                sProbeV[slot].x = -(rows[1][0] * lv->x + rows[1][1] * lv->y + rows[1][2] * lv->z + rows[1][3]);
                sProbeV[slot].w = rows[3][0] * lv->x + rows[3][1] * lv->y + rows[3][2] * lv->z + rows[3][3];
                sProbeStale[slot] = 0;
            }
            t[i] = &sProbeV[slot];
        }
        if (t[0]->w != 1.0f) {
            stereo_probe_tri(t);
        }
    }
    if (!gfx_citro3d_idx_push(ix[0], ix[1], ix[2])) {
        return;
    }
    buf_vbo_len = buf_vbo_len ? buf_vbo_len : 1;
    if (++buf_vbo_num_tris == MAX_BUFFERED) {
        gfx_flush();
    }
}
#endif

static void gfx_sp_tri1_impl(uint8_t vtx1_idx, uint8_t vtx2_idx, uint8_t vtx3_idx);
static void gfx_sp_tri1(uint8_t vtx1_idx, uint8_t vtx2_idx, uint8_t vtx3_idx) {
    uint64_t t0 = PERF_T(), f0 = gPortPerfFlush;
    PROF_SET(PROF_TRI);
    gfx_sp_tri1_impl(vtx1_idx, vtx2_idx, vtx3_idx);
    PROF_SET(PROF_DL);
    gPortPerfTri += (PERF_T() - t0) - (gPortPerfFlush - f0);
}

static void gfx_sp_tri1_impl(uint8_t vtx1_idx, uint8_t vtx2_idx, uint8_t vtx3_idx) {
#ifdef PORT_ICONGEN
    if (mesh_capturing()) sMeshDbgTri1++;
#endif
    gfx_select_target();
    gfx_port_tri_count++;
#ifdef __3DS__
    { extern u32 gPortPerfTrisIn; gPortPerfTrisIn++; gPortPerfRoomTris += sInRoomDl != 0; }
#endif
    struct LoadedVertex *v1 = &rsp.loaded_vertices[vtx1_idx];
    struct LoadedVertex *v2 = &rsp.loaded_vertices[vtx2_idx];
    struct LoadedVertex *v3 = &rsp.loaded_vertices[vtx3_idx];
    struct LoadedVertex *v_arr[3] = {v1, v2, v3};
#ifndef __3DS__
    {
        static int trilog_target = -2;
        if (trilog_target == -2) {
            const char* e = getenv("PORT_TRILOG");
            trilog_target = e ? atoi(e) : -1;
        }
        if (trilog_target >= 0 && (int)gfx_port_frame_index == trilog_target
            && gfx_port_tri_count <= 8) {
            fprintf(stderr, "[tri %u] clip=%x|%x|%x cc=%08x gm=%08x\n", gfx_port_tri_count, v1->clip_rej, v2->clip_rej,
                    v3->clip_rej, (uint32_t)rdp.combine_mode, rsp.geometry_mode);
        }
    }
#endif
    
    //if (rand()%2) return;
    
    if (v1->clip_rej & v2->clip_rej & v3->clip_rej) {
        // The whole triangle lies outside the visible area
        return;
    }
    
    /* PORT PERF: cache the debug env lookup — this is per-triangle; an uncached getenv()
     * here scans the environment for every tri (thousands/frame) and crushes emulated fps. */
#ifdef __3DS__
    /* PORT PERF (2026-10-03): getenv is always NULL on the 3DS; the per-triangle checks of these debug switches (and of
     * the GPU path's draw state below while no command changed it) showed in the Old 3DS profile (tools/pcprof.py) */
    const int nocull = 0;
    if (gPortGpuVtx) { /* the GPU culls (and clips) */
        if (!sTriStateOk && !gpu_tri_state()) return; /* sTriStateOk: no state command since an accepted triangle */
    } else
#else
    static int nocull = -2;
    if (nocull == -2) nocull = getenv("PORT_NOCULL") ? 1 : 0;
#endif
    if (nocull) { /* skip */ } else
    if ((rsp.geometry_mode & G_CULL_BOTH) != 0) {
        /* PORT PERF (2026-09-30): orientation without the three divisions (~19 cycles each on the ARM11 VFP).
         * det(x, y, w) = w1 w2 w3 * O(p1, p2, p3), with O the NDC orientation. The former value was
         * (p1 - p2) x (p3 - p2) = -O, negated once per vertex behind the eye (= times sign(w1 w2 w3)):
         * its sign is exactly that of -det. Only the sign is used below. */
        float cross = -(v1->x * (v2->y * v3->w - v3->y * v2->w) - v2->x * (v1->y * v3->w - v3->y * v1->w) +
                        v3->x * (v1->y * v2->w - v2->y * v1->w));
        
        switch (rsp.geometry_mode & G_CULL_BOTH) {
            case G_CULL_FRONT:
                if (cross <= 0) return;
                break;
            case G_CULL_BACK:
                if (cross >= 0) return;
                break;
            case G_CULL_BOTH:
                // Why is this even an option?
                return;
        }
    }
    
    if (sTriStateOk) {
        PROF_SET(PROF_TRI_BUILD);
        goto emit_vertices; /* render state unchanged since the previous triangle */
    }
#ifdef __3DS__
    { extern u32 gPortPerfSlowTris; gPortPerfSlowTris++; }
#endif

    bool depth_test = (rsp.geometry_mode & G_ZBUFFER) == G_ZBUFFER;
    if (depth_test != rendering_state.depth_test) {
        gfx_flush();
        gfx_rapi->set_depth_test(depth_test);
        rendering_state.depth_test = depth_test;
    }
    
    bool z_upd = (rdp.other_mode_l & Z_UPD) == Z_UPD;
    if (z_upd != rendering_state.depth_mask) {
        gfx_flush();
        gfx_rapi->set_depth_mask(z_upd);
        rendering_state.depth_mask = z_upd;
    }
    
    bool zmode_decal = (rdp.other_mode_l & ZMODE_DEC) == ZMODE_DEC;
    if (zmode_decal != rendering_state.decal_mode) {
        gfx_flush();
        gfx_rapi->set_zmode_decal(zmode_decal);
        rendering_state.decal_mode = zmode_decal;
    }
    
    if (rdp.viewport_or_scissor_changed) {
        if (memcmp(&rdp.viewport, &rendering_state.viewport, sizeof(rdp.viewport)) != 0) {
            gfx_flush();
            gfx_rapi->set_viewport(rdp.viewport.x, rdp.viewport.y, rdp.viewport.width, rdp.viewport.height);
            rendering_state.viewport = rdp.viewport;
        }
        if (memcmp(&rdp.scissor, &rendering_state.scissor, sizeof(rdp.scissor)) != 0) {
            gfx_flush();
            gfx_rapi->set_scissor(rdp.scissor.x, rdp.scissor.y, rdp.scissor.width, rdp.scissor.height);
            rendering_state.scissor = rdp.scissor;
        }
        rdp.viewport_or_scissor_changed = false;
    }
    
    bool two_cycle = (rdp.other_mode_h & (3U << G_MDSFT_CYCLETYPE)) == G_CYC_2CYCLE;

    /* PORT (2026-09-24): alpha semantics follow libultraship's Interpreter::GfxSpTri1 (MIT):
     *  - blend: a blender cycle mixes with memory via (1 - A)  [was: "A_MEM not selected"];
     *  - use_alpha (alpha combiner active) = blend || texture_edge;
     *  - cutout: texture_edge, or G_AC_THRESHOLD on a blended draw. The backend picks the alpha
     *    test from these + the blend state (edge: keep a>0.19, drawn opaque; threshold: a>=8/256).
     * Opaque draws keep blending AND alpha test off, so their alpha is irrelevant (the key makes it 1). */
    bool blend = (((rdp.other_mode_l & (3U << 20)) == (G_BL_CLR_MEM << 20)) && ((rdp.other_mode_l & (3U << 16)) == (G_BL_1MA << 16))) ||
                 (((rdp.other_mode_l & (3U << 22)) == (G_BL_CLR_MEM << 22)) && ((rdp.other_mode_l & (3U << 18)) == (G_BL_1MA << 18)));
    bool use_fog = (rdp.other_mode_l >> 30) == G_BL_CLR_FOG;
    bool texture_edge = (rdp.other_mode_l & CVG_X_ALPHA) == CVG_X_ALPHA;
    bool alpha_threshold = (rdp.other_mode_l & (3U << G_MDSFT_ALPHACOMPARE)) == G_AC_THRESHOLD;
    bool use_alpha = blend || texture_edge;
    bool cutout = texture_edge || (alpha_threshold && blend);

    /* the key only changes with the combine words and the two flags: decode once per change */
    static uint64_t sKeyRaw = ~0ull, sKeyVal;
    static uint32_t sKeyFlags = ~0u, sKeyUsage;
    uint32_t keyFlags = (two_cycle ? 1 : 0) | (use_alpha ? 2 : 0);
    if (rdp.combine_mode != sKeyRaw || keyFlags != sKeyFlags) {
        sKeyRaw = rdp.combine_mode;
        sKeyFlags = keyFlags;
        sKeyVal = gfx_cc_key(two_cycle, use_alpha);
        sKeyUsage = gfx_cc_usage(sKeyVal);
    }
    uint64_t cc_id0 = sKeyVal;
    uint32_t usage = sKeyUsage;
    uint32_t cc_id1 = (usage & (SHADER_OPT_TEX0 | SHADER_OPT_TEX1)) | (use_alpha ? SHADER_OPT_ALPHA : 0) |
                      (use_fog ? SHADER_OPT_FOG : 0) | (cutout ? SHADER_OPT_TEXTURE_EDGE : 0);
    if (rendering_state.shader_program == NULL || cc_id0 != rendering_state.cc_id0 || cc_id1 != rendering_state.cc_id1) {
        gfx_flush();
        struct ShaderProgram *prg = gfx_rapi->lookup_shader(cc_id0, cc_id1);
        gfx_rapi->unload_shader(rendering_state.shader_program);
        if (prg == NULL) {
            prg = gfx_rapi->create_and_load_new_shader(cc_id0, cc_id1);
        } else {
            gfx_rapi->load_shader(prg);
        }
        rendering_state.shader_program = prg;
        rendering_state.cc_id0 = cc_id0;
        rendering_state.cc_id1 = cc_id1;
    }
    struct ShaderProgram *prg = rendering_state.shader_program;

    /* constant inputs (only the ones the key reads, so unrelated color changes don't split batches) */
    struct GfxCombineConsts consts;
    memset(&consts, 0, sizeof(consts));
    if (usage & ((1u << (8 + CCS_PRIM)) | (1u << (8 + CCS_PRIMA)))) {
        memcpy(consts.prim, &rdp.prim_color, 4);
    }
    if (usage & ((1u << (8 + CCS_ENV)) | (1u << (8 + CCS_ENVA)))) {
        memcpy(consts.env, &rdp.env_color, 4);
    }
    if (use_fog) {
        memcpy(consts.fog, &rdp.fog_color, 4);
    }
    if (usage & (1u << (8 + CCS_PRIMLODF))) {
        consts.prim_lod_frac = rdp.prim_lod_frac;
    }
    if (usage & (1u << (8 + CCS_LODF))) { /* fast3d approximation: by distance */
        float w1 = v1->w;
#ifdef __3DS__
        if (gPortGpuVtx && vtx1_idx < MAX_VERTICES && sGpuSlot != NULL && sLoadSlot[vtx1_idx] < sGpuSlotN) {
            /* GPU path: the vertex is in model space; its w is the slot's w row (one dot product) */
            const float* r = sGpuSlot[sLoadSlot[vtx1_idx]].rows[2][3];
            w1 = r[0] * v1->x + r[1] * v1->y + r[2] * v1->z + r[3];
        }
#endif
        float distance_frac = (w1 - 3000.0f) / 3000.0f;
        if (distance_frac < 0.0f) distance_frac = 0.0f;
        if (distance_frac > 1.0f) distance_frac = 1.0f;
        consts.lod_frac = distance_frac * 255.0f;
    }
    if (memcmp(&consts, &rendering_state.consts, sizeof(consts)) != 0) {
        gfx_flush();
        gfx_rapi->set_combine_consts(&consts);
        rendering_state.consts = consts;
    }

    if (blend != rendering_state.alpha_blend) {
        gfx_flush();
        gfx_rapi->set_use_alpha(blend); /* blending on/off; alpha combiner is a shader option */
        rendering_state.alpha_blend = blend;
    }
    uint8_t num_inputs;
    bool used_textures[2];
    gfx_rapi->shader_get_info(prg, &num_inputs, used_textures);

    /* texture unit i samples tile first_tile + i (no LOD: unit 1 stays on the base tile when the
     * first tile is >= 2, as in libultraship) */
    const struct TileDesc* utile[2];
    float tex_width[2] = { 1.0f, 1.0f }, tex_height[2] = { 1.0f, 1.0f };
    bool linear_filter = (rdp.other_mode_h & (3U << G_MDSFT_TEXTFILT)) != G_TF_POINT;
    for (int i = 0; i < 2; i++) {
        int ti = (i == 1 && rdp.first_tile >= 2) ? rdp.first_tile : (rdp.first_tile + i) & 7;
        utile[i] = &rdp.tiles[ti];
        if (used_textures[i]) {
            if (rdp.textures_changed[i]) {
                gfx_flush();
                import_texture(i, ti);
                rdp.textures_changed[i] = false;
            }
            const struct TileDesc* t = utile[i];
            struct TextureHashmapNode* node = rendering_state.textures[i];
            if (linear_filter != node->linear_filter || t->cms != node->cms || t->cmt != node->cmt) {
                gfx_flush();
                gfx_rapi->set_sampler_parameters(i, linear_filter, t->cms, t->cmt);
                node->linear_filter = linear_filter;
                node->cms = t->cms;
                node->cmt = t->cmt;
            }
            if (node->width != 0 && node->height != 0) {
                tex_width[i] = node->width;
                tex_height[i] = node->height;
            } else {
                tex_width[i] = (t->lrs - t->uls + 4) / 4;
                tex_height[i] = (t->lrt - t->ult + 4) / 4;
            }
        }
    }
    
    bool z_is_from_0_to_1 = gfx_rapi->z_is_from_0_to_1();
#ifdef __3DS__
    port_draw_snapshot(); /* first triangle of a batch: record the state it's drawn with */
#endif
    
    sTC.inv_tex_w[0] = 1.0f / tex_width[0];
    sTC.inv_tex_w[1] = 1.0f / tex_width[1];
    sTC.inv_tex_h[0] = 1.0f / tex_height[0];
    sTC.inv_tex_h[1] = 1.0f / tex_height[1];
    sTC.used_textures[0] = used_textures[0];
    sTC.used_textures[1] = used_textures[1];
    sTC.utile[0] = utile[0];
    sTC.utile[1] = utile[1];
    for (int t = 0; t < 2; t++) { /* libultraship's per-tile coordinate transform, as one multiply-add */
        const struct TileDesc* tl = utile[t];
        float su = 1.0f / 32.0f, sv = 1.0f / 32.0f;
        if (tl->shifts != 0) {
            su = tl->shifts <= 10 ? su / (1 << tl->shifts) : su * (1 << (16 - tl->shifts));
        }
        if (tl->shiftt != 0) {
            sv = tl->shiftt <= 10 ? sv / (1 << tl->shiftt) : sv * (1 << (16 - tl->shiftt));
        }
        sTC.u_scale[t] = su;
        sTC.v_scale[t] = sv;
        sTC.u_off[t] = tl->uls / 4.0f;
        sTC.v_off[t] = tl->ult / 4.0f;
    }
    sTC.linear_filter = linear_filter;
    sTC.z_is_from_0_to_1 = z_is_from_0_to_1;
    sPVState++; /* packed vertices depend on sTC */
#ifdef __3DS__
    sBatchId++;
#endif
    /* LOD fraction is derived from each triangle's w: keep re-evaluating state for those draws */
    sTriStateOk = !(usage & (1u << (8 + CCS_LODF)));

emit_vertices:;
    const uint8_t vidx[3] = { vtx1_idx, vtx2_idx, vtx3_idx };
#ifdef PORT_ICONGEN
    if (mesh_capturing()) {
        sMeshDbgTri++;
        if (sMeshN < MESH_MAX && vtx1_idx < MAX_VERTICES && vtx2_idx < MAX_VERTICES &&
            vtx3_idx < MAX_VERTICES) {
            MeshTri* m = &sMeshTris[sMeshN++];
            const struct TileDesc* t0 = &rdp.tiles[rdp.first_tile & 7];
            bool blend = (((rdp.other_mode_l & (3U << 20)) == (G_BL_CLR_MEM << 20)) && ((rdp.other_mode_l & (3U << 16)) == (G_BL_1MA << 16))) ||
                         (((rdp.other_mode_l & (3U << 22)) == (G_BL_CLR_MEM << 22)) && ((rdp.other_mode_l & (3U << 18)) == (G_BL_1MA << 18)));
            m->combine = rdp.combine_mode;
            memcpy(&m->prim, &rdp.prim_color, 4);
            memcpy(&m->env, &rdp.env_color, 4);
            m->tex = (sTC.used_textures[0] && rendering_state.textures[0] != NULL) ? rendering_state.textures[0]->texture_id : 0xFFFFFFFFu;
            if (m->tex < 4096 && sMeshTex[m->tex].rgba == NULL) { /* decoded again now: the tile's state is this draw's */
                int ti = rdp.first_tile & 7;
                sMeshRedecode = 1;
                import_texture_impl(0, ti);
                sMeshRedecode = 0;
            }
            m->flags = (sTC.used_textures[0] ? 1 : 0) |
                       ((rdp.other_mode_h & (3U << G_MDSFT_CYCLETYPE)) == G_CYC_2CYCLE ? 2 : 0) |
                       ((rdp.other_mode_l & CVG_X_ALPHA) ? 4 : 0) | (blend ? 8 : 0) |
                       ((uint32_t)(t0->cms & 3) << 8) | ((uint32_t)(t0->cmt & 3) << 12);
            for (int k = 0; k < 3; k++) {
                const struct LoadedVertex* lv = &rsp.loaded_vertices[vidx[k]];
                float uv[2][2];
                gpu_uv(lv, uv);
                m->v[k][0] = sMeshPos[vidx[k]][0], m->v[k][1] = sMeshPos[vidx[k]][1], m->v[k][2] = sMeshPos[vidx[k]][2];
                m->v[k][3] = uv[0][0], m->v[k][4] = uv[0][1];
                m->c[k] = (uint32_t)lv->color.r | ((uint32_t)lv->color.g << 8) | ((uint32_t)lv->color.b << 16) |
                          ((uint32_t)lv->color.a << 24);
            }
            sMeshFrameHasTris = 1;
        }
    }
#endif
#ifdef __3DS__
    if (gPortGpuVtx) { /* GPU path: indices (+ any vertex new to this batch), nothing else per triangle */
        PROF_SET(PROF_EMIT);
        gpu_emit_tri(vidx);
        return;
    }
#endif
    PVtx pv[3];
    for (int i = 0; i < 3; i++) {
        PVtx* p = &pv[i];
        /* PORT PERF (2026-09-30): a loaded vertex's packed form depends only on the vertex and the render
         * state (sTC); triangles sharing it (strips, TRI2, quads) reuse it. sPVState bumps whenever sTC is
         * rebuilt and every frame; G_VTX clears the stamp of each slot it writes. */
        if (sPVStamp[vidx[i]] == sPVState) {
            *p = sPVCache[vidx[i]];
            continue;
        }
        p->x = v_arr[i]->x;
        p->y = v_arr[i]->y;
        p->z = v_arr[i]->z;
        p->w = v_arr[i]->w;
        for (int t = 0; t < 2; t++) {
            float u = 0.0f, v = 0.0f;
            if (sTC.used_textures[t]) { /* libultraship's per-tile texture coordinates */
                const struct TileDesc* tl = sTC.utile[t];
                /* (u / 32) shifted, minus the tile origin: scale/offset precomputed per state (sTC) */
                u = v_arr[i]->u * sTC.u_scale[t] - sTC.u_off[t];
                v = v_arr[i]->v * sTC.v_scale[t] - sTC.v_off[t];
                (void)tl;
                if (sTC.linear_filter) {
                    // Linear filter adds 0.5f to the coordinates (texture rectangles too: this
                    // backend's rectangle UVs need it; without it HUD digits blurred - fbdiff)
                    u += 0.5f;
                    v += 0.5f;
                }
                u *= sTC.inv_tex_w[t];
                v *= sTC.inv_tex_h[t];
            }
            p->uv[t][0] = u;
            p->uv[t][1] = v;
        }
        /* shade (with G_FOG the RSP's fog factor replaces shade alpha, as on the N64) */
        p->c[0] = v_arr[i]->color.r * (1.0f / 255.0f);
        p->c[1] = v_arr[i]->color.g * (1.0f / 255.0f);
        p->c[2] = v_arr[i]->color.b * (1.0f / 255.0f);
        p->c[3] = v_arr[i]->color.a * (1.0f / 255.0f);
        p->s = stereo_offset(p->w);
        sPVCache[vidx[i]] = *p;
        sPVStamp[vidx[i]] = sPVState;
    }
    {
        uint64_t te = PERF_T(), fe = gPortPerfFlush;
#ifdef __3DS__
        sCurVidx = vidx; /* unsplit, the triangle's vertices are the loaded ones (reusable) */
        if (gPortReplayRec) { /* replay recipes of split/clipped pieces refer to these three */
            int i;
            sRecTriOk = 1;
            for (i = 0; i < 3; i++) {
                sRecTriSrc[i] = sLoadSrc[vidx[i]];
                sRecTriOk &= sRecTriSrc[i] != REC_NONE;
                sRecTriPos[i][0] = pv[i].x, sRecTriPos[i][1] = pv[i].y, sRecTriPos[i][2] = pv[i].w;
            }
        }
#endif
        PROF_SET(PROF_EMIT);
        gfx_emit_tri(&pv[0], &pv[1], &pv[2], sTC.z_is_from_0_to_1);
#ifdef __3DS__
        sCurVidx = NULL;
        sRecTriOk = 0;
#endif
        gPortPerfEmit += (PERF_T() - te) - (gPortPerfFlush - fe);
    }
}

static void gfx_sp_geometry_mode(uint32_t clear, uint32_t set) {
    rsp.geometry_mode &= ~clear;
    rsp.geometry_mode |= set;
}

static void gfx_calc_and_set_viewport(const Vp_t *viewport) {
    // 2 bits fraction
    float width = 2.0f * viewport->vscale[0] / 4.0f;
    float height = 2.0f * viewport->vscale[1] / 4.0f;
    float x = (viewport->vtrans[0] / 4.0f) - width / 2.0f;
    float y = SCREEN_HEIGHT - ((viewport->vtrans[1] / 4.0f) + height / 2.0f);
    rsp.half_px_x = width > 0.0f ? 1.0f / width : 0.0f;
    sGpuSlotCur = -1;
    rsp.half_px_y = height > 0.0f ? 1.0f / height : 0.0f;
    rdp.vp_raw[0] = x;
    rdp.vp_raw[1] = y;
    rdp.vp_raw[2] = width;
    rdp.vp_raw[3] = height;
    gfx_apply_viewport();
}

/* rdp.viewport from the raw N64 viewport, for the current target's dimensions */
static void gfx_apply_viewport(void) {
    float x = rdp.vp_raw[0], y = rdp.vp_raw[1], width = rdp.vp_raw[2], height = rdp.vp_raw[3];
    
    /* PORT (2026-09-25): N64 screen x maps to x * RATIO_Y + pillar (4:3 content centered in the wider
     * target), and gfx_sp_vertex squeezes clip x by the aspect ratio around the viewport's center, so
     * the viewport keeps a widened width but must be CENTERED on the mapped N64 center. Scaling x by
     * RATIO_X (sm64) is only right for viewports centered on screen: the A button's 45-pixel viewport
     * landed ~12 px off (found by tools/statediff fbdiff). */
    float center = (x + width / 2.0f) * RATIO_Y + GFX_PILLAR_X;
    width *= RATIO_X;
    height *= RATIO_Y;
    x = center - width / 2.0f;
    y *= RATIO_Y;
    
    rdp.viewport.x = x;
    rdp.viewport.y = y;
    rdp.viewport.width = width;
    rdp.viewport.height = height;

    rdp.viewport_or_scissor_changed = true;
}

static void gfx_sp_movemem(uint8_t index, uint8_t offset, const void* data) {
    switch (index) {
        case G_MV_VIEWPORT:
            gfx_calc_and_set_viewport((const Vp_t *) data);
            break;
#if 0
        case G_MV_LOOKATY:
        case G_MV_LOOKATX:
            memcpy(rsp.current_lookat + (index - G_MV_LOOKATY) / 2, data, sizeof(Light_t));
            //rsp.lights_changed = 1;
            break;
#endif
#ifdef F3DEX_GBI_2
        case G_MV_LIGHT: {
            int lightidx = offset / 24 - 2;
            if (lightidx >= 0 && lightidx <= MAX_LIGHTS) {
                // NOTE: reads out of bounds if it is an ambient light
                memcpy(rsp.current_lights + lightidx, data, sizeof(Light_t));
            } else if (lightidx < 0) { /* G_MVO_LOOKATX / G_MVO_LOOKATY */
                /* guLookAt/guLookAtHilite always write col = colc = (0,0,0) for X and (0,0x80,0) for Y.
                 * Anything else is not a LookAt: the N64 boot logo's gsSPLookAt(0x01002E50) points one
                 * byte past the end of nintendo_rogo_static (0x2E50 bytes), so it reads whatever follows
                 * in RAM (a bug of the original that happens to look fine on the N64). Keep the LookAt
                 * the game built just before (ConsoleLogo_Draw's func_8002EABC) instead of garbage. */
                const uint8_t* b = (const uint8_t*)data;
                int y = offset / 24; /* 0 = X, 1 = Y */
                uint8_t want = y ? 0x80 : 0x00;
                /* native (built by game code: guLookAtHilite): col/colc at bytes 0-2 / 4-6, dir at 8-10,
                 * 12-15 zero. Stored in a ROM segment the same bytes sit at k ^ 7 (all DMA'd data): col at
                 * 7,6,5 / colc at 3,2,1, dir at 15,14,13, logical 12-15 at physical 11-8 (zero). */
                if (b[0] == 0 && b[1] == want && b[2] == 0 && b[4] == 0 && b[5] == want && b[6] == 0 &&
                    (b[12] | b[13] | b[14] | b[15]) == 0) {
                    memcpy(rsp.current_lookat + y, data, sizeof(Light_t));
                } else if (b[7] == 0 && b[6] == want && b[5] == 0 && b[3] == 0 && b[2] == want && b[1] == 0 &&
                           (b[8] | b[9] | b[10] | b[11]) == 0) {
                    uint8_t tmp[16];
                    for (int k = 0; k < 16; k++) tmp[k] = b[k ^ 7];
                    memcpy(rsp.current_lookat + y, tmp, sizeof(Light_t));
                }
            }
            rsp.lights_changed = true;
            break;
        }
#else
        case G_MV_L0:
        case G_MV_L1:
        case G_MV_L2:
            // NOTE: reads out of bounds if it is an ambient light
            memcpy(rsp.current_lights + (index - G_MV_L0) / 2, data, sizeof(Light_t));
            break;
#endif
    }
}

static void gfx_sp_moveword(uint8_t index, uint16_t offset, uint32_t data) {
    switch (index) {
        case G_MW_SEGMENT:
            gfx_port_segments[offset / 4] = data;
            break;
        case G_MW_NUMLIGHT:
#ifdef F3DEX_GBI_2
            rsp.current_num_lights = data / 24 + 1; // add ambient light
            if (rsp.current_num_lights > MAX_LIGHTS + 1) {
                rsp.current_num_lights = MAX_LIGHTS + 1;
            }
#else
            // Ambient light is included
            // The 31th bit is a flag that lights should be recalculated
            rsp.current_num_lights = (data - 0x80000000U) / 32;
#endif
            rsp.lights_changed = 1;
            break;
        case G_MW_FOG:
            rsp.fog_mul = (int16_t)(data >> 16);
            rsp.fog_offset = (int16_t)data;
            break;
        case G_MW_CLIP: /* 4 words: FR_NEG_FRUSTRATIO_n = n, FR_POS_FRUSTRATIO_n = 0x10000 - n */
            rsp.clip_ratio = (uint8_t)((data & 0xFFFF) > 0x8000 ? 0x10000 - (data & 0xFFFF) : data);
            break;
    }
}

static void gfx_sp_texture(uint16_t sc, uint16_t tc, uint8_t level, uint8_t tile, uint8_t on) {
    rsp.texture_scaling_factor.s = sc;
    rsp.texture_scaling_factor.t = tc;
    if (rdp.first_tile != (tile & 7)) {
        rdp.textures_changed[0] = true;
        rdp.textures_changed[1] = true;
    }
    rdp.first_tile = tile & 7;
}

static void gfx_dp_set_scissor(uint32_t mode, uint32_t ulx, uint32_t uly, uint32_t lrx, uint32_t lry) {
    rdp.scissor_raw[0] = ulx;
    rdp.scissor_raw[1] = uly;
    rdp.scissor_raw[2] = lrx;
    rdp.scissor_raw[3] = lry;
    gfx_apply_scissor();
}

/* PORT (2026-09-29): 0 (default) = the N64's 4:3 image centered with black side bars, like the
 * original. 1 = "hor+" widescreen: full-width scissors are widened to the whole 400-pixel target so
 * 3D also draws into the sides; OoT was not designed for it (pre-rendered backgrounds, fades and
 * cutscene fills stay 4:3, geometry outside the N64 view shows), so it is opt-in. */
int gPortWidescreen = 0;
/* pre-rendered rooms draw their background image with S2DEX BG_COPY/BG_1CYC: frames that have one (or
 * whose previous frame had one, which covers draws issued before the background) stay 4:3 */
static int sBgThisFrame, sBgLastFrame;
int gPortPrerenderedFrame; /* stereo (gfx_3ds.c): the previous frame had a pre-rendered background */
static int sStereoRoomThisFrame; /* a pre-rendered skybox room (shops, houses: G_NOOP stereo mode 2) */
/* PORT (2026-10-03): ...and for about a second after the last such frame. When the camera switches angle in a
 * pre-rendered room, the new picture can take a frame or two to arrive; those frames went wide and the room's 3D
 * geometry flashed in the side bars (hardware v41: "the drawing of triangles when moving the camera"). */
static int sPrerenderedRecent; /* logic frames to stay 4:3 */
#define WIDE_ACTIVE() (gPortWidescreen && !sBgThisFrame && !sBgLastFrame && !sStereoRoomThisFrame && \
                       sPrerenderedRecent == 0 && sDrawTarget == NULL)
static int sRectFullWidth; /* gfx_dp_fill_rectangle -> gfx_draw_rectangle: stretch to the whole target */

static void gfx_apply_scissor(void) {
    uint32_t ulx = rdp.scissor_raw[0], uly = rdp.scissor_raw[1], lrx = rdp.scissor_raw[2], lry = rdp.scissor_raw[3];
    /* PORT (2026-09-25): pixels map like the content (x * RATIO_Y + pillar), except that in widescreen
     * mode a full-width scissor keeps the whole target so the widened 3D view fills the sides */
    float x = ulx / 4.0f * RATIO_Y + GFX_PILLAR_X;
    float y = (SCREEN_HEIGHT - lry / 4.0f) * RATIO_Y;
    float width = (lrx - ulx) / 4.0f * RATIO_Y;
    float height = (lry - uly) / 4.0f * RATIO_Y;
    if (WIDE_ACTIVE() && ulx == 0 && lrx >= SCREEN_WIDTH * 4 - 4) {
        x = 0.0f;
        width = gfx_current_dimensions.width;
    }
    
    rdp.scissor.x = x;
    rdp.scissor.y = y;
    rdp.scissor.width = width;
    rdp.scissor.height = height;
    
    rdp.viewport_or_scissor_changed = true;
}

#ifdef __3DS__
/* PORT (2026-10-03): widescreen pause background (gfx_3ds.c Port3ds_WideCapFor): the game copies its 320-pixel
 * capture to the screen in full-row strips in COPY mode (PreRender_CopyImage); those strips load from the 400-pixel
 * capture instead and their rectangles span the whole target */
static const void* sWideCapImg;     /* the wide capture standing for the current texture image, if any */
static uint8_t sWideLoaded[2];      /* per TMEM slot: the last load came from the wide capture */
/* WIDE_ACTIVE for the color image the game has set: the draw target follows it only at the next draw
 * (gfx_select_target), and during the pause the frame's earlier draw is Link's off-screen preview, so the first
 * strip was judged off-screen, stayed 320 wide and left black corners at the top (hardware v42) */
#define WIDE_ACTIVE_CIMG() (gPortWidescreen && !sBgThisFrame && !sBgLastFrame && !sStereoRoomThisFrame && \
                            sPrerenderedRecent == 0 && gfx_cimg_is_screen())
int gPortFrameWide;                 /* gfx_3ds.c: this frame's 3D filled the whole width (set at the frame's end) */
#endif
static void gfx_dp_set_texture_image(uint32_t format, uint32_t size, uint32_t width, const void* addr) {
    rdp.texture_to_load.addr = addr;
    rdp.texture_to_load.siz = size;
    rdp.texture_to_load.width = width + 1;
#ifdef __3DS__
    {
        extern const void* Port3ds_WideCapFor(const void* gameBuf);
        sWideCapImg = (width + 1 == 320 && size == G_IM_SIZ_16b) ? Port3ds_WideCapFor(addr) : NULL;
    }
#endif
}

static void gfx_dp_set_tile(uint8_t fmt, uint32_t siz, uint32_t line, uint32_t tmem, uint8_t tile, uint32_t palette, uint32_t cmt, uint32_t maskt, uint32_t shiftt, uint32_t cms, uint32_t masks, uint32_t shifts) {
    struct TileDesc* t = &rdp.tiles[tile & 7];
    /* RDP: without a mask a coordinate neither wraps nor mirrors - it clamps (libultraship did this for
     * WRAP only; MIRROR with mask 0 clamps too) */
    if (!(cms & G_TX_CLAMP) && masks == G_TX_NOMASK) {
        cms = G_TX_CLAMP;
    }
    if (!(cmt & G_TX_CLAMP) && maskt == G_TX_NOMASK) {
        cmt = G_TX_CLAMP;
    }
    t->fmt = fmt;
    t->siz = siz;
    t->palette = palette;
    t->cms = cms;
    t->cmt = cmt;
    t->shifts = shifts;
    t->shiftt = shiftt;
    t->line_size_bytes = line * 8;
    t->tmem = tmem;
    rdp.textures_changed[0] = true;
    rdp.textures_changed[1] = true;
}

static void gfx_dp_set_tile_size(uint8_t tile, uint16_t uls, uint16_t ult, uint16_t lrs, uint16_t lrt) {
    struct TileDesc* t = &rdp.tiles[tile & 7];
    t->uls = uls;
    t->ult = ult;
    t->lrs = lrs;
    t->lrt = lrt;
    rdp.textures_changed[0] = true;
    rdp.textures_changed[1] = true;
}

static void gfx_dp_load_tlut(uint8_t tile, uint32_t high_index) {
    const uint8_t* src = rdp.texture_to_load.addr;
    uint32_t dest = (rdp.tiles[tile & 7].tmem >= 256) ? rdp.tiles[tile & 7].tmem - 256 : 0; /* TLUT entry index */
    uint32_t count = high_index + 1; /* gDPLoadTLUTCmd carries count - 1 (15 or 255) */
    uint32_t i;
    SUPPORT_CHECK(rdp.texture_to_load.siz == G_IM_SIZ_16b);
    rdp.palette = src;
    if (src == NULL) {
        return;
    }
    uintptr_t sw = gfx_src_swizzle(src);
    if (dest + count <= 256 && (count & 3) == 0 && ((sw == 7u && ((uintptr_t)src & 7) == 0) || (sw == 3u && ((uintptr_t)src & 3) == 0))) {
        /* PORT PERF (2026-10-03): whole words. Entries are big-endian u16s in byte-reversed groups (8 bytes for game
         * memory, 4 for u32 assets), so each entry is one half of a native u32: no per-byte work (this loop was ~18% of
         * the display-list walk's own time at Old 3DS speed, tools/pcprof.py) */
        const uint32_t* w = (const uint32_t*)src;
        uint16_t* t = &rdp.tlut[dest];
        if (sw == 7u) {
            for (i = 0; i < count; i += 4, w += 2, t += 4) {
                uint32_t lo = w[0], hi = w[1];
                t[0] = (uint16_t)(hi >> 16), t[1] = (uint16_t)hi, t[2] = (uint16_t)(lo >> 16), t[3] = (uint16_t)lo;
            }
        } else {
            for (i = 0; i < count; i += 2, w++, t += 2) {
                uint32_t v = w[0];
                t[0] = (uint16_t)(v >> 16), t[1] = (uint16_t)v;
            }
        }
    } else {
        for (i = 0; i < count && dest + i < 256; i++) {
            rdp.tlut[dest + i] = (TEX_SRC_BYTE_X(src + i * 2, sw) << 8) | TEX_SRC_BYTE_X(src + i * 2 + 1, sw);
        }
    }
    sTlutHashValid = 0; /* palette hashes are recomputed on the next CI lookup */
    rdp.textures_changed[0] = true;
    rdp.textures_changed[1] = true;
}

/* CI textures depend on the palette colors they index: part of the texture cache key */
/* PORT PERF (2026-09-30): hashes of the 16 CI4 banks and the whole CI8 palette, computed once per TLUT
 * change instead of on every CI texture lookup (256 multiply-xors each, ~1-2 us on the 3DS CPU). */
/* PORT PERF (2026-10-05): lazily, only the hash a lookup needs (one CI4 bank, or the CI8 palette): every TLUT load
 * cleared them and the next CI lookup re-hashed all 16 banks plus all 256 colours (import_texture was ~6% of the
 * Kokiri Forest walk, tools/pcprof.py). sTlutHashValid: bit k = bank k valid, bit 16 = the 256-colour hash. */
static uint32_t gfx_ci_palette_hash(uint32_t fmt, uint32_t siz) {
    uint32_t i;
    if (fmt != G_IM_FMT_CI) {
        return 0;
    }
    if (siz == G_IM_SIZ_4b) {
        uint32_t k = sImpTile->palette & 15;
        if (!(sTlutHashValid & (1u << k))) {
            uint32_t h = 2166136261u;
            for (i = 0; i < 16; i++) {
                h = (h ^ rdp.tlut[k * 16 + i]) * 16777619u;
            }
            sTlutHash16[k] = h | 1; /* never 0, which means "not CI" */
            sTlutHashValid |= 1u << k;
        }
        return sTlutHash16[k];
    }
    if (!(sTlutHashValid & (1u << 16))) {
        uint32_t h256 = 2166136261u;
        for (i = 0; i < 256; i++) {
            h256 = (h256 ^ rdp.tlut[i]) * 16777619u;
        }
        sTlutHash256 = h256 | 1;
        sTlutHashValid |= 1u << 16;
    }
    return sTlutHash256;
}

static void gfx_dp_load_block(uint8_t tile, uint32_t uls, uint32_t ult, uint32_t lrs, uint32_t dxt) {
    uint32_t slot = rdp.tiles[tile & 7].tmem != 0; /* TMEM slot (see struct TileDesc) */
    SUPPORT_CHECK(uls == 0);
    SUPPORT_CHECK(ult == 0);
    
    // The lrs field rather seems to be number of pixels to load
    uint32_t word_size_shift;
    switch (rdp.texture_to_load.siz) {
        case G_IM_SIZ_4b:
            word_size_shift = 0; // Or -1? It's unused in SM64 anyway.
            break;
        case G_IM_SIZ_8b:
            word_size_shift = 0;
            break;
        case G_IM_SIZ_16b:
            word_size_shift = 1;
            break;
        case G_IM_SIZ_32b:
            word_size_shift = 2;
            break;
    }
    uint32_t size_bytes = (lrs + 1) << word_size_shift;
    rdp.loaded_texture[slot].size_bytes = size_bytes;
    if (size_bytes > 4096) { static int _w2=0; if(!_w2){_w2=1; fprintf(stderr, "[gfx] texture >4096B clamped\n");} size_bytes = 4096; }
    rdp.loaded_texture[slot].addr = rdp.texture_to_load.addr;
    rdp.loaded_texture[slot].line_size_bytes = rdp.loaded_texture[slot].size_bytes;
    rdp.loaded_texture[slot].full_image_line_size_bytes = rdp.loaded_texture[slot].size_bytes;
    
    rdp.textures_changed[0] = true;
    rdp.textures_changed[1] = true;
}

/* PORT (2026-09-24): G_LOADTILE -- load a sub-rectangle of the texture image. Logic follows
 * libultraship's Interpreter::GfxDpLoadTile (MIT, Kenix3): the loaded texture starts at the
 * sub-rect's first texel, rows are tile_width texels, and the source stride is the full image
 * width from G_SETTIMG. It was unhandled (tools/gbi_audit.py), so tile-loaded textures kept the
 * previous load's data. */
static void gfx_dp_load_tile(uint8_t tile, uint32_t uls, uint32_t ult, uint32_t lrs, uint32_t lrt) {
    uint32_t word_size_shift = 0, slot;
    uint32_t offset_x, offset_y, tile_width, tile_height, tile_line, full_line;
    switch (rdp.texture_to_load.siz) {
        case G_IM_SIZ_16b: word_size_shift = 1; break;
        case G_IM_SIZ_32b: word_size_shift = 2; break;
        default: word_size_shift = 0; break; /* 4b/8b as in libultraship */
    }
#ifdef __3DS__
    slot = rdp.tiles[tile & 7].tmem != 0;
    sWideLoaded[slot] = 0;
    if (sWideCapImg != NULL && WIDE_ACTIVE_CIMG() && uls == 0 && (lrs >> G_TEXTURE_IMAGE_FRAC) == 319 &&
        (rdp.other_mode_h & (3U << G_MDSFT_CYCLETYPE)) == G_CYC_COPY) {
        rdp.texture_to_load.addr = sWideCapImg; /* the same rows, 400 pixels wide */
        rdp.texture_to_load.width = 400;
        lrs = 399 << G_TEXTURE_IMAGE_FRAC;
        sWideLoaded[slot] = 1;
    }
#endif
    offset_x = uls >> G_TEXTURE_IMAGE_FRAC;
    offset_y = ult >> G_TEXTURE_IMAGE_FRAC;
    tile_width = ((lrs - uls) >> G_TEXTURE_IMAGE_FRAC) + 1;
    tile_height = ((lrt - ult) >> G_TEXTURE_IMAGE_FRAC) + 1;
    tile_line = tile_width << word_size_shift;
    full_line = rdp.texture_to_load.width << word_size_shift;
    slot = rdp.tiles[tile & 7].tmem != 0;
    rdp.loaded_texture[slot].addr = rdp.texture_to_load.addr + full_line * offset_y + (offset_x << word_size_shift);
    rdp.loaded_texture[slot].size_bytes = tile_line * tile_height;
    rdp.loaded_texture[slot].line_size_bytes = tile_line;
    rdp.loaded_texture[slot].full_image_line_size_bytes = full_line;
    rdp.textures_changed[0] = true;
    rdp.textures_changed[1] = true;
}

/* raw combine "(0 - 0) * 0 + D" for both cycles and channels */
#define CC_RAW_D(d_rgb, d_alpha)                                                                         \
    (((uint64_t)(GCCc0w0(G_CCMUX_0, G_CCMUX_0, G_ACMUX_0, G_ACMUX_0) | GCCc1w0(G_CCMUX_0, G_CCMUX_0)) << 32) | \
     (GCCc0w1(G_CCMUX_0, d_rgb, G_ACMUX_0, d_alpha) |                                                       \
      GCCc1w1(G_CCMUX_0, G_ACMUX_0, G_ACMUX_0, d_rgb, G_ACMUX_0, d_alpha)))

static void gfx_dp_set_combine_mode(uint64_t raw) {
    rdp.combine_mode = raw;
}

static void gfx_dp_set_env_color(uint8_t r, uint8_t g, uint8_t b, uint8_t a) {
    rdp.env_color.r = r;
    rdp.env_color.g = g;
    rdp.env_color.b = b;
    rdp.env_color.a = a;
}

static void gfx_dp_set_prim_color(uint8_t r, uint8_t g, uint8_t b, uint8_t a) {
    rdp.prim_color.r = r;
    rdp.prim_color.g = g;
    rdp.prim_color.b = b;
    rdp.prim_color.a = a;
}

static void gfx_dp_set_fog_color(uint8_t r, uint8_t g, uint8_t b, uint8_t a) {
    rdp.fog_color.r = r;
    rdp.fog_color.g = g;
    rdp.fog_color.b = b;
    rdp.fog_color.a = a;
}

static void gfx_dp_set_fill_color(uint32_t packed_color) {
    uint16_t col16 = (uint16_t)packed_color;
    uint32_t r = col16 >> 11;
    uint32_t g = (col16 >> 6) & 0x1f;
    uint32_t b = (col16 >> 1) & 0x1f;
    uint32_t a = col16 & 1;
    rdp.fill_color.r = SCALE_5_8(r);
    rdp.fill_color.g = SCALE_5_8(g);
    rdp.fill_color.b = SCALE_5_8(b);
    rdp.fill_color.a = a * 255;
}

static void gfx_draw_rectangle(int32_t ulx, int32_t uly, int32_t lrx, int32_t lry) {
    gfx_select_target();
    uint32_t saved_other_mode_h = rdp.other_mode_h;
    uint32_t cycle_type = (rdp.other_mode_h & (3U << G_MDSFT_CYCLETYPE));
    
    if (cycle_type == G_CYC_COPY) {
        rdp.other_mode_h = (rdp.other_mode_h & ~(3U << G_MDSFT_TEXTFILT)) | G_TF_POINT;
    }
    
    // U10.2 coordinates
    float ulxf = ulx;
    float ulyf = uly;
    float lrxf = lrx;
    float lryf = lry;
    
    ulxf = ulxf / (4.0f * HALF_SCREEN_WIDTH) - 1.0f;
    ulyf = -(ulyf / (4.0f * HALF_SCREEN_HEIGHT)) + 1.0f;
    lrxf = lrxf / (4.0f * HALF_SCREEN_WIDTH) - 1.0f;
    lryf = -(lryf / (4.0f * HALF_SCREEN_HEIGHT)) + 1.0f;
    
    ulxf = gfx_adjust_x_for_aspect_ratio(ulxf);
    lrxf = gfx_adjust_x_for_aspect_ratio(lrxf);
    if (sRectFullWidth) { /* widescreen: a full-width fill (fade, letterbox, flash) covers the whole screen */
        ulxf = -1.0f;
        lrxf = 1.0f;
        sRectFullWidth = 0;
    }
    
    struct LoadedVertex* ul = &rsp.loaded_vertices[MAX_VERTICES + 0];
    struct LoadedVertex* ll = &rsp.loaded_vertices[MAX_VERTICES + 1];
    struct LoadedVertex* lr = &rsp.loaded_vertices[MAX_VERTICES + 2];
    struct LoadedVertex* ur = &rsp.loaded_vertices[MAX_VERTICES + 3];
    
    ul->x = ulxf;
    ul->y = ulyf;
    ul->z = -1.0f;
    ul->w = 1.0f;
    
    ll->x = ulxf;
    ll->y = lryf;
    ll->z = -1.0f;
    ll->w = 1.0f;
    
    lr->x = lrxf;
    lr->y = lryf;
    lr->z = -1.0f;
    lr->w = 1.0f;
    
    ur->x = lrxf;
    ur->y = ulyf;
    ur->z = -1.0f;
    ur->w = 1.0f;
    
    // The coordinates for texture rectangle shall bypass the viewport setting
    struct XYWidthHeight default_viewport = {0, 0, gfx_current_dimensions.width, gfx_current_dimensions.height};
    struct XYWidthHeight viewport_saved = rdp.viewport;
    uint32_t geometry_mode_saved = rsp.geometry_mode;
    
    rdp.viewport = default_viewport;
    rdp.viewport_or_scissor_changed = true;
    rsp.geometry_mode = 0;
    
    sTriStateOk = false; /* the rectangle's temporary viewport/geometry mode must not be cached... */
    gfx_sp_tri1(MAX_VERTICES + 0, MAX_VERTICES + 1, MAX_VERTICES + 3);
    gfx_sp_tri1(MAX_VERTICES + 1, MAX_VERTICES + 2, MAX_VERTICES + 3);
    sTriStateOk = false; /* ...or reused by the triangles that follow it */

    rsp.geometry_mode = geometry_mode_saved;
    rdp.viewport = viewport_saved;
    rdp.viewport_or_scissor_changed = true;
    
    if (cycle_type == G_CYC_COPY) {
        rdp.other_mode_h = saved_other_mode_h;
    }
}

static void gfx_dp_texture_rectangle_impl(int32_t ulx, int32_t uly, int32_t lrx, int32_t lry, uint8_t tile, int16_t uls, int16_t ult, int16_t dsdx, int16_t dtdy, bool flip);
static void gfx_dp_texture_rectangle(int32_t ulx, int32_t uly, int32_t lrx, int32_t lry, uint8_t tile, int16_t uls, int16_t ult, int16_t dsdx, int16_t dtdy, bool flip) {
    PROF_SET(PROF_RECT);
    gfx_dp_texture_rectangle_impl(ulx, uly, lrx, lry, tile, uls, ult, dsdx, dtdy, flip);
    PROF_SET(PROF_DL);
}
static void gfx_dp_texture_rectangle_impl(int32_t ulx, int32_t uly, int32_t lrx, int32_t lry, uint8_t tile, int16_t uls, int16_t ult, int16_t dsdx, int16_t dtdy, bool flip) {
    uint64_t saved_combine_mode = rdp.combine_mode;
    uint8_t saved_first_tile = rdp.first_tile;
    if (rdp.first_tile != (tile & 7)) { /* the rectangle draws with its own tile (libultraship) */
        rdp.textures_changed[0] = true;
        rdp.textures_changed[1] = true;
        rdp.first_tile = tile & 7;
    }
    rdp.drawing_rect = true;
#ifdef __3DS__
    if (sWideLoaded[rdp.tiles[tile & 7].tmem != 0] && (rdp.other_mode_h & (3U << G_MDSFT_CYCLETYPE)) == G_CYC_COPY &&
        ulx == 0 && WIDE_ACTIVE_CIMG()) {
        /* the wide capture's strip: 400 texels, from the target's left edge (N64 x = -40) to its right (360) */
        struct TileDesc* t = &rdp.tiles[tile & 7];
        t->line_size_bytes = 400 * 2;
        t->lrs = t->uls + (399 << G_TEXTURE_IMAGE_FRAC);
        ulx -= 40 << 2;
        lrx += 40 << 2;
        rdp.textures_changed[0] = true;
        rdp.textures_changed[1] = true;
    }
#endif
    if ((rdp.other_mode_h & (3U << G_MDSFT_CYCLETYPE)) == G_CYC_COPY) {
        // Per RDP Command Summary Set Tile's shift s and this dsdx should be set to 4 texels
        // Divide by 4 to get 1 instead
        dsdx >>= 2;
        
        // Color combiner is turned off in copy mode
        gfx_dp_set_combine_mode(CC_RAW_D(G_CCMUX_TEXEL0, G_ACMUX_TEXEL0));
        
        // Per documentation one extra pixel is added in this modes to each edge
        lrx += 1 << 2;
        lry += 1 << 2;
    }
    
    /* PORT (2026-09-27): at exactly 1 texel per pixel the RDP's bilinear filter samples texel centers,
     * i.e. returns the texels unfiltered; here 2x2 supersamples land a quarter texel off-center and
     * bilinear smeared 1-pixel HUD/minimap lines (tools/statediff fbdiff). Point-sample instead. */
    uint32_t saved_omh_filter = rdp.other_mode_h;
    bool one_to_one = (dsdx == (1 << 10) || dsdx == -(1 << 10)) && (dtdy == (1 << 10) || dtdy == -(1 << 10));
    if (one_to_one) {
        rdp.other_mode_h = (rdp.other_mode_h & ~(3U << G_MDSFT_TEXTFILT)) | G_TF_POINT;
    }

    // uls and ult are S10.5
    // dsdx and dtdy are S5.10
    // lrx, lry, ulx, uly are U10.2
    // lrs, lrt are S10.5
    if (flip) {
        dsdx = -dsdx;
        dtdy = -dtdy;
    }
    int16_t width = !flip ? lrx - ulx : lry - uly;
    int16_t height = !flip ? lry - uly : lrx - ulx;
    float lrs = ((uls << 7) + dsdx * width) >> 7;
    float lrt = ((ult << 7) + dtdy * height) >> 7;
    
    struct LoadedVertex* ul = &rsp.loaded_vertices[MAX_VERTICES + 0];
    struct LoadedVertex* ll = &rsp.loaded_vertices[MAX_VERTICES + 1];
    struct LoadedVertex* lr = &rsp.loaded_vertices[MAX_VERTICES + 2];
    struct LoadedVertex* ur = &rsp.loaded_vertices[MAX_VERTICES + 3];
    ul->u = uls;
    ul->v = ult;
    lr->u = lrs;
    lr->v = lrt;
    if (!flip) {
        ll->u = uls;
        ll->v = lrt;
        ur->u = lrs;
        ur->v = ult;
    } else {
        ll->u = lrs;
        ll->v = ult;
        ur->u = uls;
        ur->v = lrt;
    }
    
    gfx_draw_rectangle(ulx, uly, lrx, lry);
    rdp.combine_mode = saved_combine_mode;
    rdp.drawing_rect = false;
    if (one_to_one) {
        rdp.other_mode_h = (rdp.other_mode_h & ~(3U << G_MDSFT_TEXTFILT)) | (saved_omh_filter & (3U << G_MDSFT_TEXTFILT));
    }
    if (rdp.first_tile != saved_first_tile) {
        rdp.textures_changed[0] = true;
        rdp.textures_changed[1] = true;
        rdp.first_tile = saved_first_tile;
    }
}

static void gfx_dp_fill_rectangle(int32_t ulx, int32_t uly, int32_t lrx, int32_t lry) {
    if (rdp.color_image_address == rdp.z_buf_address) {
        // Don't clear Z buffer here since we already did it with glClear
        return;
    }
    uint32_t mode = (rdp.other_mode_h & (3U << G_MDSFT_CYCLETYPE));
    
    if (mode == G_CYC_COPY || mode == G_CYC_FILL) {
        // Per documentation one extra pixel is added in this modes to each edge
        lrx += 1 << 2;
        lry += 1 << 2;
    } else {
        /* PORT (2026-09-30): 1/2-cycle fills exclude their lower-right edge, so the game's full-screen
         * fades (gDPFillRectangle(0, 0, 319, 239)) leave the last row and column uncovered. On a TV
         * that line is overscan; the 3DS shows every pixel (2x supersampled: a visible stripe under
         * fade-to-black). Fills reaching the last row/column cover them. */
        if (lry >= (SCREEN_HEIGHT - 1) * 4 && lry < SCREEN_HEIGHT * 4) lry = SCREEN_HEIGHT * 4;
        if (lrx >= (SCREEN_WIDTH - 1) * 4 && lrx < SCREEN_WIDTH * 4) lrx = SCREEN_WIDTH * 4;
    }
    
    for (int i = MAX_VERTICES; i < MAX_VERTICES + 4; i++) {
        struct LoadedVertex* v = &rsp.loaded_vertices[i];
        v->color = rdp.fill_color;
    }
    
    /* widescreen: untextured fills spanning the N64's full width stretch to the 400-pixel screen */
    /* 1/2-cycle fills (Environment_FillScreen, fbdemo fades) are gDPFillRectangle(0, 0, 319, 239): no
     * extra pixel is added in those modes, so the right edge is 319 << 2; FILL mode reaches 320 << 2
     * above. Requiring 320 << 2 left every fade/flash 4:3 in widescreen. */
    sRectFullWidth = WIDE_ACTIVE() && ulx <= 0 && lrx >= (SCREEN_WIDTH - 1) * 4;
    uint64_t saved_combine_mode = rdp.combine_mode;
    /* only FILL mode writes the fill color; 1/2-cycle fill rects go through the combiner (prim-colored
     * fades, letterboxes, sky rects) - as in libultraship's GfxDpFillRectangle */
    if (mode == G_CYC_FILL) {
        gfx_dp_set_combine_mode(CC_RAW_D(G_CCMUX_SHADE, G_ACMUX_SHADE));
    }
    gfx_draw_rectangle(ulx, uly, lrx, lry);
    rdp.combine_mode = saved_combine_mode;
}

static void gfx_dp_set_z_image(void *z_buf_address) {
    rdp.z_buf_address = z_buf_address;
}

static void gfx_dp_set_color_image(uint32_t format, uint32_t size, uint32_t width, void* address) {
#ifdef PORT_ICONGEN
    {
        static void* sLast[8];
        extern void PortDbgX(const char*, unsigned);
        int k, seen = 0;
        for (k = 0; k < 8; k++) seen |= sLast[k] == address;
        if (!seen) {
            for (k = 7; k > 0; k--) sLast[k] = sLast[k - 1];
            sLast[0] = address;
            PortDbgX("[mesh] color image", (unsigned)(uintptr_t)address);
            PortDbgX("[mesh] preview buffer", (unsigned)(uintptr_t)gPortIconGenBuf);
        }
    }
#endif
    rdp.color_image_address = address;
    rdp.color_image_width = width + 1; /* the command carries width - 1 */
    sTargetDirty = true;
}

static void gfx_sp_set_other_mode(uint32_t shift, uint32_t num_bits, uint64_t mode) {
    uint64_t mask = (((uint64_t)1 << num_bits) - 1) << shift;
    uint64_t om = rdp.other_mode_l | ((uint64_t)rdp.other_mode_h << 32);
    om = (om & ~mask) | mode;
    rdp.other_mode_l = (uint32_t)om;
    rdp.other_mode_h = (uint32_t)(om >> 32);
}


static inline void *seg_addr(uintptr_t w1) {
    uintptr_t v;
#ifdef __3DS__
    /* 3DS address model: our host RAM (binary .data, gfx pools, arena) lives at
     * physical-equivalent addresses below 0x20000000. The game/decomp produces
     * three flavours of pointer:
     *   - N64 KSEG0/KSEG1 (0x80000000+): strip to physical (== host address).
     *   - true native host pointer (>= 0x10000000, e.g. linear heap): use as-is.
     *   - low value (< 0x10000000): either a real host pointer in .data, OR an
     *     N64 segmented address. If the segment slot is unset it's native;
     *     otherwise translate base+offset (and strip if the base was KSEG0). */
    if (w1 == 0) {
        return NULL;
    }
    /* NOTE (2026-07-31): a "D7 part 2" that resolved SMALL-offset seg refs into heap-loaded
     * segments (to fix the file-select's seg-1 title_static file slots) was tried and
     * REVERTED — it garbled gameplay scene geometry (scene seg-2 small-offset refs are a
     * mix of genuine refs AND native relocated pointers, indistinguishable by offset size).
     * The file-select's black file slots need a narrower fix (only its specific UI segments)
     * that does not touch scene resolution. See roadmap §5.2d. */
    /* Any pointer INTO the loaded binary image [0x00100000, __end__) is a native
     * relocated pointer (a gGfxPools master-DL branch, or a DL/vertex/texture
     * pointer baked into a native-compiled asset), NOT an N64 segment offset. It
     * must pass through even when its high nibble collides with a SET segment —
     * e.g. asset DL pointers relocated to 0x01ffxxxx (nibble 1) vs a loaded
     * segment 1. Segment-translating them sends the interpreter into the unmapped
     * gap above the image. Genuine segment refs to DMA'd room/object data are
     * >= 0x03000000 (> __end__) and still translate below. Mirror of
     * PortSegmentedToVirtual; subsumes the old gGfxPools-only special case. */
    if (w1 >= 0x00100000u && w1 < (uintptr_t)__end__) {
        return (void *) w1;
    }
    if (w1 >= 0x80000000u) {
        return (void *) (w1 & 0x1FFFFFFFu); /* KSEG0/KSEG1 -> physical/host */
    }
    if (w1 >= 0x10000000u) {
        return (void *) w1; /* native host pointer */
    }
    /* D7 fix (2026-07-31): the 3DS app heap (newlib malloc, where DMA'd assets + the
     * game arena live) is at 0x08000000-0x0FFFFFFF, colliding with OoT segment numbers
     * 8-0xF. A native heap pointer in DL data (e.g. a sub-DL the file-select built in a
     * heap buffer, w1=0x08243670) carries its FULL heap address, so its low 24 bits are
     * large (>= 0x100000, since arena allocations start at ~0x0824xxxx); a genuine
     * seg-8..F reference has a SMALL offset (< 1MB) into its UI/keep asset. Treat the
     * large-offset case as a native pointer BEFORE segment translation — otherwise
     * seg_addr did gfx_port_segments[8]+0x243670 = out-of-bounds garbage -> file-select
     * DL runaways. Small-offset (< 0x100000) values still translate as real seg refs. */
    if (w1 >= 0x08000000u && (w1 & 0x00FFFFFFu) >= 0x00100000u) {
        return (void *) w1;
    }
    {
        uintptr_t base = gfx_port_segments[(w1 << 4) >> 28];
        if (base == 0) {
            return (void *) w1; /* unset segment -> native pointer */
        }
        v = base + (w1 & 0x00FFFFFF);
        if (v >= 0x80000000u) {
            v &= 0x1FFFFFFFu; /* segment base was a KSEG0 address */
        }
        return (void *) v;
    }
#else
    if (w1 >= 0x10000000u || w1 == 0) {
        return (void *) w1; /* native pointer (or NULL) */
    }
    v = gfx_port_segments[(w1 << 4) >> 28] + (w1 & 0x00FFFFFF);
    if (v != 0 && v < 0x10000000u) {
        v += 0x80000000u; /* physical emulated-RDRAM base */
    }
    return (void *) v;
#endif
}

/* G_MTX operand. PORT (2026-09-25): 0x01000000 is the billboard matrix (segment 1 =
 * play->billboardMtx, the only segment-1 reference in OoT, always a gsSPMatrix operand), but it
 * also lies inside the loaded binary, so seg_addr passed it through as a native pointer and every
 * billboarded draw (torch glows, moon, flames, effects) multiplied by wallmaster vertex bytes -
 * found by tools/statediff fbdiff (Fire Temple torch glows as a screen-wide band). */
static inline void *seg_addr_mtx(uintptr_t w1) {
#ifdef __3DS__
    if (w1 == 0x01000000u && gfx_port_segments[1] != 0) {
        uintptr_t v = gfx_port_segments[1];
        return (void *) (v >= 0x80000000u ? v & 0x1FFFFFFFu : v);
    }
#endif
    return seg_addr(w1);
}

#define C0(pos, width) ((cmd->words.w0 >> (pos)) & ((1U << width) - 1))
#define C1(pos, width) ((cmd->words.w1 >> (pos)) & ((1U << width) - 1))

/* PORT (2026-09-24): S2DEX background rectangles (pre-rendered rooms: Market, shops, houses).
 * The room DL switches microcode with gSPLoadUcodeL(gspS2DEX2d_fifo) and draws with
 * gSPBgRectCopy / gSPBgRect1Cyc. Logic follows libultraship's Gfxs2dexBgCopy/Gfxs2dexBg1cyc (MIT):
 * load the image and draw one textured rectangle -- here in 16-row strips so each strip fits the
 * importer's buffers (the whole 320x240 RGBA16 image is 150 KB). */
static bool sUcodeS2dex = false;
static void gfx_s2dex_bg_rect(const uObjBg* bg, bool copy) {
    if (!sBgThisFrame) {
        sBgThisFrame = 1; /* pre-rendered background: keep this frame 4:3 */
        gfx_flush();
        gfx_apply_scissor();
    }
    const uint8_t* img = seg_addr((uintptr_t)bg->b.imagePtr);
    uint32_t w = bg->b.imageW >> 2, h = bg->b.imageH >> 2, row, n;
    uint32_t line = (w * 2 + 7) >> 3;
    int32_t fx = bg->b.frameX, fy = bg->b.frameY;
    const uint32_t kStrip = 16;
    if (img == NULL || w == 0 || h == 0 || bg->b.imageSiz != G_IM_SIZ_16b) return; /* OoT backgrounds are 16-bit */
#ifdef __3DS__
    { /* stereo: the pre-rendered room sits at a fixed middle depth, not at the screen (see G_NOOP tags) */
        extern void gfx_citro3d_set_stereo_mode(int mode);
        gfx_flush();
        gfx_citro3d_set_stereo_mode(2);
    }
#endif
    for (row = 0; row < h; row += kStrip) {
        n = (h - row < kStrip) ? h - row : kStrip;
        gfx_dp_set_texture_image(bg->b.imageFmt, G_IM_SIZ_16b, w - 1, img);
        gfx_dp_set_tile(bg->b.imageFmt, G_IM_SIZ_16b, line, 0, G_TX_LOADTILE, 0, 0, 0, 0, 0, 0, 0);
        gfx_dp_load_tile(G_TX_LOADTILE, 0, row << 2, (w - 1) << 2, (row + n - 1) << 2);
        gfx_dp_set_tile(bg->b.imageFmt, G_IM_SIZ_16b, line, 0, G_TX_RENDERTILE, bg->b.imagePal, 0, 0, 0, 0, 0, 0);
        gfx_dp_set_tile_size(G_TX_RENDERTILE, 0, row << 2, (w - 1) << 2, (row + n - 1) << 2);
        if (copy) {
            gfx_dp_texture_rectangle(fx, fy + (int32_t)(row << 2), fx + (int32_t)(w << 2) - 4,
                                     fy + (int32_t)((row + n) << 2) - 4, G_TX_RENDERTILE, 0, (int16_t)(row << 5),
                                     4 << 10, 1 << 10, false);
        } else {
            gfx_dp_texture_rectangle(fx, fy + (int32_t)(row << 2), fx + (int32_t)bg->b.frameW,
                                     fy + (int32_t)((row + n) << 2), G_TX_RENDERTILE, 0, (int16_t)(row << 5),
                                     1 << 10, 1 << 10, false);
        }
    }
#ifdef __3DS__
    {
        extern void gfx_citro3d_set_stereo_mode(int mode);
        gfx_flush();
        gfx_citro3d_set_stereo_mode(0);
    }
#endif
}

static uint32_t rdp_half1; /* G_RDPHALF_1 word for G_BRANCH_Z */
#ifdef __3DS__
static uint32_t sDlBranches; /* display-list branches this frame (runaway guard, gfx_run_dl) */
#endif

static const uint8_t sKeepsTriState[256] = {
    [G_VTX] = 1, [G_TRI1] = 1, [G_TRI2] = 1, [G_QUAD] = 1, [G_MTX] = 1, [(uint8_t)G_POPMTX] = 1, [G_DL] = 1,
    [(uint8_t)G_ENDDL] = 1, [(uint8_t)G_NOOP] = 1,
    /* PORT PERF (2026-09-30): the syncs are no-ops here (~1000/frame) */
    [(uint8_t)G_RDPPIPESYNC] = 1, [(uint8_t)G_RDPLOADSYNC] = 1, [(uint8_t)G_RDPTILESYNC] = 1, [(uint8_t)G_RDPFULLSYNC] = 1,
#ifdef F3DEX_GBI_2
    [(uint8_t)G_RDPHALF_1] = 1, [(uint8_t)G_BRANCH_Z] = 1, [(uint8_t)G_CULLDL] = 1,
#endif
};
static void gfx_run_dl(Gfx* cmd) {
    int dummy = 0;
#ifdef __3DS__
    extern unsigned PortMem_ReadableEnd(unsigned addr);
    uintptr_t valid_until = 0; /* cmd reads are safe below this mapped-block end */
#endif
    for (;;) {
#ifdef __3DS__
        /* Crash-proof the walk: an un-terminated or garbage DL (bad seg_addr,
         * missing G_ENDDL, or garbage bytes decoded as a G_DL branch) would read
         * off its buffer into unmapped memory and data-abort. Before reading a
         * command, make sure both of its 8 bytes are inside a mapped block; if
         * cmd has crossed out of the last known-good block, re-probe. One
         * svcQueryMemory per 4KB is negligible. */
        if ((uintptr_t)cmd + sizeof(Gfx) > valid_until) {
            unsigned end = PortMem_ReadableEnd((unsigned)(uintptr_t)cmd);
            if (end == 0 || (uintptr_t)cmd + sizeof(Gfx) > (uintptr_t)end) {
                PortLogFastX("[GFX] stop unmapped DL cmd", (unsigned)(uintptr_t)cmd);
                return;
            }
            valid_until = end;
        }
        /* PORT PERF (2026-10-05): the display list is read sequentially; an Old 3DS (no L2) waits 50-190 cycles per
         * missed 32-byte line (hardware v55 memory probe), so the line two commands-lines ahead is requested now (PLD:
         * a hint, never faults, also past the end of the list) */
        __builtin_prefetch(cmd + 8);
#endif
        uint32_t opcode = cmd->words.w0 >> 24;
#ifdef __3DS__
        sPortCurCmd = cmd; /* draw attribution: which DL command produced a draw */
        if ((++gPortPerfDlCmds & 255) == 0) {
            extern void Port3ds_MaybePumpAudio(void);
            Port3ds_MaybePumpAudio(); /* audio preempts rendering on each retrace, like the N64 */
        }
#ifdef PORT_PERF_STAGES
        { extern u32 gPortPerfOpCounts[256]; gPortPerfOpCounts[opcode & 0xFF]++; }
#endif
#endif
        if (opcode == (uint8_t)G_LOAD_UCODE) { /* track which microcode the DL runs under */
            extern uint64_t gspS2DEX2d_fifoTextStart[];
            sUcodeS2dex = (seg_addr(cmd->words.w1) == (void*)gspS2DEX2d_fifoTextStart);
            ++cmd;
            continue;
        }
        if (sUcodeS2dex && (opcode <= 0x0B || opcode == 0xDA || opcode == 0xDC)) {
            /* S2DEX-only opcodes (same numbers as F3DEX2 VTX/TRI/...): draw backgrounds, skip the rest */
            if (opcode == G_BG_COPY || opcode == G_BG_1CYC) {
                gfx_s2dex_bg_rect((const uObjBg*)seg_addr(cmd->words.w1), opcode == G_BG_COPY);
            }
            ++cmd;
            continue;
        }
#ifdef PORT_GBIAUDIT
        { extern unsigned gPortGbiCounts[256]; gPortGbiCounts[opcode & 0xFF]++; }
#endif
        /* commands that cannot change triangle render state keep the fast path (a table: one load per command
         * instead of a second switch, PORT PERF 2026-10-03) */
        if (!sKeepsTriState[opcode & 0xFF]) {
            sTriStateOk = false;
        }

#if defined(__3DS__) && defined(PORT_PERF_STAGES)
        uint64_t tOp = PERF_T(); /* perf_stages: time per opcode (G_DL sub-lists excluded) */
#endif
        switch (opcode) {
#ifdef __3DS__
            /* PORT (2026-09-30): stereo depth tags (gDPNoOpTag 0x3D5E3D0m; z_vr_box_draw.c): m = 0 normal,
             * 1 infinity (sky), 2 pre-rendered room picture, 3 flat at screen depth (HUD). */
            case (uint8_t)G_NOOP:
                if (((cmd->words.w0 >> 16) & 0xFF) == PORT_INTERP_TAG) {
                    interp_group(cmd->words.w0, cmd->words.w1);
                }
#ifdef PORT_ACTOR_PROF
                else if ((cmd->words.w1 & 0xFFFF0000u) == 0x3D5D0000u) { /* z_actor.c: an actor type's drawing */
                    static u64 sActT0;
                    static int sActId = -1;
                    extern u64 gPortActorTicks[512];
                    extern u32 gPortActorCalls[512];
                    u64 now = svcGetSystemTick();
                    if (sActId >= 0) {
                        gPortActorTicks[sActId] += now - sActT0;
                        gPortActorCalls[sActId]++;
                    }
                    sActId = (cmd->words.w1 & 0xFFFF) == 0xFFFF ? -1 : (int)(cmd->words.w1 & 0x1FF);
                    sActT0 = now;
                }
#endif
                else if ((cmd->words.w1 & 0xFFFFFFFEu) == 0x3D5E5200u) {
                    /* z_room.c: room geometry begins (0) / ends (1) */
                    sInRoomDl = (cmd->words.w1 & 1) == 0;
                    {
                        static u64 sRoomT0;
                        extern u64 gPortPerfRoomTicks;
                        if (sInRoomDl) {
                            sRoomT0 = svcGetSystemTick();
                        } else if (sRoomT0 != 0) {
                            gPortPerfRoomTicks += svcGetSystemTick() - sRoomT0;
                            sRoomT0 = 0;
                        }
                    }
                } else if ((cmd->words.w1 & 0xFFFFFF00u) == 0x3D5E3D00u) {
                    extern void gfx_citro3d_set_stereo_mode(int mode);
                    gfx_flush();
                    gfx_citro3d_set_stereo_mode((int)(cmd->words.w1 & 0xFF));
                    sStereoDrawMode = (int)(cmd->words.w1 & 0xFF);
                    if ((cmd->words.w1 & 0xFF) == 2 && !sStereoRoomThisFrame) {
                        sStereoRoomThisFrame = 1; /* a pre-rendered room's picture: this frame stays 4:3 */
                        gfx_apply_scissor();
                    }
                }
                break;
#endif
            // RSP commands:
            case G_MTX:
#ifdef F3DEX_GBI_2
            {
                uint64_t tm = PERF_T();
                PROF_SET(PROF_MTX);
                gfx_sp_matrix(C0(0, 8) ^ G_MTX_PUSH, (const int32_t *) seg_addr_mtx(cmd->words.w1));
                gPortPerfMtx += PERF_T() - tm;
            }
#else
                gfx_sp_matrix(C0(16, 8), (const int32_t *) seg_addr_mtx(cmd->words.w1));
#endif
                break;
            case (uint8_t)G_POPMTX:
#ifdef F3DEX_GBI_2
                gfx_sp_pop_matrix(cmd->words.w1 / 64);
#else
                gfx_sp_pop_matrix(1);
#endif
                break;
            case G_MOVEMEM:
#ifdef F3DEX_GBI_2
                gfx_sp_movemem(C0(0, 8), C0(8, 8) * 8, seg_addr(cmd->words.w1));
#else
                gfx_sp_movemem(C0(16, 8), 0, seg_addr(cmd->words.w1));
#endif
                break;
            case (uint8_t)G_MOVEWORD:
#ifdef F3DEX_GBI_2
                gfx_sp_moveword(C0(16, 8), C0(0, 16), cmd->words.w1);
#else
                gfx_sp_moveword(C0(0, 8), C0(8, 16), cmd->words.w1);
#endif
                break;
            case (uint8_t)G_TEXTURE:
#ifdef F3DEX_GBI_2
                gfx_sp_texture(C1(16, 16), C1(0, 16), C0(11, 3), C0(8, 3), C0(1, 7));
#else
                gfx_sp_texture(C1(16, 16), C1(0, 16), C0(11, 3), C0(8, 3), C0(0, 8));
#endif
                break;
            case G_VTX:
#ifdef F3DEX_GBI_2
                gfx_sp_vertex(C0(12, 8), C0(1, 7) - C0(12, 8), seg_addr(cmd->words.w1));
#elif defined(F3DEX_GBI) || defined(F3DLP_GBI)
                gfx_sp_vertex(C0(10, 6), C0(16, 8) / 2, seg_addr(cmd->words.w1));
#else
                gfx_sp_vertex((C0(0, 16)) / sizeof(Vtx), C0(16, 4), seg_addr(cmd->words.w1));
#endif
                break;
            case G_DL:
#ifdef __3DS__
                gPortPerfDlCalls++;
#endif
#ifdef __3DS__
                /* a runaway (garbage decoded as a branch) can only loop through branches: they are counted here
                 * instead of every command (PORT PERF 2026-10-05; linear runs end at unmapped memory, see above) */
                if (++sDlBranches > 400000u) {
                    PortLogFastX("[GFX] RUNAWAY stop cmd", (unsigned)(uintptr_t)cmd);
                    return;
                }
#endif
                if (C0(16, 1) == 0) {
                    // Push return address
                    gfx_run_dl((Gfx *)seg_addr(cmd->words.w1));
                } else {
                    cmd = (Gfx *)seg_addr(cmd->words.w1);
                    --cmd; // increase after break
                }
                break;
            case (uint8_t)G_ENDDL:
                return;
#ifdef F3DEX_GBI_2
            /* PORT (2026-09-27): F3DEX2 LOD commands, unhandled before: close-range geometry behind
             * gsSPBranchLessZraw never drew (Gerudo Valley's waterfall; tools/statediff fbdiff).
             * G_RDPHALF_1 carries the branch target; G_BRANCH_Z branches (no return) when the vertex's
             * depth is <= zval, compared in clip-space z like libultraship (Ship of Harkinian). */
            case (uint8_t)G_RDPHALF_1:
                rdp_half1 = cmd->words.w1;
                break;
            case (uint8_t)G_BRANCH_Z: {
                uint32_t vi = C0(0, 12) / 2;
                if (vi < MAX_VERTICES && rsp.loaded_vertices[vi].z <= (float)(int32_t)cmd->words.w1) {
#ifdef __3DS__
                    if (++sDlBranches > 400000u) {
                        PortLogFastX("[GFX] RUNAWAY stop cmd", (unsigned)(uintptr_t)cmd);
                        return;
                    }
#endif
                    cmd = (Gfx *)seg_addr(rdp_half1);
                    --cmd; // increase after break
                }
                break;
            }
            case (uint8_t)G_CULLDL: { /* end the DL if vertices v0..vn are all outside one plane */
                uint32_t v0 = C0(0, 16) / 2, vn = cmd->words.w1 / 2, k;
                uint8_t rej = 0xFF;
                for (k = v0; k <= vn && k < MAX_VERTICES; k++) {
                    rej &= rsp.loaded_vertices[k].clip_rej;
                }
                if (v0 <= vn && vn < MAX_VERTICES && rej != 0) {
                    return;
                }
                break;
            }
#endif
#ifdef F3DEX_GBI_2
            case G_GEOMETRYMODE:
                gfx_sp_geometry_mode(~C0(0, 24), cmd->words.w1);
                break;
#else
            case (uint8_t)G_SETGEOMETRYMODE:
                gfx_sp_geometry_mode(0, cmd->words.w1);
                break;
            case (uint8_t)G_CLEARGEOMETRYMODE:
                gfx_sp_geometry_mode(cmd->words.w1, 0);
                break;
#endif
            case (uint8_t)G_TRI1:
#ifdef F3DEX_GBI_2
#ifdef __3DS__
                if (raw_tri_fast(C0(16, 8) / 2, C0(8, 8) / 2, C0(0, 8) / 2)) {
                    break;
                }
#endif
                gfx_sp_tri1(C0(16, 8) / 2, C0(8, 8) / 2, C0(0, 8) / 2);
#elif defined(F3DEX_GBI) || defined(F3DLP_GBI)
                gfx_sp_tri1(C1(16, 8) / 2, C1(8, 8) / 2, C1(0, 8) / 2);
#else
                gfx_sp_tri1(C1(16, 8) / 10, C1(8, 8) / 10, C1(0, 8) / 10);
#endif
                break;
#if defined(F3DEX_GBI) || defined(F3DLP_GBI)
            case (uint8_t)G_TRI2:
#ifdef F3DEX_GBI_2
            /* PORT (2026-09-24): F3DEX2 G_QUAD (gSP1Quadrangle, 411 sites incl. the pause-menu
             * pages) packs two triangles exactly like G_TRI2 and the microcode draws it the same
             * way. It had no case, so every quad was silently skipped (found by tools/gbi_audit.py). */
            case (uint8_t)G_QUAD:
#endif
#ifdef __3DS__
                if (!raw_tri_fast(C0(16, 8) / 2, C0(8, 8) / 2, C0(0, 8) / 2)) {
                    gfx_sp_tri1(C0(16, 8) / 2, C0(8, 8) / 2, C0(0, 8) / 2);
                }
                if (!raw_tri_fast(C1(16, 8) / 2, C1(8, 8) / 2, C1(0, 8) / 2)) {
                    gfx_sp_tri1(C1(16, 8) / 2, C1(8, 8) / 2, C1(0, 8) / 2);
                }
#else
                gfx_sp_tri1(C0(16, 8) / 2, C0(8, 8) / 2, C0(0, 8) / 2);
                gfx_sp_tri1(C1(16, 8) / 2, C1(8, 8) / 2, C1(0, 8) / 2);
#endif
                break;
#endif
            case (uint8_t)G_SETOTHERMODE_L:
#ifdef F3DEX_GBI_2
                gfx_sp_set_other_mode(31 - C0(8, 8) - C0(0, 8), C0(0, 8) + 1, cmd->words.w1);
#else
                gfx_sp_set_other_mode(C0(8, 8), C0(0, 8), cmd->words.w1);
#endif
                break;
            case (uint8_t)G_SETOTHERMODE_H:
#ifdef F3DEX_GBI_2
                gfx_sp_set_other_mode(63 - C0(8, 8) - C0(0, 8), C0(0, 8) + 1, (uint64_t) cmd->words.w1 << 32);
#else
                gfx_sp_set_other_mode(C0(8, 8) + 32, C0(0, 8), (uint64_t) cmd->words.w1 << 32);
#endif
                break;
            case (uint8_t)G_RDPSETOTHERMODE:
                /* PORT (2026-09-24): gDPSetOtherMode sets the WHOLE other mode in one command
                 * (hi 24 bits in w0, lo word in w1). OoT's Gfx_SetupDL_* presets (z_rcp.c, 72
                 * uses) rely on it; unhandled, every draw after a preset inherited stale state
                 * (e.g. the HUD drew in FILL cycle with the scene's FOG blender -> flat dark boxes). */
                rdp.other_mode_h = cmd->words.w0 & 0x00FFFFFFu;
                rdp.other_mode_l = cmd->words.w1;
                break;
            
            // RDP Commands:
            case G_SETTIMG:
                gfx_dp_set_texture_image(C0(21, 3), C0(19, 2), C0(0, 12), seg_addr(cmd->words.w1));
                break;
            case G_LOADBLOCK:
                gfx_dp_load_block(C1(24, 3), C0(12, 12), C0(0, 12), C1(12, 12), C1(0, 12));
                break;
            case G_LOADTILE:
                gfx_dp_load_tile(C1(24, 3), C0(12, 12), C0(0, 12), C1(12, 12), C1(0, 12));
                break;
            case G_SETTILE:
                gfx_dp_set_tile(C0(21, 3), C0(19, 2), C0(9, 9), C0(0, 9), C1(24, 3), C1(20, 4), C1(18, 2), C1(14, 4), C1(10, 4), C1(8, 2), C1(4, 4), C1(0, 4));
                break;
            case G_SETTILESIZE:
                gfx_dp_set_tile_size(C1(24, 3), C0(12, 12), C0(0, 12), C1(12, 12), C1(0, 12));
                break;
            case G_LOADTLUT:
                gfx_dp_load_tlut(C1(24, 3), C1(14, 10));
                break;
            case G_SETENVCOLOR:
                gfx_dp_set_env_color(C1(24, 8), C1(16, 8), C1(8, 8), C1(0, 8));
                break;
            case G_SETPRIMCOLOR:
                gfx_dp_set_prim_color(C1(24, 8), C1(16, 8), C1(8, 8), C1(0, 8));
                rdp.prim_lod_frac = C0(0, 8);
                break;
            case G_SETFOGCOLOR:
                gfx_dp_set_fog_color(C1(24, 8), C1(16, 8), C1(8, 8), C1(0, 8));
                break;
            case G_SETFILLCOLOR:
                gfx_dp_set_fill_color(cmd->words.w1);
                break;
            case G_SETCOMBINE:
                gfx_dp_set_combine_mode(((uint64_t)C0(0, 24) << 32) | cmd->words.w1);
                break;
            // G_SETPRIMCOLOR, G_CCMUX_PRIMITIVE, G_ACMUX_PRIMITIVE, is used by Goddard
            // G_CCMUX_TEXEL1, LOD_FRACTION is used in Bowser room 1
            case G_TEXRECT:
            case G_TEXRECTFLIP:
            {
                int32_t lrx, lry, tile, ulx, uly;
                uint32_t uls, ult, dsdx, dtdy;
#ifdef F3DEX_GBI_2E
                lrx = (int32_t)(C0(0, 24) << 8) >> 8;
                lry = (int32_t)(C1(0, 24) << 8) >> 8;
                ++cmd;
                ulx = (int32_t)(C0(0, 24) << 8) >> 8;
                uly = (int32_t)(C1(0, 24) << 8) >> 8;
                ++cmd;
                uls = C0(16, 16);
                ult = C0(0, 16);
                dsdx = C1(16, 16);
                dtdy = C1(0, 16);
#else
                lrx = C0(12, 12);
                lry = C0(0, 12);
                tile = C1(24, 3);
                ulx = C1(12, 12);
                uly = C1(0, 12);
                ++cmd;
                uls = C1(16, 16);
                ult = C1(0, 16);
                ++cmd;
                dsdx = C1(16, 16);
                dtdy = C1(0, 16);
#endif
                gfx_dp_texture_rectangle(ulx, uly, lrx, lry, tile, uls, ult, dsdx, dtdy, opcode == G_TEXRECTFLIP);
                break;
            }
            case G_FILLRECT:
#ifdef F3DEX_GBI_2E
            {
                int32_t lrx, lry, ulx, uly;
                lrx = (int32_t)(C0(0, 24) << 8) >> 8;
                lry = (int32_t)(C1(0, 24) << 8) >> 8;
                ++cmd;
                ulx = (int32_t)(C0(0, 24) << 8) >> 8;
                uly = (int32_t)(C1(0, 24) << 8) >> 8;
                gfx_dp_fill_rectangle(ulx, uly, lrx, lry);
                break;
            }
#else
                gfx_dp_fill_rectangle(C1(12, 12), C1(0, 12), C0(12, 12), C0(0, 12));
                break;
#endif
            case G_SETSCISSOR:
                gfx_dp_set_scissor(C1(24, 2), C0(12, 12), C0(0, 12), C1(12, 12), C1(0, 12));
                break;
            case G_SETZIMG:
                gfx_dp_set_z_image(seg_addr(cmd->words.w1));
                break;
            case G_SETCIMG:
                gfx_dp_set_color_image(C0(21, 3), C0(19, 2), C0(0, 11), seg_addr(cmd->words.w1));
                break;
        }
#if defined(__3DS__) && defined(PORT_PERF_STAGES)
        if (gPortPerfStagesOn && opcode != G_DL) {
            extern u64 gPortPerfOpTicks[256];
            gPortPerfOpTicks[opcode & 0xFF] += PERF_T() - tOp;
        }
#endif
        ++cmd;
    }
}

static void gfx_sp_reset() {
    rsp.modelview_matrix_stack_size = 1;
    sMPDirty = true;
    rsp.current_num_lights = 2;
    rsp.lights_changed = true;
}

void gfx_get_dimensions(uint32_t *width, uint32_t *height) {
    gfx_wapi->get_dimensions(width, height);
}

void gfx_init(struct GfxWindowManagerAPI *wapi, struct GfxRenderingAPI *rapi) {
    gfx_wapi = wapi;
    gfx_rapi = rapi;
    gfx_wapi->init();
    gfx_rapi->init();
    
    // Used in the 120 star TAS
}

int gPortEventsExternal; /* 3ds_main.c render thread: the game thread runs the HOME/sleep events itself */
void gfx_handle_events(void) {
    gfx_wapi->handle_events();
}

void gfx_start_frame(void) {
#ifdef __3DS__
    gfx_texture_cache_apply_queued(); /* data the game loaded since the last frame (render thread) */
    gfx_citro3d_raw_mode(0); /* frames start on the GPU path's program */
    /* latched per frame: the A/B switch flips gPortRawVtxWant from the game thread (render thread) */
    gPortRawVtx = gPortRawVtxWant && gPortGpuVtx && gfx_citro3d_raw_ready();
    sRawDraw = 0;
    sRawLitCur = -1;
    sRawLightsN = 0;
    sInRoomDl = 0;
    sDlBranches = 0;
#endif
    sTriStateOk = false;
    /* the LookAt does not carry over to the next frame (kept, the last actor hilite's LookAt env-mapped
     * the next frame's room geometry: Chamber of the Sages pedestal worst; see gfx_sp_vertex) */
    memset(rsp.current_lookat, 0, sizeof(rsp.current_lookat));
    rsp.lights_changed = true;
    sPVState++; /* the stereo convergence changes per frame */
#ifdef __3DS__
    sGpuSlotN = 0, sGpuSlotCur = -1; /* GPU path: slots are per frame; draw state re-sent on first use */
    sGpuCull = -1, sGpuFogOn = -1, sGpuConvSet = -1.0f, sGpuLin = -1;
    sGpuPalN = sGpuPalSent = 0, sGpuPalEpoch++;
#endif
#ifdef __3DS__
    sBatchId++;
#endif
    /* (gfx_start_frame runs once per logic frame: the in-between frames are replays) */
    sPrerenderedRecent = (sBgThisFrame || sStereoRoomThisFrame) ? 20 : sPrerenderedRecent > 0 ? sPrerenderedRecent - 1 : 0;
    sBgLastFrame = sBgThisFrame; gPortPrerenderedFrame = sBgLastFrame || sStereoRoomThisFrame; sStereoRoomThisFrame = 0;
    sBgThisFrame = 0;
    if (!gPortEventsExternal) {
        gfx_wapi->handle_events();
    }
    sDrawTarget = NULL; /* the backend starts each frame on the screen */
    sTargetDirty = true;
    gfx_wapi->get_dimensions(&gfx_current_dimensions.width, &gfx_current_dimensions.height);
    if (gfx_current_dimensions.height == 0) {
        // Avoid division by zero
        gfx_current_dimensions.height = 1;
    }
    gfx_current_dimensions.aspect_ratio = (float)gfx_current_dimensions.width / (float)gfx_current_dimensions.height;
    gfx_apply_scissor(); /* widescreen toggled or a pre-rendered room entered/left */
}

#ifdef PORT_GBIAUDIT
/* DEBUG TOOL (opt-in, PORT_EXTRA=-DPORT_GBIAUDIT): runtime opcode histogram, dumped at frames
 * 600 and 1500 as "GBI op=XX n=count" so tools/gbi_audit.py's static list can be confirmed. */
unsigned gPortGbiCounts[256];
static void gbi_audit_dump(unsigned frame) {
    extern void PortDbgX(const char*, unsigned);
    int i;
    PortDbgX("GBI dump at frame", frame);
    for (i = 0; i < 256; i++) if (gPortGbiCounts[i]) { PortDbgX("GBI op", (unsigned)i); PortDbgX("GBI   n", gPortGbiCounts[i]); }
}
#endif
int gPortInterpExtra; /* 1 while an in-between (interpolated) pass re-runs the display list */

void gfx_run(Gfx *commands) {
    if (!gPortInterpExtra) gfx_port_frame_index++;
#ifdef __3DS__
    sTjOn = gPortTjDumpFrame >= 0 && (int)gfx_port_frame_index == gPortTjDumpFrame;
    if (sTjOn && sTj == NULL) {
        sTj = malloc(sizeof(*sTj) * TJ_MAX);
    }
    sTjN = 0;
#endif
#ifdef PORT_GBIAUDIT
    if (gfx_port_frame_index == 600 || gfx_port_frame_index == 1500) gbi_audit_dump(gfx_port_frame_index);
#endif
    {
        int i;
        for (i = 0; i < 16; i++) gfx_port_segments[i] = gSegments[i];
    }
    gfx_sp_reset();
    
    //puts("New frame");
    
    if (!gfx_wapi->start_frame()) {
        dropped_frame = true;
        return;
    }
    dropped_frame = false;
    
    double t0 = gfx_wapi->get_time();
    gfx_rapi->start_frame();
#ifdef __3DS__
    if (gPortReplayRec) {
        extern void gfx_citro3d_rec_begin(void);
        extern int gfx_citro3d_rec_direct(int k);
        int k, top;
        gfx_citro3d_rec_begin();
        /* GPU path: the walk draws the update's first shown frame itself (gfx_citro3d.c replay by copy) */
        if (gPortGpuVtx) {
            gfx_citro3d_rec_direct(gPortReplayFirstK);
        }
        for (k = 0; k < 2; k++) { /* the t = 1/3, 2/3 stacks start like the real one */
            for (top = 0; top < 11; top++) {
                memcpy(sAltMV[k][top], rsp.modelview_matrix_stack[top], sizeof(sAltMV[k][top]));
            }
            memcpy(sAltP[k], rsp.P_matrix, sizeof(sAltP[k]));
        }
    }
#endif
    PROF_SET(PROF_DL);
    gfx_run_dl(commands);
    gfx_flush();
#ifdef __3DS__
    {
        void* target = sDrawTarget;
        sDrawTarget = NULL; /* (the frame ends on the screen) */
        gPortFrameWide = WIDE_ACTIVE();
        sDrawTarget = target;
        sTargetDirty = true;
    }
    if (sTjOn) {
        FILE* f = fopen("sdmc:/3ds/oot/tjdump.bin", "wb");
        sTjOn = 0;
        if (f != NULL && sTj != NULL) {
            fwrite(&sTjN, 4, 1, f);
            fwrite(sTj, sizeof(*sTj), sTjN, f);
        }
        if (f != NULL) {
            fclose(f);
        }
    }
#endif
    { extern void PortGfx_FrameReady(void); PortGfx_FrameReady(); }
#ifdef PORT_ICONGEN
    {
        static int sMeshFrames;
        if (sMeshFrameHasTris && ++sMeshFrames == 10) { /* the preview's 10th frame: every texture imported */
            mesh_write();
        }
    }
    if (sMeshDbgVtx || sMeshDbgTri || sMeshDbgTri1) {
        extern void PortDbgX(const char*, unsigned);
        PortDbgX("[mesh] frame: vertex loads", (unsigned)sMeshDbgVtx);
        PortDbgX("[mesh] frame: tri1 calls", (unsigned)sMeshDbgTri1);
        PortDbgX("[mesh] frame: triangles emitted", (unsigned)sMeshDbgTri);
        sMeshDbgVtx = sMeshDbgTri = sMeshDbgTri1 = 0;
    }
    sMeshN = 0, sMeshFrameHasTris = 0;
#endif
    double t1 = gfx_wapi->get_time();
    //printf("Process %f %f\n", t1, t1 - t0);
#ifdef __3DS__
    if (gPortReplayRec) {
        /* the walk only recorded: draw its first shown frame (t = 1/3, 2/3, or the logic frame) */
        extern int gfx_citro3d_rec_end(void);
        extern void gfx_citro3d_replay(void);
        int k, drawn;
        gPortReplayRec = 0;
        drawn = gfx_citro3d_rec_end(); /* (may break the recording: before k is read) */
        k = gPortReplayBroken ? 2 : gPortReplayFirstK;
        PROF_SET(PROF_REPLAY);
        if (gPortGpuVtx) {
            extern void gfx_citro3d_replay_variant(int k);
            gfx_citro3d_replay_variant(k); /* palette matrices of that in-between frame; no VBO rewrite */
        } else if (k != 2) {
            gfx_replay_positions(k, gfx_rapi->z_is_from_0_to_1());
        }
        if (!drawn) {
            gfx_citro3d_replay();
        }
        if (gPortGpuVtx) {
            extern void gfx_citro3d_replay_variant(int k);
            gfx_citro3d_replay_variant(2);
        }
        PROF_SET(PROF_SWAP);
        gPortInterpExtra = k < 2;
        gfx_wapi->swap_buffers_begin();
        gPortInterpExtra = 0;
        port_draw_log_commit();
        return;
    }
#endif
    PROF_SET(PROF_SWAP);
    gfx_wapi->swap_buffers_begin();
#ifdef __3DS__
    port_draw_log_commit();
#endif
}

#ifdef __3DS__
int gPortReplayFirstK = 2; /* 3ds_main.c: which replay the recording frame shows (0: t = 1/3, 1: 2/3, 2: t = 1) */

/* 3ds_main.c: another shown frame of the recorded logic frame (k as above), on its own retrace */
void gfx_replay_frame(int k) {
    extern void gfx_citro3d_replay(void);
    gPortInterpExtra = k < 2;
    gfx_wapi->start_frame();
    PROF_SET(PROF_REPLAY);
    if (gPortGpuVtx) {
        extern void gfx_citro3d_replay_variant(int k);
        gfx_citro3d_replay_variant(k);
        gfx_citro3d_replay();
        gfx_citro3d_replay_variant(2);
    } else {
        gfx_replay_positions(k, gfx_rapi->z_is_from_0_to_1());
        gfx_citro3d_replay();
    }
    PROF_SET(PROF_SWAP);
    gfx_wapi->swap_buffers_begin();
    gPortInterpExtra = 0;
}
#endif

void gfx_end_frame(void) {
    if (!dropped_frame) {
        gfx_wapi->swap_buffers_end();
    }
}

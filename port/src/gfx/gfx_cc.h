#ifndef GFX_CC_H
#define GFX_CC_H

#include <stdint.h>

/* PORT (2026-09-25): the N64 color combiner, carried losslessly from gfx_pc.c to the backend.
 * Both cycles, both channels, every input of (A-B)*C+D, including ONE, COMBINED and the alpha
 * broadcasts that the sm64-era 3-bit encoding (8 codes, max 2 colors, cycle 1 dropped) could not
 * represent. The backend compiles a key into PICA TEV stages (gfx_citro3d.c).
 *
 * Key: shader_id0 = 64 bits of sources, 4 bits each at (cycle * 32 + channel * 16 + slot * 4),
 * channel 0 = RGB, 1 = alpha; slot 0..3 = A, B, C, D. In the ALPHA channel every source means its
 * alpha component (CCS_TEX0 = texel0 alpha, CCS_PRIM = prim alpha, ...). 1-cycle draws have cycle 1
 * = (0 - 0) * 0 + COMBINED. Cycle-1 TEXEL0/TEXEL1 are already swapped (the RDP's 2nd cycle sees the
 * next tile), so CCS_TEX0 always means texture unit 0. shader_id1 = SHADER_OPT_* flags. */
enum {
    CCS_0,
    CCS_1,
    CCS_TEX0,
    CCS_TEX1,
    CCS_TEX0A,
    CCS_TEX1A,
    CCS_SHADE,
    CCS_SHADEA,
    CCS_PRIM,
    CCS_PRIMA,
    CCS_ENV,
    CCS_ENVA,
    CCS_LODF,
    CCS_PRIMLODF,
    CCS_COMB,
    CCS_COMBA
};

#define CC_SRC(id0, cycle, ch, slot) ((uint8_t)(((id0) >> ((cycle) * 32 + (ch) * 16 + (slot) * 4)) & 0xF))

#define SHADER_OPT_ALPHA (1 << 0)        /* alpha channel is meaningful (blending or alpha test) */
#define SHADER_OPT_FOG (1 << 1)          /* blender cycle 1 mixes fog color by shade alpha */
#define SHADER_OPT_TEXTURE_EDGE (1 << 2) /* cutout (alpha test) */
#define SHADER_OPT_TEX0 (1 << 3)         /* texture units referenced by the key */
#define SHADER_OPT_TEX1 (1 << 4)

/* per-draw constant inputs of the combiner (RGBA8888, big-endian component order r,g,b,a) */
struct GfxCombineConsts {
    uint8_t prim[4], env[4], fog[4];
    uint8_t prim_lod_frac, lod_frac;
};

#endif

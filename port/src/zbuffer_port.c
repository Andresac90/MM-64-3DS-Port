/* zbuffer_port.c - answer the game's CPU reads of the N64 z-buffer (gZBuffer[y][x]) from the 3DS depth
 * buffer. Readers: point-light glows (z_lights.c, e.g. Navi) and the sun's lens-flare depth test
 * (z_kankyo.c Environment_GetPixelDepth).
 *
 * Mapping, measured by tools/statediff/zcal.py against ares' real gZBuffer (Deku Tree, frame 100:
 * median error 3 of 0x7FC0 screen-z units, 90% within -17..+3):
 *  - depth = low 24 bits of each D24S8 word; value = (ndc_z + 1) / 2 * 0xFFFFFF (vertex packing negates
 *    z, C3D_DepthMap(-1, 0) negates it back), so N64 screen z = depth / 0xFFFFFF * 0x7FC0, matching
 *    z_lights.c's own model (viewport z scale = translate = G_MAXZ / 2, times 32)
 *  - N64 pixel (x, y) -> window (x + 40, y) (4:3 pillarboxed in 400x240), times the supersampling
 *    factor (first subsample); the portrait buffer's column is mirrored window y, its row is window x
 * The result is encoded like the RDP stores z (3-bit exponent, 11-bit mantissa, 2-bit dz) using the
 * inverse of z_kankyo.c's sZBufValConversionTable.
 * Known difference: the N64 only clears z inside the scissor, so letterbox bars keep stale depth;
 * here they read as cleared (far). */
#include <stddef.h>
#include <stdint.h>

extern void Port3ds_RequestDepth(void);
extern const uint32_t* Port3ds_GetDepth(int* width, int* height);

#define N64_Z_FAR 0xFFFC /* GPACK_ZDZ(G_MAXFBZ, 0): the value the game clears the z-buffer to */

static const struct {
    uint8_t shift;
    uint32_t base;
} sZTable[8] = {
    { 6, 0x0000 << 3 }, { 5, 0x4000 << 3 }, { 4, 0x6000 << 3 }, { 3, 0x7000 << 3 },
    { 2, 0x7800 << 3 }, { 1, 0x7C00 << 3 }, { 0, 0x7E00 << 3 }, { 0, 0x7F00 << 3 },
};

static uint16_t encode_n64_z(uint32_t z18) {
    int e = 7;
    uint32_t m;

    while (e > 0 && z18 < sZTable[e].base) {
        e--;
    }
    m = (z18 - sZTable[e].base) >> sZTable[e].shift;
    if (m > 0x7FF) {
        m = 0x7FF;
    }
    return (uint16_t)((e << 13) | (m << 2));
}

uint16_t PortZBuf_Read(int x, int y) {
    int w, h, sx, sy, u, v;
    const uint32_t* depth;
    uint32_t d;

    Port3ds_RequestDepth(); /* keeps the per-frame readback alive while the game samples depth */
    depth = Port3ds_GetDepth(&w, &h);
    if (depth == NULL || x < 0 || x >= 320 || y < 0 || y >= 240) {
        return N64_Z_FAR;
    }
    sx = h / 400;
    sy = w / 240;
    /* Subsample choice and alignment measured against ares' gZBuffer over full frames (5 captures,
     * tools/statediff): the pixel's first x subsample and the window-y subsample just above it agree
     * best (3.7% of pixels off by >64 z units, vs 4.3% one subsample lower and 8.0% for the last
     * subsample; the latter flipped a torch glow at an object edge in Gerudo Training Ground). */
    /* PORT (2026-09-27): 3D geometry moved half an N64 pixel right/down (gfx_pc.c gfx_sp_vertex, measured
     * RDP triangle offset), so the same samples are one subsample further along both axes */
    u = w - 1 - (y * sy);
    if (u < 0) {
        u = 0;
    }
    v = (x + 40) * sx + 1;
    if (v > h - 1) {
        v = h - 1;
    }
    d = depth[v * w + u] & 0xFFFFFF;
    if (d >= 0xFFFFFF) {
        return N64_Z_FAR;
    }
    /* +1 screen-z unit: the N64 stores surfaces slightly deeper than the game's own projection of the
     * same point. Measured over 229,589 same-surface pixels in 4 captures (tools/statediff): median of
     * N64 - 3DS = +1, and +1 minimizes the mean error. Matters for glows sitting on their own geometry
     * (torch flames): N64 draws them, an unbiased 3DS value fails the strict test and hides them. */
    return encode_n64_z((((d * 0x7FC0ull) / 0xFFFFFF) + 1) << 3);
}

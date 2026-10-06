/*
 * jpeg_port.c — pre-rendered room backgrounds (JPEG) for the 3DS port.
 *
 * On N64, Room_DecodeJpeg (z_room.c) recognises a JFIF image by its first word, Huffman-decodes it on
 * the CPU, runs the IDCT/colour conversion as an RSP task (njpgdspMain), and copies the resulting
 * 320x240 RGBA5551 image back over the JPEG data IN PLACE; the S2DEX background draw then reads it.
 * In the port the JPEG sits in a natively-compiled u64 array (bytes swizzled per 8 on the LE ARM11),
 * so the N64 marker check never matched and no RSP JPEG task exists. This decodes the same file and
 * writes the image back in the same swizzled u64 layout every other compiled texture has, so the
 * standard texture path draws it unchanged. A second visit finds no marker and skips decoding.
 *
 * PORT (2026-09-27): decoder model MEASURED against the N64's output (ares framebuffer, where COPY-mode
 * backgrounds are the decoded texels; tools/statediff/jpegref.py): exact IDCT, chroma replicated
 * (nearest, no smoothing), JFIF YCbCr->RGB, TRUNCATED to 5 bits. 94.9% of background pixels exact
 * (the rest within one 5-bit step) vs 82% for rounding and ~70% for stb_image (smooth chroma).
 * All 35 OoT backgrounds are 320x240 baseline 4:2:0; any baseline sampling is handled.
 */
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define BG_WIDTH 320
#define BG_HEIGHT 240
#define BG_BYTES (BG_WIDTH * BG_HEIGHT * 2) /* the JPEG array and the decoded image share this size */

extern void gfx_texture_cache_invalidate_range(const void* start, uint32_t size);

/* logical (N64 big-endian order) byte i of a natively-compiled u64 array starting at 8-aligned base */
static uint8_t swz_get(const void* base, uint32_t i) {
    return *(const uint8_t*)(((uintptr_t)base + i) ^ 7u);
}
static void swz_put(void* base, uint32_t i, uint8_t v) {
    *(uint8_t*)(((uintptr_t)base + i) ^ 7u) = v;
}

static const uint8_t kZigzag[64] = { 0,  1,  8,  16, 9,  2,  3,  10, 17, 24, 32, 25, 18, 11, 4,  5,
                                     12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13, 6,  7,  14, 21, 28,
                                     35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
                                     58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63 };

typedef struct {
    uint16_t code[256]; /* canonical codes, in symbol order */
    uint8_t len[256];
    uint8_t sym[256];
    int n;
} Huff;

typedef struct {
    int id, h, v, tq, td, ta, pred;
    int bw, bh;    /* size in blocks */
    float* plane;  /* bw*8 x bh*8 samples (level-shifted for Y) */
} Comp;

typedef struct {
    const uint8_t* d;
    int len, p, acc, n;
} Bits;

static int bits_bit(Bits* b) {
    if (b->n == 0) {
        int v = b->p < b->len ? b->d[b->p++] : 0;
        if (v == 0xFF && b->p < b->len && b->d[b->p] == 0) {
            b->p++;
        }
        b->acc = v;
        b->n = 8;
    }
    b->n--;
    return (b->acc >> b->n) & 1;
}

static int bits_get(Bits* b, int k) {
    int v = 0;
    while (k-- > 0) {
        v = (v << 1) | bits_bit(b);
    }
    return v;
}

static int huff_decode(Bits* b, const Huff* h) {
    int code = 0, len, i = 0;
    for (len = 1; len <= 16; len++) {
        code = (code << 1) | bits_bit(b);
        for (; i < h->n && h->len[i] == len; i++) {
            if (h->code[i] == code) {
                return h->sym[i];
            }
        }
    }
    return -1;
}

static int extend(int v, int t) {
    return v < (1 << (t - 1)) ? v - (1 << t) + 1 : v;
}

static float sIdctM[8][8]; /* [u][x] */

static void idct_init(void) {
    static int done;
    int u, x;
    if (done) return;
    for (u = 0; u < 8; u++) {
        for (x = 0; x < 8; x++) {
            sIdctM[u][x] = (float)((u == 0 ? 1.0 / sqrt(2.0) : 1.0) * cos((2 * x + 1) * u * M_PI / 16) / 2);
        }
    }
    done = 1;
}

/* in: dequantized natural-order coefficients; out: 8x8 samples into plane at (ox, oy) */
static void idct_block(const float in[64], float* plane, int stride, int ox, int oy, float level) {
    float tmp[8][8];
    int u, v, x, y;
    for (u = 0; u < 8; u++) { /* rows of coefficients (v) -> columns x */
        for (x = 0; x < 8; x++) {
            float s = 0;
            for (v = 0; v < 8; v++) s += in[u * 8 + v] * sIdctM[v][x];
            tmp[u][x] = s;
        }
    }
    for (y = 0; y < 8; y++) {
        for (x = 0; x < 8; x++) {
            float s = 0;
            for (u = 0; u < 8; u++) s += tmp[u][x] * sIdctM[u][y];
            plane[(oy + y) * stride + ox + x] = s + level;
        }
    }
}

/* baseline JPEG -> RGBA5551 (N64 model above). Returns 0 on unsupported/corrupt input. */
static int decode(const uint8_t* d, int len, uint16_t* out) {
    uint16_t qt[4][64];
    Huff ht[2][4];
    Comp comps[3];
    int nc = 0, w = 0, h = 0, restart = 0, p = 2, i, c, hmax = 1, vmax = 1, mcux, mcuy, mx, my, ok = 0;
    Bits bits;
    memset(ht, 0, sizeof(ht));
    memset(comps, 0, sizeof(comps));
    idct_init();
    while (p + 4 <= len) {
        int m, ln;
        const uint8_t* seg;
        if (d[p] != 0xFF) return 0;
        m = d[p + 1];
        ln = (d[p + 2] << 8) | d[p + 3];
        seg = d + p + 4;
        if (m == 0xDB) {
            for (i = 0; i < ln - 2;) {
                int pq = seg[i] >> 4, tq = seg[i] & 3, k;
                for (k = 0; k < 64; k++) {
                    qt[tq][kZigzag[k]] = pq ? (seg[i + 1 + 2 * k] << 8) | seg[i + 2 + 2 * k] : seg[i + 1 + k];
                }
                i += 1 + (pq ? 128 : 64);
            }
        } else if (m == 0xC0) {
            h = (seg[1] << 8) | seg[2];
            w = (seg[3] << 8) | seg[4];
            nc = seg[5];
            if (nc != 3) return 0;
            for (c = 0; c < nc; c++) {
                comps[c].id = seg[6 + 3 * c];
                comps[c].h = seg[7 + 3 * c] >> 4;
                comps[c].v = seg[7 + 3 * c] & 15;
                comps[c].tq = seg[8 + 3 * c] & 3;
                if (comps[c].h > hmax) hmax = comps[c].h;
                if (comps[c].v > vmax) vmax = comps[c].v;
            }
        } else if (m == 0xC4) {
            for (i = 0; i < ln - 2;) {
                int tc = seg[i] >> 4, th = seg[i] & 3, n = 0, l, k, code = 0;
                Huff* hh = &ht[tc & 1][th];
                const uint8_t* counts = seg + i + 1;
                for (l = 0; l < 16; l++) n += counts[l];
                hh->n = n;
                for (l = 1, k = 0; l <= 16; l++, code <<= 1) {
                    int j;
                    for (j = 0; j < counts[l - 1]; j++, k++) {
                        hh->code[k] = (uint16_t)code++;
                        hh->len[k] = (uint8_t)l;
                        hh->sym[k] = seg[i + 17 + k];
                    }
                }
                i += 17 + n;
            }
        } else if (m == 0xDD) {
            restart = (seg[0] << 8) | seg[1];
        } else if (m == 0xDA) {
            int ns = seg[0];
            for (i = 0; i < ns; i++) {
                for (c = 0; c < nc; c++) {
                    if (comps[c].id == seg[1 + 2 * i]) {
                        comps[c].td = seg[2 + 2 * i] >> 4;
                        comps[c].ta = seg[2 + 2 * i] & 15;
                    }
                }
            }
            bits.d = d + p + 2 + ln;
            bits.len = len - (p + 2 + ln);
            bits.p = bits.acc = bits.n = 0;
            ok = 1;
            break;
        } else if (m == 0xC1 || m == 0xC2 || m == 0xC3) {
            return 0; /* not baseline */
        }
        p += 2 + ln;
    }
    if (!ok || w <= 0 || h <= 0) return 0;
    mcux = (w + 8 * hmax - 1) / (8 * hmax);
    mcuy = (h + 8 * vmax - 1) / (8 * vmax);
    for (c = 0; c < nc; c++) {
        comps[c].bw = mcux * comps[c].h;
        comps[c].bh = mcuy * comps[c].v;
        comps[c].plane = (float*)malloc(sizeof(float) * comps[c].bw * 8 * comps[c].bh * 8);
        if (comps[c].plane == NULL) goto fail;
    }
    for (my = 0; my < mcuy; my++) {
        for (mx = 0; mx < mcux; mx++) {
            int k = my * mcux + mx;
            if (restart && k && k % restart == 0) {
                bits.n = 0; /* byte-align, skip RSTn */
                if (bits.p + 1 < bits.len && bits.d[bits.p] == 0xFF && bits.d[bits.p + 1] >= 0xD0 &&
                    bits.d[bits.p + 1] <= 0xD7) {
                    bits.p += 2;
                }
                for (c = 0; c < nc; c++) comps[c].pred = 0;
            }
            for (c = 0; c < nc; c++) {
                Comp* cp = &comps[c];
                int by, bx;
                for (by = 0; by < cp->v; by++) {
                    for (bx = 0; bx < cp->h; bx++) {
                        float blk[64];
                        int coef[64], t, j;
                        memset(coef, 0, sizeof(coef));
                        t = huff_decode(&bits, &ht[0][cp->td]);
                        if (t < 0) goto fail;
                        cp->pred += t ? extend(bits_get(&bits, t), t) : 0;
                        coef[0] = cp->pred;
                        for (j = 1; j < 64;) {
                            int rs = huff_decode(&bits, &ht[1][cp->ta]), r, s;
                            if (rs < 0) goto fail;
                            r = rs >> 4;
                            s = rs & 15;
                            if (s == 0) {
                                if (r == 15) {
                                    j += 16;
                                    continue;
                                }
                                break;
                            }
                            j += r;
                            if (j > 63) break;
                            coef[kZigzag[j]] = extend(bits_get(&bits, s), s);
                            j++;
                        }
                        for (j = 0; j < 64; j++) blk[j] = (float)(coef[j] * qt[cp->tq][j]);
                        idct_block(blk, cp->plane, cp->bw * 8, (mx * cp->h + bx) * 8, (my * cp->v + by) * 8,
                                   c == 0 ? 128.0f : 0.0f);
                    }
                }
            }
        }
    }
    {
        int x, y;
        for (y = 0; y < BG_HEIGHT; y++) {
            for (x = 0; x < BG_WIDTH; x++) {
                float Y, Cb, Cr, rgb[3];
                int k, v5[3];
                if (x >= w || y >= h) {
                    out[y * BG_WIDTH + x] = 1;
                    continue;
                }
                /* chroma replicated (nearest) */
                Y = comps[0].plane[(y * comps[0].v / vmax) * comps[0].bw * 8 + x * comps[0].h / hmax];
                Cb = comps[1].plane[(y * comps[1].v / vmax) * comps[1].bw * 8 + x * comps[1].h / hmax];
                Cr = comps[2].plane[(y * comps[2].v / vmax) * comps[2].bw * 8 + x * comps[2].h / hmax];
                rgb[0] = Y + 1.402f * Cr;
                rgb[1] = Y - 0.344136f * Cb - 0.714136f * Cr;
                rgb[2] = Y + 1.772f * Cb;
                for (k = 0; k < 3; k++) {
                    int iv = (int)floorf(rgb[k]); /* truncation, as measured */
                    v5[k] = (iv < 0 ? 0 : iv > 255 ? 255 : iv) >> 3;
                }
                out[y * BG_WIDTH + x] = (uint16_t)((v5[0] << 11) | (v5[1] << 6) | (v5[2] << 1) | 1);
            }
        }
    }
    for (c = 0; c < nc; c++) free(comps[c].plane);
    return 1;
fail:
    for (c = 0; c < nc; c++) free(comps[c].plane);
    return 0;
}

/* Returns 1 if a JPEG was decoded (in place), 0 if the data was not a JPEG (e.g. already decoded). */
int PortJpeg_DecodeRoomImageInPlace(void* data) {
    uint8_t* jpg;
    uint16_t* img;
    int x, ok;

    if (data == NULL || ((uintptr_t)data & 7) != 0) return 0;
    if (swz_get(data, 0) != 0xFF || swz_get(data, 1) != 0xD8 || swz_get(data, 2) != 0xFF || swz_get(data, 3) != 0xE0)
        return 0;

    jpg = (uint8_t*)malloc(BG_BYTES);
    img = (uint16_t*)malloc(BG_BYTES);
    if (jpg == NULL || img == NULL) {
        free(jpg);
        free(img);
        return 0;
    }
    for (x = 0; x < BG_BYTES; x++) jpg[x] = swz_get(data, (uint32_t)x);
    ok = decode(jpg, BG_BYTES, img);
    free(jpg);
    if (ok) {
        for (x = 0; x < BG_WIDTH * BG_HEIGHT; x++) {
            swz_put(data, (uint32_t)x * 2, (uint8_t)(img[x] >> 8));
            swz_put(data, (uint32_t)x * 2 + 1, (uint8_t)(img[x] & 0xFF));
        }
        gfx_texture_cache_invalidate_range(data, BG_BYTES);
    }
    free(img);
    return ok;
}

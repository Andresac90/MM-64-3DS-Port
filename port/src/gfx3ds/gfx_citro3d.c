#ifdef TARGET_N3DS

#include "gfx_3ds.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#ifndef _LANGUAGE_C
#define _LANGUAGE_C
#endif

#define u64 __u64
#define s64 __s64
#define u32 __u32
#define vu32 __vu32
#define vs32 __vs32
#define s32 __s32
#define u16 __u16
#define s16 __s16
#define u8 __u8
#define s8 __s8
#include <3ds/types.h>
#undef u64 
#undef s64 
#undef u32 
#undef vu32 
#undef vs32 
#undef s32 
#undef u16 
#undef s16 
#undef u8
#undef s8

#include <PR/gbi.h>

#include <3ds.h>
#include <citro3d.h>
#include "c3d_fast.h"
extern void Port3ds_CacheFlush(const void* p, u32 size); /* gfx_3ds.c */

#include "gfx_cc.h"
#include "gfx_rendering_api.h"

static DVLB_s* sVShaderDvlb;
static void* sVboBuffer;

extern const u8 shader_shbin[];
/* objcopy's _size symbol is ABSOLUTE (its address is the size): reading it as a u32 reads address
 * 0xB0, which Azahar allows but real hardware faults on. Use end - start. */
extern const u8 shader_shbin_end[];

/* PORT (2026-09-25): N64 color combiner -> PICA TEV compiler.
 *
 * gfx_pc.c hands over the complete combiner (gfx_cc.h: both cycles, both channels, every input of
 * (A-B)*C+D). Each cycle/channel is lowered to 1..3 TEV operations (lowerChannel), RGB and alpha run
 * side by side in the same stages, cycle 1 follows cycle 0, and fog (blender cycle 1: lerp to the
 * fog color by shade alpha) is a last stage. Inputs:
 *   TEXEL0/1 -> GPU_TEXTURE0/1, SHADE -> GPU_PRIMARY_COLOR (the vertex color is always shade),
 *   PRIM, ENV, their alpha broadcasts, LOD fractions, ONE, ZERO, fog color -> per-stage CONSTANT
 *   (each stage has its own RGB and alpha constant, filled per draw from GfxCombineConsts),
 *   a second constant in one stage -> the TEV buffer's initial color (GPU_PREVIOUS_BUFFER),
 *   COMBINED -> GPU_PREVIOUS in cycle 1's first stage, else the TEV buffer (written by the last
 *   cycle-0 stage; PICA: stage s reads the buffer as updated by stages <= s-2).
 * Keys this can't express exactly are compiled approximately and reported once (PortDbgX). */

enum { KC_NONE, KC_ZERO, KC_ONE, KC_PRIM, KC_PRIMA, KC_ENV, KC_ENVA, KC_LODF, KC_PRIMLODF, KC_FOG, KC_HALF };
enum { OK_CONST, OK_TEX0, OK_TEX1, OK_SHADE, OK_PREV, OK_COMB };

typedef struct {
    u8 kind, alpha, inv, k; /* alpha: use the source's alpha; inv: 1 - x; k: KC_* for OK_CONST */
} Opnd;

typedef struct {
    u8 func; /* GPU_COMBINEFUNC */
    Opnd src[3];
    u8 scale; /* 1: the stage's result times 2 (then clamped) */
} ChanOp;

typedef struct {
    u8 func[2];     /* [channel] GPU_COMBINEFUNC */
    u8 src[2][3];   /* GPU_TEVSRC */
    u8 op[2][3];    /* GPU_TEVOP_RGB / GPU_TEVOP_A */
    u8 konst[2];    /* KC_* in this stage's CONSTANT, per channel */
    u8 scale[2];    /* [channel] 1 = GPU_TEVSCALE_2 */
} TevStage;

#define TEV_STAGES 6

struct ShaderProgram {
    uint64_t shader_id0;
    uint32_t shader_id1;
    bool used_textures[2];
    u8 num_stages;
    TevStage stages[TEV_STAGES];
    u8 buf_konst[2];  /* KC_* held in the TEV buffer's initial color, per channel */
    u8 buf_update[2]; /* TEV buffer update masks (stages 0..3), per channel */
};

/* OoT uses many distinct combiners (the sm64 [64] pool overflowed); lookups only happen when the key changes */
#define SHADER_POOL_CAP 1024
static struct ShaderProgram sShaderProgramPool[SHADER_POOL_CAP];
static uint16_t sShaderProgramPoolSize;

static int sCurShader = 0;
static struct GfxCombineConsts sConsts;

static C3D_Tex sTexturePool[4096];
/* PORT (2026-10-01): in the GPU vertex path texture unit 0 samples in projective mode (s/q, t/q per pixel)
 * for the screen-linear shading mode (shader_gpu.v.pica); the shader sends q = 1 otherwise, which samples
 * exactly like 2D. Only unit 0's binding gets the projective type, through a copy of the texture: the pool
 * keeps 2D (a pool texture with the projective type bound to unit 1 - the two-texture sky blend - sampled
 * wrong in Azahar: grey title-screen sky, tools/statediff bootflow.py title). */
extern int gPortGpuVtx, gPortShadeLinear;
static C3D_Tex sTex0Proj;
static void texBindUnit(int unit, C3D_Tex* t) {
    if (unit == 0 && gPortGpuVtx && gPortShadeLinear) {
        sTex0Proj = *t;
        sTex0Proj.param = (t->param & ~GPU_TEXTURE_MODE(7)) | GPU_TEXTURE_MODE(GPU_TEX_PROJECTION);
        C3D_TexBind(0, &sTex0Proj);
    } else {
        C3D_TexBind(unit, t);
    }
}
static float sTexturePoolScaleS[4096];
static float sTexturePoolScaleT[4096];
static int sTextureIndex;
static int sCurTex = 0;

static int sTexUnits[2];

static bool gfx_citro3d_z_is_from_0_to_1(void)
{
    return true;
}

#define VTX_FLOATS 13 /* gfx_pc.c packs pos(4) uv0(2) uv1(2) shade(4) stereo(1); the VBO holds the same layout */

static bool sDepthTestOn = false;
static bool sDepthUpdateOn = true;
static bool sDepthDecal = false;

static bool sUseBlend;

static int sBufIdx = 0;

/* PORT (2026-09-30): 60 fps replay (gfx_pc.c "replay"). While recording, every state call and draw is
 * logged instead of drawn (state calls still run: the texture/shader bookkeeping and citro3d's state
 * are needed by the walk); gfx_citro3d_replay re-issues the log for each shown frame, after gfx_pc.c
 * rewrote the VBO positions. Texture uploads are not logged - the textures stay in their slots. */
enum { OP_SHADER, OP_CONSTS, OP_TEX, OP_SAMPLER, OP_DTEST, OP_DMASK, OP_DECAL, OP_VIEWPORT, OP_SCISSOR,
       OP_ALPHA, OP_DRAWID, OP_STEREO, OP_DRAWIDX, OP_GPUPAL, OP_GPUPARAM, OP_CULL, OP_RAWMODE, OP_RAWPARAM, OP_TEXPARAM };
typedef struct {
    u8 op;
    int v[4];
} RecOp;
#define REC_OPS 12288
#define REC_CONSTS 4096
static RecOp* sOps;
static struct GfxCombineConsts* sOpConsts;
static int sOpN, sConstN, sRec, sRecOverflow;
static int sRecDirect; /* the walk draws the update's first shown frame itself (replay by copy: captured as it goes) */
static void recOp(u8 op, int a, int b, int c, int d) {
    if (sOpN >= REC_OPS) {
        sRecOverflow = 1;
        return;
    }
    sOps[sOpN].op = op;
    sOps[sOpN].v[0] = a, sOps[sOpN].v[1] = b, sOps[sOpN].v[2] = c, sOps[sOpN].v[3] = d;
    sOpN++;
}

static void gfx_citro3d_unload_shader(struct ShaderProgram *old_prg) {

}

static Opnd opnd(u8 kind, u8 alpha, u8 k) {
    Opnd o = { kind, alpha, 0, k };
    return o;
}

static bool opndEq(Opnd a, Opnd b) {
    return a.kind == b.kind && a.alpha == b.alpha && a.inv == b.inv && a.k == b.k;
}

static bool isK(Opnd a, u8 k) {
    return a.kind == OK_CONST && !a.inv && a.k == k;
}

/* gfx_cc.h source -> operand, for channel ch (0 RGB, 1 alpha: every source is its alpha) */
static Opnd ccsOpnd(u8 s, int ch) {
    switch (s) {
        case CCS_1: return opnd(OK_CONST, ch, KC_ONE);
        case CCS_TEX0: return opnd(OK_TEX0, ch, 0);
        case CCS_TEX1: return opnd(OK_TEX1, ch, 0);
        case CCS_TEX0A: return opnd(OK_TEX0, 1, 0);
        case CCS_TEX1A: return opnd(OK_TEX1, 1, 0);
        case CCS_SHADE: return opnd(OK_SHADE, ch, 0);
        case CCS_SHADEA: return opnd(OK_SHADE, 1, 0);
        case CCS_PRIM: return opnd(OK_CONST, ch, KC_PRIM);
        case CCS_PRIMA: return opnd(OK_CONST, ch, ch ? KC_PRIM : KC_PRIMA);
        case CCS_ENV: return opnd(OK_CONST, ch, KC_ENV);
        case CCS_ENVA: return opnd(OK_CONST, ch, ch ? KC_ENV : KC_ENVA);
        case CCS_LODF: return opnd(OK_CONST, ch, KC_LODF);
        case CCS_PRIMLODF: return opnd(OK_CONST, ch, KC_PRIMLODF);
        case CCS_COMB: return opnd(OK_COMB, ch, 0);
        case CCS_COMBA: return opnd(OK_COMB, 1, 0);
    }
    return opnd(OK_CONST, ch, KC_ZERO);
}

static int emit(ChanOp* out, int n, u8 func, Opnd a, Opnd b, Opnd c) {
    out[n].func = func;
    out[n].scale = 0;
    out[n].src[0] = a;
    out[n].src[1] = b;
    out[n].src[2] = c;
    return n + 1;
}

/* (A-B)*C+D as TEV operations (PICA clamps every stage to 0..1). Returns the count (<= 3);
 * 0 = the channel passes the previous cycle through unchanged. */
static int lowerChannel(Opnd a, Opnd b, Opnd c, Opnd d, int ch, int cycle, ChanOp* out) {
    Opnd z = opnd(OK_CONST, ch, KC_ZERO), prev = opnd(OK_PREV, ch, 0);
    int n = 0;
    if (isK(c, KC_ZERO) || opndEq(a, b)) {
        if (cycle == 1 && d.kind == OK_COMB && d.alpha == ch) {
            return 0;
        }
        return emit(out, n, GPU_REPLACE, d, z, z);
    }
    if (isK(b, KC_ZERO)) {
        if (isK(d, KC_ZERO)) {
            if (isK(a, KC_ONE)) return emit(out, n, GPU_REPLACE, c, z, z);
            if (isK(c, KC_ONE)) return emit(out, n, GPU_REPLACE, a, z, z);
            return emit(out, n, GPU_MODULATE, a, c, z);
        }
        if (isK(a, KC_ONE)) return emit(out, n, GPU_ADD, c, d, z);
        if (isK(c, KC_ONE)) return emit(out, n, GPU_ADD, a, d, z);
        return emit(out, n, GPU_MULTIPLY_ADD, a, c, d);
    }
    if (opndEq(b, d)) {
        return emit(out, n, GPU_INTERPOLATE, a, b, c); /* a*c + b*(1-c) */
    }
    if (isK(a, KC_ONE) && b.kind != OK_PREV) { /* (1-B)*C+D */
        Opnd nb = b;
        nb.inv = 1;
        if (isK(d, KC_ZERO)) return emit(out, n, GPU_MODULATE, nb, c, z);
        if (isK(c, KC_ONE)) return emit(out, n, GPU_ADD, nb, d, z);
        return emit(out, n, GPU_MULTIPLY_ADD, nb, c, d);
    }
    if (isK(b, KC_ONE) && a.kind != OK_PREV) { /* (A-1)*C+D = D - (1-A)*C: no intermediate above 1 */
        Opnd na = a;
        na.inv = 1;
        n = emit(out, n, GPU_MODULATE, na, c, z);
        return emit(out, n, GPU_SUBTRACT, d, prev, z);
    }
    if (isK(d, KC_ZERO)) { /* max(A-B, 0)*C is exact: C >= 0 and the N64 clamps the result */
        n = emit(out, n, GPU_SUBTRACT, a, b, z);
        if (isK(c, KC_ONE)) return n;
        return emit(out, n, GPU_MODULATE, prev, c, z);
    }
    if (isK(c, KC_ONE)) {
        n = emit(out, n, GPU_ADD, a, d, z);
        return emit(out, n, GPU_SUBTRACT, prev, b, z);
    }
    if (opndEq(a, d) && a.kind != OK_PREV && b.kind != OK_PREV && c.kind != OK_PREV) {
        /* PORT (2026-10-05): (A-B)*C + A = 2A - (B*C + A*(1-C)) = 2 * (A - lerp(A, B, C) / 2): every intermediate stays in
         * 0..1 and the only clamp is the last stage's - the N64's own. The order below computed A*C + max(A - B*C, 0),
         * too bright wherever A < B*C: the blue warp's crystal, (TEXEL0 - PRIM) * PRIM_LOD_FRAC + TEXEL0 with the
         * fraction at 1 (= 2T - P on the N64), came out white instead of blue and hid Link (Chamber of the Sages,
         * tools/statediff fbdiff: 10% of the pixels off, user report). */
        n = emit(out, n, GPU_INTERPOLATE, b, a, c);                      /* B*C + A*(1-C) */
        n = emit(out, n, GPU_MODULATE, prev, opnd(OK_CONST, ch, KC_HALF), z); /* ... / 2 */
        n = emit(out, n, GPU_SUBTRACT, a, prev, z);                      /* A - ..., times 2 */
        out[n - 1].scale = 1;
        return n;
    }
    if (a.kind != OK_PREV && b.kind != OK_PREV && c.kind != OK_PREV && d.kind != OK_PREV) {
        /* PORT (2026-09-30): A*C + (D - B*C). Every PICA stage clamps to 0..1; the previous order
         * interp(A,B,C) + D - B saturated at the "+ D" whenever the true intermediate passed 1, then the
         * "- B" made it far too dark: the N64 boot logo's "(TEXEL1 - PRIM) * ENV_A + TEXEL0" text came out
         * dark blue instead of cyan-white (tools/statediff fbdiff, boot@60). This order errs only when
         * D < B*C (a bias larger than the base), which "base texture + shine/tint" combines avoid. */
        n = emit(out, n, GPU_MODULATE, b, c, z);         /* B*C */
        n = emit(out, n, GPU_SUBTRACT, d, prev, z);      /* D - B*C */
        return emit(out, n, GPU_MULTIPLY_ADD, a, c, prev); /* A*C + ... */
    }
    n = emit(out, n, GPU_INTERPOLATE, a, b, c); /* (A-B)*C + B */
    n = emit(out, n, GPU_ADD, prev, d, z);      /* ... + D */
    return emit(out, n, GPU_SUBTRACT, prev, b, z); /* ... - B */
}

/* compile state */
typedef struct {
    struct ShaderProgram* prg;
    bool exact;
    int cyc0Last;         /* last cycle-0 stage */
    bool prevIsComb[2];   /* per component (0 rgb, 1 alpha): PREVIOUS still holds cycle 0's output */
    bool bufComb[2];      /* buffer component carries cycle 0's output (updated by stage cyc0Last) */
    int bufKLast[2];      /* last stage reading the buffer's initial color */
} TevCompile;

static int arityOf(u8 func) {
    return func == GPU_REPLACE ? 1 : (func == GPU_INTERPOLATE || func == GPU_MULTIPLY_ADD) ? 3 : 2;
}

/* buffer component c as constant k at stage s: its initial color is visible until the cycle-0
 * output lands in it (PICA: stage s reads buffer updates of stages <= s-2). PORT (2026-09-29): and
 * only FROM STAGE 1: stage 0's PREVIOUS_BUFFER reads 0 (the buffer color register feeds the buffer
 * that stage 1 sees; Citra/Azahar model it the same way). Using it at stage 0 turned the constant
 * black, e.g. the hearts' (PRIM - ENV) * TEXEL0 + ENV drew a black outline instead of ENV. */
static bool bufKonstOk(TevCompile* tc, int c, u8 k, int s) {
    struct ShaderProgram* prg = tc->prg;
    if (s == 0) {
        return false;
    }
    if (prg->buf_konst[c] != KC_NONE && prg->buf_konst[c] != k) {
        return false;
    }
    if (tc->bufComb[c] && s > tc->cyc0Last + 1) {
        return false;
    }
    return true;
}

/* place op for channel ch into stage s; returns false (placing nothing) if it needs a second
 * constant with nowhere to put it: *needK then names that constant */
static bool placeOp(TevCompile* tc, TevStage* st, int s, int ch, int cycle, const ChanOp* op, u8* needK) {
    struct ShaderProgram* prg = tc->prg;
    int arity = arityOf(op->func);
    u8 konst = st->konst[ch], bufK = prg->buf_konst[ch];
    u8 src[3], ops[3];
    bool useBufK = false, useBufComb[2] = { false, false };
    for (int i = 0; i < 3; i++) {
        Opnd o = op->src[i];
        bool alpha = ch == 1 || o.alpha;
        if (i >= arity) { /* unused by the function: claim nothing */
            src[i] = GPU_PREVIOUS;
            ops[i] = ch ? GPU_TEVOP_A_SRC_ALPHA : GPU_TEVOP_RGB_SRC_COLOR;
            continue;
        }
        if (o.kind == OK_COMB && cycle == 0) { /* cleared by gfx_cc_key; keep safe */
            o.kind = OK_CONST;
            o.k = KC_ZERO;
        }
        switch (o.kind) {
            case OK_TEX0: src[i] = GPU_TEXTURE0; break;
            case OK_TEX1: src[i] = GPU_TEXTURE1; break;
            case OK_SHADE: src[i] = GPU_PRIMARY_COLOR; break;
            case OK_PREV: src[i] = GPU_PREVIOUS; break;
            case OK_COMB:
                if (tc->prevIsComb[alpha ? 1 : 0]) {
                    src[i] = GPU_PREVIOUS;
                } else {
                    src[i] = GPU_PREVIOUS_BUFFER;
                    useBufComb[alpha ? 1 : 0] = true;
                }
                break;
            case OK_CONST:
                alpha = ch == 1; /* constants carry per-channel values (KC_PRIMA etc.) */
                if (konst == KC_NONE || konst == o.k) {
                    konst = o.k;
                    src[i] = GPU_CONSTANT;
                } else if ((bufK == KC_NONE || bufK == o.k) && bufKonstOk(tc, ch, o.k, s)) {
                    bufK = o.k;
                    useBufK = true;
                    src[i] = GPU_PREVIOUS_BUFFER;
                } else {
                    *needK = o.k;
                    return false;
                }
                break;
        }
        if (ch == 0) {
            ops[i] = alpha ? (o.inv ? GPU_TEVOP_RGB_ONE_MINUS_SRC_ALPHA : GPU_TEVOP_RGB_SRC_ALPHA)
                           : (o.inv ? GPU_TEVOP_RGB_ONE_MINUS_SRC_COLOR : GPU_TEVOP_RGB_SRC_COLOR);
        } else {
            ops[i] = o.inv ? GPU_TEVOP_A_ONE_MINUS_SRC_ALPHA : GPU_TEVOP_A_SRC_ALPHA;
        }
    }
    for (int c = 0; c < 2; c++) {
        if (useBufComb[c]) {
            if (tc->cyc0Last < 0 || tc->cyc0Last > 3 || s < tc->cyc0Last + 2 || tc->bufKLast[c] > tc->cyc0Last + 1) {
                tc->exact = false;
            } else {
                tc->bufComb[c] = true;
                prg->buf_update[c] = 1 << tc->cyc0Last;
            }
        }
    }
    if (useBufK) {
        prg->buf_konst[ch] = bufK;
        if (s > tc->bufKLast[ch]) {
            tc->bufKLast[ch] = s;
        }
    }
    st->konst[ch] = konst;
    st->func[ch] = op->func;
    st->scale[ch] = op->scale;
    for (int i = 0; i < 3; i++) {
        st->src[ch][i] = src[i];
        st->op[ch][i] = ops[i];
    }
    return true;
}

static ChanOp passOp(int ch) {
    ChanOp op;
    op.func = GPU_REPLACE;
    op.scale = 0;
    op.src[0] = opnd(OK_PREV, ch, 0);
    op.src[1] = op.src[2] = opnd(OK_CONST, ch, KC_ZERO);
    return op;
}

/* compile a key; returns false if some input had to be approximated */
static bool compileTev(struct ShaderProgram* prg) {
    uint64_t id0 = prg->shader_id0;
    TevCompile tc;
    int ns = 0;
    memset(&tc, 0, sizeof(tc));
    tc.prg = prg;
    tc.exact = true;
    tc.cyc0Last = -1;
    tc.bufKLast[0] = tc.bufKLast[1] = -1;
    memset(prg->stages, 0, sizeof(prg->stages));
    prg->buf_konst[0] = prg->buf_konst[1] = KC_NONE;
    prg->buf_update[0] = prg->buf_update[1] = 0;

    for (int cycle = 0; cycle < 2; cycle++) {
        ChanOp ops[2][4];
        int nops[2], next[2] = { 0, 0 };
        for (int ch = 0; ch < 2; ch++) {
            nops[ch] = lowerChannel(ccsOpnd(CC_SRC(id0, cycle, ch, 0), ch), ccsOpnd(CC_SRC(id0, cycle, ch, 1), ch),
                                    ccsOpnd(CC_SRC(id0, cycle, ch, 2), ch), ccsOpnd(CC_SRC(id0, cycle, ch, 3), ch), ch,
                                    cycle, ops[ch]);
        }
        if (cycle == 1) {
            tc.prevIsComb[0] = tc.prevIsComb[1] = true;
        }
        while (next[0] < nops[0] || next[1] < nops[1]) {
            if (ns >= TEV_STAGES) {
                tc.exact = false;
                goto done;
            }
            TevStage* st = &prg->stages[ns];
            bool changed[2] = { false, false };
            for (int ch = 0; ch < 2; ch++) {
                u8 needK = KC_NONE;
                if (next[ch] < nops[ch]) {
                    ChanOp* op = &ops[ch][next[ch]];
                    if (placeOp(&tc, st, ns, ch, cycle, op, &needK)) {
                        next[ch]++;
                        changed[ch] = true;
                        continue;
                    }
                    /* two constants: preload the second into PREVIOUS with this stage, if the op
                     * doesn't need PREVIOUS itself (COMBINED then comes from the buffer) */
                    bool usesPrev = false;
                    for (int i = 0; i < arityOf(op->func); i++) {
                        usesPrev |= op->src[i].kind == OK_PREV;
                    }
                    ChanOp pre;
                    pre.func = GPU_REPLACE;
                    pre.scale = 0;
                    pre.src[0] = opnd(OK_CONST, ch, needK);
                    pre.src[1] = pre.src[2] = pre.src[0];
                    if (!usesPrev && placeOp(&tc, st, ns, ch, cycle, &pre, &needK)) {
                        for (int i = 0; i < 3; i++) {
                            if (op->src[i].kind == OK_CONST && op->src[i].k == pre.src[0].k) {
                                u8 inv = op->src[i].inv;
                                op->src[i] = opnd(OK_PREV, ch, 0);
                                op->src[i].inv = inv;
                            }
                        }
                        changed[ch] = true;
                        continue;
                    }
                    tc.exact = false; /* approximate: drop the op's second constant */
                    for (int i = 0; i < 3; i++) {
                        if (op->src[i].kind == OK_CONST && op->src[i].k == needK) {
                            op->src[i].k = st->konst[ch];
                        }
                    }
                    if (placeOp(&tc, st, ns, ch, cycle, op, &needK)) {
                        next[ch]++;
                        changed[ch] = true;
                        continue;
                    }
                }
                ChanOp pass = passOp(ch);
                if (ns == 0) { /* nothing to pass through at stage 0 */
                    pass.src[0] = opnd(OK_CONST, ch, ch ? KC_ONE : KC_ZERO);
                }
                placeOp(&tc, st, ns, ch, cycle, &pass, &needK);
            }
            for (int ch = 0; ch < 2; ch++) {
                if (changed[ch]) {
                    tc.prevIsComb[ch] = false;
                }
            }
            ns++;
        }
        if (cycle == 0) {
            tc.cyc0Last = ns - 1;
        }
    }
done:
    if (prg->shader_id1 & SHADER_OPT_FOG) {
        if (ns < TEV_STAGES) {
            TevStage* st = &prg->stages[ns++];
            st->func[0] = GPU_INTERPOLATE; /* fog * f + prev * (1 - f), f = shade alpha */
            st->src[0][0] = GPU_CONSTANT;
            st->src[0][1] = GPU_PREVIOUS;
            st->src[0][2] = GPU_PRIMARY_COLOR;
            st->op[0][0] = GPU_TEVOP_RGB_SRC_COLOR;
            st->op[0][1] = GPU_TEVOP_RGB_SRC_COLOR;
            st->op[0][2] = GPU_TEVOP_RGB_SRC_ALPHA;
            st->konst[0] = KC_FOG;
            st->func[1] = GPU_REPLACE;
            st->src[1][0] = st->src[1][1] = st->src[1][2] = GPU_PREVIOUS;
            st->op[1][0] = st->op[1][1] = st->op[1][2] = GPU_TEVOP_A_SRC_ALPHA;
        } else {
            tc.exact = false;
        }
    }
    prg->num_stages = ns;
    return tc.exact;
}

/* value of constant k for channel ch, as 0..255 */
static u8 konstVal(u8 k, int ch, int comp) {
    const struct GfxCombineConsts* c = &sConsts;
    switch (k) {
        case KC_ONE: return 255;
        case KC_PRIM: return c->prim[ch ? 3 : comp];
        case KC_PRIMA: return c->prim[3];
        case KC_ENV: return c->env[ch ? 3 : comp];
        case KC_ENVA: return c->env[3];
        case KC_LODF: return c->lod_frac;
        case KC_PRIMLODF: return c->prim_lod_frac;
        case KC_FOG: return c->fog[ch ? 3 : comp];
        case KC_HALF: return 128;
    }
    return 0;
}

static u32 konstColor(u8 krgb, u8 ka) { /* PICA constant: 0xAABBGGRR */
    return konstVal(krgb, 0, 0) | (konstVal(krgb, 0, 1) << 8) | (konstVal(krgb, 0, 2) << 16) |
           ((u32)konstVal(ka, 1, 3) << 24);
}

/* program the TEV for the current shader and constants */
u32 gPortC3dCalls[6]; /* per report: shader loads, const sets, tex binds (same tex), tex binds, samplers, uploads */
static void updateShader(void)
{
    gPortC3dCalls[0]++;
    if (sCurShader < 0 || sCurShader >= SHADER_POOL_CAP) {
        sCurShader = 0;
    }
    const struct ShaderProgram* prg = &sShaderProgramPool[sCurShader];
    for (int s = 0; s < TEV_STAGES; s++) {
        C3D_TexEnv* e = C3D_GetTexEnv(s);
        C3D_TexEnvInit(e);
        if (s >= prg->num_stages) {
            continue;
        }
        const TevStage* st = &prg->stages[s];
        C3D_TexEnvSrc(e, C3D_RGB, st->src[0][0], st->src[0][1], st->src[0][2]);
        C3D_TexEnvSrc(e, C3D_Alpha, st->src[1][0], st->src[1][1], st->src[1][2]);
        C3D_TexEnvOpRgb(e, st->op[0][0], st->op[0][1], st->op[0][2]);
        C3D_TexEnvOpAlpha(e, st->op[1][0], st->op[1][1], st->op[1][2]);
        C3D_TexEnvFunc(e, C3D_RGB, st->func[0]);
        C3D_TexEnvFunc(e, C3D_Alpha, st->func[1]);
        if (st->scale[0] | st->scale[1]) {
            C3D_TexEnvScale(e, C3D_RGB, st->scale[0] ? GPU_TEVSCALE_2 : GPU_TEVSCALE_1);
            C3D_TexEnvScale(e, C3D_Alpha, st->scale[1] ? GPU_TEVSCALE_2 : GPU_TEVSCALE_1);
        }
        C3D_TexEnvColor(e, konstColor(st->konst[0], st->konst[1]));
    }
    C3D_TexEnvBufUpdate(C3D_RGB, prg->buf_update[0]);
    C3D_TexEnvBufUpdate(C3D_Alpha, prg->buf_update[1]);
    C3D_TexEnvBufColor(konstColor(prg->buf_konst[0], prg->buf_konst[1]));
}

static void gfx_citro3d_load_shader(struct ShaderProgram *new_prg) {
    sCurShader = new_prg - sShaderProgramPool;
    if (sRec) recOp(OP_SHADER, sCurShader, 0, 0, 0);
    updateShader();
}

static void gfx_citro3d_set_combine_consts(const struct GfxCombineConsts *consts) {
    /* only the constant colors change: rewrite them, not the whole TEV program */
    const struct ShaderProgram* prg = &sShaderProgramPool[sCurShader];
    int s;
    gPortC3dCalls[1]++;
    sConsts = *consts;
    if (sRec) {
        if (sConstN < REC_CONSTS) {
            sOpConsts[sConstN] = *consts;
            recOp(OP_CONSTS, sConstN++, 0, 0, 0);
        } else {
            sRecOverflow = 1;
        }
    }
    for (s = 0; s < prg->num_stages; s++) {
        if (prg->stages[s].konst[0] != KC_NONE || prg->stages[s].konst[1] != KC_NONE) {
            C3D_TexEnvColor(C3D_GetTexEnv(s), konstColor(prg->stages[s].konst[0], prg->stages[s].konst[1]));
        }
    }
    if (prg->buf_konst[0] != KC_NONE || prg->buf_konst[1] != KC_NONE) {
        C3D_TexEnvBufColor(konstColor(prg->buf_konst[0], prg->buf_konst[1]));
    }
}

static struct ShaderProgram *gfx_citro3d_create_and_load_new_shader(uint64_t shader_id0, uint32_t shader_id1) {
    struct ShaderProgram *prg;
    if (sShaderProgramPoolSize >= SHADER_POOL_CAP) {
        /* pool full: recycle the last slot (wrong-but-safe) */
        prg = &sShaderProgramPool[SHADER_POOL_CAP - 1];
    } else {
        prg = &sShaderProgramPool[sShaderProgramPoolSize++];
    }
    prg->shader_id0 = shader_id0;
    prg->shader_id1 = shader_id1;
    prg->used_textures[0] = (shader_id1 & SHADER_OPT_TEX0) != 0;
    prg->used_textures[1] = (shader_id1 & SHADER_OPT_TEX1) != 0;
    if (!compileTev(prg)) {
        static int sReported;
        if (sReported < 16) {
            extern void PortDbgX(const char* label, unsigned val);
            sReported++;
            PortDbgX("TEV approx key hi", (unsigned)(shader_id0 >> 32));
            PortDbgX("TEV approx key lo", (unsigned)shader_id0);
            PortDbgX("TEV approx opts", (unsigned)shader_id1);
        }
    }
    gfx_citro3d_load_shader(prg);
    return prg;
}

static struct ShaderProgram *gfx_citro3d_lookup_shader(uint64_t shader_id0, uint32_t shader_id1) {
    for (size_t i = 0; i < sShaderProgramPoolSize; i++) {
        if (sShaderProgramPool[i].shader_id0 == shader_id0 && sShaderProgramPool[i].shader_id1 == shader_id1) {
            return &sShaderProgramPool[i];
        }
    }
    return NULL;
}

static void gfx_citro3d_shader_get_info(struct ShaderProgram *prg, uint8_t *num_inputs, bool used_textures[2]) {
    *num_inputs = 1;
    used_textures[0] = prg->used_textures[0];
    used_textures[1] = prg->used_textures[1];
}

static u32 gfx_citro3d_new_texture(void) {
    if(sTextureIndex == 4096)
    {
        printf("Out of texures!");
        return 0;
    }
    return sTextureIndex++;
}

/* PORT PERF (2026-10-01): a unit already bound to this texture is not re-bound - C3D_TexBind marks the
 * unit dirty and citro3d re-sends its texture registers with the next draw (~32 of ~168 binds per frame
 * were the same texture again). An upload into a bound texture or a sampler change re-binds explicitly. */
static bool sTexBoundOk[2];

static void gfx_citro3d_select_texture(int tile, u32 texture_id) {
    gPortC3dCalls[sTexUnits[tile] == (int)texture_id ? 2 : 3]++;
    if (sRec) recOp(OP_TEX, tile, (int)texture_id, 0, 0);
    if (!(sTexBoundOk[tile] && sTexUnits[tile] == (int)texture_id)) {
        texBindUnit(tile, &sTexturePool[texture_id]);
        sTexBoundOk[tile] = true;
    }
    sCurTex = texture_id;
    sTexUnits[tile] = texture_id;
}

static u32 sTexBuf[16 * 1024] __attribute__((aligned(32)));

static int sTileOrder[] =
{
    0,  1,   4,  5,
    2,  3,   6,  7,

    8,  9,  12, 13,  
    10, 11,  14, 15
};

static void performTexSwizzle(const u8* src, u32* dst, u32 w, u32 h)
{
    int offs = 0;
    for(int y = 0; y < h; y += 8)
    {
        for(int x = 0; x < w; x += 8)
        {
            for (int i = 0; i < 64; i++)
            {
                int x2 = i & 7;
                int y2 = i >> 3;
                int pos = sTileOrder[(x2 & 3) + ((y2 & 3) << 2)] + ((x2 >> 2) << 4) + ((y2 >> 2) << 5);
                u32 c = ((const u32*)src)[(y + y2) * w + x + x2];
                dst[offs + pos] = ((c & 0xFF) << 24) | (((c >> 8) & 0xFF) << 16) | (((c >> 16) & 0xFF) << 8) | (c >> 24);
            }
            dst += 64;
        }
    }
}

static void gfx_citro3d_upload_texture(uint8_t *rgba32_buf, int width, int height) {
    gPortC3dCalls[5]++;
    if(width < 8 || height < 8 || (width & (width - 1)) || (height & (height - 1)))
    {
        int newWidth = width < 8 ? 8 : (1 << (32 - __builtin_clz(width - 1)));
        int newHeight = height < 8 ? 8 : (1 << (32 - __builtin_clz(height - 1)));
        if(newWidth * newHeight * 4 > sizeof(sTexBuf))
        {
            printf("Tex buffer overflow!\n");
            return;
        }
        int offs = 0;
        for(int y = 0; y < newHeight; y += 8)
        {
            for(int x = 0; x < newWidth; x += 8)
            {
                for (int i = 0; i < 64; i++)
                {
                    int x2 = i % 8;
                    int y2 = i / 8;

                    int realX = x + x2;
                    if(realX >= width)
                        realX -= width;

                    int realY = y + y2;
                    if(realY >= height)
                        realY -= height;

                    int pos = sTileOrder[x2 % 4 + y2 % 4 * 4] + 16 * (x2 / 4) + 32 * (y2 / 4);
                    u32 c = ((u32*)rgba32_buf)[realY * width + realX];
                    ((u32*)sTexBuf)[offs + pos] = ((c & 0xFF) << 24) | (((c >> 8) & 0xFF) << 16) | (((c >> 16) & 0xFF) << 8) | (c >> 24);
                }
                offs += 64;
            }
        }
        sTexturePoolScaleS[sCurTex] = width / (float)newWidth;
        sTexturePoolScaleT[sCurTex] = height / (float)newHeight;
        width = newWidth;
        height = newHeight;
    }
    else
    {
        /* PORT (MM bring-up): the same bound as the branch above. Unchecked, a power-of-two texture over 16K texels
         * (MM: larger images than OoT's) overran sTexBuf and corrupted the statics after it - texture records the
         * GPU then read garbage addresses from (Azahar: "Read from unknown GPU address"). */
        if (width * height * 4 > (int)sizeof(sTexBuf)) {
            static int sLogged;
            if (sLogged++ < 4) {
                extern void PortDbgX(const char* label, unsigned val);
                PortDbgX("[gfx] texture too large for sTexBuf: w", width);
                PortDbgX("  h", height);
            }
            return;
        }
        sTexturePoolScaleS[sCurTex] = 1.f;
        sTexturePoolScaleT[sCurTex] = 1.f;
        performTexSwizzle(rgba32_buf, sTexBuf, width, height);
    }
    /* Cache slots get recycled (gfx_pc pool wraps at 512); C3D_TexInit on an
     * already-initialized tex leaks its previous linear buffer. Free it first,
     * or reuse the buffer when the dimensions match. */
    if (sTexturePool[sCurTex].data != NULL &&
        (sTexturePool[sCurTex].width != width || sTexturePool[sCurTex].height != height)) {
        C3D_TexDelete(&sTexturePool[sCurTex]);
        sTexturePool[sCurTex].data = NULL;
    }
    if (sTexturePool[sCurTex].data == NULL) {
        C3D_TexInit(&sTexturePool[sCurTex], width, height, GPU_RGBA8);
    }
#ifdef PORT_TEX_DEBUG /* (debug: textures that are large, failed, or end past the linear heap) */
    {
        extern u32 __ctru_linear_heap, __ctru_linear_heap_size;
        uintptr_t d = (uintptr_t)sTexturePool[sCurTex].data;
        uintptr_t end = d + (uintptr_t)width * height * 4;
        if (d == 0 || width * height > 256 * 256 || end > __ctru_linear_heap + __ctru_linear_heap_size) {
            extern void PortDbgX(const char* label, unsigned val);
            PortDbgX("[texdbg] tex w", width); PortDbgX("  h", height); PortDbgX("  data", (unsigned)d);
            PortDbgX("  slot", sCurTex);
        }
    }
#endif
    C3D_TexUpload(&sTexturePool[sCurTex], sTexBuf);
    C3D_TexFlush(&sTexturePool[sCurTex]);
    {
        int u;
        for (u = 0; u < 2; u++) { /* bound here: its registers (address, size) must be re-sent */
            if (sTexUnits[u] == sCurTex) {
                texBindUnit(u, &sTexturePool[sCurTex]);
                sTexBoundOk[u] = true;
            }
        }
    }
}

static uint32_t gfx_cm_to_opengl(uint32_t val) {
    if (val & G_TX_CLAMP)
         return GPU_CLAMP_TO_EDGE;
    return (val & G_TX_MIRROR) ? GPU_MIRRORED_REPEAT : GPU_REPEAT;
}

static void gfx_citro3d_set_sampler_parameters(int tile, bool linear_filter, uint32_t cms, uint32_t cmt) {
    gPortC3dCalls[4]++;
    if (sRec) recOp(OP_SAMPLER, tile, linear_filter, (int)cms, (int)cmt);
    C3D_TexSetFilter(&sTexturePool[sTexUnits[tile]], linear_filter ? GPU_LINEAR : GPU_NEAREST, linear_filter ? GPU_LINEAR : GPU_NEAREST);
    C3D_TexSetWrap(&sTexturePool[sTexUnits[tile]], gfx_cm_to_opengl(cms), gfx_cm_to_opengl(cmt));
    texBindUnit(tile, &sTexturePool[sTexUnits[tile]]); /* the parameters are sent with the binding */
    sTexBoundOk[tile] = true;
}

static void updateDepth()
{
    C3D_DepthTest(sDepthTestOn, GPU_LEQUAL, sDepthUpdateOn ? GPU_WRITE_ALL : GPU_WRITE_COLOR);
    C3D_DepthMap(true, sDepthDecal ? -1 : -1, sDepthDecal ? -0.001f : 0);
}

static void gfx_citro3d_set_depth_test(bool depth_test) {
    if (sRec) recOp(OP_DTEST, depth_test, 0, 0, 0);
    sDepthTestOn = depth_test;
    updateDepth();
}

static void gfx_citro3d_set_depth_mask(bool z_upd) {
    if (sRec) recOp(OP_DMASK, z_upd, 0, 0, 0);
    sDepthUpdateOn = z_upd;
    updateDepth();
}

static void gfx_citro3d_set_zmode_decal(bool zmode_decal) {
    if (sRec) recOp(OP_DECAL, zmode_decal, 0, 0, 0);
    sDepthDecal = zmode_decal;
    updateDepth();
}

/* The top-screen target is PORTRAIT (240x400, rotated): x/y and width/height swap (sm64 3DS port),
 * and the vertex setup writes (y, -x), so N64 x runs down the target's second axis: a rectangle at x
 * sits at 400 - (x + width) there. PORT (2026-09-25): without that mirror every sub-viewport/scissor
 * was mirrored horizontally (full-screen ones are symmetric, so only small ones showed it: the A
 * button drawn over the hearts, found by tools/statediff fbdiff). */
#define TOP_W 400

extern int Port3ds_IsOffscreen(void);

/* stereo (gfx_3ds.c): the last top-screen viewport/scissor as given (x, y, w, h), re-issued per eye in
 * normal-mode target coordinates; kept in every mode so a switch into stereo starts with the right ones */
static int sVp[4] = { 0, 0, 400, 240 }, sSc[4] = { 0, 0, 400, 240 };

static void gfx_citro3d_set_viewport(int x, int y, int width, int height) {
    if (sRec) recOp(OP_VIEWPORT, x, y, width, height);
    if (Port3ds_IsOffscreen()) { /* 1x 320x240-space off-screen target (gfx_3ds.c) */
        C3D_SetViewport(y, 320 - (x + width), height, width);
        return;
    }
    sVp[0] = x, sVp[1] = y, sVp[2] = width, sVp[3] = height;
    x = TOP_W - (x + width);
    if (gGfx3DSMode == GFX_3DS_MODE_STEREO) {
        C3D_SetViewport(y, x, height, width);
    } else if (gGfx3DSMode == GFX_3DS_MODE_AA_22 || gGfx3DSMode == GFX_3DS_MODE_WIDE_AA_12)
        C3D_SetViewport(y * 2, x * 2, height * 2, width * 2);
    else if (gGfx3DSMode == GFX_3DS_MODE_WIDE)
        C3D_SetViewport(y, x * 2, height, width * 2);
    else
        C3D_SetViewport(y, x, height, width);
}

static void gfx_citro3d_set_scissor(int x, int y, int width, int height)
{
    if (sRec) recOp(OP_SCISSOR, x, y, width, height);
    if (Port3ds_IsOffscreen()) {
        C3D_SetScissor(GPU_SCISSOR_NORMAL, y, 320 - (x + width), y + height, 320 - x);
        return;
    }
    sSc[0] = x, sSc[1] = y, sSc[2] = width, sSc[3] = height;
    x = TOP_W - (x + width);
    if (gGfx3DSMode == GFX_3DS_MODE_STEREO) {
        C3D_SetScissor(GPU_SCISSOR_NORMAL, y, x, y + height, x + width);
    } else if (gGfx3DSMode == GFX_3DS_MODE_NORMAL)
        C3D_SetScissor(GPU_SCISSOR_NORMAL, y, x, y + height, x + width);
    else if (gGfx3DSMode == GFX_3DS_MODE_AA_22 || gGfx3DSMode == GFX_3DS_MODE_WIDE_AA_12)
        C3D_SetScissor(GPU_SCISSOR_NORMAL, y * 2, x * 2, (y + height) * 2, (x + width) * 2);
    else if (gGfx3DSMode == GFX_3DS_MODE_WIDE)
        C3D_SetScissor(GPU_SCISSOR_NORMAL, y, x * 2, y + height, (x + width) * 2);
}

/* PORT (2026-09-24): alpha test per draw, libultraship semantics. Opaque: off. Cutout without
 * blending (texture edge): keep alpha > 0.19 and draw opaque. Cutout with blending (threshold):
 * discard alpha < 8/256. Other translucent draws: discard alpha 0 only (keeps depth clean). */
static void applyAlphaTest(void)
{
    /* PORT PERF (2026-10-01): only when it changes - every C3D_ state call marks state dirty and citro3d
     * re-sends those GPU registers with the next draw (this ran for every draw, twice in 3D) */
    static int sLast = -1;
    u32 id = sShaderProgramPool[sCurShader].shader_id1;
    int want = !(id & SHADER_OPT_ALPHA) ? 0 : (id & SHADER_OPT_TEXTURE_EDGE) ? (sUseBlend ? 1 : 2) : 3;
    if (want == sLast)
        return;
    sLast = want;
    if (want == 0)
        C3D_AlphaTest(false, GPU_ALWAYS, 0);
    else if (want != 3)
        C3D_AlphaTest(true, GPU_GREATER, want == 1 ? 7 : 48);
    else
        C3D_AlphaTest(true, GPU_GREATER, 0);
}

static void applyBlend()
{
    if (sUseBlend)
        C3D_AlphaBlend(GPU_BLEND_ADD, GPU_BLEND_ADD, GPU_SRC_ALPHA, GPU_ONE_MINUS_SRC_ALPHA, GPU_SRC_ALPHA, GPU_ONE_MINUS_SRC_ALPHA);
    else
        C3D_AlphaBlend(GPU_BLEND_ADD, GPU_BLEND_ADD, GPU_ONE, GPU_ZERO, GPU_ONE, GPU_ZERO);
}

static void gfx_citro3d_set_use_alpha(bool use_alpha)
{
    if (sRec) recOp(OP_ALPHA, use_alpha, 0, 0, 0);
    sUseBlend = use_alpha;
    applyBlend();
}

/* tools/statediff per-draw attribution: gfx_pc.c sets an id per flush while the color readback runs;
 * every fragment that passes the depth test writes it into the (otherwise unused) stencil buffer. */
static int sDrawId;

void gfx_citro3d_set_draw_id(int id) {
    if (sRec) recOp(OP_DRAWID, id, 0, 0, 0);
    sDrawId = id;
}

static void applyDrawId(void) {
    static int sLast = -1;
    if (sDrawId == sLast) {
        return;
    }
    sLast = sDrawId;
    if (sDrawId != 0) {
        C3D_StencilTest(true, GPU_ALWAYS, sDrawId, 0xFF, 0xFF);
        C3D_StencilOp(GPU_STENCIL_KEEP, GPU_STENCIL_KEEP, GPU_STENCIL_REPLACE);
    } else {
        C3D_StencilTest(false, GPU_ALWAYS, 0, 0xFF, 0x00);
    }
}

/* eye uniform (shader.v.pica): (shift at infinity, convergence w, pop-out limit, constant shift).
 * PORT (2026-09-30, v2 after hardware feedback "not deep enough, Link pops out in 2D rooms"): the
 * convergence follows Link's distance (gPortStereoFocusW, smoothed in gfx_3ds.c), so Link sits at the
 * screen plane, the world recedes behind him and closer things come forward a little (limit below);
 * the 2D HUD (w < 1.5) stays at the screen. Before: a fixed convergence of 80 put Link almost as deep
 * as the horizon (little relative depth) and nothing could come forward. */
#define STEREO_POPOUT_LIMIT (-0.10f) /* closest things come forward at most 10% of the full depth:
                                      * comfort guideline - keep almost everything behind the screen */
#define STEREO_ROOM_DEPTH 0.75f      /* pre-rendered room picture: behind Link, who is at ~0.5 there (the
                                      * convergence halves in those rooms, gfx_3ds.c) - he stands inside
                                      * the picture instead of in front of it (hardware feedback) */
extern float gPortStereoConv;        /* gfx_3ds.c: smoothed convergence distance */
static int sEyeLoc = -1;
static float sEyeCur[4] = { -1.0f, -1.0f, -1.0f, -1.0f };

/* depth mode from gfx_pc.c (G_NOOP tags, S2DEX backgrounds): 0 = by distance, 1 = infinity (sky),
 * 2 = pre-rendered room picture, 3 = flat at screen depth (the HUD) */
static int sStereoMode;

void gfx_citro3d_set_stereo_mode(int mode) {
    if (sRec) recOp(OP_STEREO, mode, 0, 0, 0);
    sStereoMode = mode;
}

static int sRemapLoc = -1;
static float sRemapCur[3] = { -1.0f, -1.0f, -1.0f };

static void setRemap(float a, float b) {
    extern int gPortRawRelax; /* gfx_pc.c speed rules: no NoN depth clamp, the PICA clips at the near plane */
    float zlim = gPortRawRelax ? 1.0e10f : 0.0f;
    if (sRemapLoc >= 0 && (a != sRemapCur[0] || b != sRemapCur[1] || zlim != sRemapCur[2])) {
        C3D_FVUnifSet(GPU_VERTEX_SHADER, sRemapLoc, a, b, zlim, 0.0f);
        sRemapCur[0] = a;
        sRemapCur[1] = b;
        sRemapCur[2] = zlim;
    }
}

/* pre-rendered rooms in 3D: the room picture is shifted sideways for depth, which uncovered its edge (a
 * "void" strip beside shop rooms, hardware v17). Everything but the HUD is zoomed horizontally by the
 * shift plus a margin - picture and 3D actors alike, so they stay aligned (a slight FOV change). */
static float sRoomZoom(float shift) {
    extern int gPortPrerenderedFrame;
    return gPortPrerenderedFrame ? fabsf(shift * STEREO_ROOM_DEPTH) * 1.5f : 0.0f;
}

static void setEye(float shift) {
    float u[4];
    if (sEyeLoc < 0) {
        return;
    }
    extern int gPortStereoFlatSceneR; /* gfx_3ds.c: menus (file select) - flat, backgrounds at half depth */
    if (shift == 0.0f || sStereoMode == 3 || (gPortStereoFlatSceneR && sStereoMode == 0)) {
        /* mono, a flat screen-depth layer (the HUD), or a menu's panels */
        u[0] = u[1] = u[2] = u[3] = 0.0f;
    } else if (gPortStereoFlatSceneR) { /* a menu's sky / room background: behind the panels */
        u[0] = 0.0f, u[1] = 0.0f, u[2] = fabsf(shift * 0.5f), u[3] = shift * 0.5f;
    } else if (sStereoMode == 1) { /* sky: the full shift, independent of the skybox box's own w */
        u[0] = 0.0f, u[1] = 0.0f, u[2] = 0.0f, u[3] = shift;
    } else if (sStereoMode == 2) { /* flat picture (w = 1): constant shift, zoomed (see sRoomZoom) */
        u[0] = 0.0f, u[1] = 0.0f, u[2] = sRoomZoom(shift), u[3] = shift * STEREO_ROOM_DEPTH;
    } else {
        u[0] = shift, u[1] = 0.0f, u[2] = sRoomZoom(shift), u[3] = 0.0f; /* depth curve per vertex (gfx_pc.c) */
    }
    if (u[0] != sEyeCur[0] || u[1] != sEyeCur[1] || u[2] != sEyeCur[2] || u[3] != sEyeCur[3]) {
        C3D_FVUnifSet(GPU_VERTEX_SHADER, sEyeLoc, u[0], u[1], u[2], u[3]);
        memcpy(sEyeCur, u, sizeof(u));
    }
}

/* submit `count` vertices of the current batch: DrawArrays from `first`, or DrawElements from `idx` -
 * once per eye in stereo (see the viewport/eye notes inside) */
static void submitDraw_impl(u32 count, u32 first, const u16* idx);
static void submitDraw(u32 count, u32 first, const u16* idx) {
    extern volatile unsigned char gPortProf;
    unsigned char prev = gPortProf;
    gPortProf = 17; /* PROF_SUBMIT (port_prof.h) */
    submitDraw_impl(count, first, idx);
    gPortProf = prev;
}
static void submitDraw_impl(u32 count, u32 first, const u16* idx) {
    applyAlphaTest();
    applyDrawId();
    if (gGfx3DSMode == GFX_3DS_MODE_STEREO && !Port3ds_IsOffscreen()) {
        /* both eyes from the same vertices: left half then right half of the stereo target */
        int e;
        for (e = 0; e < 2; e++) {
            int off = e * STEREO_EYE_OFFSET;
            int vx = TOP_W - (sVp[0] + sVp[2]), sx = TOP_W - (sSc[0] + sSc[2]);
            int vy = vx + off, vh = sVp[2];
            /* The PICA framebuffer is stored bottom-up: viewport long-axis offset 0 lands in the SECOND
             * half of the buffer in memory, which sEyeOut[1] sends to GFX_RIGHT. So e = 0 draws the RIGHT
             * eye and gets -shift (uncrossed disparity = the world behind the screen). Confirmed on
             * hardware: v10 (this sign) had depth behind the screen; v11 (flipped, after misreading the
             * stereo_fb.bin dump, whose first half is the LEFT eye) made everything pop out and hurt. */
            setEye(e == 0 ? -gPortStereoSep : gPortStereoSep);
            /* PICA viewport origins are signed 10-bit: a right-eye viewport starting at >= 512 on the long
             * axis (the A button's at 524) wrapped negative and vanished. Start it at 400 instead, taller,
             * and remap clip y in the shader so the geometry lands on the same pixels. */
            if (vy > 511) {
                int dy = vy - off;
                setRemap((float)vh / (float)(dy + vh), (float)dy / (float)(dy + vh));
                C3D_SetViewport(sVp[1], off, sVp[3], dy + vh);
            } else {
                setRemap(1.0f, 0.0f);
                C3D_SetViewport(sVp[1], vy, sVp[3], vh);
            }
            C3D_SetScissor(GPU_SCISSOR_NORMAL, sSc[1], sx + off, sSc[1] + sSc[3], sx + sSc[2] + off);
            if (idx != NULL) {
                C3D_DrawElements(GPU_TRIANGLES, count, C3D_UNSIGNED_SHORT, idx);
            } else {
                C3D_DrawArrays(GPU_TRIANGLES, first, count);
            }
        }
    } else {
        setEye(0.0f);
        setRemap(1.0f, 0.0f);
        if (idx != NULL) {
            C3D_DrawElements(GPU_TRIANGLES, count, C3D_UNSIGNED_SHORT, idx);
        } else {
            C3D_DrawArrays(GPU_TRIANGLES, first, count);
        }
    }
}

#define VBO_BYTES (2 * 1024 * 1024)
#define VBO_VERTS (VBO_BYTES / (VTX_FLOATS * 4))

/* ---- PORT (2026-10-01): GPU vertex path (gpu_vtx=1, shader_gpu.v.pica) ----
 * Vertices in model space (struct GpuVtx, gfx_pc.c writes them), transformed by a palette of up to
 * GPU_PAL matrices in vertex-shader uniforms. Fog/stereo parameters and the cull mode are per draw. */
int gPortGpuVtx = 1;              /* settings gpu_vtx=0/1, fixed at init. PORT PERF (2026-10-02): the GPU path is the
                                   * default - as accurate as the CPU path (fbdiff) and faster on both consoles
                                   * (hardware v27: New 3DS walk 9-16 vs 15-22 ms, Old 3DS speed 9.9-10.4 vs 9.6
                                   * updates/s before frame skip) */
#define GPU_PAL 18
#define GPU_STRIDE 56             /* sizeof(GpuVtx) in gfx_pc.c */
#define GPU_VERTS (VBO_BYTES / GPU_STRIDE)
static int sPalLoc = -1, sFogpLoc = -1, sStpLoc = -1;
static int sReplayK = 2;          /* the replay being drawn: 0 = t 1/3, 1 = t 2/3, 2 = the logic frame */
static uint16_t sPalSlotNow[GPU_PAL]; /* the slot whose rows each palette entry holds (replay by copy) */
static u32 sPalTouched; /* written in this replay: palette entry e = bit e, stp = bit 31 (older ones are stale) */
static float sGpuConv, sGpuFog[3];
#define REC_PALS 4096
static uint16_t (*sOpPal)[GPU_PAL]; /* recorded palettes (slot ids), OP_GPUPAL v[0] = index, v[1] = count */
static int sOpPalN;
static float (*sOpParam)[5];        /* recorded fog/stereo/screen-linear parameters, OP_GPUPARAM v[0] = index */
static int sOpParamN;


/* front-end vertex (gfx_pc.c: x y z w u0 v0 u1 v1 r g b a stereo) -> the PICA layout: portrait target
 * (y, -x), N64 z negated, texcoords scaled into the power-of-two PICA texture of the bound units */
static inline void writeVertex(float* dst, const float* src, float s0, float t0, float s1, float t1) {
    dst[0] = src[1];
    dst[1] = -src[0];
    dst[2] = -src[2];
    dst[3] = src[3];
    dst[4] = src[4] * s0;
    dst[5] = 1 - (src[5] * t0);
    dst[6] = src[6] * s1;
    dst[7] = 1 - (src[7] * t1);
    dst[8] = src[8];
    dst[9] = src[9];
    dst[10] = src[10];
    dst[11] = src[11];
    dst[12] = src[12]; /* stereo offset (gfx_pc.c stereo_offset) */
}

static void gfx_citro3d_draw_triangles(float buf_vbo[], size_t buf_vbo_len, size_t buf_vbo_num_tris)
{
    if (sRec) { /* the array path is not recorded (A/B bench only): this frame is not replayable */
        sRecOverflow = 1;
    }
    if (sBufIdx * VTX_FLOATS + buf_vbo_len > VBO_BYTES / 4)
    {
        printf("Poly buf over!\n");
        return;
    }
    /* Invariant: gfx_pc.c packs VTX_FLOATS per vertex. A mismatch means the front end and this backend
     * disagree on the vertex layout; reading with the wrong stride walks off the buffer. */
    if (buf_vbo_len != buf_vbo_num_tris * 3 * VTX_FLOATS) {
        static int sReported;
        if (sReported < 4) {
            extern void PortDbgX(const char* label, unsigned val);
            sReported++;
            PortDbgX("VTX-STRIDE MISMATCH buf_vbo_len", (unsigned)buf_vbo_len);
            PortDbgX("  tris", (unsigned)buf_vbo_num_tris);
        }
        return;
    }
    /* texcoords are normalized to the uploaded texture; scale into the power-of-two PICA texture */
    float s0 = sTexturePoolScaleS[sTexUnits[0]], t0 = sTexturePoolScaleT[sTexUnits[0]];
    float s1 = sTexturePoolScaleS[sTexUnits[1]], t1 = sTexturePoolScaleT[sTexUnits[1]];
    const float* src = buf_vbo;
    float* dst = &((float*)sVboBuffer)[sBufIdx * VTX_FLOATS];
    for (size_t i = 0; i < 3 * buf_vbo_num_tris; i++, src += VTX_FLOATS, dst += VTX_FLOATS) {
        writeVertex(dst, src, s0, t0, s1, t1);
    }
    submitDraw(buf_vbo_num_tris * 3, sBufIdx, NULL);
    sBufIdx += buf_vbo_num_tris * 3;
    {
        extern u32 gPortPerfTris, gPortPerfDraws;
        gPortPerfTris += buf_vbo_num_tris;
        gPortPerfDraws++;
    }
}

/* PORT PERF (2026-09-30): indexed batches. The front end (gfx_pc.c) writes each vertex once per batch
 * straight into the VBO in the final layout (a loaded N64 vertex shared by several triangles is reused
 * by index) and each triangle adds 3 indices; the batch is drawn with DrawElements. The array path
 * above wrote 3 vertices per triangle into a staging buffer and copied/re-arranged all of them here
 * (hardware profile: triangle handling ~16 us each on Old 3DS). */
#define IDX_CAP (96 * 1024)
static u16* sIdxBuf;
static u32 sIdxPos, sIdxStart;

/* n vertices at the end of this frame's VBO; NULL when full (index of the first in *first) */
float* gfx_citro3d_vtx_reserve(u32 n, u32* first) {
    if (sVboBuffer == NULL || sBufIdx + n > VBO_VERTS) {
        return NULL;
    }
    *first = sBufIdx;
    sBufIdx += n;
    return &((float*)sVboBuffer)[*first * VTX_FLOATS];
}

void gfx_citro3d_vtx_write(float* dst, const float* src) {
    writeVertex(dst, src, sTexturePoolScaleS[sTexUnits[0]], sTexturePoolScaleT[sTexUnits[0]],
                sTexturePoolScaleS[sTexUnits[1]], sTexturePoolScaleT[sTexUnits[1]]);
}

/* the VBO base, capacity (vertices), the running index and the bound units' texcoord scales, so the
 * front end can write vertices inline (one call per batch instead of two per vertex) */
float* gfx_citro3d_vbo_info(int** pos, u32* cap, float scale[4]) {
    *pos = &sBufIdx;
    *cap = VBO_VERTS;
    scale[0] = sTexturePoolScaleS[sTexUnits[0]];
    scale[1] = sTexturePoolScaleT[sTexUnits[0]];
    scale[2] = sTexturePoolScaleS[sTexUnits[1]];
    scale[3] = sTexturePoolScaleT[sTexUnits[1]];
    return (float*)sVboBuffer;
}

/* one triangle's indices; 0 when the index buffer is full */
int gfx_citro3d_idx_push(u32 a, u32 b, u32 c) {
    if (sIdxBuf == NULL || sIdxPos + 3 > IDX_CAP) {
        return 0;
    }
    sIdxBuf[sIdxPos++] = (u16)a;
    sIdxBuf[sIdxPos++] = (u16)b;
    sIdxBuf[sIdxPos++] = (u16)c;
    return 1;
}

/* draw the indices pushed since the last batch */
void gfx_citro3d_draw_indexed(void) {
    u32 n = sIdxPos - sIdxStart;
    if (n == 0) {
        return;
    }
    if (sRec) { /* drawn by the replays (direct capture: also now) */
        recOp(OP_DRAWIDX, (int)sIdxStart, (int)n, 0, 0);
        if (!sRecDirect) {
            sIdxStart = sIdxPos;
            return;
        }
    }
    submitDraw(n, 0, sIdxBuf + sIdxStart);
    sIdxStart = sIdxPos;
    {
        extern u32 gPortPerfTris, gPortPerfDraws;
        gPortPerfTris += n / 3;
        gPortPerfDraws++;
    }
}

extern void gfx_gpu_slot_rows(uint16_t slot, int k, float rows[4][4]);

static void gpuUploadPalette(const uint16_t* slots, int start, int n) {
    int i;
    float rows[4][4];
    if (sPalLoc < 0 || n <= 0 || start < 0 || start + n > GPU_PAL) {
        return;
    }
    for (i = 0; i < n; i++) {
        float* u = (float*)C3D_FVUnifWritePtr(GPU_VERTEX_SHADER, sPalLoc + (start + i) * 4, 4);
        sPalSlotNow[start + i] = slots[i];
        sPalTouched |= 1u << (start + i);
        gfx_gpu_slot_rows(slots[i], sReplayK, rows);
        /* citro3d uniforms are stored (w, z, y, x) per vec4 */
        int r;
        for (r = 0; r < 4; r++) {
            u[r * 4 + 0] = rows[r][3];
            u[r * 4 + 1] = rows[r][2];
            u[r * 4 + 2] = rows[r][1];
            u[r * 4 + 3] = rows[r][0];
        }
    }
}

static void gpuUploadParams(const float p[5]) {
    /* p: fog mul, fog offset, fog on, stereo convergence, screen-linear mode */
    if (sFogpLoc >= 0) {
        C3D_FVUnifSet(GPU_VERTEX_SHADER, sFogpLoc, p[0], p[1], p[2], 1.0f / 255.0f);
    }
    if (sStpLoc >= 0) {
        /* y = 1 - t of the replay being drawn (skinned vertices' delta toward the previous frame) */
        float back = sReplayK == 0 ? (2.0f / 3.0f) : sReplayK == 1 ? (1.0f / 3.0f) : 0.0f;
        C3D_FVUnifSet(GPU_VERTEX_SHADER, sStpLoc, p[3], back, p[4], 0.0005f);
        sPalTouched |= 1u << 31;
    }
}

/* gfx_pc.c, before a GPU-path draw: palette entries start..start+n-1 (slot ids) added since the last one */
void gfx_citro3d_gpu_palette(const uint16_t* slots, int start, int n) {
    if (sRec) {
        if (sOpPalN < REC_PALS) {
            memcpy(sOpPal[sOpPalN], slots, sizeof(uint16_t) * n);
            recOp(OP_GPUPAL, sOpPalN++, start, n, 0);
        } else {
            sRecOverflow = 1;
        }
    }
    gpuUploadPalette(slots, start, n);
}

/* fog multiplier/offset (already folded for z' = -(z + w) / 2), fog on, stereo convergence, screen-linear
 * shading (shader_gpu.v.pica) */
void gfx_citro3d_gpu_params(float fogMul, float fogOff, float fogOn, float conv, float lin) {
    float p[5] = { fogMul, fogOff, fogOn, conv, lin };
    if (sRec) {
        if (sOpParamN < REC_PALS) {
            memcpy(sOpParam[sOpParamN], p, sizeof(p));
            recOp(OP_GPUPARAM, sOpParamN++, 0, 0, 0);
        } else {
            sRecOverflow = 1;
        }
    }
    gpuUploadParams(p);
}

/* N64 cull mode (G_CULL_BACK etc. >> 9: 0 none, 1 front, 2 back, 3 both) */
static int sCullMode;
void gfx_citro3d_gpu_cull(int mode) {
    if (sRec) recOp(OP_CULL, mode, 0, 0, 0);
    sCullMode = mode;
    /* The portrait mapping (y, -x) is a rotation (keeps the winding); the N64 front face is
     * counter-clockwise on screen. Both = everything culled: the CPU path drew nothing either. */
    C3D_CullFace(mode == 1 ? GPU_CULL_FRONT_CCW : mode == 2 ? GPU_CULL_BACK_CCW : GPU_CULL_NONE);
}

/* the GPU-path vertex buffer: base, capacity (vertices) and the running index */
void* gfx_citro3d_gpu_vbo(int** pos, u32* cap, float scale[4]) {
    *pos = &sBufIdx;
    *cap = GPU_VERTS;
    scale[0] = sTexturePoolScaleS[sTexUnits[0]];
    scale[1] = sTexturePoolScaleT[sTexUnits[0]];
    scale[2] = sTexturePoolScaleS[sTexUnits[1]];
    scale[3] = sTexturePoolScaleT[sTexUnits[1]];
    return sVboBuffer;
}

/* ---- PORT PERF (2026-10-04): raw vertex path (gfx_pc.c gPortRawVtx, shader_raw.v.pica) ----
 * The game's 16-byte N64 vertices go to the GPU as they are; the shader does the matrix, lights, texture coordinate
 * scale and fog. Same binary as the GPU path (uniforms shared by name, so the palette and eye/fog values carry over
 * a switch); its own vertex buffer and format. Draws switch between the two programs (rectangles, texgen and skinned
 * loads stay on the GPU path). */
#define RAW_VBO_BYTES (768 * 1024)
#define RAW_STRIDE 16
#define REC_RAWP 2048
typedef struct {
    float uvc0[4], uvc1[4], lit[4], lamb[3], ldir[4][3], lcol[4][3]; /* lit: shader_raw.v.pica's lit uniform */
} RawParams;
static shaderProgram_s sProgRaw;
static bool sProgRawInit;
static void* sRawVbo;
static int sRawIdx;
static int sRawMode;              /* the raw program is bound */
static int sUvc0Loc = -1, sUvc1Loc = -1, sLitLoc = -1, sLambLoc = -1, sLdirLoc = -1, sLcolLoc = -1;
static C3D_AttrInfo sAttrGpu, sAttrRaw;
static C3D_BufInfo sBufGpu, sBufRaw;
static RawParams* sOpRawP;
static int sOpRawPN;
static shaderProgram_s sProg[2];
static bool sProgInit[2];

u32 gPortPerfRawSwitches; /* perf report: program + vertex format switches between the raw and GPU paths */
static C3Df_Config sCfgGpu, sCfgRaw;
static void rawBind(int raw) {
    gPortPerfRawSwitches++;
    /* only the registers that differ between the two configurations are sent (c3d_fast.c) */
    C3Df_SelectConfig(raw);
    sRawMode = raw;
}

/* gfx_pc.c, between draws: switch the program and vertex format (the caller flushed) */
void gfx_citro3d_raw_mode(int raw) {
    raw = raw != 0;
    if (raw == sRawMode || !sProgRawInit) {
        return;
    }
    if (sRec) recOp(OP_RAWMODE, raw, 0, 0, 0);
    rawBind(raw);
}

int gfx_citro3d_raw_ready(void) {
    return sProgRawInit && sRawVbo != NULL && sOpRawP != NULL;
}

/* PORT PERF (2026-10-04): only the values that changed are set: each set uniform is re-sent with the next draw, and
 * the command buffer is linear memory, whose writes cost the most on an Old 3DS. These uniforms are the raw program's
 * own (nothing else sets them), so the last values sent stay valid across program switches and frames. */
static RawParams sRawSent;
static int sRawSentOk; /* 0: nothing sent yet; bit 1: uvc0/uvc1/lit, bit 2: the lights */
static void rawUpload(const RawParams* p) {
    int i;
    if (sUvc0Loc < 0) return;
    if (!(sRawSentOk & 1) || memcmp(p->uvc0, sRawSent.uvc0, sizeof(p->uvc0)) != 0) {
        C3D_FVUnifSet(GPU_VERTEX_SHADER, sUvc0Loc, p->uvc0[0], p->uvc0[1], p->uvc0[2], p->uvc0[3]);
        memcpy(sRawSent.uvc0, p->uvc0, sizeof(p->uvc0));
    }
    if (!(sRawSentOk & 1) || memcmp(p->uvc1, sRawSent.uvc1, sizeof(p->uvc1)) != 0) {
        C3D_FVUnifSet(GPU_VERTEX_SHADER, sUvc1Loc, p->uvc1[0], p->uvc1[1], p->uvc1[2], p->uvc1[3]);
        memcpy(sRawSent.uvc1, p->uvc1, sizeof(p->uvc1));
    }
    if (!(sRawSentOk & 1) || memcmp(p->lit, sRawSent.lit, sizeof(p->lit)) != 0) {
        C3D_FVUnifSet(GPU_VERTEX_SHADER, sLitLoc, p->lit[0], p->lit[1], p->lit[2], p->lit[3]);
        memcpy(sRawSent.lit, p->lit, sizeof(p->lit));
    }
    sRawSentOk |= 1;
    if (p->lit[0] != 0.0f && (!(sRawSentOk & 2) || memcmp(p->lamb, sRawSent.lamb, sizeof(p->lamb)) != 0 ||
                           memcmp(p->ldir, sRawSent.ldir, sizeof(p->ldir)) != 0 ||
                           memcmp(p->lcol, sRawSent.lcol, sizeof(p->lcol)) != 0)) {
        C3D_FVUnifSet(GPU_VERTEX_SHADER, sLambLoc, p->lamb[0], p->lamb[1], p->lamb[2], 0.0f);
        for (i = 0; i < 4; i++) {
            C3D_FVUnifSet(GPU_VERTEX_SHADER, sLdirLoc + i, p->ldir[i][0], p->ldir[i][1], p->ldir[i][2], 0.0f);
            C3D_FVUnifSet(GPU_VERTEX_SHADER, sLcolLoc + i, p->lcol[i][0], p->lcol[i][1], p->lcol[i][2], 0.0f);
        }
        memcpy(sRawSent.lamb, p->lamb, sizeof(p->lamb));
        memcpy(sRawSent.ldir, p->ldir, sizeof(p->ldir));
        memcpy(sRawSent.lcol, p->lcol, sizeof(p->lcol));
        sRawSentOk |= 2;
    }
}

/* (replay by copy, canonUniforms) the raw path's uniforms at fixed values, and the next raw draw sends its own */
static void rawCanonUniforms(void) {
    int i;
    if (sUvc0Loc < 0) {
        return;
    }
    C3D_FVUnifSet(GPU_VERTEX_SHADER, sUvc0Loc, 0.0f, 0.0f, 0.0f, 0.0f);
    C3D_FVUnifSet(GPU_VERTEX_SHADER, sUvc1Loc, 0.0f, 0.0f, 0.0f, 0.0f);
    C3D_FVUnifSet(GPU_VERTEX_SHADER, sLitLoc, 0.0f, 0.0f, 0.0f, 0.0f);
    C3D_FVUnifSet(GPU_VERTEX_SHADER, sLambLoc, 0.0f, 0.0f, 0.0f, 0.0f);
    for (i = 0; i < 4; i++) {
        C3D_FVUnifSet(GPU_VERTEX_SHADER, sLdirLoc + i, 0.0f, 0.0f, 0.0f, 0.0f);
        C3D_FVUnifSet(GPU_VERTEX_SHADER, sLcolLoc + i, 0.0f, 0.0f, 0.0f, 0.0f);
    }
    sRawSentOk = 0;
}

/* gfx_pc.c, before a raw draw: its texture coordinate coefficients and lights */
void gfx_citro3d_raw_params(const float uvc0[4], const float uvc1[4], const float lit[4], const float lamb[3],
                            const float ldir[4][3], const float lcol[4][3]) {
    RawParams p;
    memcpy(p.uvc0, uvc0, sizeof(p.uvc0));
    memcpy(p.uvc1, uvc1, sizeof(p.uvc1));
    memcpy(p.lit, lit, sizeof(p.lit));
    if (lit[0] != 0.0f) {
        memcpy(p.lamb, lamb, sizeof(p.lamb));
        memcpy(p.ldir, ldir, sizeof(p.ldir));
        memcpy(p.lcol, lcol, sizeof(p.lcol));
    }
    if (sRec) {
        if (sOpRawPN < REC_RAWP) {
            sOpRawP[sOpRawPN] = p;
            recOp(OP_RAWPARAM, sOpRawPN++, 0, 0, 0);
        } else {
            sRecOverflow = 1;
        }
    }
    rawUpload(&p);
}

/* the raw vertex buffer: base, capacity (vertices) and the running index */
void* gfx_citro3d_raw_vbo(int** pos, u32* cap) {
    *pos = &sRawIdx;
    *cap = RAW_VBO_BYTES / RAW_STRIDE;
    return sRawVbo;
}

static void rawInit(void) {
    if (sProgRawInit || sVShaderDvlb == NULL || sVShaderDvlb->numDVLE < 3) {
        return;
    }
    sRawVbo = linearAlloc(RAW_VBO_BYTES);
    sOpRawP = malloc(sizeof(RawParams) * REC_RAWP);
    if (sRawVbo == NULL || sOpRawP == NULL) {
        return;
    }
    shaderProgramInit(&sProgRaw);
    shaderProgramSetVsh(&sProgRaw, &sVShaderDvlb->DVLE[2]);
    sUvc0Loc = shaderInstanceGetUniformLocation(sProgRaw.vertexShader, "uvc0");
    sUvc1Loc = shaderInstanceGetUniformLocation(sProgRaw.vertexShader, "uvc1");
    sLitLoc = shaderInstanceGetUniformLocation(sProgRaw.vertexShader, "lit");
    sLambLoc = shaderInstanceGetUniformLocation(sProgRaw.vertexShader, "lamb");
    sLdirLoc = shaderInstanceGetUniformLocation(sProgRaw.vertexShader, "ldir");
    sLcolLoc = shaderInstanceGetUniformLocation(sProgRaw.vertexShader, "lcol");
    /* N64 Vtx: s16 ob[3], u16 flag (the palette index here), s16 tc[2], u8 cn[4] */
    AttrInfo_Init(&sAttrRaw);
    AttrInfo_AddLoader(&sAttrRaw, 0, GPU_SHORT, 4);         // v0 = x, y, z, palette index
    AttrInfo_AddLoader(&sAttrRaw, 1, GPU_SHORT, 2);         // v1 = s, t
    AttrInfo_AddLoader(&sAttrRaw, 2, GPU_UNSIGNED_BYTE, 4); // v2 = colour, or normal + alpha
    BufInfo_Init(&sBufRaw);
    BufInfo_Add(&sBufRaw, sRawVbo, RAW_STRIDE, 3, 0x210);
    sCfgGpu.prog = &sProg[1], sCfgGpu.attr = &sAttrGpu, sCfgGpu.buf = &sBufGpu;
    sCfgRaw.prog = &sProgRaw, sCfgRaw.attr = &sAttrRaw, sCfgRaw.buf = &sBufRaw;
    C3Df_SetConfigs(&sCfgGpu, &sCfgRaw);
    sProgRawInit = true;
    {
        extern void PortDbgX(const char*, unsigned);
        PortDbgX("[gfx] raw vertex path ready", 1);
    }
}
static void gfx_citro3d_setup_mode(int gpu) {
    extern void PortDbgX(const char*, unsigned);
    C3D_AttrInfo* attrInfo;
    C3D_BufInfo* bufInfo;
    if (!sProgInit[gpu]) {
        shaderProgramInit(&sProg[gpu]);
        shaderProgramSetVsh(&sProg[gpu], &sVShaderDvlb->DVLE[gpu]);
        sProgInit[gpu] = true;
    }
    C3D_BindProgram(&sProg[gpu]);
    sEyeLoc = shaderInstanceGetUniformLocation(sProg[gpu].vertexShader, "eye");
    sRemapLoc = shaderInstanceGetUniformLocation(sProg[gpu].vertexShader, "remap");
    sPalLoc = sFogpLoc = sStpLoc = -1;
    if (gpu) {
        sPalLoc = shaderInstanceGetUniformLocation(sProg[gpu].vertexShader, "pal");
        sFogpLoc = shaderInstanceGetUniformLocation(sProg[gpu].vertexShader, "fogp");
        sStpLoc = shaderInstanceGetUniformLocation(sProg[gpu].vertexShader, "stp");
        if (sOpPal == NULL) {
            sOpPal = malloc(sizeof(*sOpPal) * REC_PALS);
            sOpParam = malloc(sizeof(*sOpParam) * REC_PALS);
        }
    }
    PortDbgX(gpu ? "[gfx] vertex path: GPU (gpu_vtx)" : "[gfx] vertex path: CPU", 1);
    PortDbgX("[gfx] uniforms pal<<24|fogp<<16|stp<<8|eye", ((unsigned)(sPalLoc & 0xFF) << 24) | ((unsigned)(sFogpLoc & 0xFF) << 16) |
                                                            ((unsigned)(sStpLoc & 0xFF) << 8) | (unsigned)(sEyeLoc & 0xFF));
    PortDbgX("[gfx] uniforms remap<<24|uvc0<<16|uvc1<<8|lit", ((unsigned)(sRemapLoc & 0xFF) << 24) | ((unsigned)(sUvc0Loc & 0xFF) << 16) |
                                                               ((unsigned)(sUvc1Loc & 0xFF) << 8) | (unsigned)(sLitLoc & 0xFF));
    PortDbgX("[gfx] uniforms lamb<<16|ldir<<8|lcol", ((unsigned)(sLambLoc & 0xFF) << 16) | ((unsigned)(sLdirLoc & 0xFF) << 8) |
                                                      (unsigned)(sLcolLoc & 0xFF));
    /* a new program: its uniforms start unset */
    sEyeCur[0] = sEyeCur[1] = sEyeCur[2] = sEyeCur[3] = -1.0f;
    sRemapCur[0] = sRemapCur[1] = sRemapCur[2] = -1.0f;
    setEye(0.0f);
    setRemap(1.0f, 0.0f);

    attrInfo = C3D_GetAttrInfo();
    AttrInfo_Init(attrInfo);
    bufInfo = C3D_GetBufInfo();
    BufInfo_Init(bufInfo);
    if (gpu) { /* struct GpuVtx: pos[4] uv0[2] uv1[2] dpos[4] (floats), shade[4] idx (bytes) */
        AttrInfo_AddLoader(attrInfo, 0, GPU_FLOAT, 4);         // v0 = position (model space w=1, or clip space)
        AttrInfo_AddLoader(attrInfo, 1, GPU_FLOAT, 2);         // v1 = texcoord 0
        AttrInfo_AddLoader(attrInfo, 2, GPU_UNSIGNED_BYTE, 4); // v2 = shade 0..255
        AttrInfo_AddLoader(attrInfo, 3, GPU_FLOAT, 2);         // v3 = texcoord 1
        AttrInfo_AddLoader(attrInfo, 4, GPU_UNSIGNED_BYTE, 1); // v4 = palette index
        AttrInfo_AddLoader(attrInfo, 5, GPU_FLOAT, 4);         // v5 = skinned delta; w = 1: fog precomputed
        /* buffer order: pos(v0) uv0(v1) uv1(v3) dpos(v5) shade(v2) idx(v4); the stride covers the 3 spare bytes */
        BufInfo_Add(bufInfo, sVboBuffer, GPU_STRIDE, 6, 0x425310);
        sAttrGpu = *attrInfo; /* (the raw path switches back to these) */
        sBufGpu = *bufInfo;
        sRawMode = 0;
    } else {
        AttrInfo_AddLoader(attrInfo, 0, GPU_FLOAT, 4); // v0=position
        AttrInfo_AddLoader(attrInfo, 1, GPU_FLOAT, 2); // v1=texcoord
        AttrInfo_AddLoader(attrInfo, 2, GPU_FLOAT, 4); // v2=color
        AttrInfo_AddLoader(attrInfo, 3, GPU_FLOAT, 2); // v3=texcoord1
        AttrInfo_AddLoader(attrInfo, 4, GPU_FLOAT, 1); // v4=stereo offset
        BufInfo_Add(bufInfo, sVboBuffer, VTX_FLOATS * 4, 5, 0x42310); // pos, uv0, uv1, color, stereo
    }
    C3D_CullFace(GPU_CULL_NONE); /* the CPU path culls itself; the GPU path sets it per draw */
}

/* gfx_pc.c / 3ds_main.c, between frames only */
void gfx_citro3d_set_gpu_mode(int gpu) {
    gpu = gpu != 0;
    if (gpu != gPortGpuVtx) {
        gPortGpuVtx = gpu;
        gfx_citro3d_setup_mode(gpu);
        sTexBoundOk[0] = sTexBoundOk[1] = false; /* unit 0's projective type follows the vertex path */
    }
}

static void gfx_citro3d_init(void)
{
    sVShaderDvlb = DVLB_ParseFile((u32*)shader_shbin, (u32)(shader_shbin_end - shader_shbin));
    // Create the VBO (vertex buffer object)
    sVboBuffer = linearAlloc(VBO_BYTES);
    sIdxBuf = linearAlloc(IDX_CAP * sizeof(u16));
#ifdef PORT_TEX_DEBUG
    { extern void PortDbgX(const char* label, unsigned val);
      PortDbgX("[texdbg] idx buf", (unsigned)(uintptr_t)sIdxBuf); PortDbgX("  vbo", (unsigned)(uintptr_t)sVboBuffer);
      PortDbgX("  raw vbo", (unsigned)(uintptr_t)sRawVbo); }
#endif
    gfx_citro3d_setup_mode(gPortGpuVtx != 0);
    rawInit();
    C3D_DepthMap(true, -1.0f, 0);
    C3D_DepthTest(false, GPU_LEQUAL, GPU_WRITE_ALL);
    C3D_AlphaTest(true, GPU_GREATER, 0x00);
}

static void gfx_citro3d_start_frame(void) {
    sBufIdx = 0;
    sRawIdx = 0;
    sIdxPos = sIdxStart = 0;
}

/* PORT (2026-09-30): the vertex buffer is cached linear memory written by the CPU every frame; the GPU
 * reads RAM. Nothing flushed it, so on hardware the GPU could draw stale vertices still sitting in the
 * data cache (the New 3DS has a 2 MB L2): intermittent hardware-only glitches - a ghost of a previous
 * frame's Link in the pause preview, stray pixel lines. Azahar has no CPU cache model, so it never
 * showed. Called by gfx_3ds.c right before C3D_FrameEnd submits the frame's commands. */
void gfx_citro3d_flush_vbo(void) {
    if (sBufIdx > 0) {
        Port3ds_CacheFlush(sVboBuffer, sBufIdx * (gPortGpuVtx ? GPU_STRIDE : VTX_FLOATS * sizeof(float)));
    }
    if (sIdxPos > 0) {
        Port3ds_CacheFlush(sIdxBuf, sIdxPos * sizeof(u16));
    }
    if (sRawIdx > 0) {
        Port3ds_CacheFlush(sRawVbo, sRawIdx * RAW_STRIDE);
    }
}

/* PORT PERF (2026-10-05): replay by copy. An update's shown frames differ only in the palette matrices
 * (gfx_gpu_slot_rows of the frame's t) and the skinned-vertex blend (stp.y): the same draws, states, textures and
 * vertices. So the walk draws the first of them itself (gfx_citro3d_rec_direct) as a self-contained command stream
 * (c3d_fast.c C3Df_CaptureBegin re-sends all state at its start), the positions of every palette row and stp upload in
 * it are noted, and each further frame is a copy of those words with only those patched - instead of re-running the
 * draw log through citro3d (a third of a walk per frame). The log is still recorded: it is replayed whenever a copy
 * could differ (another render target or mode - 3D switched -, a full command buffer, a capture that did not fit) and
 * when the walk cannot draw directly (CPU vertex path). settings replay_copy=0 turns copies off. */
int gPortReplayCopy = 1;
int gPortReplayCopyCheck; /* settings replay_copy_check=1: the log is replayed (from a full state, as captured) and its
                           * command words compared with the patched copy's - any difference is a bug */
u32 gPortPerfReplayCopies, gPortPerfReplayCopyWords, gPortPerfCopyChecked, gPortPerfCopyBad;
u32 gPortCopyBadInfo[6]; /* the first mismatch: offset, words (log, copy), stream lengths (log << 16 | copy, x/4) */
typedef struct {
    u32 off;    /* word 0 of the vec4, from the start of the stream */
    u16 slot;   /* palette: the slot whose row it holds */
    u8 row;     /* palette: row 0-3 */
    u8 flags;   /* 1: the command header follows word 0 (first vec4 of a command), 2: stp (patch y = word 2),
                 * 4: stale - sent with the frame's full state before the frame wrote it (no draw reads it) */
} CapPatch;
#define CAP_PATCHES 16384
enum { CAP_NONE, CAP_ARMED, CAP_RUNNING, CAP_READY };
static int sCapState, sCapOverflow, sCapPN, sCapMode;
static CapPatch* sCapP;
static u32 *sCapWords, *sCapBase, *sCapCmdBuf;
static u32 sCapN, sCapCap, sCapTris, sCapDraws;
static const void* sCapTarget;
extern const void* Port3ds_DrawTargetId(void);
extern const float* gfx_gpu_slot_row_ptr(uint16_t slot, int k);

/* c3d_fast.c: a float-uniform run [first, first + count) was written at `data` */
static void capUnifRun(const u32* data, int first, int count) {
    int v, v0, v1;
    if (sStpLoc >= first && sStpLoc < first + count) {
        v0 = sStpLoc - first, v1 = v0 + 1; /* (stp sits apart from the palette: handled on its own below) */
    } else {
        v0 = v1 = 0;
    }
    for (v = v0; v < v1; v++) {
        int w = v * 4, wi = w & 255;
        if (sCapPN >= CAP_PATCHES) {
            sCapOverflow = 1;
            return;
        }
        sCapP[sCapPN].off = (u32)(data + (w >> 8) * 258 + (wi ? wi + 1 : 0) - sCapBase);
        sCapP[sCapPN].slot = 0, sCapP[sCapPN].row = 0;
        sCapP[sCapPN].flags = 2 | (wi == 0 ? 1 : 0) | ((sPalTouched >> 31) ? 0 : 4);
        sCapPN++;
    }
    v0 = sPalLoc - first, v1 = sPalLoc + GPU_PAL * 4 - first;
    v0 = v0 < 0 ? 0 : v0, v1 = v1 > count ? count : v1;
    for (v = v0; v < v1; v++) {
        int u = first + v - sPalLoc, w = v * 4, wi = w & 255;
        if (sCapPN >= CAP_PATCHES) {
            sCapOverflow = 1;
            return;
        }
        /* a run goes out as commands of 256 words (64 vec4s): [w0][header][w1..w255][pad] = 258 words */
        sCapP[sCapPN].off = (u32)(data + (w >> 8) * 258 + (wi ? wi + 1 : 0) - sCapBase);
        sCapP[sCapPN].slot = sPalSlotNow[u >> 2];
        sCapP[sCapPN].row = (u8)(u & 3);
        sCapP[sCapPN].flags = (wi == 0 ? 1 : 0) | ((sPalTouched >> (u >> 2)) & 1 ? 0 : 4);
        sCapPN++;
    }
}

/* a captured or replayed frame starts from fixed values of the uniforms each frame sets before use (palette, fog/stp,
 * raw-path coefficients and lights), not from what the previous frame left: the full-state upload at its start is then
 * the same words whichever frame came before (the walk follows the previous update, a replay follows the walk) */
static void canonUniforms(void) {
    if (sPalLoc >= 0) {
        memset(C3D_FVUnifWritePtr(GPU_VERTEX_SHADER, sPalLoc, GPU_PAL * 4), 0, sizeof(float) * 16 * GPU_PAL);
    }
    if (sFogpLoc >= 0) {
        C3D_FVUnifSet(GPU_VERTEX_SHADER, sFogpLoc, 0.0f, 0.0f, 0.0f, 1.0f / 255.0f); /* (w: the shade byte scale) */
    }
    if (sStpLoc >= 0) {
        C3D_FVUnifSet(GPU_VERTEX_SHADER, sStpLoc, 0.0f, 0.0f, 0.0f, 0.0f);
    }
    rawCanonUniforms();
}

static int capBegin(void) {
    extern u32 gPortPerfTris, gPortPerfDraws;
    if (sCapP == NULL) {
        sCapP = malloc(sizeof(CapPatch) * CAP_PATCHES);
        if (sCapP == NULL) {
            return 0;
        }
    }
    sCapPN = 0, sCapOverflow = 0;
    sPalTouched = 0;
    canonUniforms();
    C3Df_CaptureBegin(capUnifRun);
    sCapCmdBuf = gpuCmdBuf;
    sCapBase = gpuCmdBuf + gpuCmdBufOffset;
    sCapTris = gPortPerfTris, sCapDraws = gPortPerfDraws;
    sCapState = CAP_RUNNING;
    return 1;
}

static void capEnd(void) {
    extern u32 gPortPerfTris, gPortPerfDraws;
    u32 n = (u32)(gpuCmdBuf + gpuCmdBufOffset - sCapBase);
    C3Df_CaptureEnd();
    sCapState = CAP_NONE;
    if (sCapOverflow || gpuCmdBuf != sCapCmdBuf) { /* (no split can happen inside a replay; checked anyway) */
        return;
    }
    if (n > sCapCap) {
        u32 cap = n + n / 4 + 1024;
        u32* w = realloc(sCapWords, cap * sizeof(u32));
        if (w == NULL) {
            return;
        }
        sCapWords = w, sCapCap = cap;
    }
    memcpy(sCapWords, sCapBase, n * sizeof(u32));
    sCapN = n;
    sCapTris = gPortPerfTris - sCapTris, sCapDraws = gPortPerfDraws - sCapDraws;
    sCapTarget = Port3ds_DrawTargetId();
    sCapMode = gGfx3DSMode;
    sCapState = CAP_READY;
}

/* the captured words at dst (copied there first unless they are the live stream itself), patched for frame k */
static void capPatchK(u32* dst, int copy, int k) {
    const CapPatch *p, *end;
    union {
        float f;
        u32 u;
    } back;
    if (copy) {
        memcpy(dst, sCapWords, sCapN * sizeof(u32));
    }
    back.f = k == 0 ? (2.0f / 3.0f) : k == 1 ? (1.0f / 3.0f) : 0.0f; /* as gpuUploadParams */
    for (p = sCapP, end = sCapP + sCapPN; p < end; p++) {
        u32* w = dst + p->off;
        int g = p->flags & 1;
        if (p->flags & 4) {
            continue; /* stale: the canonical value (canonUniforms), the same for every frame */
        } else if (p->flags & 2) {
            w[2 + g] = back.u;
        } else {
            /* citro3d uniform words are (w, z, y, x): the row's coefficients 3, 2, 1, 0 */
            u32 r[4];
            memcpy(r, gfx_gpu_slot_row_ptr(p->slot, k) + p->row * 4, sizeof(r));
            w[0] = r[3], w[1 + g] = r[2], w[2 + g] = r[1], w[3 + g] = r[0];
        }
    }
}

static void capPatch(u32* dst) {
    capPatchK(dst, 1, sReplayK);
}

/* direct capture cut short (an off-screen render, a broken recording): the frame must show the logic frame after all.
 * What was drawn is still in the command buffer, unsubmitted: patched in place to t = 1, and citro3d's copies of the
 * palette and stp rewritten for t = 1, for the draws that follow. 0 if the words could not be patched. */
static int capToLogicFrame(void) {
    int ok = !sCapOverflow && gpuCmdBuf == sCapCmdBuf;
    int e;
    if (ok) {
        u32 n = (u32)(gpuCmdBuf + gpuCmdBufOffset - sCapBase);
        u32 keepN = sCapN;
        sCapN = n; /* (the patch records all lie inside) */
        capPatchK(sCapBase, 0, 2);
        sCapN = keepN;
    }
    sReplayK = 2;
    for (e = 0; e < GPU_PAL; e++) {
        gpuUploadPalette(&sPalSlotNow[e], e, 1);
    }
    if (sStpLoc >= 0) {
        float* u = (float*)C3D_FVUnifWritePtr(GPU_VERTEX_SHADER, sStpLoc, 1);
        u[2] = 0.0f; /* (w, z, y, x): y = 1 - t */
    }
    return ok;
}

static int capUsable(void) {
    return Port3ds_DrawTargetId() == sCapTarget && gGfx3DSMode == sCapMode && gpuCmdBuf != NULL &&
           gpuCmdBufOffset + sCapN + 64 <= gpuCmdBufSize;
}

/* one in-between frame from the captured stream, patched for sReplayK; 0 = cannot (the caller replays the log) */
static int capEmit(void) {
    extern u32 gPortPerfTris, gPortPerfDraws;
    if (!capUsable()) {
        return 0;
    }
    capPatch(gpuCmdBuf + gpuCmdBufOffset);
    gpuCmdBufOffset += sCapN;
    C3Df_AfterCopy(sPalLoc, GPU_PAL * 4, sStpLoc);
    gPortPerfTris += sCapTris, gPortPerfDraws += sCapDraws;
    gPortPerfReplayCopies++;
    gPortPerfReplayCopyWords += sCapN;
    return 1;
}

/* the draw state the walk starts from, as the log's first entries: gfx_pc.c only sends what changed since the previous
 * frame, so without them a replay started from the state the walk ENDED in (its first draws could get another alpha
 * test, texture or cull mode than the walk's) */
static void recInitialState(void) {
    int t;
    recOp(OP_SHADER, sCurShader, 0, 0, 0);
    if (sConstN < REC_CONSTS) {
        sOpConsts[sConstN] = sConsts;
        recOp(OP_CONSTS, sConstN++, 0, 0, 0);
    }
    for (t = 0; t < 2; t++) { /* the bound textures with their sampler bits as they are now */
        recOp(OP_TEXPARAM, t, sTexUnits[t], (int)sTexturePool[sTexUnits[t]].param, 0);
    }
    recOp(OP_DTEST, sDepthTestOn, 0, 0, 0);
    recOp(OP_DMASK, sDepthUpdateOn, 0, 0, 0);
    recOp(OP_DECAL, sDepthDecal, 0, 0, 0);
    recOp(OP_ALPHA, sUseBlend, 0, 0, 0);
    recOp(OP_DRAWID, sDrawId, 0, 0, 0);
    recOp(OP_STEREO, sStereoMode, 0, 0, 0);
    recOp(OP_CULL, sCullMode, 0, 0, 0);
}

/* gfx_pc.c, right after gfx_citro3d_rec_begin: the walk also draws, as the update's first shown frame k, and its command
 * words are captured for the frames after it - no replay of the log for that frame (Old 3DS: about a third of the walk).
 * 0 = not possible here (the walk only records, as before). */
int gfx_citro3d_rec_direct(int k) {
    extern void Port3ds_ResetFrameViewport(void);
    if (!sRec || !gPortReplayCopy || !gPortGpuVtx || sPalLoc < 0) {
        return 0;
    }
    Port3ds_ResetFrameViewport(); /* (as each replayed frame starts) */
    if (!capBegin()) {
        return 0;
    }
    sReplayK = k;
    sRecDirect = 1;
    return 1;
}

void gfx_citro3d_rec_begin(void) {
    if (sOps == NULL) {
        sOps = malloc(sizeof(RecOp) * REC_OPS);
        sOpConsts = malloc(sizeof(struct GfxCombineConsts) * REC_CONSTS);
    }
    sOpN = sConstN = 0;
    sOpPalN = sOpParamN = 0;
    sOpRawPN = 0;
    sRecOverflow = sOps == NULL || sOpConsts == NULL || (gPortGpuVtx && (sOpPal == NULL || sOpParam == NULL));
    sRec = !sRecOverflow;
    sCapState = CAP_NONE;
    sRecDirect = 0;
    if (sRec) {
        recInitialState();
    }
}

/* returns 1 when the walk drew the update's first shown frame itself (direct capture): no replay for it */
int gfx_citro3d_rec_end(void) {
    extern int gPortReplayBroken;
    sRec = 0;
    if (sRecOverflow) {
        gPortReplayBroken = 1;
    }
    if (sRecDirect) {
        sRecDirect = 0;
        if (gPortReplayBroken) { /* no frames follow: this one shows the logic frame */
            C3Df_CaptureEnd();
            capToLogicFrame();
            sCapState = CAP_NONE;
        } else {
            capEnd(); /* READY, or NONE if the capture did not fit (the frames after it replay the log) */
        }
        return 1;
    }
    /* the next replay is the update's first shown frame: capture it for the others (GPU path: the vertex buffer is
     * the same for all of them) */
    sCapState = gPortReplayCopy && gPortGpuVtx && !gPortReplayBroken && sPalLoc >= 0 ? CAP_ARMED : CAP_NONE;
    return 0;
}

/* GPU path: which in-between frame the next replay draws (palette matrices, skinned deltas) */
void gfx_citro3d_replay_variant(int k) {
    sReplayK = k;
}

/* re-issue the recorded state calls and draws (one shown frame) */
void gfx_citro3d_replay(void) {
    extern void Port3ds_ResetFrameViewport(void);
    int i, cap = 0, check = 0;
    u32* checkAt = NULL;
    /* every shown frame starts from the frame's own viewport, as the walk did (C3D_FrameDrawOn): the first replay used
     * to start from the walk's last one */
    Port3ds_ResetFrameViewport();
    sPalTouched = 0;
    if (gPortReplayCopy && gPortGpuVtx) {
        canonUniforms();
    }
    if (sCapState == CAP_READY) {
        if (gPortReplayCopyCheck && capUsable()) {
            check = 1;
            C3Df_CaptureBegin(NULL);
            checkAt = gpuCmdBuf + gpuCmdBufOffset;
        } else if (capEmit()) {
            return;
        } else {
            sCapState = CAP_NONE; /* the rest of this update replays the log */
        }
    } else if (sCapState == CAP_ARMED) {
        cap = capBegin();
    }
    if (sRawMode) {
        rawBind(0); /* the recording started on the GPU path's program (gfx_start_frame) */
    }
    for (i = 0; i < sOpN; i++) {
        const RecOp* o = &sOps[i];
        switch (o->op) {
            case OP_SHADER:
                gfx_citro3d_load_shader(&sShaderProgramPool[o->v[0]]);
                break;
            case OP_CONSTS:
                gfx_citro3d_set_combine_consts(&sOpConsts[o->v[0]]);
                break;
            case OP_TEX:
                gfx_citro3d_select_texture(o->v[0], (u32)o->v[1]);
                break;
            case OP_SAMPLER:
                gfx_citro3d_set_sampler_parameters(o->v[0], o->v[1] != 0, (uint32_t)o->v[2], (uint32_t)o->v[3]);
                break;
            case OP_DTEST:
                gfx_citro3d_set_depth_test(o->v[0] != 0);
                break;
            case OP_DMASK:
                gfx_citro3d_set_depth_mask(o->v[0] != 0);
                break;
            case OP_DECAL:
                gfx_citro3d_set_zmode_decal(o->v[0] != 0);
                break;
            case OP_VIEWPORT:
                gfx_citro3d_set_viewport(o->v[0], o->v[1], o->v[2], o->v[3]);
                break;
            case OP_SCISSOR:
                gfx_citro3d_set_scissor(o->v[0], o->v[1], o->v[2], o->v[3]);
                break;
            case OP_ALPHA:
                gfx_citro3d_set_use_alpha(o->v[0] != 0);
                break;
            case OP_DRAWID:
                gfx_citro3d_set_draw_id(o->v[0]);
                break;
            case OP_STEREO:
                gfx_citro3d_set_stereo_mode(o->v[0]);
                break;
            case OP_GPUPAL:
                gpuUploadPalette(sOpPal[o->v[0]], o->v[1], o->v[2]);
                break;
            case OP_GPUPARAM:
                gpuUploadParams(sOpParam[o->v[0]]);
                break;
            case OP_CULL:
                gfx_citro3d_gpu_cull(o->v[0]);
                break;
            case OP_RAWMODE:
                rawBind(o->v[0]);
                break;
            case OP_RAWPARAM:
                rawUpload(&sOpRawP[o->v[0]]);
                break;
            case OP_TEXPARAM:
                sTexturePool[o->v[1]].param = (u32)o->v[2];
                texBindUnit(o->v[0], &sTexturePool[o->v[1]]);
                sTexUnits[o->v[0]] = o->v[1];
                sTexBoundOk[o->v[0]] = true;
                break;
            case OP_DRAWIDX:
                submitDraw((u32)o->v[1], 0, sIdxBuf + o->v[0]);
                {
                    extern u32 gPortPerfTris, gPortPerfDraws;
                    gPortPerfTris += (u32)o->v[1] / 3;
                    gPortPerfDraws++;
                }
                break;
        }
    }
    if (cap) {
        capEnd();
    }
    if (check) {
        static u32* sCheck;
        static u32 sCheckCap;
        u32 n = (u32)(gpuCmdBuf + gpuCmdBufOffset - checkAt), j;
        C3Df_CaptureEnd();
        if (sCheckCap < sCapN) {
            free(sCheck);
            sCheckCap = sCapN + sCapN / 4;
            sCheck = malloc(sCheckCap * sizeof(u32));
            if (sCheck == NULL) {
                sCheckCap = 0;
                return;
            }
        }
        capPatch(sCheck);
        gPortPerfCopyChecked++;
        for (j = 0; j < n && j < sCapN && checkAt[j] == sCheck[j]; j++) {
        }
        if (j < n || n != sCapN) {
            if (gPortPerfCopyBad++ == 0) {
                /* the command holding word j: register << 16 | parameter index, and the uniform index last configured */
                u32 i2 = 0, reg = 0xFFFF, par = 0, unif = 0xFFFF;
                while (i2 + 1 < n) {
                    u32 hdr = checkAt[i2 + 1], extra = (hdr >> 20) & 0xFF, r = hdr & 0xFFFF, inc = hdr >> 31, q;
                    u32 len = 2 + extra + (extra & 1);
                    if (r == GPUREG_VSH_FLOATUNIFORM_CONFIG && i2 < j) {
                        unif = checkAt[i2] & 0xFF;
                    }
                    if (j < i2 + len) {
                        q = j == i2 ? 0 : j == i2 + 1 ? 0xFF : j - i2 - 1;
                        reg = r + (inc && q != 0xFF ? q : 0), par = q;
                        break;
                    }
                    i2 += len;
                }
                gPortCopyBadInfo[4] = (reg << 16) | (par & 0xFFFF);

                gPortCopyBadInfo[5] = unif;
                gPortCopyBadInfo[0] = j;
                gPortCopyBadInfo[1] = j < n ? checkAt[j] : 0xFFFFFFFFu;
                gPortCopyBadInfo[2] = j < sCapN ? sCheck[j] : 0xFFFFFFFFu;
                gPortCopyBadInfo[3] = (n << 16) | (sCapN & 0xFFFF);
            }
        }
    }
}

/* the walk meets something replay cannot cover (an off-screen render): draw what was recorded so far
 * now, at the logic frame's own positions, and stop recording - the rest of the frame draws directly */
void gfx_citro3d_rec_abort(void) {
    if (!sRec) {
        return;
    }
    sRec = 0;
    if (sRecDirect) { /* already drawn, as an in-between frame: becomes the logic frame */
        sRecDirect = 0;
        C3Df_CaptureEnd();
        capToLogicFrame();
        sCapState = CAP_NONE;
        sOpN = sConstN = 0;
        sOpPalN = sOpParamN = 0;
        sOpRawPN = 0;
        return;
    }
    sCapState = CAP_NONE;
    gfx_citro3d_replay();
    sOpN = sConstN = 0;
    sOpPalN = sOpParamN = 0;
    sOpRawPN = 0;
}

struct GfxRenderingAPI gfx_citro3d_api = {
    gfx_citro3d_z_is_from_0_to_1,
    gfx_citro3d_unload_shader,
    gfx_citro3d_load_shader,
    gfx_citro3d_create_and_load_new_shader,
    gfx_citro3d_lookup_shader,
    gfx_citro3d_shader_get_info,
    gfx_citro3d_set_combine_consts,
    gfx_citro3d_new_texture,
    gfx_citro3d_select_texture,
    gfx_citro3d_upload_texture,
    gfx_citro3d_set_sampler_parameters,
    gfx_citro3d_set_depth_test,
    gfx_citro3d_set_depth_mask,
    gfx_citro3d_set_zmode_decal,
    gfx_citro3d_set_viewport,
    gfx_citro3d_set_scissor,
    gfx_citro3d_set_use_alpha,
    gfx_citro3d_draw_triangles,
    gfx_citro3d_init,
    gfx_citro3d_start_frame
};

#endif

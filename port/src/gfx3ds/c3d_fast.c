/*
 * c3d_fast.c - faster replacements for two citro3d 1.7.1 source files (uniforms.c, drawElements.c), linked in place
 * of the library's objects (every symbol of those objects is defined here, so the archive members are not pulled).
 *
 * ALTERED SOURCE VERSION of citro3d (https://github.com/devkitPro/citro3d), marked as such per its license:
 *   - C3D_UpdateUniforms skips clean groups of four float-uniform dirty flags with one word test (it ran for both
 *     shader stages on every draw, scanning 2 x 96 flags one byte at a time);
 *   - C3D_DrawElements writes its fixed register sequence straight into the command buffer instead of 14 calls to
 *     GPUCMD_Add;
 *   - (port additions, marked "port" below) C3D_UpdateUniforms reports where its uniform data lands while a frame is
 *     captured for replay by copy; cheap program switches.
 * Together about 11% of the Old 3DS CPU in Azahar PC samples (tools/pcprof.py, 2026-10-03). Behaviour is unchanged.
 *
 * Original license (citro3d):
 *
 * Copyright (C) 2014-2018 fincs
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any
 * damages arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any
 * purpose, including commercial applications, and to alter it and
 * redistribute it freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you
 *    must not claim that you wrote the original software. If you use
 *    this software in a product, an acknowledgment in the product
 *    documentation would be appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and
 *    must not be misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source
 *    distribution.
 */
#include <3ds.h>
#include <stddef.h>
#include <citro3d.h>

/* ---- citro3d internal.h (1.7.1): the context layout, checked against the library's own offsets below ---- */
typedef struct {
    gxCmdQueue_s gxQueue;
    u32* cmdBuf;
    size_t cmdBufSize;
    float cmdBufUsage;
    u32 flags;
    shaderProgram_s* program;
    C3D_AttrInfo attrInfo;
    C3D_BufInfo bufInfo;
    /* (the rest is not used here) */
} C3Df_ContextHead;

/* the library reads ctx->flags at 0x20 and ctx->bufInfo.base_paddr at 0x40 (renderqueue.o / drawElements.o) */
_Static_assert(offsetof(C3Df_ContextHead, flags) == 0x20, "citro3d context layout changed");
_Static_assert(offsetof(C3Df_ContextHead, bufInfo) == 0x40, "citro3d context layout changed");

#define C3DiF_DrawUsed BIT(1)
#include "c3d_fast.h"
#include "port_prof.h" /* (port: hardware sampling-profiler stages) */
extern unsigned char __C3D_Context[];
extern void C3Di_UpdateContext(void);

/* ---- uniforms.c ---- */
C3D_FVec C3D_FVUnif[2][C3D_FVUNIF_COUNT];
C3D_IVec C3D_IVUnif[2][C3D_IVUNIF_COUNT];
u16 C3D_BoolUnifs[2];

bool C3D_FVUnifDirty[2][C3D_FVUNIF_COUNT] __attribute__((aligned(4)));
bool C3D_IVUnifDirty[2][C3D_IVUNIF_COUNT] __attribute__((aligned(4)));
bool C3D_BoolUnifsDirty[2];

_Static_assert(sizeof(bool) == 1 && (C3D_FVUNIF_COUNT % 4) == 0, "dirty flags scanned four at a time");

static struct {
    bool dirty;
    int count;
    float24Uniform_s* data;
} C3Di_ShaderFVecData[2];

static bool C3Di_FVUnifEverDirty[2][C3D_FVUNIF_COUNT];
static C3Df_UnifRunFn sUnifRunFn; /* (port) */
static bool C3Di_IVUnifEverDirty[2][C3D_IVUNIF_COUNT];

void C3D_UpdateUniforms(GPU_SHADER_TYPE type) {
    int offset = type == GPU_GEOMETRY_SHADER ? (GPUREG_GSH_BOOLUNIFORM - GPUREG_VSH_BOOLUNIFORM) : 0;
    const u32* dirty4 = (const u32*)C3D_FVUnifDirty[type];
    int i = 0;
    PROF_PUSH(PROF_C3D_UNIF);

    // Update FVec uniforms that come from shader constants
    if (C3Di_ShaderFVecData[type].dirty) {
        while (i < C3Di_ShaderFVecData[type].count) {
            float24Uniform_s* u = &C3Di_ShaderFVecData[type].data[i++];
            GPUCMD_AddIncrementalWrites(GPUREG_VSH_FLOATUNIFORM_CONFIG + offset, (u32*)u, 4);
            C3D_FVUnifDirty[type][u->id] = false;
        }
        C3Di_ShaderFVecData[type].dirty = false;
        i = 0;
    }

    // Update FVec uniforms
    while (i < C3D_FVUNIF_COUNT) {
        if ((i & 3) == 0 && dirty4[i >> 2] == 0) { /* four clean flags at once */
            i += 4;
            continue;
        }
        if (!C3D_FVUnifDirty[type][i]) {
            i++;
            continue;
        }

        // Find the number of consecutive dirty uniforms
        int j;
        for (j = i; j < C3D_FVUNIF_COUNT && C3D_FVUnifDirty[type][j]; j++)
            ;

        // Upload the uniforms
        GPUCMD_AddWrite(GPUREG_VSH_FLOATUNIFORM_CONFIG + offset, 0x80000000 | i);
        if (sUnifRunFn != NULL && type == GPU_VERTEX_SHADER) { /* (port: replay capture, see C3Df_CaptureBegin) */
            sUnifRunFn(gpuCmdBuf + gpuCmdBufOffset, i, j - i);
        }
        GPUCMD_AddWrites(GPUREG_VSH_FLOATUNIFORM_DATA + offset, (u32*)&C3D_FVUnif[type][i], (j - i) * 4);

        // Clear the dirty flag
        int k;
        for (k = i; k < j; k++) {
            C3D_FVUnifDirty[type][k] = false;
            C3Di_FVUnifEverDirty[type][k] = true;
        }

        // Advance
        i = j;
    }

    // Update IVec uniforms
    if (*(const u32*)C3D_IVUnifDirty[type] != 0) {
        for (i = 0; i < C3D_IVUNIF_COUNT; i++) {
            if (!C3D_IVUnifDirty[type][i]) continue;

            GPUCMD_AddWrite(GPUREG_VSH_INTUNIFORM_I0 + offset + i, C3D_IVUnif[type][i]);
            C3D_IVUnifDirty[type][i] = false;
            C3Di_IVUnifEverDirty[type][i] = false;
        }
    }

    // Update bool uniforms
    if (C3D_BoolUnifsDirty[type]) {
        GPUCMD_AddWrite(GPUREG_VSH_BOOLUNIFORM + offset, 0x7FFF0000 | C3D_BoolUnifs[type]);
        C3D_BoolUnifsDirty[type] = false;
    }
    PROF_POP();
}

void C3Di_DirtyUniforms(GPU_SHADER_TYPE type) {
    int i;
    C3D_BoolUnifsDirty[type] = true;
    if (C3Di_ShaderFVecData[type].count)
        C3Di_ShaderFVecData[type].dirty = true;
    for (i = 0; i < C3D_FVUNIF_COUNT; i++)
        C3D_FVUnifDirty[type][i] = C3D_FVUnifDirty[type][i] || C3Di_FVUnifEverDirty[type][i];
    for (i = 0; i < C3D_IVUNIF_COUNT; i++)
        C3D_IVUnifDirty[type][i] = C3D_IVUnifDirty[type][i] || C3Di_IVUnifEverDirty[type][i];
}

void C3Di_LoadShaderUniforms(shaderInstance_s* si) {
    GPU_SHADER_TYPE type = si->dvle->type;
    if (si->boolUniformMask) {
        C3D_BoolUnifs[type] &= ~si->boolUniformMask;
        C3D_BoolUnifs[type] |= si->boolUniforms;
    }

    if (type == GPU_GEOMETRY_SHADER)
        C3D_BoolUnifs[type] &= ~BIT(15);
    C3D_BoolUnifsDirty[type] = true;

    if (si->intUniformMask) {
        int i;
        for (i = 0; i < 4; i++) {
            if (si->intUniformMask & BIT(i)) {
                C3D_IVUnif[type][i] = si->intUniforms[i];
                C3D_IVUnifDirty[type][i] = true;
            }
        }
    }
    C3Di_ShaderFVecData[type].dirty = true;
    C3Di_ShaderFVecData[type].count = si->numFloat24Uniforms;
    C3Di_ShaderFVecData[type].data = si->float24Uniforms;
}

void C3Di_ClearShaderUniforms(GPU_SHADER_TYPE type) {
    C3Di_ShaderFVecData[type].dirty = false;
    C3Di_ShaderFVecData[type].count = 0;
    C3Di_ShaderFVecData[type].data = NULL;
}

/* ---- port (2026-10-04, not part of citro3d): cheap switches between two program + vertex-format configurations ----
 * The renderer alternates between two vertex shaders of one binary (GPU path / raw path, gfx_citro3d.c rawBind), up
 * to ~50 times per frame. Through citro3d each switch re-sent the whole program setup, attribute layout, all twelve
 * attribute buffers and the program's constants (~100 command words). Here both configurations are captured once
 * (what citro3d emits for each, parsed into register writes) and a switch sends only the writes whose values differ
 * (entry point, attribute formats, buffer 0, ...). citro3d is still told about every switch (its program/attribute/
 * buffer state stays right for anything it re-sends later, e.g. after the HOME menu); only its re-emission is
 * replaced, and only while nothing else of that state is pending. Falls back to plain citro3d whenever it cannot. */
typedef struct {
    u16 reg;
    u8 mask;
    u32 val;
} C3Df_Write;
static const C3Df_Config* sCfg[2];
static u32 sSwWords[2][128];
static int sSwCount[2];
static int sSwState;    /* 0 not captured yet, 1 ready, -1 off (always plain citro3d) */
static int sConstsSame; /* both programs carry the same constants: they stay on the GPU across switches */

#define C3DiF_AttrInfo BIT(2)
#define C3DiF_BufInfo BIT(3)
#define C3DiF_Program BIT(8)
#define C3DiF_VshCode BIT(11)
#define C3DiF_GshCode BIT(12)
#define CFG_FLAGS (C3DiF_Program | C3DiF_AttrInfo | C3DiF_BufInfo | C3DiF_VshCode | C3DiF_GshCode)

static void cfgBind(const C3Df_Config* c) {
    C3D_BindProgram(c->prog);
    C3D_SetAttrInfo(c->attr);
    C3D_SetBufInfo(c->buf);
}

/* command words -> register writes (header: register, byte mask, extra parameters, consecutive flag; each command padded
 * to 8 bytes) */
static int parseWrites(const u32* w, int n, C3Df_Write* out, int max) {
    int i = 0, k = 0;
    while (i + 1 < n) {
        u32 hdr = w[i + 1];
        int reg = hdr & 0xFFFF, mask = (hdr >> 16) & 0xF, extra = (hdr >> 20) & 0xFF, inc = (int)(hdr >> 31), j;
        if (k + 1 + extra > max || i + 2 + extra > n) {
            return -1;
        }
        out[k].reg = (u16)reg, out[k].mask = (u8)mask, out[k].val = w[i], k++;
        for (j = 0; j < extra; j++, k++) {
            out[k].reg = (u16)(reg + (inc ? j + 1 : 0)), out[k].mask = (u8)mask, out[k].val = w[i + 2 + j];
        }
        i += 2 + extra;
        i += i & 1;
    }
    return k;
}

/* the writes to `reg` in a and in b, in order, are the same */
static bool sameWrites(const C3Df_Write* a, int na, const C3Df_Write* b, int nb, int reg) {
    int i = 0, j = 0;
    for (;;) {
        while (i < na && a[i].reg != reg) i++;
        while (j < nb && b[j].reg != reg) j++;
        if (i == na || j == nb) {
            return i == na && j == nb;
        }
        if (a[i].mask != b[j].mask || a[i].val != b[j].val) {
            return false;
        }
        i++, j++;
    }
}

static bool isPortReg(int reg) { /* shader code / constant upload ports: their writes only make sense in sequence */
    return (reg >= GPUREG_GSH_CODETRANSFER_END && reg <= GPUREG_GSH_OPDESCS_DATA + 7) ||
           (reg >= GPUREG_VSH_CODETRANSFER_END && reg <= GPUREG_VSH_OPDESCS_DATA + 7);
}

static void cfgCapture(void) {
    static C3Df_Write w[2][192];
    C3Df_ContextHead* ctx = (C3Df_ContextHead*)(void*)__C3D_Context;
    int n[2], o0, o1, t, i;
    const shaderInstance_s* a = sCfg[0]->prog->vertexShader;
    const shaderInstance_s* b = sCfg[1]->prog->vertexShader;
    sSwState = -1;
    C3Di_UpdateContext(); /* whatever is pending goes out first (it would with the next draw) */
    o0 = gpuCmdBufOffset;
    cfgBind(sCfg[1]);
    C3Di_UpdateContext();
    o1 = gpuCmdBufOffset;
    cfgBind(sCfg[0]); /* ends in configuration 0 */
    C3Di_UpdateContext();
    if ((ctx->flags & CFG_FLAGS) != 0) {
        return;
    }
    n[1] = parseWrites(gpuCmdBuf + o0, o1 - o0, w[1], 192);
    n[0] = parseWrites(gpuCmdBuf + o1, gpuCmdBufOffset - o1, w[0], 192);
    if (n[0] < 0 || n[1] < 0) {
        return;
    }
    for (t = 0; t < 2; t++) {
        sSwCount[t] = 0;
        for (i = 0; i < n[t]; i++) {
            const C3Df_Write* x = &w[t][i];
            if (sameWrites(w[0], n[0], w[1], n[1], x->reg)) {
                continue;
            }
            if (isPortReg(x->reg) || sSwCount[t] + 2 > (int)(sizeof(sSwWords[t]) / sizeof(u32))) {
                return; /* (would need the whole sequence: stay on plain citro3d) */
            }
            sSwWords[t][sSwCount[t]++] = x->val;
            sSwWords[t][sSwCount[t]++] = GPUCMD_HEADER(0, x->mask, x->reg);
        }
    }
    sConstsSame = a->numFloat24Uniforms == b->numFloat24Uniforms &&
                  (a->numFloat24Uniforms == 0 ||
                   memcmp(a->float24Uniforms, b->float24Uniforms, sizeof(float24Uniform_s) * a->numFloat24Uniforms) == 0);
    sSwState = 1;
}

void C3Df_SetConfigs(const C3Df_Config* c0, const C3Df_Config* c1) {
    sCfg[0] = c0, sCfg[1] = c1;
    sSwState = 0;
}

int gPortFastSwitch = 1; /* settings fastswitch=0: plain citro3d switches */
void C3Df_SelectConfig(int t) {
    C3Df_ContextHead* ctx = (C3Df_ContextHead*)(void*)__C3D_Context;
    bool clean = gPortFastSwitch && gpuCmdBuf != NULL && (ctx->flags & CFG_FLAGS) == 0;
    if (sCfg[0] == NULL || sCfg[1] == NULL) {
        return;
    }
    if (sSwState == 0 && clean) {
        cfgCapture();
        clean = (ctx->flags & CFG_FLAGS) == 0;
    }
    cfgBind(sCfg[t]); /* citro3d's own state follows every switch */
    if (sSwState == 1 && clean && (ctx->flags & (C3DiF_VshCode | C3DiF_GshCode)) == 0 &&
        gpuCmdBufOffset + (u32)sSwCount[t] <= gpuCmdBufSize) {
        ctx->flags &= ~(C3DiF_Program | C3DiF_AttrInfo | C3DiF_BufInfo);
        /* (The program's constants are still re-sent by citro3d with the next draw, 12 words. Both programs carry the
         * same ones, so skipping them is correct for the PICA, but Azahar then drew one Jabu-Jabu surface too bright in
         * one of seven runs - kept until that is understood: sConstsSame is only reported.) */
        memcpy(gpuCmdBuf + gpuCmdBufOffset, sSwWords[t], (size_t)sSwCount[t] * sizeof(u32));
        gpuCmdBufOffset += (u32)sSwCount[t];
    }
}

int C3Df_SwitchWords(int t) { /* perf report: command words per switch (0: plain citro3d); bit 15: same constants */
    return sSwState == 1 ? (sSwCount[t] | (sConstsSame ? 0x8000 : 0)) : 0;
}

/* ---- port (2026-10-05, not part of citro3d): replay by copy ----
 * gfx_citro3d.c draws an update's in-between frames by copying the first one's command words and patching the
 * per-frame matrices into them. For that the first one's words must not depend on what was on the GPU before: every
 * piece of state citro3d tracks is marked for re-sending here (the list citro3d itself re-sends after the HOME menu,
 * minus the shader code, which never changes), and the vertex-shader float uniform runs it writes are reported to
 * `fn` (data = the run's first word in the command buffer; a run is split into commands of 256 words, each with its
 * header after the first word). */
#define C3DiF_Effect BIT(4)
#define C3DiF_FrameBuf BIT(5)
#define C3DiF_Viewport BIT(6)
#define C3DiF_Scissor BIT(7)
#define C3DiF_TexEnvBuf BIT(9)
#define C3DiF_LightEnv BIT(10)
#define C3DiF_FogLut BIT(17)
#define C3DiF_TexAll (7 << 23)
#define C3DiF_TexEnvAll (0x3F << 26)
void C3Df_CaptureBegin(C3Df_UnifRunFn fn) {
    C3Df_ContextHead* ctx = (C3Df_ContextHead*)(void*)__C3D_Context;
    ctx->flags |= C3DiF_AttrInfo | C3DiF_BufInfo | C3DiF_Effect | C3DiF_FrameBuf | C3DiF_Viewport | C3DiF_Scissor |
                  C3DiF_Program | C3DiF_TexAll | C3DiF_TexEnvBuf | C3DiF_TexEnvAll | C3DiF_LightEnv | C3DiF_FogLut;
    C3Di_DirtyUniforms(GPU_VERTEX_SHADER);
    C3Di_DirtyUniforms(GPU_GEOMETRY_SHADER);
    sUnifRunFn = fn;
}

void C3Df_CaptureEnd(void) {
    sUnifRunFn = NULL;
}

/* after a copy: it drew into the frame buffer, and the uniforms it patched (first..first+count-1, extra) now differ
 * from citro3d's copies - re-sent with the next draw */
void C3Df_AfterCopy(int first, int count, int extra) {
    C3Df_ContextHead* ctx = (C3Df_ContextHead*)(void*)__C3D_Context;
    int i;
    ctx->flags |= C3DiF_DrawUsed;
    for (i = first; i >= 0 && i < first + count && i < C3D_FVUNIF_COUNT; i++) {
        C3D_FVUnifDirty[GPU_VERTEX_SHADER][i] = true;
    }
    if (extra >= 0 && extra < C3D_FVUNIF_COUNT) {
        C3D_FVUnifDirty[GPU_VERTEX_SHADER][extra] = true;
    }
}

/* ---- drawElements.c ---- */
void C3D_DrawElements(GPU_Primitive_t primitive, int count, int type, const void* indices) {
    C3Df_ContextHead* ctx = (C3Df_ContextHead*)(void*)__C3D_Context;
    u32 pa = osConvertVirtToPhys(indices);
    u32 base = ctx->bufInfo.base_paddr;
    u32* p;
    if (pa < base) return;

    PROF_PUSH(PROF_C3D_CTX);
    C3Di_UpdateContext();
    PROF_SET(PROF_C3D_DRAW);

    /* the same 15 single-register writes as citro3d's GPUCMD_AddWrite / GPUCMD_AddMaskedWrite calls: each is the
     * value followed by its header (one parameter, no padding) */
    if (!gpuCmdBuf || gpuCmdBufOffset + 2 * 15 > gpuCmdBufSize)
        svcBreak(USERBREAK_PANIC); /* as GPUCMD_Add on overflow */
    p = gpuCmdBuf + gpuCmdBufOffset;
#define W(reg, mask, val) (p[0] = (u32)(val), p[1] = GPUCMD_HEADER(0, (mask), (reg)), p += 2)
    // Set primitive type
    W(GPUREG_PRIMITIVE_CONFIG, 2, primitive != GPU_TRIANGLES ? primitive : GPU_GEOMETRY_PRIM);
    // Start a new primitive (breaks off a triangle strip/fan)
    W(GPUREG_RESTART_PRIMITIVE, 0xF, 1);
    // Configure the index buffer
    W(GPUREG_INDEXBUFFER_CONFIG, 0xF, (pa - base) | ((u32)type << 31));
    // Number of vertices
    W(GPUREG_NUMVERTICES, 0xF, count);
    // First vertex
    W(GPUREG_VERTEX_OFFSET, 0xF, 0);
    // Enable triangle element drawing mode if necessary
    if (primitive == GPU_TRIANGLES) {
        W(GPUREG_GEOSTAGE_CONFIG, 2, 0x100);
        W(GPUREG_GEOSTAGE_CONFIG2, 2, 0x100);
    }
    // Enable drawing mode
    W(GPUREG_START_DRAW_FUNC0, 1, 0);
    // Trigger element drawing
    W(GPUREG_DRAWELEMENTS, 0xF, 1);
    // Go back to configuration mode
    W(GPUREG_START_DRAW_FUNC0, 1, 1);
    // Disable triangle element drawing mode if necessary
    if (primitive == GPU_TRIANGLES) {
        W(GPUREG_GEOSTAGE_CONFIG, 2, 0);
        W(GPUREG_GEOSTAGE_CONFIG2, 2, 0);
    }
    // Clear the post-vertex cache
    W(GPUREG_VTX_FUNC, 0xF, 1);
    W(GPUREG_PRIMITIVE_CONFIG, 0x8, 0);
    W(GPUREG_PRIMITIVE_CONFIG, 0x8, 0);
#undef W
    gpuCmdBufOffset = p - gpuCmdBuf;

    ctx->flags |= C3DiF_DrawUsed;
    PROF_POP();
}

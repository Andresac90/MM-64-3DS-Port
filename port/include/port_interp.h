#ifndef PORT_INTERP_H
#define PORT_INTERP_H

/* PORT (2026-09-30): 60 fps frame-interpolation tags (renderer: gfx_pc.c interp_*).
 *
 * Same model as Zelda64Recomp's RT64 matrix groups (gEXMatrixGroup, patches/..._transform_tagging.c) and
 * Ship of Harkinian's FrameInterpolation_RecordOpenChild: the game tells the renderer WHICH transform
 * each matrix is, with a stable id, so the renderer can pair it with the same transform in the previous
 * logic frame and draw in-between frames at 60 fps. Groups nest; the renderer combines the ids along
 * the path (actor -> limb), and numbers the matrices inside a group in order.
 *
 * Encoded as G_NOOP (ignored by any other GBI consumer): w0 = G_NOOP | TAG << 16 | op << 8 | flags,
 * w1 = id (local to the parent group). */

#define PORT_INTERP_TAG 0x6E
#define PORT_INTERP_OP_PUSH 1
#define PORT_INTERP_OP_POP 2

/* flags */
#define PORT_INTERP_SKIP 0x01  /* record, but draw this frame as it is (camera cut, teleport, spawn) */
#define PORT_INTERP_VERTS 0x02 /* also blend CPU-written vertex positions (Skin system: Epona...) */

/* local ids */
#define PORT_INTERP_ID_CAMERA 0xCA3E0001u
#define PORT_INTERP_ID_SKYBOX 0x5CB00001u
#define PORT_INTERP_ID_LIMB(i) (0x11B00000u + (unsigned)(i))
#define PORT_INTERP_ID_POSTLIMB(i) (0x11C00000u + (unsigned)(i))
#define PORT_INTERP_ID_SKIN(i) (0x5C100000u + (unsigned)(i))

#ifdef __3DS__
#define gPortInterpPush(pkt, id, flags)                                                                   \
    do {                                                                                                  \
        Gfx* _g = (Gfx*)(pkt);                                                                            \
        _g->words.w0 = ((unsigned)(unsigned char)G_NOOP << 24) | (PORT_INTERP_TAG << 16) | (PORT_INTERP_OP_PUSH << 8) | \
                       ((flags)&0xFF);                                                                    \
        _g->words.w1 = (unsigned)(id);                                                                    \
    } while (0)
#define gPortInterpPop(pkt)                                                                                    \
    do {                                                                                                       \
        Gfx* _g = (Gfx*)(pkt);                                                                                 \
        _g->words.w0 = ((unsigned)(unsigned char)G_NOOP << 24) | (PORT_INTERP_TAG << 16) | (PORT_INTERP_OP_POP << 8); \
        _g->words.w1 = 0;                                                                                      \
    } while (0)
/* push/pop on both the opaque and translucent lists (outside OPEN_DISPS blocks too) */
#define gPortInterpPush2(gfxCtx, id, flags)                  \
    do {                                                     \
        gPortInterpPush((gfxCtx)->polyOpa.p++, id, flags);   \
        gPortInterpPush((gfxCtx)->polyXlu.p++, id, flags);   \
    } while (0)
#define gPortInterpPop2(gfxCtx)                 \
    do {                                        \
        gPortInterpPop((gfxCtx)->polyOpa.p++);  \
        gPortInterpPop((gfxCtx)->polyXlu.p++);  \
    } while (0)
/* the camera jumped this logic frame (View_Apply heuristic, like Zelda64Recomp camera_transform_tagging.c) */
extern int gPortInterpCameraCut;
#else
#define gPortInterpPush(pkt, id, flags) (void)0
#define gPortInterpPop(pkt) (void)0
#define gPortInterpPush2(gfxCtx, id, flags) (void)0
#define gPortInterpPop2(gfxCtx) (void)0
#endif

#endif

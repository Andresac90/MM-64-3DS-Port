#ifndef GFX_3DS_H
#define GFX_3DS_H

#include "gfx_window_manager_api.h"

#define N3DS_USE_ANTIALIASING
#define N3DS_USE_WIDE_800PX

typedef enum
{
    GFX_3DS_MODE_NORMAL,
    GFX_3DS_MODE_AA_22,
    GFX_3DS_MODE_WIDE,
    GFX_3DS_MODE_WIDE_AA_12,
    GFX_3DS_MODE_STEREO /* 3D slider up: both eyes 400x240 side by side in one 240x800 target */
} Gfx3DSMode;

/* stereoscopic 3D (gfx_3ds.c): per-eye horizontal shift at infinity in NDC, 0 = mono */
extern float gPortStereoSep;
#define STEREO_EYE_OFFSET 400 /* long-axis offset of the right eye in the stereo target */

extern struct GfxWindowManagerAPI gfx_3ds;
extern Gfx3DSMode gGfx3DSMode;

#endif
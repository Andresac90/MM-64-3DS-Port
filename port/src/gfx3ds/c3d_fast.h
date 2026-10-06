#ifndef C3D_FAST_H
#define C3D_FAST_H
/* port/src/gfx3ds/c3d_fast.c: cheap switches between two shader program + vertex format configurations */
#include <citro3d.h>
typedef struct {
    shaderProgram_s* prog;
    C3D_AttrInfo* attr;
    C3D_BufInfo* buf;
} C3Df_Config;
void C3Df_SetConfigs(const C3Df_Config* c0, const C3Df_Config* c1);
void C3Df_SelectConfig(int t);
int C3Df_SwitchWords(int t);
/* replay by copy (gfx_citro3d.c): capture one frame's commands as a self-contained stream */
typedef void (*C3Df_UnifRunFn)(const u32* data, int first, int count);
void C3Df_CaptureBegin(C3Df_UnifRunFn fn);
void C3Df_CaptureEnd(void);
void C3Df_AfterCopy(int first, int count, int extra);
#endif

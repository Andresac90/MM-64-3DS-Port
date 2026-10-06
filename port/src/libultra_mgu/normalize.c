#include "ultra64.h"
#include "math.h" /* PORT: MM (OoT: z_math.h) */
#include "z64math.h" /* SQ */

void guNormalize(f32* x, f32* y, f32* z) {
    f32 m = 1 / sqrtf(SQ(*x) + SQ(*y) + SQ(*z));

    *x *= m;
    *y *= m;
    *z *= m;
}

#ifndef PORT_PR_GBI_WRAPPER_H
#define PORT_PR_GBI_WRAPPER_H
/* PORT: the renderer (sm64-port files) includes <PR/gbi.h> without the N64 types; MM's gbi.h expects them
 * declared first. Declare them, then use MM's own header. */
#include "PR/ultratypes.h"
#include_next <PR/gbi.h>
#endif

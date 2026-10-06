/*
 * sleep_shim.c — replaces src/libc64/sleep.c. The original sleeps by waiting
 * on an OS timer message; here the host just sleeps.
 */
#include "ultra64.h"

extern int usleep();

#define CYCLES_PER_USEC 47 /* ~46.875MHz N64 counter */

void Sleep_Cycles(u64 cycles) { usleep((u32)(cycles / CYCLES_PER_USEC)); }
void Sleep_Nsec(u32 nsec) { usleep(nsec / 1000); }
void Sleep_Usec(u32 usec) { usleep(usec); }
void Sleep_Msec(u32 ms) { usleep(ms * 1000); }
void Sleep_Sec(u32 sec) { usleep(sec * 1000000); }

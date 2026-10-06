/*
 * 3ds_compat.c — newlib compatibility for the port's freestanding shims.
 * The shims declare `extern void* stderr` and call fprintf(stderr, ...), but
 * newlib makes `stderr` a macro (no global symbol). Provide a real `stderr`
 * symbol, routed to fd 1 so shim debug output lands on the bottom-screen
 * console (consoleInit GFX_BOTTOM) — useful for on-hardware debugging.
 */
#include <stdio.h>
#undef stderr

FILE* stderr;

/* Called explicitly from main() AFTER consoleInit — NOT a crt0 constructor
 * (running fdopen during early startup risked crashing before main). */
void PortCompat_InitStreams(void) {
    stderr = fdopen(1, "w");        /* fd 1 -> 3DS console */
    if (stderr) setvbuf(stderr, NULL, _IONBF, 0);
}

/* PORT (2026-09-30): once the bottom screen is the touch panel (3ds_main.c), discard the shims'
 * fprintf(stderr, ...) output so it cannot draw over the panel. */
static int PortCompat_Discard(void* cookie, const char* buf, size_t n) {
    (void)cookie;
    (void)buf;
    return (int)n;
}
void PortCompat_SilenceStderr(void) {
    FILE* sink = funopen(NULL, NULL, PortCompat_Discard, NULL, NULL);
    if (sink != NULL) {
        stderr = sink;
        stdout = sink; /* the bottom screen is the touch panel now: no console text over it */
    }
}

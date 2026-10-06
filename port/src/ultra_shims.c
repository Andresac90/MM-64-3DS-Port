#include <string.h>
/*
 * ultra_shims.c — libultra replacement layer for the OoT port.
 * Single-threaded synchronous model: the platform frame loop owns execution;
 * N64 threads are recorded but never scheduled. Message queues are real.
 */
#include "ultra64.h"
/* freestanding bring-up: libc via implicit decls (32-bit: int==ptr) */
extern int fprintf();
extern void* stderr;
struct PortTimespec { long tv_sec; long tv_nsec; };
extern int clock_gettime(int clk, struct PortTimespec* ts);
#define PORT_CLOCK_MONOTONIC 1
extern void* memset();

/* ------------------------------------------------------------------ */
/* Message queues — faithful ring-buffer semantics, minus blocking.    */
/* In a single-threaded world a blocking recv on an empty queue would  */
/* deadlock; we log it so bring-up shows us each spot that needs the   */
/* frame loop to feed it. */

void osCreateMesgQueue(OSMesgQueue* mq, OSMesg* msg, s32 count) {
    mq->validCount = 0;
    mq->first = 0;
    mq->msgCount = count;
    mq->msg = msg;
}

s32 osSendMesg(OSMesgQueue* mq, OSMesg msg, s32 flag) {
    if (mq == NULL) { return -1; }
    if (mq->validCount >= mq->msgCount) {
        (void)flag;
        return -1; /* BLOCK defused: single-threaded bring-up */
    }
    mq->msg[(mq->first + mq->validCount) % mq->msgCount] = msg;
    mq->validCount++;
    return 0;
}

s32 osJamMesg(OSMesgQueue* mq, OSMesg msg, s32 flag) {
    if (mq == NULL) { return -1; }
    if (mq->validCount >= mq->msgCount) {
        (void)flag;
        return -1; /* BLOCK defused: single-threaded bring-up */
    }
    mq->first = (mq->first + mq->msgCount - 1) % mq->msgCount;
    mq->msg[mq->first] = msg;
    mq->validCount++;
    return 0;
}

s32 osRecvMesg(OSMesgQueue* mq, OSMesg* msg, s32 flag) {
    if (mq == NULL) { return -1; }
    if (mq->validCount == 0) {
        (void)flag;
        return -1; /* BLOCK defused: single-threaded bring-up */
    }
    if (msg != NULL) {
        *msg = mq->msg[mq->first];
    }
    mq->first = (mq->first + 1) % mq->msgCount;
    mq->validCount--;
    return 0;
}

/* ------------------------------------------------------------------ */
/* Threads — recorded, never run. The frame loop calls what matters.   */

static OSThread* sThreads[32];
static int sThreadCount = 0;

void osCreateThread(OSThread* t, OSId id, void (*entry)(void*), void* arg, void* sp, OSPri pri) {
    (void)sp;
    t->id = id;
    t->priority = pri;
    if (sThreadCount < 32) {
        sThreads[sThreadCount++] = t;
    }
    fprintf(stderr, "[shim] thread %d created (entry %p) — not scheduled\n", (int)id, (void*)entry);
}

void osStartThread(OSThread* t) { (void)t; }
void osStopThread(OSThread* t) { (void)t; }
void osDestroyThread(OSThread* t) { (void)t; }
void osYieldThread(void) {}
OSId osGetThreadId(OSThread* t) { return (t != NULL) ? t->id : 0; }
OSPri osGetThreadPri(OSThread* t) { return (t != NULL) ? t->priority : 0; }
void osSetThreadPri(OSThread* t, OSPri pri) { if (t != NULL) { t->priority = pri; } }

/* ------------------------------------------------------------------ */
/* Events, interrupts, timers                                          */

static OSMesgQueue* sEventQueues[OS_NUM_EVENTS];
static OSMesg sEventMesgs[OS_NUM_EVENTS];

void osSetEventMesg(OSEvent e, OSMesgQueue* mq, OSMesg msg) {
    if (e < OS_NUM_EVENTS) {
        sEventQueues[e] = mq;
        sEventMesgs[e] = msg;
    }
}

/* The platform loop calls this to deliver VI retrace etc. */
void PortShim_RaiseEvent(OSEvent e) {
    if (e < OS_NUM_EVENTS && sEventQueues[e] != NULL) {
        osSendMesg(sEventQueues[e], sEventMesgs[e], OS_MESG_NOBLOCK);
    }
}

OSIntMask osSetIntMask(OSIntMask mask) { (void)mask; return OS_IM_ALL; }
u32 osGetIntMask(void) { return OS_IM_ALL; }

#ifdef __3DS__
extern u64 svcGetSystemTick(void);
/* OS_CLOCK_RATE ticks (46.875 MHz counter on N64) from the 268.111856 MHz system tick: tick * 46875000 / 268111856
 * as a 32.32 fixed-point product (clock_gettime's 64-bit software divisions were measurable on the Old 3DS) */
static u64 TicksNow(void) {
    const u64 k = 750905219ULL; /* 46875000 / 268111856 * 2^32 */
    u64 t = svcGetSystemTick();
    return (t >> 32) * k + (((t & 0xFFFFFFFFULL) * k) >> 32);
}
#else
static u64 TicksNow(void) {
    struct PortTimespec ts;
    clock_gettime(PORT_CLOCK_MONOTONIC, &ts);
    /* OS_CLOCK_RATE ticks (46.875 MHz counter on N64) */
    return (u64)ts.tv_sec * 46875000ULL + (u64)ts.tv_nsec * 46875ULL / 1000000ULL;
}
#endif

#ifndef __3DS__
/* on 3DS, libctru provides osGetTime — avoid the symbol collision */
OSTime osGetTime(void) { return TicksNow(); }
#endif
void osSetTime(OSTime t) { (void)t; }
u32 osGetCount(void) { return (u32)TicksNow(); }

s32 osSetTimer(OSTimer* timer, OSTime countdown, OSTime interval, OSMesgQueue* mq, OSMesg msg) {
    (void)timer; (void)countdown; (void)interval; (void)mq; (void)msg;
    return 0; /* frame loop will deliver time-based messages */
}
s32 osStopTimer(OSTimer* timer) { (void)timer; return 0; }

/* ------------------------------------------------------------------ */
/* Cache / TLB / misc CPU — meaningless on the host                    */

void osInvalDCache(void* addr, s32 size) { (void)addr; (void)size; }
void osInvalICache(void* addr, s32 size) { (void)addr; (void)size; }
void osWritebackDCache(void* addr, s32 size) { (void)addr; (void)size; }
void osWritebackDCacheAll(void) {}
u32 osVirtualToPhysical(void* addr) { return (u32)(uintptr_t)addr; }
void osMapTLBRdb(void) {}
void osUnmapTLBAll(void) {}
s32 osAfterPreNMI(void) { return 0; }
void osInitialize(void) {}
u32 osGetMemSize(void) { return 8 * 1024 * 1024; }

/* ------------------------------------------------------------------ */
/* VI — frame presentation is the platform layer's job                 */

static void* sNextFb = NULL;
static void* sCurrentFb = NULL;

void osViSetMode(OSViMode* mode) { (void)mode; }
void osViSetSpecialFeatures(u32 func) { (void)func; }
void osViSetXScale(f32 v) { (void)v; }
void osViSetYScale(f32 v) { (void)v; }
void osViExtendVStart(u32 v) { (void)v; }
void osViBlack(u8 active) { (void)active; }
static u32 sSwapCount = 0;
void osViSwapBuffer(void* fb) {
    sNextFb = fb;
    sSwapCount++;
    if ((sSwapCount % 60) == 1) {
        fprintf(stderr, "[vi] frame %d (fb %p)\n", sSwapCount, fb);
    }
}
void* osViGetNextFramebuffer(void) { return sNextFb; }
void* osViGetCurrentFramebuffer(void) { return (sCurrentFb != NULL) ? sCurrentFb : sNextFb; }
void osViSetEvent(OSMesgQueue* mq, OSMesg msg, u32 retraceCount) {
    (void)retraceCount;
    osSetEventMesg(OS_EVENT_VI, mq, msg);
}
void osCreateViManager(OSPri pri) { (void)pri; }

/* ------------------------------------------------------------------ */
/* Controller — zeroed pad until the input backend lands               */

s32 osContInit(OSMesgQueue* mq, u8* bitpattern, OSContStatus* status) {
    (void)mq;
    *bitpattern = 1; /* controller 1 present */
    memset(status, 0, 4 * sizeof(OSContStatus));
    status[0].type = CONT_TYPE_NORMAL;
    return 0;
}
s32 osContStartReadData(OSMesgQueue* mq) { PortShim_RaiseEvent(OS_EVENT_SI); (void)mq; return 0; }

/* keyboard -> pad, implemented in pc_gfx.c */
extern unsigned short PortInput_GetPad(signed char* outX, signed char* outY);

void osContGetReadData(OSContPad* pad) {
    signed char sx = 0, sy = 0;
    memset(pad, 0, 4 * sizeof(OSContPad));
    pad[0].button = PortInput_GetPad(&sx, &sy);
    pad[0].stick_x = sx;
    pad[0].stick_y = sy;
}
s32 osContStartQuery(OSMesgQueue* mq) { (void)mq; return 0; }
void osContGetQuery(OSContStatus* status) { memset(status, 0, 4 * sizeof(OSContStatus)); }
s32 osContSetCh(u8 ch) { (void)ch; return 0; }

/* Pump the game's PadMgr from live 3DS buttons once per frame. Normally a
 * dedicated controller thread runs PadMgr_HandleRetrace on each VI retrace, but
 * in the synchronous model that thread never executes, so gPadMgr.inputs[] stays
 * zero and the game ignores input. We drive it cooperatively here. We call the
 * two inner steps directly rather than PadMgr_HandleRetrace: the latter's
 * osContGetQuery path would zero validCtrlrsMask and mark controller 1 absent.
 * Called from PortGfx_RunTask (3ds_main.c) after the hardware poll. */
#include "padmgr.h"
extern void PadMgr_UpdateInputs(PadMgr* padMgr); /* global in padmgr.c, not in the header */
void Port3ds_PumpInput(void) {
    osContGetReadData(gPadMgr.pads); /* pads[0] <- live 3DS buttons via the shim above */
    PadMgr_UpdateInputs(&gPadMgr);   /* fills inputs[] incl. press/release edges */
}

/* Pump the audio driver once per frame. AudioMgr_ThreadEntry never runs
 * (osStartThread is a no-op), so AudioThread_Update is never called and no audio
 * RSP task is ever built or dispatched. Drive the per-retrace handler directly:
 * it forwards the previous frame's Acmd task to the scheduler (-> the C audio
 * microcode in audio_microcode.c) and builds the next one. Mirrors the PadMgr
 * pattern above. Called from PortGfx_RunTask (3ds_main.c). */
#include "audiomgr.h"
#include "regs.h"
#include "global.h" /* PORT: was OoT audio.h */
extern AudioMgr sAudioMgr;                         /* global instance in main.c */
extern void AudioMgr_HandleRetrace(AudioMgr* audioMgr);
extern void PortDbgX(const char* label, unsigned val);
static void Port3ds_PumpAudio_impl(void);
void Port3ds_PumpAudio(void) {
    extern volatile unsigned char gPortProf;
    extern void Port3ds_ScanBetweenUpdates(void);
    unsigned char prev = gPortProf;
    Port3ds_ScanBetweenUpdates(); /* (3ds_main.c: taps between updates) */
    gPortProf = 10; /* PROF_AUDIO (port_prof.h) */
    Port3ds_PumpAudio_impl();
    gPortProf = prev;
}
static void Port3ds_PumpAudio_impl(void) {
    /* The threadless port never runs cic6105/AudioMgr_ThreadEntry which set this
     * to ALL, so it can be stuck inhibiting audio updates. Force ALL each frame. */
    R_AUDIOMGR_ACTIVITY_LEVEL = AUDIOMGR_ACTIVITY_LEVEL_ALL;
    /* Flush the game's queued audio commands to the audio thread each frame.
     * Normally Audio_Update does this, but a scene audio-reset sets D_80133418=1
     * which blocks Audio_Update's body -> the reset command never flushes ->
     * never acks -> deadlock. Flushing here lets AudioThread_Update process the
     * reset so it acks (func_800E5EDC) and clears D_80133418, unblocking audio. */
    /* Break the spec-reset handshake deadlock: func_800F71BC (scene audio reset)
     * sets D_80133418=1 and waits for func_800E5EDC's ack (a message on
     * audioResetQueueP with the matching specId), which the free-running audio
     * thread would post on reset completion. The synchronous port's reset already
     * completed (resetStatus=0), so post the ack ourselves when a wait is pending.
     * This lets func_800FAD34 clear the flag AND run func_800F7170 (restart SFX +
     * unmute), which force-clearing the flag would skip. */
    { extern unsigned char D_80133418;
      if (D_80133418 != 0) {
          osSendMesg(gAudioCtx.audioResetQueueP, (OSMesg)(unsigned)gAudioCtx.specId, OS_MESG_NOBLOCK);
      } }
    /* NOTE: do NOT flush cmds here - Audio_Update (now unblocked) owns the
     * ScheduleProcessCmds flush; a second flush corrupts the read-pos/STOP state. */
    {
        extern u64 gPortPerfAudioMain;
        extern u64 svcGetSystemTick(void);
        u64 t0 = svcGetSystemTick();
        AudioMgr_HandleRetrace(&sAudioMgr);
        gPortPerfAudioMain += svcGetSystemTick() - t0; /* engine + any wait for the previous task */
    }
}

/* Rumble + Controller Pak: absent hardware */
s32 osMotorInit(OSMesgQueue* mq, OSPfs* pfs, s32 channel) { (void)mq; (void)pfs; (void)channel; return 1; }
s32 __osMotorAccess(OSPfs* pfs, s32 flag) { (void)pfs; (void)flag; return 1; }

/* PORT (2026-09-29): touch panel state (3ds_main.c draws it; see port_minimap.h and
 * docs/3ds-touch-panel.md). The game side fills gPortMinimap / gPortHud*; the input side sets the
 * gPortTouch* one-shot requests that z_player.c consumes. */
#include "global.h" /* PORT: was OoT save.h */
#include "global.h" /* PORT: was OoT item.h */
#include "port_minimap.h"
#include "global.h" /* PORT: was OoT inventory.h */
#include "global.h" /* PORT: was OoT interface.h */
#include "global.h" /* PORT: was OoT dma.h */
#include "segment_symbols.h"
PortMinimap gPortMinimap;
int gPortMinimapOnBottom = 1;
const unsigned char* gPortHudIconSeg;
unsigned int gPortHudSerial;
int gPortHudKeys = -1;
int gPortHudNavi;
int gPortHudTop = 1;
volatile int gPortTouchOcarina, gPortTouchBoots;
volatile int gPortTouchPage = -1;

static short Port_AmmoFor(int item) {
    switch (item) {
        case ITEM_DEKU_STICK: case ITEM_DEKU_NUT: case ITEM_BOMB: case ITEM_BOW: case ITEM_SLINGSHOT:
        case ITEM_BOMBCHU: case ITEM_MAGIC_BEAN:
            return AMMO(item);
        case ITEM_BOW_FIRE: case ITEM_BOW_ICE: case ITEM_BOW_LIGHT:
            return AMMO(ITEM_BOW);
        default:
            return -1;
    }
}

void Port_GetHudInfo(PortHudInfo* h) {
    int i;
    memset(h, 0, sizeof(*h));
    h->rupees = h->keys = -1;
    h->ocarina = ITEM_NONE;
    for (i = 0; i < 3; i++) h->cItem[i] = ITEM_NONE, h->cDisabled[i] = 1, h->cAmmo[i] = -1;
    if (gSaveContext.gameMode != GAMEMODE_NORMAL || gSaveContext.save.info.playerData.healthCapacity == 0) {
        return; /* boot logo, title, file select: no save loaded yet */
    }
    h->valid = 1;
    h->rupees = gSaveContext.save.info.playerData.rupees;
    h->keys = gPortHudKeys;
    h->health = gSaveContext.save.info.playerData.health;
    h->healthCapacity = gSaveContext.save.info.playerData.healthCapacity;
    h->magic = gSaveContext.save.info.playerData.magic;
    h->magicCapacity = gSaveContext.save.info.playerData.magicLevel != 0 ? gSaveContext.magicCapacity : 0;
    h->boots = CUR_EQUIP_VALUE(EQUIP_TYPE_BOOTS);
    h->ocarina = INV_CONTENT(ITEM_OCARINA_FAIRY);
    for (i = 0; i < 3; i++) {
        h->cItem[i] = gSaveContext.save.info.equips.buttonItems[i + 1];
        h->cDisabled[i] = gSaveContext.buttonStatus[i + 1] == BTN_DISABLED;
        h->cAmmo[i] = Port_AmmoFor(h->cItem[i]);
    }
}

/* 32x32 RGBA32 icon for any item id (icon_item_static), read from ROM once and cached; NULL if out of
 * range. For panel buttons whose item is not on a C button (boots, ocarina). Game thread only. */
const unsigned char* Port_GetItemIcon(int itemId) {
    enum { N = 8 };
    static unsigned char sIcons[N][ITEM_ICON_SIZE] __attribute__((aligned(8)));
    static short sIds[N] = { -1, -1, -1, -1, -1, -1, -1, -1 };
    static int sNext;
    int i;
    if (itemId < 0 || itemId > ITEM_BOOTS_HOVER) return NULL;
    for (i = 0; i < N; i++) if (sIds[i] == itemId) return sIcons[i];
    i = sNext;
    sNext = (sNext + 1) % N;
    DmaMgr_RequestSync(sIcons[i], GET_ITEM_ICON_VROM(itemId), ITEM_ICON_SIZE);
    sIds[i] = (short)itemId;
    return sIcons[i];
}

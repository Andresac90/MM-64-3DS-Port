/*
 * sched_shim.c — replaces src/code/sched.c. Synchronous scheduler: tasks are
 * executed the moment Sched_Notify fires. Graphics tasks go to the platform
 * renderer hook; audio tasks complete instantly (no RSP).
 */
#include "ultra64.h"
#include "global.h" /* PORT: MM: Scheduler in scheduler.h via global.h */

extern int fprintf();
extern void* stderr;

/* renderer hook — pc_gfx.c overrides; default counts frames silently */
void __attribute__((weak)) PortGfx_RunTask(OSTask* task) {
    static u32 frames = 0;
    frames++;
    if ((frames % 256) == 1) {
        fprintf(stderr, "[gfx] task %u (dl %p) — renderer not attached\n",
                frames, (void*)task->t.data_ptr);
    }
}

void Sched_Init(Scheduler* sc, void* stack, OSPri priority, u8 viModeType, UNK_TYPE arg4, IrqMgr* irqMgr) {
    (void)stack; (void)priority; (void)viModeType; (void)arg4; (void)irqMgr;
    osCreateMesgQueue(&sc->cmdQueue, sc->cmdMsgBuf, 8);
    osCreateMesgQueue(&sc->interruptQueue, sc->interruptMsgBuf, 8);
}

static void Sched_RunTask(OSScTask* task) {
    if (task == NULL) {
        return;
    }
    if (task->list.t.type == M_GFXTASK) {
        PortGfx_RunTask(&task->list);
        if ((task->flags & OS_SC_SWAPBUFFER) && task->framebuffer != NULL) { /* (MM: framebuffer is the CfbInfo) */
            osViSwapBuffer(((CfbInfo*)task->framebuffer)->swapBuffer);
        }
    }
    /* audio tasks: execute the Acmd list (C reimpl of aspMain) on the audio worker core, like the RSP
     * running beside the CPU (port/src/audio_3ds.c) */
    if (task->list.t.type == M_AUDTASK) {
        extern void Port3ds_AudioTaskRun(void* task);
        _Static_assert(sizeof(OSTask) == 64, "audio_3ds.c copies OSTask as 64 bytes");
        Port3ds_AudioTaskRun(&task->list);
    }
    if (task->msgQ != NULL) {
        osSendMesg(task->msgQ, task->msg, OS_MESG_NOBLOCK);
    }
}

/* (MM: the game wakes the scheduler with Sched_SendNotifyMsg) */
void Sched_SendNotifyMsg(Scheduler* sc) {
    OSMesg msg;

    while (osRecvMesg(&sc->cmdQueue, &msg, OS_MESG_NOBLOCK) == 0) {
        Sched_RunTask((OSScTask*)msg);
    }
}

void Sched_Notify(Scheduler* sc) { Sched_SendNotifyMsg(sc); } /* (the OoT name, for port code) */
void Sched_SendAudioCancelMsg(Scheduler* sc) { (void)sc; } /* tasks run at once: nothing to cancel */
void Sched_SendGfxCancelMsg(Scheduler* sc) { (void)sc; }
void Sched_FlushTaskQueue(void) {}

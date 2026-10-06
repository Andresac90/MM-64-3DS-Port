/*
 * ultra_shims2.c — second shim wave: RSP tasks, PI/cart, AI audio interface,
 * Controller Pak, and microcode blob symbols.
 */
#include "ultra64.h"
extern int fprintf(); extern void* stderr;
extern void* memset();

/* ------------------------------------------------------------------ */
/* RSP microcode blobs — never executed; only their symbols are linked. */
/* sys_ucode.c hands these to the scheduler, which we replace.          */

u64 rspbootTextStart[1];
u64 rspbootTextEnd[1];
u64 gspF3DZEX2_NoN_PosLight_fifoTextStart[1];
u64 gspF3DZEX2_NoN_PosLight_fifoTextEnd[1];
u64 gspF3DZEX2_NoN_PosLight_fifoDataStart[1];
u64 gspF3DZEX2_NoN_PosLight_fifoDataEnd[1];
u64 gspF3DZEX2_NoN_fifoTextStart[1];
u64 gspF3DZEX2_NoN_fifoTextEnd[1];
u64 gspF3DZEX2_NoN_fifoDataStart[1];
u64 gspF3DZEX2_NoN_fifoDataEnd[1];
u64 gspS2DEX2d_fifoTextStart[1];
u64 gspS2DEX2d_fifoTextEnd[1];
u64 gspS2DEX2d_fifoDataStart[1];
u64 gspS2DEX2d_fifoDataEnd[1];
u64 aspMainTextStart[1];
u64 aspMainTextEnd[1];
u64 aspMainDataStart[1];
u64 aspMainDataEnd[1];
u64 njpgdspMainTextStart[1];
u64 njpgdspMainTextEnd[1];
u64 njpgdspMainDataStart[1];
u64 njpgdspMainDataEnd[1];

/* ------------------------------------------------------------------ */
/* RSP task submission. The platform loop intercepts graphics tasks    */
/* before they reach here (scheduler replacement); audio tasks are     */
/* replaced by direct synthesis calls. Anything landing here is a      */
/* bring-up signal, not a crash.                                       */

void osSpTaskLoad(OSTask* task) { (void)task; }
void osSpTaskStartGo(OSTask* task) {
    fprintf(stderr, "[shim] osSpTaskStartGo(type=%d) — RSP task ignored\n",
            (int)task->t.type);
}
void osSpTaskYield(void) {}
OSYieldResult osSpTaskYielded(OSTask* task) { (void)task; return 0; }

u32 osDpGetStatus(void) { return 0; }
void osDpSetStatus(u32 status) { (void)status; }

/* ------------------------------------------------------------------ */
/* PI / cartridge                                                      */

static OSMesgQueue sPiCmdQueue;
static OSMesg sPiCmdMsgs[8];
static OSPiHandle sCartHandle;

OSPiHandle* osCartRomInit(void) {
    memset(&sCartHandle, 0, sizeof(sCartHandle));
    return &sCartHandle;
}
OSPiHandle* osDriveRomInit(void) { return NULL; }
s32 osLeoDiskInit(void) { return -1; }

void osCreatePiManager(OSPri pri, OSMesgQueue* cmdQ, OSMesg* cmdBuf, s32 cmdMsgCnt) {
    (void)pri; (void)cmdQ; (void)cmdBuf; (void)cmdMsgCnt;
    osCreateMesgQueue(&sPiCmdQueue, sPiCmdMsgs, 8);
}
OSMesgQueue* osPiGetCmdQueue(void) { return &sPiCmdQueue; }

/* EPI raw IO: only the boot chain touches these. */
/* PORT (2026-09-24): read the real cartridge word. Locale_Init reads the ROM header at 0x3C
 * and indexes it as BYTES (regionInfo[2] = country code 'E' for US), so copy the 4 bytes in
 * ROM memory order. It was stubbed to 0 -> region unknown -> save language JPN (Japanese
 * pause-menu text on the US ROM). */
extern void PortDma_ReadRomRaw(void* dst, u32 offset, u32 n);
s32 osEPiReadIo(OSPiHandle* h, u32 devAddr, u32* data) {
    (void)h;
    PortDma_ReadRomRaw(data, devAddr & 0x0FFFFFFFu, 4);
    return 0;
}
s32 osEPiWriteIo(OSPiHandle* h, u32 devAddr, u32 data) { (void)h; (void)devAddr; (void)data; return 0; }

/* Cartridge DMA → ROM image (PortDma in dma_shim.c). */
extern s32 DmaMgr_AudioDmaHandler(OSPiHandle* pihandle, OSIoMesg* mb, s32 direction);
#ifdef __3DS__
extern s32 PortSram_Dma(OSIoMesg* mb, s32 direction); /* save data -> SD file */
#endif
s32 osEPiStartDma(OSPiHandle* h, OSIoMesg* mb, s32 direction) {
#ifdef __3DS__
    /* SRAM save window (cart 0x08000000, 32KB) is persisted to SD, not the ROM. */
    u32 dev = (u32)mb->devAddr;
    if (dev >= 0x08000000u && dev < 0x08008000u) {
        return PortSram_Dma(mb, direction);
    }
#endif
    return DmaMgr_AudioDmaHandler(h, mb, direction);
}

/* ------------------------------------------------------------------ */
/* AI — audio DAC interface; real output lands with the audio backend. */

/* osAiSetNextBuffer is provided by the game's own src/audio/internal/os.c. */
static u32 sAiFreq = 32000;
s32 osAiSetFrequency(u32 freq) { sAiFreq = freq; return freq; }
/* bytes still queued for playback (N64: left in the current AI DMA); drives the engine's
 * per-frame audio length feedback (AudioThread_UpdateImpl) so production tracks ndsp's rate */
u32 osAiGetLength(void) {
    extern int Port3ds_AudioQueuedFrames(void);
    return (u32)Port3ds_AudioQueuedFrames() * 4;
}

/* ------------------------------------------------------------------ */
/* Controller Pak — absent; every call reports no pak.                 */

s32 osPfsInitPak(OSMesgQueue* mq, OSPfs* pfs, s32 channel) { (void)mq; (void)pfs; (void)channel; return 1; }
s32 osPfsIsPlug(OSMesgQueue* mq, u8* pattern) { (void)mq; *pattern = 0; return 0; }
s32 osPfsFreeBlocks(OSPfs* pfs, s32* bytes) { (void)pfs; *bytes = 0; return 1; }
s32 osPfsAllocateFile(OSPfs* pfs, u16 cc, u32 gc, u8* gn, u8* en, s32 size, s32* fileNo) {
    (void)pfs; (void)cc; (void)gc; (void)gn; (void)en; (void)size; (void)fileNo; return 1;
}
s32 osPfsDeleteFile(OSPfs* pfs, u16 cc, u32 gc, u8* gn, u8* en) {
    (void)pfs; (void)cc; (void)gc; (void)gn; (void)en; return 1;
}
s32 osPfsFindFile(OSPfs* pfs, u16 cc, u32 gc, u8* gn, u8* en, s32* fileNo) {
    (void)pfs; (void)cc; (void)gc; (void)gn; (void)en; (void)fileNo; return 1;
}
s32 osPfsFileState(OSPfs* pfs, s32 fileNo, OSPfsState* state) { (void)pfs; (void)fileNo; (void)state; return 1; }
s32 osPfsReadWriteFile(OSPfs* pfs, s32 fileNo, u8 flag, s32 offset, s32 size, u8* data) {
    (void)pfs; (void)fileNo; (void)flag; (void)offset; (void)size; (void)data; return 1;
}
s32 osPfsChecker(OSPfs* pfs) { (void)pfs; return 1; }

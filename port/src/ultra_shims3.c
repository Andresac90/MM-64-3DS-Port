/*
 * ultra_shims3.c — raw RCP/SI register access, interrupt/FPU control, and the
 * 64DD "Leo" library (drive absent: every call reports failure/no disk).
 * Signatures match include/ultra64/*.h exactly.
 */
#include "ultra64.h"

/* --- RSP/RDP/SI raw access --- */
u32 __osSpGetStatus(void) { return 0; }
void __osSpSetStatus(u32 status) { (void)status; }
s32 __osSpSetPc(void* pc) { (void)pc; return 0; }
s32 __osSpRawStartDma(s32 direction, void* devAddr, void* dramAddr, u32 size) {
    (void)direction; (void)devAddr; (void)dramAddr; (void)size;
    return 0;
}
u32 __osSpDeviceBusy(void) { return 0; }
u32 __osDpDeviceBusy(void) { return 0; }
s32 __osSiDeviceBusy(void) { return 0; }

/* --- interrupt + FPU control --- */
s32 __osDisableInt(void) { return 0; }
void __osRestoreInt(s32 mask) { (void)mask; }
void __osSetFpcCsr(u32 v) { (void)v; }
u32 __osGetFpcCsr(void) { return 0; }

/* --- fault/exception introspection (fault_n64.c) --- */
OSThread* __osGetCurrFaultedThread(void) { return 0; }
OSThread* __osGetActiveQueue(void) { return 0; }

/* --- PI access table + CIC boot blob --- */
OSPiHandle* __osPiTable = 0;
u64 cic6105TextStart[1];

/* --- 64DD (Leo) library: no drive --- */
#define LEO_NO_DRIVE 0x23 /* arbitrary nonzero error */
s32 LeoDriveExist(void) { return 0; }
s32 LeoCJCreateLeoManager(s32 cmdPri, s32 intPri, OSMesgQueue* mq, OSMesg* msgBuf) {
    (void)cmdPri; (void)intPri; (void)mq; (void)msgBuf;
    return LEO_NO_DRIVE;
}
s32 LeoCACreateLeoManager(s32 cmdPri, s32 intPri, OSMesgQueue* mq, OSMesg* msgBuf) {
    (void)cmdPri; (void)intPri; (void)mq; (void)msgBuf;
    return LEO_NO_DRIVE;
}
s32 LeoReset(void) { return LEO_NO_DRIVE; }
s32 LeoResetClear(void) { return LEO_NO_DRIVE; }
s32 LeoReadWrite(void* req, s32 direction, u32 lba, void* buf, u32 nLbas) {
    (void)req; (void)direction; (void)lba; (void)buf; (void)nLbas;
    return LEO_NO_DRIVE;
}
s32 LeoSeek(u32 lba) { (void)lba; return LEO_NO_DRIVE; }
s32 LeoSpdlMotor(s32 mode) { (void)mode; return LEO_NO_DRIVE; }
s32 LeoTestUnitReady(void* status) { (void)status; return LEO_NO_DRIVE; }
s32 LeoClearQueue(void) { return LEO_NO_DRIVE; }
u32 LeoLBAToByte(u32 lba, u32 nLbas) { (void)lba; (void)nLbas; return 0; }
u32 LeoByteToLBA(u32 startLba, u32 nBytes, void* nLbas) {
    (void)startLba; (void)nBytes; (void)nLbas;
    return 0;
}
s32 LeoGetKAdr(s32 sound) { (void)sound; return 0; }
s32 LeoGetAAdr(u32 lba, void* result) { (void)lba; (void)result; return 0; }
s32 LeoGetAAdr2(u32 lba, void* a, void* b, void* c) {
    (void)lba; (void)a; (void)b; (void)c;
    return 0;
}
s32 DmaMgr_DmaFromDriveRom(void* ram, uintptr_t rom, u32 size) {
    (void)ram; (void)rom; (void)size;
    return -1;
}

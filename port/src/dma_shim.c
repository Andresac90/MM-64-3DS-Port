/*
 * dma_shim.c — replaces src/boot/z_std_dma.c.
 * All "ROM" reads are served from the decompressed ROM (baserom-decompressed.z64),
 * which uses VROM addressing — the same address space as _*SegmentRomStart symbol
 * values and the dmadata table.
 *
 * 3DS: the ROM (~55MB) is far too big to hold in RAM alongside the binary, and
 * our malloc is the N64 arena (not ready at PortDma_Init time). So on 3DS we
 * STREAM: keep the FILE* open and fseek+fread each request directly into the
 * caller's RAM buffer. PC keeps the whole ROM in a malloc'd image.
 *
 * Bring-up note: data served from the ROM is big-endian, exactly as on cartridge.
 * Natively-linked asset symbols (liboot_assets.a) are little-endian and served via
 * the VROM->native blob remap (gVromMap) without touching the ROM file.
 */
#include "ultra64.h"
extern int fprintf(); extern void* stderr;
extern void* fopen(); extern int fseek(); extern long ftell(); extern u32 fread(); extern int fclose();
extern u32 fwrite(); extern int rename(); extern int remove();
extern void* malloc(); extern void exit(); extern void* memcpy(); extern void* memset();
#define SEEK_SET 0
#define SEEK_END 2

static u32 sRomSize = 0;

#ifdef __3DS__
static void* sRomFile = NULL;   /* kept open; streamed per request */
static void PortDma_CacheAudio(void);

void PortDma_Init(const char* romPath) {
    sRomFile = fopen(romPath, "rb");
    if (sRomFile == NULL) {
        fprintf(stderr, "[dma] cannot open %s\n", romPath);
        exit(1);
    }
    fseek(sRomFile, 0, SEEK_END);
    sRomSize = (u32)ftell(sRomFile);
    fseek(sRomFile, 0, SEEK_SET);
    fprintf(stderr, "[dma] rom streaming: %u bytes\n", sRomSize);
    PortDma_CacheAudio();
}

static void ReadRom(void* dst, uintptr_t vrom, u32 n) {
    fseek(sRomFile, (long)vrom, SEEK_SET);
    fread(dst, 1, n, sRomFile);
}
#define ROM_READY() (sRomFile != NULL)

/* PORT (2026-09-29): the audio engine streams cartridge-medium samples with many small DMAs every audio
 * update (fast cart reads on N64). Served from the SD card each one was an fseek+fread, which made audio
 * lag and stalled frames on hardware. The audio segments are contiguous in the ROM (sequences, soundfonts,
 * sample table: ~4.8 MB), so they are read into linear memory once at boot and served from there.
 * If the allocation fails, audio DMAs keep streaming. */
extern u8 _AudiobankSegmentRomStart[];
extern u8 _AudiotableSegmentRomEnd[];
extern void* linearAlloc(u32 size);
static u8* sAudioRom = NULL;
static uintptr_t sAudioRomStart, sAudioRomEnd;

static void PortDma_CacheAudio(void) {
    u32 n;
    sAudioRomStart = (uintptr_t)_AudiobankSegmentRomStart;
    sAudioRomEnd = (uintptr_t)_AudiotableSegmentRomEnd;
    if (sAudioRomEnd <= sAudioRomStart || sAudioRomEnd > sRomSize) {
        return;
    }
    n = (u32)(sAudioRomEnd - sAudioRomStart);
    sAudioRom = linearAlloc(n);
    if (sAudioRom == NULL) {
        fprintf(stderr, "[dma] no memory to cache audio (%u bytes): streaming it\n", n);
        return;
    }
    ReadRom(sAudioRom, sAudioRomStart, n);
    fprintf(stderr, "[dma] audio ROM cached in RAM: %u bytes\n", n);
}

/* audio-range request served from RAM? (then no SD read, and no texture cache can live there) */
static int PortDma_ReadAudio(void* ram, uintptr_t vrom, u32 size) {
    if (sAudioRom == NULL || vrom < sAudioRomStart || vrom + size > sAudioRomEnd || vrom + size < vrom) {
        return 0;
    }
    memcpy(ram, sAudioRom + (vrom - sAudioRomStart), size);
    return 1;
}

#else  /* PC: whole ROM in RAM */
static u8* sRomImage = NULL;

void PortDma_Init(const char* romPath) {
    void* f = fopen(romPath, "rb");
    if (f == NULL) {
        fprintf(stderr, "[dma] cannot open %s\n", romPath);
        exit(1);
    }
    fseek(f, 0, SEEK_END);
    sRomSize = (u32)ftell(f);
    fseek(f, 0, SEEK_SET);
    sRomImage = malloc(sRomSize);
    if (fread(sRomImage, 1, sRomSize, f) != sRomSize) {
        fprintf(stderr, "[dma] short read on %s\n", romPath);
        exit(1);
    }
    fclose(f);
    fprintf(stderr, "[dma] rom image loaded: %u bytes\n", sRomSize);
}

static void ReadRom(void* dst, uintptr_t vrom, u32 n) {
    memcpy(dst, sRomImage + vrom, n);
}
#define ROM_READY() (sRomImage != NULL)
#endif

/* VROM->native asset map (generated: port/src_gen/vrom_map.c). Requests
 * inside a mapped file copy from the natively-compiled (little-endian)
 * data; everything else (audio banks, unmapped files) falls back to the
 * big-endian ROM, byte-identical to cartridge DMA. */
typedef struct { void* romStart; void* romEnd; void* native; } VromMapEntry;
extern VromMapEntry gVromMap[];
extern u32 gVromMapCount;

#ifdef __3DS__
extern void PortDbgX(const char* label, unsigned val);
static u32 sDmaLogN = 0;
#endif

/* Raw cartridge read (no byte-order conversion) for the PI I/O shim: osEPiReadIo. */
void PortDma_ReadRomRaw(void* dst, u32 offset, u32 n) {
    if (ROM_READY()) ReadRom(dst, offset, n); else memset(dst, 0, n);
}

extern void gfx_texture_cache_invalidate_range(const void* start, u32 size);
static void Dma_Copy_Impl(void* ram, uintptr_t vrom, u32 size);
/* Every DMA destination may hold a cached texture (buffers are reused in place): invalidate it. */
static void Dma_Copy(void* ram, uintptr_t vrom, u32 size) {
#ifdef __3DS__
    if (PortDma_ReadAudio(ram, vrom, size)) {
        return;
    }
#endif
    Dma_Copy_Impl(ram, vrom, size);
    gfx_texture_cache_invalidate_range(ram, size);
}
static void Dma_Copy_Impl(void* ram, uintptr_t vrom, u32 size) {
    u32 i;

    if (!ROM_READY()) {
        fprintf(stderr, "[dma] request before init (vrom %08x)\n", (u32)vrom);
        exit(1);
    }
    /* PORT: guard against a corrupt DMA request. During Hyrule Field load the game issues a
     * garbage request (vrom 0xf8d44728, size 0xfffffffc == -4). The OOB path below then did
     * memset(ram, 0, size) — a ~4GB clear that floods the heap with unmapped writes and hangs
     * Play_Init. No real asset exceeds the ROM, so a size past the ROM (or a vrom past it) is
     * bogus; skip it instead of running wild. (Root cause of the bad request is a separate
     * scene/segment bug to chase, but the shim must never 4GB-memset.) */
    if (size > sRomSize || vrom > (uintptr_t)sRomSize) {
        PortDbgX("dma bogus vrom", (unsigned)vrom);
        PortDbgX("  bogus size", size);
        return;
    }
    for (i = 0; i < gVromMapCount; i++) {
        uintptr_t start = (uintptr_t)gVromMap[i].romStart;
        uintptr_t end = (uintptr_t)gVromMap[i].romEnd;

        if (vrom >= start && vrom < end) {
            u32 avail = end - vrom;
            u32 n = (size <= avail) ? size : avail;

#ifdef __3DS__
            if (size >= 0x8000u && sDmaLogN < 60u) {
                sDmaLogN++;
                PortDbgX("DMA native vrom", (unsigned)vrom);
                PortDbgX("  -> ram", (unsigned)(uintptr_t)ram);
                PortDbgX("  size", size);
            }
#endif
            memcpy(ram, (u8*)gVromMap[i].native + (vrom - start), n);
            if (n < size) { /* request crosses file end: tail from ROM */
                ReadRom((u8*)ram + n, vrom + n, size - n);
            }
            return;
        }
    }
    if (vrom + size > sRomSize) {
        fprintf(stderr, "[dma] OOB: vrom %08x size %x\n", (u32)vrom, size);
        memset(ram, 0, size);
        return;
    }
#ifdef __3DS__
    if (size >= 0x8000u && sDmaLogN < 60u) {
        sDmaLogN++;
        PortDbgX("DMA ROM(BE) vrom", (unsigned)vrom);
        PortDbgX("  -> ram", (unsigned)(uintptr_t)ram);
        PortDbgX("  size", size);
    }
#endif
    ReadRom(ram, vrom, size);
}

/* --- public DmaMgr API (signatures per src/boot/z_std_dma.c) --- */

typedef struct DmaRequest {
    uintptr_t vromAddr;
    void* dramAddr;
    u32 size;
    const char* filename;
    int line;
    OSMesgQueue* notifyQueue;
    OSMesg notifyMsg;
} DmaRequest;

s32 DmaMgr_RequestAsync(DmaRequest* req, void* ram, uintptr_t vrom, u32 size,
                        u32 unk, OSMesgQueue* queue, OSMesg msg) {
    (void)unk;
    Dma_Copy(ram, vrom, size);
    if (queue != NULL) {
        osSendMesg(queue, msg, OS_MESG_NOBLOCK);
    }
    if (req != NULL) {
        req->vromAddr = vrom;
        req->dramAddr = ram;
        req->size = size;
    }
    return 0;
}

s32 DmaMgr_RequestSync(void* ram, uintptr_t vrom, u32 size) {
    Dma_Copy(ram, vrom, size);
    return 0;
}

void DmaMgr_Init(void) {}
void DmaMgr_Stop(void) {}

/* Audio loads go through a separate handler pointer on N64; same data here. */
s32 DmaMgr_AudioDmaHandler(OSPiHandle* pihandle, OSIoMesg* mb, s32 direction) {
    (void)pihandle; (void)direction;
    Dma_Copy(mb->dramAddr, (uintptr_t)mb->devAddr, mb->size);
    if (mb->hdr.retQueue != NULL) {
        osSendMesg(mb->hdr.retQueue, NULL, OS_MESG_NOBLOCK);
    }
    return 0;
}

#ifdef __3DS__
/* --- SRAM save persistence ---------------------------------------------
 * The game's SsSram driver DMAs save data to/from cart address 0x08000000
 * (32KB). On N64 that's battery-backed cartridge SRAM; here we back it with a
 * file on the SD card so saves survive power-off. Called from the osEPiStartDma
 * shim (ultra_shims2.c) for any DMA in the [0x08000000, 0x08008000) window —
 * that shim is the only place that still has the read/write `direction`. */
#define SRAM_FILE      "sdmc:/3ds/oot/save.bin"
#define SRAM_TMP       "sdmc:/3ds/oot/save.tmp"
#define SRAM_BAK       "sdmc:/3ds/oot/save.bak"
#define PORT_SRAM_SIZE 0x8000u
static u8 sSram[PORT_SRAM_SIZE];
static int sSramLoaded = 0;
static int sSramDirty, sSramDirtyAge;

static void SramLoad(void) {
    void* f = fopen(SRAM_FILE, "rb");
    if (f == NULL) {
        f = fopen(SRAM_BAK, "rb"); /* a save interrupted between the two renames below */
    }
    if (f != NULL) { fread(sSram, 1, PORT_SRAM_SIZE, f); fclose(f); }
    /* no file -> a fresh cartridge: SRAM reads 0xFF (as ares' blank SRAM). PORT (2026-09-28): it was zeroed,
     * and an all-zero slot passes the game's checksum (0 == 0), so file select loaded empty "valid" saves
     * instead of initializing new ones (tools/statediff/bootflow.py: 86 SaveContext differences) */
    else { memset(sSram, 0xFF, PORT_SRAM_SIZE); }
    sSramLoaded = 1;
}
/* PORT (2026-10-02): the new image is written completely before it replaces the old one, so a power loss or a
 * removed SD card during a save leaves the previous save (save.bin, or save.bak between the renames), never a
 * truncated file. Rewriting save.bin in place destroyed it in that case. */
static void SramFlush(void) {
    extern void PortDbgX(const char* label, unsigned val);
    extern u64 svcGetSystemTick(void);
    u64 t0 = svcGetSystemTick();
    void* f = fopen(SRAM_TMP, "wb");
    u32 n;
    if (f == NULL) {
        PortDbgX("[save] FAILED to create save.tmp", 0);
        return;
    }
    n = fwrite(sSram, 1, PORT_SRAM_SIZE, f);
    if (fclose(f) != 0 || n != PORT_SRAM_SIZE) {
        remove(SRAM_TMP);
        PortDbgX("[save] FAILED to write save.tmp, bytes", n);
        return;
    }
    remove(SRAM_BAK);
    rename(SRAM_FILE, SRAM_BAK); /* fails harmlessly when there is no save yet */
    if (rename(SRAM_TMP, SRAM_FILE) != 0) {
        PortDbgX("[save] FAILED to rename save.tmp to save.bin", 0);
        return;
    }
    /* (one line per save, always: release checklist "saving is quick" is timed from it) */
    PortDbgX("[save] written, ms", (unsigned)((svcGetSystemTick() - t0) / 268112u)); /* (ticks at 268.11 MHz) */
}

/* PORT (2026-10-02): one file write per save. The game writes a save as several SRAM DMAs (the slot, its
 * backup copy, the header), and each one rewrote the whole file; on 3DS SD cards creating a file can take
 * very long (the Super Mario 64 3DS port measured seconds). The image is written 10 updates (0.5 s) after the
 * last SRAM write (PortSram_Tick, every update) and at once when the software is closed (PortSram_FlushNow). */
void PortSram_Tick(void) {
    if (sSramDirty && ++sSramDirtyAge >= 10) {
        SramFlush();
        sSramDirty = 0;
    }
}
void PortSram_FlushNow(void) {
    if (sSramDirty) {
        SramFlush();
        sSramDirty = 0;
    }
}

s32 PortSram_Dma(OSIoMesg* mb, s32 direction) {
    u32 off = (u32)mb->devAddr - 0x08000000u;
    if (!sSramLoaded) SramLoad();
    if (off < PORT_SRAM_SIZE && off + mb->size <= PORT_SRAM_SIZE) {
        if (direction == OS_WRITE) {
            memcpy(sSram + off, mb->dramAddr, mb->size);
            sSramDirty = 1; /* written by PortSram_Tick once the game has finished saving */
            sSramDirtyAge = 0;
        } else {
            memcpy(mb->dramAddr, sSram + off, mb->size);
        }
    }
    if (mb->hdr.retQueue != NULL) {
        osSendMesg(mb->hdr.retQueue, NULL, OS_MESG_NOBLOCK);
    }
    return 0;
}
#endif

/*
 * audio_microcode.c - C implementation of OoT's N64 audio RSP microcode (aspMain).
 * synthesis.c builds an Acmd list each audio frame (M_AUDTASK); on N64 the RSP executes it
 * into PCM in the AI buffer. Here the ARM11 does, synchronously, from PortAudio_RunTask.
 *
 * The per-command math follows the reference C implementation used by Ship of Harkinian
 * (soh/soh/mixer.c, derived from the sm64-port mixer), which is validated against OoT's
 * microcode. The command WORD decoding follows this repo's include/ultra64/abi.h macros and
 * synthesis.c's AudioSynth_* encoders.
 *
 * Endianness: everything the microcode touches in RDRAM is native (little-endian) s16 on the
 * port: codebooks and loop predictor states are byteswapped at soundfont load
 * (AudioLoad_ByteswapFont), and every other buffer (ADPCM/resample/filter states, reverb ring
 * buffers, AI buffers) is written only by this microcode or native game code. Compressed
 * sample data is consumed bytewise. So DMEM is native s16 and loads/saves are plain copies.
 *
 * Output: the AI buffer is handed to ndsp by osAiSetNextBuffer (src/audio/internal/os.c), the
 * same hand-off the N64 audio interface gets, so each engine frame is played whole.
 */
#include "ultra64.h"
#include "ultra64/abi.h"
#include <string.h>

extern void PortDbgX(const char* label, unsigned val);

int sPortAudioUcodeEnable = 1;

#define ROUND_UP_64(v) (((v) + 63) & ~63)
#define ROUND_UP_32(v) (((v) + 31) & ~31)
#define ROUND_UP_16(v) (((v) + 15) & ~15)
#define ROUND_UP_8(v) (((v) + 7) & ~7)
#define ROUND_DOWN_16(v) ((v) & ~0xf)

/* 4 KiB of RSP DMEM plus slack on both sides. Every DMEM length is capped at DMEM_SIZE
 * (DMEM_LEN) and addresses wrap at 4 KiB like the RSP's, so even a bogus command (the engine
 * can emit a ~64 KiB aClearBuffer when a note's play position runs past its loop end) stays
 * inside this array instead of trashing neighbouring variables. */
#define DMEM_SIZE 0x1000
#define DMEM_SLACK 0x2000
#define DMEM_LEN(n) ((u32)(n) > DMEM_SIZE ? DMEM_SIZE : (u32)(n))
static union {
    s16 as_s16[(DMEM_SLACK + DMEM_SIZE + DMEM_SLACK) / 2];
    u8 as_u8[DMEM_SLACK + DMEM_SIZE + DMEM_SLACK];
} sDmem;
#define BUF_U8(a) (sDmem.as_u8 + DMEM_SLACK + ((a) & (DMEM_SIZE - 1)))
#define BUF_S16(a) ((s16*)BUF_U8((a) & ~1))

typedef s16 PortAdpcmState[16];

static struct {
    u16 in;
    u16 out;
    u16 nbytes;
    u16 vol[2];
    u16 rate[2];
    u16 vol_wet;
    u16 rate_wet;
    s16* adpcm_loop_state;
    s16 adpcm_table[8][2][8];
    u16 filter_count;
    s16 filter[8];
} rspa;

static const s16 resample_table[64][4] = {
    { 0x0c39, 0x66ad, 0x0d46, 0xffdf }, { 0x0b39, 0x6696, 0x0e5f, 0xffd8 }, { 0x0a44, 0x6669, 0x0f83, 0xffd0 },
    { 0x095a, 0x6626, 0x10b4, 0xffc8 }, { 0x087d, 0x65cd, 0x11f0, 0xffbf }, { 0x07ab, 0x655e, 0x1338, 0xffb6 },
    { 0x06e4, 0x64d9, 0x148c, 0xffac }, { 0x0628, 0x643f, 0x15eb, 0xffa1 }, { 0x0577, 0x638f, 0x1756, 0xff96 },
    { 0x04d1, 0x62cb, 0x18cb, 0xff8a }, { 0x0435, 0x61f3, 0x1a4c, 0xff7e }, { 0x03a4, 0x6106, 0x1bd7, 0xff71 },
    { 0x031c, 0x6007, 0x1d6c, 0xff64 }, { 0x029f, 0x5ef5, 0x1f0b, 0xff56 }, { 0x022a, 0x5dd0, 0x20b3, 0xff48 },
    { 0x01be, 0x5c9a, 0x2264, 0xff3a }, { 0x015b, 0x5b53, 0x241e, 0xff2c }, { 0x0101, 0x59fc, 0x25e0, 0xff1e },
    { 0x00ae, 0x5896, 0x27a9, 0xff10 }, { 0x0063, 0x5720, 0x297a, 0xff02 }, { 0x001f, 0x559d, 0x2b50, 0xfef4 },
    { 0xffe2, 0x540d, 0x2d2c, 0xfee8 }, { 0xffac, 0x5270, 0x2f0d, 0xfedb }, { 0xff7c, 0x50c7, 0x30f3, 0xfed0 },
    { 0xff53, 0x4f14, 0x32dc, 0xfec6 }, { 0xff2e, 0x4d57, 0x34c8, 0xfebd }, { 0xff0f, 0x4b91, 0x36b6, 0xfeb6 },
    { 0xfef5, 0x49c2, 0x38a5, 0xfeb0 }, { 0xfedf, 0x47ed, 0x3a95, 0xfeac }, { 0xfece, 0x4611, 0x3c85, 0xfeab },
    { 0xfec0, 0x4430, 0x3e74, 0xfeac }, { 0xfeb6, 0x424a, 0x4060, 0xfeaf }, { 0xfeaf, 0x4060, 0x424a, 0xfeb6 },
    { 0xfeac, 0x3e74, 0x4430, 0xfec0 }, { 0xfeab, 0x3c85, 0x4611, 0xfece }, { 0xfeac, 0x3a95, 0x47ed, 0xfedf },
    { 0xfeb0, 0x38a5, 0x49c2, 0xfef5 }, { 0xfeb6, 0x36b6, 0x4b91, 0xff0f }, { 0xfebd, 0x34c8, 0x4d57, 0xff2e },
    { 0xfec6, 0x32dc, 0x4f14, 0xff53 }, { 0xfed0, 0x30f3, 0x50c7, 0xff7c }, { 0xfedb, 0x2f0d, 0x5270, 0xffac },
    { 0xfee8, 0x2d2c, 0x540d, 0xffe2 }, { 0xfef4, 0x2b50, 0x559d, 0x001f }, { 0xff02, 0x297a, 0x5720, 0x0063 },
    { 0xff10, 0x27a9, 0x5896, 0x00ae }, { 0xff1e, 0x25e0, 0x59fc, 0x0101 }, { 0xff2c, 0x241e, 0x5b53, 0x015b },
    { 0xff3a, 0x2264, 0x5c9a, 0x01be }, { 0xff48, 0x20b3, 0x5dd0, 0x022a }, { 0xff56, 0x1f0b, 0x5ef5, 0x029f },
    { 0xff64, 0x1d6c, 0x6007, 0x031c }, { 0xff71, 0x1bd7, 0x6106, 0x03a4 }, { 0xff7e, 0x1a4c, 0x61f3, 0x0435 },
    { 0xff8a, 0x18cb, 0x62cb, 0x04d1 }, { 0xff96, 0x1756, 0x638f, 0x0577 }, { 0xffa1, 0x15eb, 0x643f, 0x0628 },
    { 0xffac, 0x148c, 0x64d9, 0x06e4 }, { 0xffb6, 0x1338, 0x655e, 0x07ab }, { 0xffbf, 0x11f0, 0x65cd, 0x087d },
    { 0xffc8, 0x10b4, 0x6626, 0x095a }, { 0xffd0, 0x0f83, 0x6669, 0x0a44 }, { 0xffd8, 0x0e5f, 0x6696, 0x0b39 },
    { 0xffdf, 0x0d46, 0x66ad, 0x0c39 }
};

static inline s16 clamp16(s32 v) {
#if defined(__3DS__) && defined(__ARM_ARCH_6K__)
    /* PORT PERF (2026-09-30): ARMv6 SSAT - saturate to 16 bits in one instruction (bit-exact with the
     * branches below; every mixer loop clamps each sample) */
    s32 r;
    __asm__("ssat %0, #16, %1" : "=r"(r) : "r"(v));
    return (s16)r;
#else
    if (v < -0x8000) {
        return -0x8000;
    } else if (v > 0x7fff) {
        return 0x7fff;
    }
    return (s16)v;
#endif
}

/* ---- RDRAM address validation --------------------------------------------------------- */

/* libctru's svcQueryMemory, declared here because <3ds.h> clashes with ultra64.h's types */
typedef struct { unsigned base_addr, size, perm, state; } PortMemInfo;
extern int svcQueryMemory(PortMemInfo* info, unsigned* pageFlags, unsigned addr);

/* Is [addr, addr+len) readable+writable process memory? The engine can hand the microcode
 * out-of-range addresses (e.g. a note whose sample changed keeps a play position past the new
 * sample's loop end -> AudioLoad_DmaSampleData's size check wraps -> pointer ~2MB past a DMA
 * buffer). On N64 that just reads other RDRAM (garbage audio); on 3DS hardware unmapped memory
 * faults (Azahar only logs it). Regions are cached so this is ~free per command. */
static int ext_range_ok(u32 addr, u32 len) {
    static u32 sBase[4], sEnd[4];
    static int sNext = 0;
    PortMemInfo mi;
    unsigned pf;
    u32 end = addr + len;
    int i;
    if (end < addr) return 0;
    for (i = 0; i < 4; i++) {
        if (addr >= sBase[i] && end <= sEnd[i]) return 1;
    }
    if (svcQueryMemory(&mi, &pf, addr) < 0) return 0;
    if (mi.state == 0 /* MEMSTATE_FREE */ || (mi.perm & 3) != 3 /* R+W */) return 0;
    sBase[sNext] = mi.base_addr;
    sEnd[sNext] = mi.base_addr + mi.size;
    sNext = (sNext + 1) & 3;
    return end <= mi.base_addr + mi.size;
}

extern u8 gAudioHeap[];

static void* ext_ptr(u32 addr, u32 len) {
    addr &= 0x7FFFFFFFu;
    /* fast path: nearly everything (sample DMA buffers, states, reverb, AI buffers) is in the audio heap */
    if (addr >= (u32)gAudioHeap && addr + len >= addr && addr + len <= (u32)gAudioHeap + 0x38000u) {
        return (void*)(uintptr_t)addr;
    }
    if (addr < 0x00100000u || !ext_range_ok(addr, len)) {
        static unsigned sBadN = 0;
        if ((sBadN++ & 1023) == 0) PortDbgX("[audio] ucode: out-of-range RDRAM addr (ignored)", addr);
        return NULL;
    }
    return (void*)(uintptr_t)addr;
}

/* 16-sample state blocks (ADPCM/S8/resample/filter). An invalid pointer gets a scratch block
 * so the command still runs (from silence) instead of being skipped. */
static s16* ext_state(u32 addr) {
    static PortAdpcmState sScratch;
    s16* p = (s16*)ext_ptr(addr, sizeof(PortAdpcmState));
    if (p == NULL) {
        memset(sScratch, 0, sizeof(sScratch));
        return sScratch;
    }
    return p;
}

/* ---- commands ------------------------------------------------------------------------- */

static void aClearBufferImpl(u16 addr, s32 nbytes) {
    nbytes = DMEM_LEN(ROUND_UP_16(nbytes));
    memset(BUF_U8(addr), 0, nbytes);
}

static void aLoadBufferImpl(u32 src, u16 dest_addr, u16 nbytes) {
    const void* s = ext_ptr(src, nbytes);
    if (s == NULL) { /* N64 would load garbage; load silence */
        memset(BUF_U8(dest_addr), 0, nbytes);
        return;
    }
    memcpy(BUF_U8(dest_addr), s, nbytes);
}

static void aSaveBufferImpl(u16 source_addr, u32 dst, u16 nbytes) {
    void* d = ext_ptr(dst, ROUND_DOWN_16(nbytes));
    if (d == NULL) return;
    memcpy(d, BUF_S16(source_addr), ROUND_DOWN_16(nbytes));
}

static void aLoadADPCMImpl(u32 nbytes, u32 src) {
    const void* s;
    if (nbytes > sizeof(rspa.adpcm_table)) nbytes = sizeof(rspa.adpcm_table);
    s = ext_ptr(src, nbytes);
    if (s == NULL) return;
    memcpy(rspa.adpcm_table, s, nbytes);
}

static void aSetBufferImpl(u16 in, u16 out, u16 nbytes) {
    rspa.in = in;
    rspa.out = out;
    rspa.nbytes = DMEM_LEN(nbytes);
}

static void aInterleaveImpl(u16 dest, u16 left, u16 right, u16 c) {
    int count = ROUND_UP_8(c) / sizeof(s16) / 4;
    s16* l = BUF_S16(left);
    s16* r = BUF_S16(right);
    s16* d = BUF_S16(dest);
    while (count > 0) {
        s16 l0 = *l++, l1 = *l++, l2 = *l++, l3 = *l++;
        s16 r0 = *r++, r1 = *r++, r2 = *r++, r3 = *r++;
        *d++ = l0; *d++ = r0;
        *d++ = l1; *d++ = r1;
        *d++ = l2; *d++ = r2;
        *d++ = l3; *d++ = r3;
        --count;
    }
}

static void aDMEMMoveImpl(u16 in_addr, u16 out_addr, s32 nbytes) {
    nbytes = DMEM_LEN(ROUND_UP_16(nbytes));
    memmove(BUF_U8(out_addr), BUF_U8(in_addr), nbytes);
}

static void aSetLoopImpl(u32 addr) {
    rspa.adpcm_loop_state = (s16*)ext_ptr(addr, sizeof(PortAdpcmState));
}

static void aADPCMdecImpl(u8 flags, s16* state) {
    u8* in = BUF_U8(rspa.in);
    s16* out = BUF_S16(rspa.out);
    s32 nbytes = ROUND_UP_32(rspa.nbytes);
    if (flags & A_INIT) {
        memset(out, 0, 16 * sizeof(s16));
    } else if (flags & A_LOOP) {
        if (rspa.adpcm_loop_state != NULL) {
            memcpy(out, rspa.adpcm_loop_state, 16 * sizeof(s16));
        } else {
            memset(out, 0, 16 * sizeof(s16));
        }
    } else {
        memcpy(out, state, 16 * sizeof(s16));
    }
    out += 16;

    while (nbytes > 0) {
        int shift = *in >> 4;
        int table_index = *in++ & 0x7;
        s16(*tbl)[8] = rspa.adpcm_table[table_index];
        int i;

        for (i = 0; i < 2; i++) {
            s16 ins[8];
            s16 prev1 = out[-1];
            s16 prev2 = out[-2];
            int j, k;
            if (flags & 4) {
                for (j = 0; j < 2; j++) {
                    ins[j * 4] = (((s32)(*in >> 6) << 30) >> 30) << shift;
                    ins[j * 4 + 1] = ((((s32)(*in >> 4) & 0x3) << 30) >> 30) << shift;
                    ins[j * 4 + 2] = ((((s32)(*in >> 2) & 0x3) << 30) >> 30) << shift;
                    ins[j * 4 + 3] = ((((s32)*in++ & 0x3) << 30) >> 30) << shift;
                }
            } else {
                for (j = 0; j < 4; j++) {
                    ins[j * 2] = (((s32)(*in >> 4) << 28) >> 28) << shift;
                    ins[j * 2 + 1] = ((((s32)*in++ & 0xf) << 28) >> 28) << shift;
                }
            }
            for (j = 0; j < 8; j++) {
                s32 acc = tbl[0][j] * prev2 + tbl[1][j] * prev1 + (ins[j] << 11);
                for (k = 0; k < j; k++) {
                    acc += tbl[1][((j - k) - 1)] * ins[k];
                }
                acc >>= 11;
                *out++ = clamp16(acc);
            }
        }
        nbytes -= 16 * sizeof(s16);
    }
    memcpy(state, out - 16, 16 * sizeof(s16));
}

static void aResampleImpl(u8 flags, u16 pitch, s16* state) {
    s16 tmp[16];
    s16* in_initial = BUF_S16(rspa.in);
    s16* in = in_initial;
    s16* out = BUF_S16(rspa.out);
    s32 nbytes = ROUND_UP_16(rspa.nbytes);
    u32 pitch_accumulator;
    int i;
    const s16* tbl;
    s32 sample;

    if (flags & A_INIT) {
        memset(tmp, 0, 5 * sizeof(s16));
    } else {
        memcpy(tmp, state, 16 * sizeof(s16));
    }
    if (flags & 2) {
        memcpy(in - 8, tmp + 8, 8 * sizeof(s16));
        in -= tmp[5] / (s32)sizeof(s16);
    }
    in -= 4;
    pitch_accumulator = (u16)tmp[4];
    memcpy(in, tmp, 4 * sizeof(s16));

    do {
        for (i = 0; i < 8; i++) {
            tbl = resample_table[pitch_accumulator * 64 >> 16];
            sample = ((in[0] * tbl[0] + 0x4000) >> 15) + ((in[1] * tbl[1] + 0x4000) >> 15) +
                     ((in[2] * tbl[2] + 0x4000) >> 15) + ((in[3] * tbl[3] + 0x4000) >> 15);
            *out++ = clamp16(sample);

            pitch_accumulator += (pitch << 1);
            in += pitch_accumulator >> 16;
            pitch_accumulator %= 0x10000;
        }
        nbytes -= 8 * sizeof(s16);
    } while (nbytes > 0);

    state[4] = (s16)pitch_accumulator;
    memcpy(state, in, 4 * sizeof(s16));
    i = (in - in_initial + 4) & 7;
    in -= i;
    if (i != 0) {
        i = -8 - i;
    }
    state[5] = i;
    memcpy(state + 8, in, 8 * sizeof(s16));
}

static void aEnvSetup1Impl(u8 initial_vol_wet, u16 rate_wet, u16 rate_left, u16 rate_right) {
    rspa.vol_wet = (u16)(initial_vol_wet << 8);
    rspa.rate_wet = rate_wet;
    rspa.rate[0] = rate_left;
    rspa.rate[1] = rate_right;
}

static void aEnvSetup2Impl(u16 initial_vol_left, u16 initial_vol_right) {
    rspa.vol[0] = initial_vol_left;
    rspa.vol[1] = initial_vol_right;
}

static void aEnvMixerImpl(u16 in_addr, u16 n_samples, int swap_reverb, int neg_3, int neg_2, int neg_left,
                          int neg_right, u32 wet_dry_addr) {
    s16* in = BUF_S16(in_addr);
    s16* dry[2] = { BUF_S16(((wet_dry_addr >> 24) & 0xFF) << 4), BUF_S16(((wet_dry_addr >> 16) & 0xFF) << 4) };
    s16* wet[2] = { BUF_S16(((wet_dry_addr >> 8) & 0xFF) << 4), BUF_S16(((wet_dry_addr) & 0xFF) << 4) };
    s16 negs[4] = { neg_left ? -1 : 0, neg_right ? -1 : 0, neg_3 ? -4 : 0, neg_2 ? -2 : 0 };
    int swapped[2] = { swap_reverb ? 1 : 0, swap_reverb ? 0 : 1 };
    int n = ROUND_UP_16(n_samples);
    u16 vols[2] = { rspa.vol[0], rspa.vol[1] };
    u16 rates[2] = { rspa.rate[0], rspa.rate[1] };
    u16 vol_wet = rspa.vol_wet;
    u16 rate_wet = rspa.rate_wet;
    int i, j;

#if defined(__3DS__) && defined(__ARM_ARCH_6K__)
    if ((((uintptr_t)in | (uintptr_t)dry[0] | (uintptr_t)dry[1] | (uintptr_t)wet[0] | (uintptr_t)wet[1]) & 3) == 0) {
        /* PORT PERF (2026-09-30): two samples per step; the four saturating accumulations are ARMv6
         * QADD16 (per lane identical to clamp16(a + b)). Same per-sample products as the loop below. */
        (void)i;
        do {
            for (i = 0; i < 8; i += 2) {
                s16 x0 = in[0], x1 = in[1];
                s16 l0 = (s16)((x0 * vols[0] >> 16) ^ negs[0]), l1 = (s16)((x1 * vols[0] >> 16) ^ negs[0]);
                s16 r0 = (s16)((x0 * vols[1] >> 16) ^ negs[1]), r1 = (s16)((x1 * vols[1] >> 16) ^ negs[1]);
                s16 s0[2] = { l0, r0 }, s1[2] = { l1, r1 };
                in += 2;
                for (j = 0; j < 2; j++) {
                    u32 d = (u16)s0[j] | ((u32)(u16)s1[j] << 16);
                    u32 w = (u16)(s16)((s0[swapped[j]] * vol_wet >> 16) ^ negs[2 + j]) |
                            ((u32)(u16)(s16)((s1[swapped[j]] * vol_wet >> 16) ^ negs[2 + j]) << 16);
                    u32* dp = (u32*)dry[j];
                    u32* wp = (u32*)wet[j];
                    __asm__("qadd16 %0, %0, %1" : "+r"(*dp) : "r"(d));
                    __asm__("qadd16 %0, %0, %1" : "+r"(*wp) : "r"(w));
                    dry[j] += 2;
                    wet[j] += 2;
                }
            }
            vols[0] += rates[0];
            vols[1] += rates[1];
            vol_wet += rate_wet;
            n -= 8;
        } while (n > 0);
        return;
    }
#endif
    do {
        for (i = 0; i < 8; i++) {
            s16 samples[2] = { *in, *in };
            in++;
            for (j = 0; j < 2; j++) {
                samples[j] = (s16)((samples[j] * vols[j] >> 16) ^ negs[j]);
            }
            for (j = 0; j < 2; j++) {
                *dry[j] = clamp16(*dry[j] + samples[j]);
                dry[j]++;
                *wet[j] = clamp16(*wet[j] + ((samples[swapped[j]] * vol_wet >> 16) ^ negs[2 + j]));
                wet[j]++;
            }
        }
        vols[0] += rates[0];
        vols[1] += rates[1];
        vol_wet += rate_wet;
        n -= 8;
    } while (n > 0);
}

static void aMixImpl(u16 count, s16 gain, u16 in_addr, u16 out_addr) {
    s32 nbytes = ROUND_UP_32(ROUND_DOWN_16(count << 4));
    s16* in = BUF_S16(in_addr);
    s16* out = BUF_S16(out_addr);
    int i;
    s32 sample;

    if (gain == -0x8000) {
        while (nbytes > 0) {
            for (i = 0; i < 16; i++) {
                sample = *out - *in++;
                *out++ = clamp16(sample);
            }
            nbytes -= 16 * sizeof(s16);
        }
    }

    while (nbytes > 0) {
        for (i = 0; i < 16; i++) {
            sample = ((*out * 0x7fff + *in++ * gain) + 0x4000) >> 15;
            *out++ = clamp16(sample);
        }
        nbytes -= 16 * sizeof(s16);
    }
}

static void aS8DecImpl(u8 flags, s16* state) {
    u8* in = BUF_U8(rspa.in);
    s16* out = BUF_S16(rspa.out);
    s32 nbytes = ROUND_UP_32(rspa.nbytes);
    int i;
    if (flags & A_INIT) {
        memset(out, 0, 16 * sizeof(s16));
    } else if (flags & A_LOOP) {
        if (rspa.adpcm_loop_state != NULL) {
            memcpy(out, rspa.adpcm_loop_state, 16 * sizeof(s16));
        } else {
            memset(out, 0, 16 * sizeof(s16));
        }
    } else {
        memcpy(out, state, 16 * sizeof(s16));
    }
    out += 16;

    while (nbytes > 0) {
        for (i = 0; i < 16; i++) {
            *out++ = (s16)(*in++ << 8);
        }
        nbytes -= 16 * sizeof(s16);
    }
    memcpy(state, out - 16, 16 * sizeof(s16));
}

static void aAddMixerImpl(u16 count, u16 in_addr, u16 out_addr) {
    s16* in = BUF_S16(in_addr);
    s16* out = BUF_S16(out_addr);
    s32 nbytes = ROUND_UP_64(ROUND_DOWN_16(count));
    int i;

    do {
        for (i = 0; i < 16; i++) {
            *out = clamp16(*out + *in++);
            out++;
        }
        nbytes -= 16 * sizeof(s16);
    } while (nbytes > 0);
}

static void aDuplicateImpl(u16 count, u16 in_addr, u16 out_addr) {
    u8* in = BUF_U8(in_addr);
    u8* out = BUF_U8(out_addr);
    u8 tmp[128];
    memcpy(tmp, in, 128);
    do {
        memcpy(out, tmp, 128);
        out += 128;
    } while (count-- > 0 && out < sDmem.as_u8 + sizeof(sDmem) - 128);
}

static void aResampleZohImpl(u16 pitch, u16 start_fract) {
    s16* in = BUF_S16(rspa.in);
    s16* out = BUF_S16(rspa.out);
    s32 nbytes = ROUND_UP_8(rspa.nbytes);
    u32 pos = start_fract;
    u32 pitch_add = pitch << 2;
    int i;

    do {
        for (i = 0; i < 4; i++) {
            *out++ = in[pos >> 17];
            pos += pitch_add;
        }
        nbytes -= 4 * sizeof(s16);
    } while (nbytes > 0);
}

static void aInterlImpl(u16 in_addr, u16 out_addr, u16 n_samples) {
    s16* in = BUF_S16(in_addr);
    s16* out = BUF_S16(out_addr);
    int n = DMEM_LEN(ROUND_UP_8(n_samples) * 2) / 2; /* samples out; reads twice as many */
    int i;

    do {
        for (i = 0; i < 8; i++) {
            *out++ = *in++;
            in++;
        }
        n -= 8;
    } while (n > 0);
}

static void aFilterImpl(u8 flags, u16 count_or_buf, u32 addr) {
    s16* state_or_filter = ext_state(addr);
    if (flags > A_INIT) {
        rspa.filter_count = DMEM_LEN(ROUND_UP_16(count_or_buf));
        memcpy(rspa.filter, state_or_filter, sizeof(rspa.filter));
    } else {
        s16 tmp[16], tmp2[8];
        int count = rspa.filter_count;
        s16* buf = BUF_S16(count_or_buf);
        int i, j;

        if (flags == A_INIT) {
            memset(tmp, 0, 8 * sizeof(s16));
            memset(tmp2, 0, 8 * sizeof(s16));
        } else {
            memcpy(tmp, state_or_filter, 8 * sizeof(s16));
            memcpy(tmp2, state_or_filter + 8, 8 * sizeof(s16));
        }

        for (i = 0; i < 8; i++) {
            rspa.filter[i] = (tmp2[i] + rspa.filter[i]) / 2;
        }

#if defined(__3DS__) && defined(__ARM_ARCH_6K__)
        /* PORT PERF (2026-09-30): the 8-tap FIR with ARMv6 SMLALD (two 16x16 products accumulated into 64
         * bits per instruction; same sums as the scalar loop). Reversed coefficients are packed in pairs
         * (frev[j] = filter[7 - j]), the input pairs are read straight from tmp. */
        s16 frev[8] __attribute__((aligned(4)));
        for (j = 0; j < 8; j++) frev[j] = rspa.filter[7 - j];
#endif
        do {
            memcpy(tmp + 8, buf, 8 * sizeof(s16));
            for (i = 0; i < 8; i++) {
#if defined(__3DS__) && defined(__ARM_ARCH_6K__)
                u32 lo = 0x4000, hi = 0; /* round term */
                const u32* fp = (const u32*)frev;
                for (j = 0; j < 8; j += 2) {
                    /* tmp[i + j] and tmp[i + j + 1] as one word (unaligned when i is odd: build it) */
                    u32 x = (u16)tmp[i + j] | ((u32)(u16)tmp[i + j + 1] << 16);
                    __asm__("smlald %0, %1, %2, %3" : "+r"(lo), "+r"(hi) : "r"(x), "r"(fp[j >> 1]));
                }
                {
                    s64 sample = (s64)(((u64)hi << 32) | lo);
                    buf[i] = clamp16((s32)(sample >> 15));
                }
#else
                s64 sample = 0x4000; // round term
                for (j = 0; j < 8; j++) {
                    sample += tmp[i + j] * rspa.filter[7 - j];
                }
                buf[i] = clamp16((s32)(sample >> 15));
#endif
            }
            memcpy(tmp, tmp + 8, 8 * sizeof(s16));
            buf += 8;
            count -= 8 * sizeof(s16);
        } while (count > 0);

        memcpy(state_or_filter, tmp, 8 * sizeof(s16));
        memcpy(state_or_filter + 8, rspa.filter, 8 * sizeof(s16));
    }
}

static void aHiLoGainImpl(u8 g, u16 count, u16 addr) {
    s16* samples = BUF_S16(addr);
    s32 nbytes = DMEM_LEN(ROUND_UP_32(count));
    int i;

    do {
        for (i = 0; i < 8; i++) {
            *samples = clamp16((*samples * g) >> 4);
            samples++;
        }
        nbytes -= 8;
    } while (nbytes > 0);
}

static void aUnkCmd19Impl(u8 f, u16 count, u16 out_addr, u16 in_addr) {
    s32 nbytes = DMEM_LEN(ROUND_UP_64(count));
    s16* in = BUF_S16(in_addr + f);
    s16* out = BUF_S16(out_addr);
    s16 tbl[32];
    int i;

    memcpy(tbl, in, 32 * sizeof(s16));
    do {
        for (i = 0; i < 32; i++) {
            out[i] = clamp16(out[i] * tbl[i]);
        }
        out += 32;
        nbytes -= 32 * sizeof(s16);
    } while (nbytes > 0);
}

#if defined(__3DS__) && defined(__ARM_ARCH_6K__)
/* PORT (2026-09-30): self-test of the ARMv6 SIMD paths against the scalar reference (random inputs);
 * logged once at the first audio task. Mismatches = the optimization is not bit-exact. */
static void PortAudio_SimdSelfTest(void) {
    u32 seed = 12345, bad = 0, n;
    for (n = 0; n < 20000; n++) {
        s32 v;
        s16 a, b;
        seed = seed * 1664525u + 1013904223u;
        v = (s32)seed >> 8; /* wide range around +-2^23 */
        a = clamp16(v);
        b = (v < -0x8000) ? -0x8000 : (v > 0x7fff) ? 0x7fff : (s16)v;
        if (a != b) bad++;
    }
    for (n = 0; n < 2000; n++) {
        s16 tmp[16], filt[8], frev[8] __attribute__((aligned(4)));
        int i, j;
        for (i = 0; i < 16; i++) { seed = seed * 1664525u + 1013904223u; tmp[i] = (s16)(seed >> 16); }
        for (i = 0; i < 8; i++) { seed = seed * 1664525u + 1013904223u; filt[i] = (s16)(seed >> 16); }
        for (j = 0; j < 8; j++) frev[j] = filt[7 - j];
        for (i = 0; i < 8; i++) {
            s64 ref = 0x4000;
            u32 lo = 0x4000, hi = 0;
            const u32* fp = (const u32*)frev;
            for (j = 0; j < 8; j++) ref += tmp[i + j] * filt[7 - j];
            for (j = 0; j < 8; j += 2) {
                u32 x = (u16)tmp[i + j] | ((u32)(u16)tmp[i + j + 1] << 16);
                __asm__("smlald %0, %1, %2, %3" : "+r"(lo), "+r"(hi) : "r"(x), "r"(fp[j >> 1]));
            }
            if ((s64)(((u64)hi << 32) | lo) != ref) bad++;
        }
    }
    for (n = 0; n < 20000; n++) { /* QADD16 lanes vs clamp16(a + b) */
        u32 a2, b2, r2;
        s16 a0, a1, b0, b1;
        seed = seed * 1664525u + 1013904223u; a2 = seed;
        seed = seed * 1664525u + 1013904223u; b2 = seed;
        a0 = (s16)a2, a1 = (s16)(a2 >> 16), b0 = (s16)b2, b1 = (s16)(b2 >> 16);
        r2 = a2;
        __asm__("qadd16 %0, %0, %1" : "+r"(r2) : "r"(b2));
        if ((s16)r2 != clamp16(a0 + b0) || (s16)(r2 >> 16) != clamp16(a1 + b1)) bad++;
    }
    PortDbgX("[audio] SIMD self-test mismatches (must be 0)", bad);
}
#endif

/* ---- dispatcher: decode the Acmd words exactly as abi.h / synthesis.c encode them --------- */

void PortAudio_RunTask(OSTask* task) {
    const Acmd* cmd;
    u32 n, k;
    if (task == NULL || task->t.data_ptr == NULL || !sPortAudioUcodeEnable) return;
    cmd = (const Acmd*)task->t.data_ptr;
    n = (u32)(task->t.data_size / sizeof(Acmd));
#if defined(__3DS__) && defined(__ARM_ARCH_6K__)
    {
        static int sTested;
        if (!sTested) {
            sTested = 1;
            PortAudio_SimdSelfTest();
        }
    }
#endif
    if (n > 8192) n = 8192;
    for (k = 0; k < n; k++) {
        u32 w0 = cmd[k].words.w0, w1 = cmd[k].words.w1;
#ifdef __3DS__
        /* perf_stages=1: time per microcode operation (3ds_main.c reports the most expensive) */
        extern int gPortPerfStagesOn;
        extern u64 gPortPerfAudOpTicks[32];
        extern u64 svcGetSystemTick(void);
        u64 tOp = gPortPerfStagesOn ? svcGetSystemTick() : 0;
#endif
        switch (w0 >> 24) {
            case A_SPNOOP:
            case A_UNK3:
                break;
            case A_ADPCM:
                aADPCMdecImpl((w0 >> 16) & 0xFF, ext_state(w1));
                break;
            case A_CLEARBUFF:
                aClearBufferImpl(w0 & 0xFFFF, w1 & 0xFFFF);
                break;
            case A_ADDMIXER:
                aAddMixerImpl(((w0 >> 16) & 0xFF) << 4, w1 >> 16, w1 & 0xFFFF);
                break;
            case A_RESAMPLE:
                aResampleImpl((w0 >> 16) & 0xFF, w0 & 0xFFFF, ext_state(w1));
                break;
            case A_RESAMPLE_ZOH:
                aResampleZohImpl(w0 & 0xFFFF, w1 & 0xFFFF);
                break;
            case A_FILTER:
                aFilterImpl((w0 >> 16) & 0xFF, w0 & 0xFFFF, w1);
                break;
            case A_SETBUFF:
                aSetBufferImpl(w0 & 0xFFFF, w1 >> 16, w1 & 0xFFFF);
                break;
            case A_DUPLICATE:
                aDuplicateImpl((w0 >> 16) & 0xFF, w0 & 0xFFFF, w1 >> 16);
                break;
            case A_DMEMMOVE:
                aDMEMMoveImpl(w0 & 0xFFFF, w1 >> 16, w1 & 0xFFFF);
                break;
            case A_LOADADPCM:
                aLoadADPCMImpl(w0 & 0xFFFFFF, w1);
                break;
            case A_MIXER:
                aMixImpl((w0 >> 16) & 0xFF, (s16)(w0 & 0xFFFF), w1 >> 16, w1 & 0xFFFF);
                break;
            case A_INTERLEAVE:
                aInterleaveImpl(w0 & 0xFFFF, w1 >> 16, w1 & 0xFFFF, ((w0 >> 16) & 0xFF) << 4);
                break;
            case A_HILOGAIN:
                aHiLoGainImpl((w0 >> 16) & 0xFF, w0 & 0xFFFF, w1 >> 16);
                break;
            case A_SETLOOP:
                aSetLoopImpl(w1);
                break;
            case A_INTERL:
                aInterlImpl(w1 >> 16, w1 & 0xFFFF, w0 & 0xFFFF);
                break;
            case A_ENVSETUP1:
                aEnvSetup1Impl((w0 >> 16) & 0xFF, w0 & 0xFFFF, w1 >> 16, w1 & 0xFFFF);
                break;
            case A_ENVMIXER:
                aEnvMixerImpl(((w0 >> 16) & 0xFF) << 4, (w0 >> 8) & 0xFF, (w0 >> 4) & 1, (w0 >> 3) & 1,
                              (w0 >> 2) & 1, (w0 >> 1) & 1, w0 & 1, w1);
                break;
            case A_LOADBUFF:
                aLoadBufferImpl(w1, w0 & 0xFFFF, ((w0 >> 16) & 0xFF) << 4);
                break;
            case A_SAVEBUFF:
                aSaveBufferImpl(w0 & 0xFFFF, w1, ((w0 >> 16) & 0xFF) << 4);
                break;
            case A_ENVSETUP2:
                aEnvSetup2Impl(w1 >> 16, w1 & 0xFFFF);
                break;
            case A_S8DEC:
                aS8DecImpl((w0 >> 16) & 0xFF, ext_state(w1));
                break;
            case A_UNK19:
                aUnkCmd19Impl((w0 >> 16) & 0xFF, w0 & 0xFFFF, w1 >> 16, w1 & 0xFFFF);
                break;
            default: {
                static unsigned sUnk = 0;
                if ((sUnk++ & 1023) == 0) PortDbgX("[audio] ucode: unknown command", w0 >> 24);
                break;
            }
        }
#ifdef __3DS__
        if (gPortPerfStagesOn) gPortPerfAudOpTicks[(w0 >> 24) & 31] += svcGetSystemTick() - tOp;
#endif
    }
}

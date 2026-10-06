/*
 * rdram_shim.c — N64 memory map emulation for the 32-bit host process.
 *   0x80000000  KSEG0 cached RDRAM   \ same 8MB, true mirror (memfd mapped
 *   0xA0000000  KSEG1 uncached RDRAM / twice — uncached DMA buffers work)
 *   0xA4000000  16MB zero-fill: RCP/PI/VI/AI/SI hardware registers.
 *               Writes vanish; reads return 0 ("idle" for every busy-poll).
 */
extern int fprintf();
extern void* stderr;
extern void* mmap();
extern int ftruncate();
extern int syscall();
extern void exit();

#define RDRAM_SIZE (8 * 1024 * 1024)
#define HWREG_BASE 0xA4000000u
#define HWREG_SIZE (16 * 1024 * 1024)

#define PORT_PROT 0x3            /* PROT_READ|PROT_WRITE */
#define MAP_SHARED_FIXED  (0x01 | 0x10)        /* MAP_SHARED|MAP_FIXED */
#define MAP_ANON_PRIV_FIX (0x02 | 0x10 | 0x20) /* MAP_PRIVATE|MAP_FIXED|MAP_ANONYMOUS */
#define SYS_memfd_create 356     /* i386 */

static void MapOrDie(void* want, void* got) {
    if (got != want) {
        fprintf(stderr, "[rdram] mmap at %p failed (got %p)\n", want, got);
        exit(1);
    }
}

void PortRdram_Init(void) {
    int fd = syscall(SYS_memfd_create, "rdram", 0);
    if (fd < 0) {
        fprintf(stderr, "[rdram] memfd_create failed\n");
        exit(1);
    }
    ftruncate(fd, RDRAM_SIZE);
    MapOrDie((void*)0x80000000u,
             mmap((void*)0x80000000u, RDRAM_SIZE, PORT_PROT, MAP_SHARED_FIXED, fd, 0));
    MapOrDie((void*)0xA0000000u,
             mmap((void*)0xA0000000u, RDRAM_SIZE, PORT_PROT, MAP_SHARED_FIXED, fd, 0));
    MapOrDie((void*)HWREG_BASE,
             mmap((void*)HWREG_BASE, HWREG_SIZE, PORT_PROT, MAP_ANON_PRIV_FIX, -1, 0));
    fprintf(stderr, "[rdram] KSEG0+KSEG1 8MB mirror + 16MB hw-reg window mapped\n");
}

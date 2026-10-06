# Majora's Mask → Nintendo 3DS: port plan and status

The port follows the [Ocarina of Time N64 3DS Port](https://github.com/Andresac90/OOT-64-3DS-Port): the
decompilation's C code is compiled natively for the 3DS's ARM11 CPU, the N64 display lists are translated by a
renderer to the 3DS GPU (citro3d), and the game data is read at run time from the player's own decompressed ROM on
the SD card. The 3DS platform layer in `port/` started as a copy of the OoT port's.

## Phases

1. **Build skeleton** (in progress)
   - [x] The decompilation builds matching from a US ROM on macOS (`mm-n64-us.z64` `f46493ea…`).
   - [x] All of MM's game code compiles for ARM11 (870 files; only `build.c`, the Japanese-only `z_sram_JPN.c`
         and `seqplayer.c`'s `MML_VERSION` needed anything).
   - [x] Port layer copied from the OoT port; `Makefile.3ds` adapted (MM defines, MM's file layout, `F3DEX_GBI_PL`).
   - [x] N64 segment tables generated from MM's ELF: `tools/port/gen_segment_tables.py` →
         `port/rom_symbols.ld`, `port/include/3ds_segment_consts.h` (1,555 segments).
   - [x] `port/include/segment_symbols.h` for MM (MM's `SEGMENT_*` helpers, no-op `DECLARE_*`).
   - [x] Port shims against MM's interfaces: libultra prototypes (`ultra_shims.c`, `ultra_shims3.c`), the
         scheduler (`sched_shim.c`: `msgQ`, `CfbInfo`, `Sched_SendNotifyMsg`), the overlay statics table
         (`tools/port/gen_overlay_statics.py`, 617 overlays), the renderer's `PR/gbi.h` wrapper. MM's own
         `sys_cfb.c` is used (its framebuffers are ordinary arrays). **All game code, the port layer and the
         shaders compile** (`make -f Makefile.3ds code`).
   - [ ] Touch panel game state for MM (`Port_GetHudInfo`, `Port_GetItemIcon` are stubs: `PORT MM TODO`).
   - [x] Native assets: `tools/port/gen_asset_tables.py` → `port/asset_srcs.mk` (901 single-file segments) and
         `port/src_gen/vrom_map.c`; each asset file gets a start marker (`port/include/port_asset_start.h`). All 901
         match their ROM segment sizes (up to the ROM's 16-byte padding). Archives (`*_yar`) and audio stay ROM data.
   - [x] Link tables: `tools/port/gen_link_symbols.py` (archive textures, audio offsets),
         `tools/port/gen_sequence_font_table.py`, `tools/port/gen_u32_asset_ranges.py`.
   - [x] First link: `build/3ds/mm.elf` (4.6 MB code, 35 MB data).
2. **First boot in Azahar** (in progress)
   - [x] Own title (`0xF0C65`), SD folder `sdmc:/3ds/mm/`, placeholder art (`tools/port/make_placeholder_art.py`).
   - [x] MM's own libc (`src/boot/libc`) excluded: GCC compiled its `memset` loop into a call to itself (hang).
   - [x] Memory: MM's `malloc`/`free` use MM's arena (`port/include/libc64/malloc.h`), on a real block of the N64
         system heap's size (`main.c`); linked overlays are never freed (`z_DLF.c`).
   - [x] Big-endian reads: Yaz0 header (`yaz0.c`), archive offset table (`sys_cmpdma.c`).
   - [x] Engine init, game states up to `Play_Init` run (audio off: `PORT_MM_AUDIO`, see phase 3).
   - [ ] Azahar stops on "Read from unknown GPU address": a buffer outside GPU memory reaches the GPU.
3. **MM-specific systems**
   - Audio: synthesis reads/writes unmapped memory in Azahar (fatal on hardware); OoT's audio patches to redo for MM.
   - Archive textures (item icons, pause pages) are big-endian ROM data: the renderer must read them as such.
   - Saves: MM writes FlashROM (`sys_flashrom.c`, `osFlash.c`), not SRAM like OoT → a flash save shim.
   - Point lighting (`F3DEX_GBI_PL`) in the renderer (`port/src/gfx/gfx_pc.c`).
   - `kanread.s` (assembly) → the OoT port's C version (`port/src/kanread_port.c`).
   - Memory: MM needs the N64 Expansion Pak (8 MB); arena sizes on the 3DS.
   - Touch screen: MM's items, masks, Bombers' Notebook, map, the three-day clock.
4. **Title screen, then gameplay** → the README's first 3DS screenshot (`tools/make_showcase.py` from the OoT port).
5. Audio, 60 fps (interpolation), stereoscopic 3D, Old 3DS, hardware tests — as in the OoT port.

## Rules (same as the OoT port)

- No ROM, extracted assets or other Nintendo data in the repository; no built `.cia` / `.3ds` published.
- The OoT port's lessons apply (`PORT_ROADMAP.md` there, "Root-caused bug classes"): N64 pointer stripping,
  identity overlay relocation, `guMtx` packing, native-pointer vs segment offsets, endian seams in packed data.

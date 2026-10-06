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
   - [ ] Port shims against MM's interfaces: `ultra_shims.c` / `ultra_shims3.c` (MM's libultra prototypes),
         `sched_shim.c` (MM's `OSScTask`), `syscfb_shim.c` (MM's `SysCfb_Init`), `overlay_statics.c` (needs
         `port/src_gen/overlay_statics_table.inc` for MM: `gen_overlay_statics.py`), the renderer files (types
         before `PR/gbi.h`).
   - [ ] Native assets: `port/asset_srcs.mk` (MM's 957 extracted asset `.c` files), the ROM→asset map
         `port/src_gen/vrom_map.c` (its OoT generator was lost: rewrite), the `gameplay_keep` blob, the 32-bit asset
         ranges (`gen_u32_asset_ranges.py`), the audio tables.
   - [ ] First link: resolve undefined symbols.
2. **First boot in Azahar** — the game's threads replaced by the port's synchronous frame loop, DMA served from
   the ROM on the SD card (`sdmc:/3ds/mm/`), `boot.log` reaching `GameState_Update`.
3. **MM-specific systems**
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

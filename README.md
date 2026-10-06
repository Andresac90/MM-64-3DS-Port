# The Legend of Zelda: Majora's Mask N64 3DS Port

A fan-made **native** Nintendo 3DS port of The Legend of Zelda: Majora's Mask (N64, US), built on the
[zeldaret/mm](https://github.com/zeldaret/mm) decompilation, following the
[Ocarina of Time N64 3DS Port](https://github.com/Andresac90/OOT-64-3DS-Port): the game's own code will run
natively on the 3DS CPU, with a new renderer, audio backend and dual-screen interface replacing the N64 hardware.

> **Status: just started.** Nothing runs on the 3DS yet. This repository is, for now, the decompilation the
> port will be built on; there is nothing to build or install for the 3DS.

> **This repository contains no ROM and no game assets.** The port will be built from your own legally obtained
> copy of the N64 game (US). No prebuilt game binaries will be distributed; see [Legal](#legal).

This project is not affiliated with, endorsed by, or sponsored by Nintendo.

## Legal

- This repository contains source code, build scripts and tools only. It does **not** contain a ROM,
  extracted game assets, or any other Nintendo data. Nintendo, Nintendo 3DS, *The Legend of Zelda* and
  *Majora's Mask* are trademarks of Nintendo; all game content belongs to Nintendo.
- The game data is extracted from **your own ROM on your own computer** at build time. For that reason no
  `.cia`, `.3dsx` or `.3ds` builds are published here or in releases. Please do not upload built binaries.
- You must own the game: dump the ROM from your own cartridge. Do not ask for or share ROMs in this
  repository's issues.
- This is a non-commercial fan project, provided as is, without warranty of any kind.

## Credits

- [zeldaret/mm](https://github.com/zeldaret/mm): the Majora's Mask decompilation this port is built on. Its
  original README is kept in [docs/DECOMP_README.md](docs/DECOMP_README.md).
- The [Ocarina of Time N64 3DS Port](https://github.com/Andresac90/OOT-64-3DS-Port), whose 3DS platform layer
  this port will reuse, and the projects credited there.

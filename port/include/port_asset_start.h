/* port_asset_start.h - included first in every native asset file (Makefile.3ds: -include, with
 * -DPORT_ASSET_SEG=<segment>). The zero-size marker port_seg_<segment> is declared before any of the file's data,
 * in .data like all of it, so with declaration order kept it sits at the segment's offset 0 - the base address
 * port/src_gen/vrom_map.c gives the DMA shim for that segment (tools/port/gen_asset_tables.py). */
#define PORT_ASSET_CAT2(a, b) a##b
#define PORT_ASSET_CAT(a, b) PORT_ASSET_CAT2(a, b)
char PORT_ASSET_CAT(port_seg_, PORT_ASSET_SEG)[0] __attribute__((section(".data"), aligned(8), used)) = {};

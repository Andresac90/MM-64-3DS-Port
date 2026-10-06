/* N64 segment base addresses referenced as data symbols; the port's segment
 * resolver (PortSegmentedToVirtual / gfx_pc seg_addr) maps 0x0Yxxxxxx values. */
D_01000000 = 0x01000000;
D_06000000 = 0x06000000;
D_0E000000 = 0x0E000000;
D_0F000000 = 0x0F000000;

/* gameplay_keep native blob (port/src_gen/gameplay_keep_blob.c, compiled VROM-exact and
 * contiguous) — vrom_map index 0 uses its RAM base. The blob's byte-0 symbol is
 * gEffSpark4Tex; point the base at it (only __gpk_blob_start is referenced). */
__gpk_blob_start = gEffSpark4Tex;

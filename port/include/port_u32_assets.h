#ifndef PORT_U32_ASSETS_H
#define PORT_U32_ASSETS_H
/* Asset arrays the decomp emits as u32[] rather than u64[] (port/src_gen/u32_asset_ranges.c, generated
 * by tools/gen_u32_asset_ranges.py): their bytes sit at address ^ 3 on the little-endian build. */
typedef struct {
    const void* addr; /* NULL if the array isn't linked (weak reference) */
    unsigned int size;
} PortU32Asset;
extern const PortU32Asset gPortU32Assets[];
extern const int gPortU32AssetCount;
#endif

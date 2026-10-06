# OoT 3DS-port: native PC build (32-bit little-endian, matching the 3DS ARM11 model)
# Stage 1: compile the game code into liboot.a. Platform layer + link come next.

OOT_REF   ?= /home/aceve/oot
BUILD     := port/build/pc
LIB       := $(BUILD)/liboot.a

CC        := gcc
CFLAGS    := -m32 -std=gnu90 -funsigned-char -fno-builtin -fno-strict-aliasing \
             -nostdinc -Os -g \
             -Wno-implicit-function-declaration -Wno-builtin-declaration-mismatch

DEFS := -DCOMPILER_GCC -DNON_MATCHING -DAVOID_UB \
        -DPLATFORM_N64=1 -DPLATFORM_GC=0 -DPLATFORM_IQUE=0 \
        -DOOT_VERSION=NTSC_1_0 -DOOT_REVISION=0 -DOOT_REGION=REGION_US \
        -DLIBULTRA_VERSION=LIBULTRA_VERSION_I -DLIBULTRA_PATCH=1 \
        -DDEBUG_FEATURES=0 -DNDEBUG -DF3DEX_GBI_2 -D_LANGUAGE_C \
        -D_MIPS_SZINT=32 -D_MIPS_SZLONG=32 -D_MIPS_SZPTR=32

INC := -Iport/include -Iinclude -Iinclude/libc -Isrc \
       -I$(OOT_REF)/build/ntsc-1.0 -I. -I$(OOT_REF)/extracted/ntsc-1.0

# Game code: everything that passed the native-compile census.
# Excluded for now (tracked in port/TODO.md):
#   src/code/fault_gc*.c       - GC fault screen, replaced by platform layer
#   src/boot/build.c           - build-info stamp, regenerated for port
#   src/audio/internal/seqplayer.c, src/audio/tables/soundfont_table.c - real fixes pending
SRCS := $(shell find src/code src/audio src/boot \
          src/overlays/gamestates src/overlays/actors/ovl_player_actor \
          -name '*.c' ! -name '*.inc.c' \
          ! -name 'fault_gc*.c' ! -name 'build.c' \
          ! -name 'seqplayer.c' ! -name 'soundfont_table.c' | sort)

OBJS := $(SRCS:%.c=$(BUILD)/%.o)

all: $(LIB)
	@echo "liboot.a: $$(ar t $(LIB) | wc -l) objects"

$(LIB): $(OBJS)
	@mkdir -p $(dir $@)
	ar rcs $@ $(OBJS)

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	@$(CC) $(CFLAGS) $(DEFS) $(INC) -c -o $@ $< || (echo "FAIL: $<" && false)

clean:
	rm -rf $(BUILD)

.PHONY: all clean

################################################################################
# image_tester — MegaWifi Image Tester for Sega Genesis
################################################################################

GDK    = $(HOME)/sgdk
PREFIX ?= m68k-elf-
CC     = $(PREFIX)gcc
AS     = $(PREFIX)as
OBJCPY = $(PREFIX)objcopy

OUT    = out
SRC    = src

# Local copy of megawifi.c — has draw hook fix + mw_sock_conn_wait timeout fix
MW_OVERRIDE_SRC = $(SRC)/megawifi.c
MW_SRC = $(GDK)/src/ext/mw

INCS   = -I$(SRC) -I$(OUT) -I$(GDK)/inc -I$(GDK)/res

CFLAGS  = -DSGDK_GCC -DMODULE_MEGAWIFI=1 -m68000 -O2 -fomit-frame-pointer
CFLAGS += -Wall -Wextra -Wno-shift-negative-value -Wno-main -Wno-unused-parameter
CFLAGS += -fno-builtin -ffunction-sections -fdata-sections -fms-extensions
CFLAGS += $(INCS)

AFLAGS  = $(CFLAGS) -x assembler-with-cpp -Wa,--register-prefix-optional,--bitwise-or

LDFLAGS = -m68000 -n -T $(GDK)/md.ld -nostdlib -fno-lto
LDFLAGS += -Wl,--gc-sections

LIBGCC  = $(shell $(CC) -m68000 -print-libgcc-file-name)
LIBMD   = $(GDK)/lib/libmd.a

ROM = $(OUT)/image_tester.bin
ELF = $(OUT)/image_tester.out

# Boot objects (from SGDK)
BOOT_HEAD_O   = $(OUT)/rom_head.o
BOOT_HEAD_BIN = $(OUT)/rom_head.bin
BOOT_SEGA_O   = $(OUT)/sega.o

# MegaWifi API objects (compiled separately to avoid LTO issues in libmd.a)
MW_OBJS = $(OUT)/megawifi.o $(OUT)/lsd.o $(OUT)/16c550.o $(OUT)/json.o

# Application
APP_OBJ = $(OUT)/image_tester.o

ALL_OBJS = $(BOOT_SEGA_O) $(APP_OBJ) $(MW_OBJS)

################################################################################

.PHONY: all run clean

all: $(OUT) $(ROM)

run: all
	mame genesis -cart $(ROM) -skip_gameinfo -window -nomaximize -resolution 1024x768 -video soft

clean:
	rm -f $(OUT)/*.o $(OUT)/*.bin $(OUT)/*.out

$(OUT):
	mkdir -p $(OUT)

################################################################################
# Boot
################################################################################

$(BOOT_HEAD_O): $(GDK)/src/boot/rom_head.c
	$(CC) $(CFLAGS) -c $< -o $@

$(BOOT_HEAD_BIN): $(BOOT_HEAD_O)
	$(OBJCPY) -O binary $< $@

$(BOOT_SEGA_O): $(GDK)/src/boot/sega.s $(BOOT_HEAD_BIN)
	$(CC) $(AFLAGS) -c $< -o $@

################################################################################
# MegaWifi API
################################################################################

$(OUT)/megawifi.o: $(MW_OVERRIDE_SRC)
	$(CC) $(CFLAGS) -c $< -o $@

$(OUT)/lsd.o: $(MW_SRC)/lsd.c
	$(CC) $(CFLAGS) -c $< -o $@

$(OUT)/16c550.o: $(MW_SRC)/16c550.c
	$(CC) $(CFLAGS) -c $< -o $@

$(OUT)/json.o: $(MW_SRC)/json.c
	$(CC) $(CFLAGS) -Wno-unused-function -c $< -o $@

################################################################################
# Application
################################################################################

$(APP_OBJ): $(SRC)/image_tester.c
	$(CC) $(CFLAGS) -c $< -o $@

################################################################################
# Link
################################################################################

$(ELF): $(ALL_OBJS) $(LIBMD)
	$(CC) $(LDFLAGS) $(ALL_OBJS) $(LIBMD) $(LIBGCC) -o $@

$(ROM): $(ELF)
	$(OBJCPY) -O binary $(ELF) $(ROM)
	@echo "ROM: $(ROM)"
	@echo "Flash to cartridge — receiver listens on port 2026"

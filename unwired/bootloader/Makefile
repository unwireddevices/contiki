CC      = arm-none-eabi-gcc
CPP     = arm-none-eabi-cpp
LD      = arm-none-eabi-gcc
AR      = arm-none-eabi-ar
TRACE_CC = @echo "  CC       " $<
TRACE_LD = @echo "  LD       " $@
TRACE_AR = @echo "  AR       " $@
TRACE_AS = @echo "  AS       " $<
Q=@

OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
NM      = arm-none-eabi-nm
SIZE    = arm-none-eabi-size
SREC_CAT = srec_cat

CONTIKI = ../..
OTA_SOURCE = ./ota-common

CPU_ABS_PATH = $(CONTIKI)/cpu/cc26xx-cc13xx
TARGET=srf06-cc26xx
OBJECTDIR=obj_$(TARGET)

LDSCRIPT = cc26xx.bootloader.ld
LDFLAGS += -mcpu=cortex-m3 -mthumb -mlittle-endian -nostartfiles
LDFLAGS += -T $(LDSCRIPT)
LDFLAGS += -g
LDFLAGS += -Wl,--gc-sections,--sort-section=alignment
LDFLAGS += -Wl,-Map=$(@:.elf=-$(TARGET).map),--cref,--no-warn-mismatch
OBJCOPY_FLAGS += -O binary --gap-fill 0xff
OBJDUMP_FLAGS += --disassemble --source --disassembler-options=force-thumb

#	TI 26XX ware sources
ifeq ($(CPU),cc13xx)
	TI_XXWARE_PATH = lib/cc13xxware
endif
ifeq ($(CPU),cc26xx)
	TI_XXWARE_PATH = lib/cc26xxware
endif

TI_XXWARE = $(CPU_ABS_PATH)/$(TI_XXWARE_PATH)
CFLAGS += -I$(CPU_ABS_PATH) -I$(OTA_SOURCE) -I./ext-flash -I$(TI_XXWARE) -I$(TI_XXWARE)/driverlib -I$(TI_XXWARE)/inc -I../../platform/unwired/$(BOARD)
SOURCEDIRS = . ./ota-common $(TI_XXWARE)/startup_files $(TI_XXWARE)/driverlib
vpath %.c $(SOURCEDIRS)
SOURCE_FILES = $(foreach dir, $(SOURCEDIRS), $(notdir $(wildcard $(dir)/*.c)))

SOURCE_FILES += ota-bootloader.c ota-common.c
vpath ota-bootloader.c ota-common.c $(OTA_SOURCE)

CFLAGS += -mcpu=cortex-m3 -mthumb -mlittle-endian
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -fshort-enums -fomit-frame-pointer -fno-strict-aliasing
CFLAGS += -Wall -std=c99
CFLAGS += -g -Os

### Are we building with code size optimisations?
ifdef WERROR
CFLAGS += -Werror
endif

### Compilation rules
### Don't treat the .elf as intermediate
.PRECIOUS: %.elf %.hex %.bin

oname = ${patsubst %.c,%.o,${patsubst %.S,%.o,$(1)}}
OBJECTFILES = ${addprefix $(OBJECTDIR)/,${call oname, $(SOURCE_FILES)}}

all: $(OBJECTFILES)

$(OBJECTDIR)/%.o: %.c | $(OBJECTDIR)
	$(TRACE_CC)
	$(Q)$(CC) $(CFLAGS) -c $< -o $@

$(OBJECTDIR):
	mkdir $@

%.elf: $(OBJECTFILES) $(LDSCRIPT)
	$(TRACE_LD)
	$(Q)$(LD) $(LDFLAGS) ${filter-out $(LDSCRIPT) %.a,$^} ${filter %.a,$^} $(TARGET_LIBFILES) -lm -o $@

%.i16hex: %.elf
	$(OBJCOPY) -O ihex $< $@

%.hex: %.i16hex
	$(SREC_CAT) $< -intel -o $@ -intel

%.bin: %.elf
	$(OBJCOPY) $(OBJCOPY_FLAGS) $< $@

%.lst: %.elf
	$(OBJDUMP) $(OBJDUMP_FLAGS) $< > $@

clean:
	-rm -f *~ *core core *.srec \
	*.lst *.map \
	*.cprg *.bin *.elf *.hex *.data contiki*.a *.firmware core-labels.S *.ihex *.ini \
	*.ce *.co
	rm -rf $(CLEAN)
	-rm -rf $(OBJECTDIR)

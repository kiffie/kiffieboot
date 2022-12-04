#
# Makefile for USB DFU Bootloader
#

CC = pic32-gcc
BIN2HEX = pic32-bin2hex
OBJCOPY = pic32-objcopy
OBJDUMP = pic32-objdump

PRG = kb3k-dfu-$(BUILD_CONFIG)

ifeq ($(BUILD_CONFIG), mx2xx-40mhz)
TARGET = 32MX250F128B
LDSCRIPT = btl_32MX2XX.ld
SYS_CLOCK = 40000000
else ifeq ($(BUILD_CONFIG), mx2xx-48mhz)
TARGET = 32MX250F128B
LDSCRIPT = btl_32MX2XX.ld
SYS_CLOCK = 48000000
else ifeq ($(BUILD_CONFIG), mx2x4-72mhz)
TARGET = 32MX274F256B
LDSCRIPT = btl_32MX274.ld
SYS_CLOCK = 72000000
else ifeq ($(BUILD_CONFIG), mx470-96mhz)
TARGET = 32MX470F512H
LDSCRIPT = btl_32MX470F512H.ld
SYS_CLOCK = 96000000
else
$(error invalid/missing BUILD_CONFIG)
endif

#ENABLE_LOGGING = true
ENABLE_CONFIGSFRS = true # include config register values in image

CFLAGS = -Wall -g -std=gnu99 -mprocessor=$(TARGET) -Os -mips16 -DSYS_CLOCK=$(SYS_CLOCK) -Iinclude

CFLAGS += -DCRT_DISABLE_BSS_INIT -DUSB_DISABLE_IRQ -DUSB_RESET_HOOK
CFLAGS += -DBL_TIMEOUT=10
ifndef ENABLE_LOGGING
CFLAGS += -DDISABLE_LOGGING
endif

LDFLAGS = $(CFLAGS) -Wl,-Map=$(PRG).map,--memorysummary,$(PRG).mem
LDFLAGS += -Wl,--script=$(LDSCRIPT) -nostartfiles

OBJECTS := crt0.o main.o usb.o
ifdef ENABLE_LOGGING
OBJECTS += terminal.o tinyprintf.o
endif
ifdef ENABLE_CONFIGSFRS
OBJECTS += config_pic32.o
endif

DEPS    = $(patsubst %.o, %.d, $(notdir $(OBJECTS)))

.PHONY: all
all: $(PRG).hex $(PRG).bin $(PRG).xxd lst


%.o: %.c
	$(CC) -c $(CFLAGS) -MMD -o $@ $<

%.o: %.S
	$(CC) -c $(CFLAGS) -MMD -o $@ $<

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

$(PRG).elf: $(OBJECTS)
	$(CC) $(LDFLAGS) -o $@ $^

$(PRG).hex: $(PRG).elf
	$(BIN2HEX) $<

$(PRG).bin: $(PRG).elf
	$(OBJCOPY) -O binary --gap-fill 0xff $< $@
	truncate --size 3056 $@

$(PRG).xxd: $(PRG).bin
	cat $(PRG).bin | xxd -include > $@

lst: $(PRG).lst

clean:
	rm -f $(OBJECTS) $(DEPS) $(PRG).elf \
	      $(PRG).map $(PRG).map $(PRG).lst $(PRG).mem

mrproper: clean
	rm -f $(PRG).hex $(PRG).bin $(PRG).xxd

-include $(DEPS)

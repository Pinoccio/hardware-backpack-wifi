SERIAL=/dev/ttyUSB0
PROGRAMMER=stk500
GCC_MCU=attiny13a
CRT_FILE=crttn13a.o
AVRDUDE_PART=t13
CFLAGS=-Wall -mmcu=$(GCC_MCU) -Os
HFUSE=0xfb
LFUSE=0x21

ifeq ($(shell avr-gcc $(CFLAGS) --print-file-name $(CRT_FILE)),$(CRT_FILE))
$(error "Gcc couldn't find $(CRT_FILE), you probably haven't applied the /usr/lib/avr/lib/avr25/tiny-stack workaround suggested in firmware.c? Short version: run ln -s /usr/lib/avr/lib/avr25 /usr/lib/avr/lib/avr25/tiny-stack")
endif

# Don't use jump tables, they make the code bigger
CFLAGS+=-fno-jump-tables

# Uncomment to enable output pins for debugging
#CFLAGS+=-DDEBUG

-include Makefile.local

all: firmware.hex

firmware.S: firmware.c protocol.h Makefile
	avr-gcc -S $(CFLAGS) $< -o $@ -fwhole-program

firmware.elf: firmware.c protocol.h Makefile
	avr-gcc -g $(CFLAGS) $< -o $@ -fwhole-program

firmware.hex: firmware.elf
	avr-objcopy -O ihex $< $@

upload: firmware.hex
	avrdude -c $(PROGRAMMER) -p $(AVRDUDE_PART) -P $(SERIAL) -U flash:w:$<

dump: firmware.elf
	avr-objdump -S $< | less

size: firmware.elf
	avr-size -C $<

read_eeprom:
	avrdude -c $(PROGRAMMER) -p $(AVRDUDE_PART) -P $(SERIAL) -V -U eeprom:r:-:h

read_fuses:
	avrdude -c $(PROGRAMMER) -p $(AVRDUDE_PART) -P $(SERIAL) -V -U hfuse:r:-:h -U lfuse:r:-:h

write_fuses:
	avrdude -c $(PROGRAMMER) -p $(AVRDUDE_PART) -P $(SERIAL) -V -U hfuse:w:$(HFUSE):m -U lfuse:w:$(LFUSE):m

# Write a random unique ID to eeprom. Variables can be override on the
# commandline, e.g. make eeprom_id MODEL=0003
eeprom_id: LAYOUT_VERSION:=01
eeprom_id: PROTO_VERSION:=01
eeprom_id: EEPROM_SIZE:=40
eeprom_id: MODEL:=0001
eeprom_id: SERIALNO:=$(shell head --bytes=3 /dev/urandom |hexdump -v -e '/1 "%02X"')
eeprom_id: REVISION:=01
eeprom_id: ID:=$(PROTO_VERSION)$(MODEL)$(REVISION)$(SERIALNO)
eeprom_id: CHECKSUM:=$(shell printf "%02x" $$(pycrc --width=8 --poly=0x2f --xor-in=0 --reflect-in=false --reflect-out=false --xor-out=0 --check-hexstring $(ID)))
eeprom_id: DATA=$(shell echo $(LAYOUT_VERSION)$(EEPROM_SIZE)$(ID)$(CHECKSUM) | tr '[:upper:]' '[:lower:]' | sed 's/../0x& /g')
eeprom_id:
	avrdude -c $(PROGRAMMER) -p $(AVRDUDE_PART) -P $(SERIAL) -V -U "eeprom:w:$(DATA):m"

clean:
	rm -f firmware.S firmware.elf firmware.hex

.PHONY: upload dump read_eeprom read_fuses write_fuses eeprom_id clean

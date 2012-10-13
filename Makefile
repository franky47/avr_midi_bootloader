# Makefile for Generic AVR MIDI Bootloader
# Francois Best 19/08/09

MCU_TARGET = atmega644p
CPU_FREQ   = 16000000L
START_ADDR = 0xF000			# Check the datasheet

# ------------------------------------------------------------------------------


# program name should not be changed...
PROGRAM    = avr_MidiBootloader

# enter the target CPU frequency



LDSECTION  = --section-start=.text=$(START_ADDR)
OBJ        = $(PROGRAM).o
OPTIMIZE   = -O2

DEFS       = 
LIBS       = 

CC         = avr-gcc

SRC_DIR	   	= src
BUILD_DIR	= build

# Override is only needed by avr-lib build system.

override CFLAGS        = -g -Wall $(OPTIMIZE) -mmcu=$(MCU_TARGET) -DF_CPU=$(CPU_FREQ) $(DEFS)
override LDFLAGS       = -Wl,$(LDSECTION)
#override LDFLAGS       = -Wl,-Map,$(PROGRAM).map,$(LDSECTION)

OBJCOPY        = avr-objcopy
OBJDUMP        = avr-objdump

all: CFLAGS += '-DMAX_TIME_COUNT=8000000L>>1'
all: $(PROGRAM).hex

$(PROGRAM).hex: $(PROGRAM).elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@
	
$(PROGRAM).elf: $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)
	
$(OBJ): $(SRC_DIR)/$(PROGRAM).c
	avr-gcc $(CFLAGS) $(LDFLAGS) -c -g -O2 -Wall -mmcu=$(MCU_TARGET) $(SRC_DIR)/$(PROGRAM).c -o $(PROGRAM).o

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

%.srec: %.elf
	$(OBJCOPY) -j .text -j .data -O srec $< $@

%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -O binary $< $@

movebins:
	mv *.o *.hex *.elf $(BUILD_DIR)	

clean:
	rm -rf *.o *.elf *.lst *.map *.sym *.lss *.eep *.srec *.bin *.hex
	

stats: $(PROGRAM).elf
	#$(OBJDUMP) -h $(PROGRAM).elf
	/usr/local/avr/bin/avr-size $(PROGRAM).elf -C --mcu=$(MCU_TARGET)

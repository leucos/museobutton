# Name: Makefile
# Project: PowerSwitch
# Author: Christian Starkjohann
# Creation Date: 2004-12-29
# Tabsize: 4
# Copyright: (c) 2005 by OBJECTIVE DEVELOPMENT Software GmbH
# License: GNU GPL v2 (see License.txt) or proprietary (CommercialLicense.txt)
# This Revision: $Id: Makefile 277 2007-03-20 10:53:33Z cs $

DEVICE = atmega8
#AVRDUDE = avrdude -c stk500v2 -P /dev/ttyS0 -p $(DEVICE)
AVRDUDE = sudo avrdude -c usbasp -p $(DEVICE)
# Choose your favorite programmer and interface above.

COMPILE = avr-gcc -Wall -Os -Iusbdrv -I. -mmcu=$(DEVICE) #-DDEBUG_LEVEL=2
# NEVER compile the final product with debugging! Any debug output will
# distort timing so that the specs can't be met.

OBJECTS_I2C = TWI_Master.o
OBJECTS = usbdrv/usbdrv.o usbdrv/usbdrvasm.o main.o 

ifdef I2C
COMPILE := $(COMPILE) -DI2C
OBJECTS = usbdrv/usbdrv.o usbdrv/usbdrvasm.o main.o TWI_Master.o
else
OBJECTS = usbdrv/usbdrv.o usbdrv/usbdrvasm.o main.o 
endif

ifdef USE_TRIANGLE
COMPILE := $(COMPILE) -DUSE_TRIANGLE
endif

ifdef USE_SAWTOOTH_UP
COMPILE := $(COMPILE) -DUSE_SAWTOOTH_UP
endif

ifdef USE_SAWTOOTH_DOWN
COMPILE := $(COMPILE) -DUSE_SAWTOOTH_DOWN
endif

ifdef USE_SIN
COMPILE := $(COMPILE) -DUSE_SIN
endif

ifdef USE_SUBPWM
COMPILE := $(COMPILE) -DUSE_SUBPWM
endif

ifdef USE_ALL_WAVES
COMPILE := $(COMPILE) -DUSE_TRIANGLE -DUSE_SAWTOOTH_UP -DUSE_SAWTOOTH_DOWN -DUSE_SIN -DUSE_SUBPWM
endif

PRG=museobutton

# symbolic targets:
all: main.hex main.eep

i2c: main_i2c

.c.o:
	$(COMPILE) -c $< -o $@

.S.o:
	$(COMPILE) -x assembler-with-cpp -c $< -o $@
# "-x assembler-with-cpp" should not be necessary since this is the default
# file type for the .S (with capital S) extension. However, upper case
# characters are not always preserved on Windows. To ensure WinAVR
# compatibility define the file type manually.

.c.s:
	$(COMPILE) -S $< -o $@

load: flash

flash:	all
	$(AVRDUDE) -U flash:w:main.hex:i


# Fuse low byte:
# 0xef = 1 1 1 0   1 1 1 1
#        ^ ^ \+/   \--+--/
#        | |  |       +------- CKSEL 3..0 (clock selection -> crystal @ 12 MHz)
#        | |  +--------------- SUT 1..0 (BOD enabled, fast rising power)
#        | +------------------ CKOUT (clock output on CKOUT pin -> disabled)
#        +-------------------- CKDIV8 (divide clock by 8 -> don't divide)
#
# Fuse high byte:
# 0xdb = 1 1 0 1   1 0 1 1
#        ^ ^ ^ ^   \-+-/ ^
#        | | | |     |   +---- RbSTDISBL (disable external reset -> enabled)
#        | | | |     +-------- BODLEVEL 2..0 (brownout trigger level -> 2.7V)
#        | | | +-------------- WDTON (watchdog timer always on -> disable)
#        | | +---------------- SPIEN (enable serial programming -> enabled)
#        | +------------------ EESAVE (preserve EEPROM on Chip Erase -> not preserved)
#        +-------------------- DWEN (debug wire enable)

fuse_tiny2313:	# only needed for attiny2313
	$(AVRDUDE) -U hfuse:w:0xdb:m -U lfuse:w:0xef:m

# Fuse low byte:
# 0x9f = 1 0 0 1   1 1 1 1
#        ^ ^ ^ ^   ^ ^ ^ ^
#        | | | |   | | | +---- CKSEL0 (select clock source -> disabled)
#        | | | |  | | +------- CKSEL1 (select clock source -> disabled)
#        | | | | | +---------- CKSEL2 (select clock source -> disabled)
#        | | | | +------------ CKSEL3 (select clock source -> disabled)
#        | | | +-------------- SUT0 (start-up time -> disabled)
#        | | +---------------- SUT1 (start-up time -> enabled)
#        | +------------------ BODEN (brwon out detector -> enabled)
#        +-------------------- BODLEVEL (brown out detect trig lvl -> disabled)
#
# Fuse high byte:
# 0xc1 = 1 1 0 0   0 0 0 1
#        ^ ^ ^ ^   ^ ^ ^ ^
#        | | | |   | | | +---- BOOTRST (select reset vector -> disabled)
#        | | | |  | | +------- BOOTSIZ0 (boot size -> enabled)
#        | | | | | +---------- BOOTSIZ1 (boot size -> enabled) 
#        | | | | +------------ EESAVE ( EEPROM preserved -> enabled)
#        | | | +-------------- CKOPT (osc options -> disabled)
#        | | +---------------- SPIEN (enable serial programming -> enabled)
#        | +------------------ WDTON (watchdog timer always on -> disabled) 
#        +-------------------- RSTDISBL (disable external reset -> disabled)

fuse_mega8:     # only needed for atmega8
	$(AVRDUDE) -U hfuse:w:0xc1:m -U lfuse:w:0x9f:m

clean:
	rm -f main.hex main.lst main.obj main.cof main.list main.map main.eep.hex main.bin *.o usbdrv/*.o main.s usbdrv/oddebug.s usbdrv/usbdrv.s *~ *.eep

# file targets:

main.bin:	$(OBJECTS)
	$(COMPILE) -o main.bin $(OBJECTS)

main.hex:	main.bin
	rm -f main.hex main.eep.hex
	avr-objcopy -j .text -j .data -O ihex main.bin main.hex
	./checksize main.bin
# do the checksize script as our last action to allow successful compilation
# on Windows with WinAVR where the Unix commands will fail.

main.eep: main.bin
	avr-objcopy -j .eeprom --change-section-lma .eeprom=0 -O ihex $< $@


eeprom: main.eep
	$(AVRDUDE) -U eeprom:w:main.eep:i

disasm:	main.bin
	avr-objdump -d main.bin



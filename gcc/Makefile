###############################################################################
#	Payload software makefile
#	Questions? Ping t-havida
#
#
###############################################################################

ARCH:=arm-none-eabi
CC:=$(ARCH)-gcc
CXX:=$(ARCH)-g++
AS:=$(ARCH)-as

OBJDIR:=obj
BINDIR:=bin/

PROJECT_ROOT:=./
INCLUDE_ROOT:=$(PROJECT_ROOT)/../inc/driverlib/ $(PROJECT_ROOT)/../inc/CMSIS/

INCLUDE_PATHS:=$(INCLUDE_ROOT)

ARMFLAGS=-mthumb
ARMFLAGS+=-mcpu=cortex-m4
ARMFLAGS+=-mfpu=fpv4-sp-d16
ARMFLAGS+=-mfloat-abi=hard
ARMFLAGS+=-DPART_TM4C123GH6PM -DTM4C123GH6PM
ARMFLAGS+=-DTARGET_IS_BLIZZARD_RB1

CFLAGS=-std=c99 -g -g3 -Wdeclaration-after-statement
CCFLAGS=-ffunction-sections -fdata-sections -nostartfiles -g -g3 -Wall -Wextra
CXXFLAGS=-fno-exceptions -fno-rtti -g -g3 -nostartfiles -Wall -Wextra

LD:=$(ARCH)-gcc
LDFLAGS:=-Wl,-eResetISR,--gc-sections,-marmelf

INCLUDE=$(INCLUDE_PATHS:%=-I%)

SIZE=$(ARCH)-size

OBJCOPY:=$(ARCH)-objcopy
PROGRAM:=payload
PROGRAM_DEPLOY:=$(payload:%:=%.hex)

CP:=cp
TOUCH:=touch
MKDIR:=mkdir -p
FLASH_LINUX:=lm4flash
FLASH_WIN:=lmflash
FLASHFLAGS_LINUX:=-E -v
FLASHFLAGS_WIN:=-r -v

ifeq ($(OS),Windows_NT)
	FLASH:=$(FLASH_WIN)
	FLASHFLAGS:=$(FLASHFLAGS_WIN)
else
	FLASH:=$(FLASH_LINUX)
	FLASHFLAGS:=$(FLASHFLAGS_LINUX)
endif

# include new .c files here

SRC_C:=main.c CU_system_TM4C123.c startup.c
SRC_CPP:=

OBJ:=$(SRC_C:.c=.o) $(SRC_CPP:.cpp=.o)
OBJ:=$(addprefix $(OBJDIR)/,$(OBJ))

NODE_NAME?=local
BUILD_ID?=$(shell date +"%c" | tr ' :' '__')

all: package


.PHONY: build
.PHONY: deploy
.PHONY: package

$(OBJDIR)/%.o: %.c | $(OBJDIR)
	$(CC) -c $(CFLAGS) $(CCFLAGS) $(ARMFLAGS) $(INCLUDE)  $< -o $@

$(OBJDIR)/%.o: %.cpp | $(OBJDIR)
	$(CXX) -c $(CCFLAGS) $(CXXFLAGS) $(ARMFLAGS) $(INCLUDE) $< -o $@

$(BINDIR)/payload.bin: $(OBJ) gcc_arm.ld | $(BINDIR)
	$(LD) $(ARMFLAGS) $(CXXFLAGS) -T gcc_arm.ld $(OBJ) $(LDFLAGS) -o $@

$(BINDIR)/payload.hex: $(BINDIR)/payload.bin
	$(OBJCOPY) -O binary $< $@

$(OBJDIR)/%.o: %.s
	echo "this does nothing yet!"

build: $(BINDIR)/payload.bin

package: $(BINDIR)/payload.hex

deploy: package
		$(FLASH) $(FLASHFLAGS) $(BINDIR)/payload.hex


clean:
	rm -rf $(OBJ) *.hex *.bin *.timestamp
	rm -rf $(OBJDIR)
	rm -rf $(BINDIR)

$(OBJDIR):
	$(MKDIR) $(OBJDIR)

$(BINDIR):
	$(MKDIR) $(BINDIR)

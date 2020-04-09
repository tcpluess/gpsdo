################################################################################
# Company:
#
# Project:        GPS frequency standard
#
# Target:         Cortex M4F
#
# Type:           makefile
#
# Description:    makefile
#
# Compiler:       GNU AS
#
# Filename:       Makefile
#
# Version:        1.0
#
# Author:         Tobias Plüss <tpluess@ieee.org>
#
# Creation-Date:  03.03.2020
################################################################################
# Modification History:
# [1.0]    03.03.2020    Tobias Plüss <tpluess@ieee.org>
# - created
################################################################################

rwildcard=$(wildcard $1$2) $(foreach d,$(wildcard $1*),$(call rwildcard,$d/,$2))

TRGT = arm-none-eabi-
CC   = $(TRGT)gcc
CP   = $(TRGT)objcopy
AS   = $(TRGT)gcc -x assembler-with-cpp
SZ   = $(TRGT)size
GDB  = $(TRGT)gdb
DUMP = $(TRGT)objdump

MCU  = cortex-m4

# List all default C defines here, like -D_DEBUG=1
DDEFS =

# List all default ASM defines here, like -D_DEBUG=1
DADEFS =

# List all default directories to look for include files here
DINCDIR =

# List the default directory to look for the libraries here
DLIBDIR =

# List all default libraries here
DLIBS =

# Define project name and Ram/Flash mode here
PROJECT        = test
RUN_FROM_FLASH = 0
USE_HARD_FPU   = 1
HEAP_SIZE      = 0
STACK_SIZE     = 8k

#
# Define linker script file here
#
ifeq ($(RUN_FROM_FLASH), 0)
LDSCRIPT = ./prj/stm32f407ve_ram.ld
FULL_PRJ = $(PROJECT)_ram
else
LDSCRIPT = ./prj/stm32f407ve_flash.ld
FULL_PRJ = $(PROJECT)_rom
endif

#
# Define FPU settings here
#
ifeq ($(USE_HARD_FPU), 0)
FPU =
else
FPU = -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__FPU_USED=1
endif


# List all user C define here, like -D_DEBUG=1
UDEFS =

# Define ASM defines here
UADEFS =

# List C source files here
SRC = $(call rwildcard,src/,*.c)

# List ASM source files here
ASRC = $(call rwildcard,src/,*.s)

# List all user directories here
UINCDIR = $(sort $(dir $(call rwildcard,src/include/,*)))

# List the user directory to look for the libraries here
ULIBDIR =

# List all user libraries here
ULIBS =

# Define optimisation level here
OPT = -O0 -g3
# OPT = -O2 -falign-functions=16 -fno-inline -fomit-frame-pointer -flto

#
# End of user defines
##############################################################################################

INCDIR  = $(patsubst %,-I%,$(DINCDIR) $(UINCDIR))
LIBDIR  = $(patsubst %,-L%,$(DLIBDIR) $(ULIBDIR))

ifeq ($(RUN_FROM_FLASH), 0)

DEFS    = $(DDEFS) $(UDEFS) -DRUN_FROM_FLASH=0 -DVECT_TAB_SRAM
else

DEFS    = $(DDEFS) $(UDEFS) -DRUN_FROM_FLASH=1
endif

ADEFS   = $(DADEFS) $(UADEFS)
OBJS    = $(addsuffix .o, $(ASRC) $(SRC))
LIBS    = $(DLIBS) $(ULIBS)
MCFLAGS = -mcpu=$(MCU) -mthumb $(FPU)

ASFLAGS  = $(MCFLAGS) $(OPT) -Wa,-amhls=$(addsuffix .lst, $<) $(ADEFS)

CPFLAGS  = $(MCFLAGS) $(OPT) -Wall -Wstrict-prototypes -fverbose-asm
CPFLAGS += -ffunction-sections -fdata-sections
CPFLAGS += -Wa,-ahlms=$(addsuffix .lst, $<) $(DEFS)
CPFLAGS += -MD -MP -MF $(@:.o=.d)

LDFLAGS  = $(MCFLAGS) -nostartfiles -T$(LDSCRIPT) -Xlinker --defsym=__HEAP_SIZE=$(HEAP_SIZE) -Xlinker --defsym=__STACK_SIZE=$(STACK_SIZE)
LDFLAGS += -Wl,-Map=lst/$(FULL_PRJ).map,--cref,--gc-sections,--no-warn-mismatch $(LIBDIR)

.PHONY: all
all: $(OBJS) bin/$(FULL_PRJ).elf bin/$(FULL_PRJ).hex bin/$(FULL_PRJ).s19 \
bin/$(FULL_PRJ).bin lst/disassembly.lst
	@$(SZ) -d bin/$(FULL_PRJ).elf

%.c.o : %.c
	@echo "CC      $<"
	@$(CC) -c $(CPFLAGS) $(INCDIR) $< -o $@

%.s.o : %.s
	@echo "AS      $<"
	@$(AS) -c $(ASFLAGS) $< -o $@

bin/$(FULL_PRJ).elf: $(OBJS)
	@echo "LD      $@"
	@$(CC) $(OBJS) $(LDFLAGS) $(LIBS) -o $@

bin/$(FULL_PRJ).hex: bin/$(FULL_PRJ).elf
	@echo "OBJCOPY $@"
	@$(CP) -O ihex $< $@

bin/$(FULL_PRJ).s19: bin/$(FULL_PRJ).elf
	@echo "OBJCOPY $@"
	@$(CP) -O srec $< $@

bin/$(FULL_PRJ).bin:  bin/$(FULL_PRJ).elf
	@echo "OBJCOPY $@"
	@$(CP) -O binary $< $@

lst/disassembly.lst: bin/$(FULL_PRJ).elf
	@echo "OBJDUMP $(FULL_PRJ).elf"
	@$(DUMP) -S -d bin/$(FULL_PRJ).elf > $@

.PHONY: doc
doc:
	@doxygen
	@if [ -e doc/latex ]; then cd doc/latex; $(MAKE); fi;

.PHONY: clean
clean:
	@-rm -rf $(OBJS)
	@-rm -rf lst/*.lst lst/*.map
	@-rm -rf bin/*.elf bin/*.hex bin/*.bin bin/*.s19
	@-rm -rf $(addsuffix .lst, $(ASRC) $(SRC))
	@-rm -rf $(addsuffix .d, $(SRC))

-include $(wildcard src/*.d)

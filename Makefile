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
# Compiler:       GNU
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
#
# [1.1]    22.03.2022    Tobias Plüss <tpluess@ieee.org>
# - added makefile itself to the dependencies
# - added linkerscript to the dependencies
# - added c++ support
# - improved assembly support
################################################################################

MAKEFLAGS := --jobs=8
MAKEFLAGS += --output-sync=target

rwildcard=$(wildcard $1$2) $(foreach d,$(wildcard $1*),$(call rwildcard,$d/,$2))

TRGT = arm-none-eabi-
CC   = $(TRGT)gcc
CXX  = $(TRGT)g++
CP   = $(TRGT)objcopy
AS   = $(TRGT)as
SZ   = $(TRGT)size
GDB  = $(TRGT)gdb
DUMP = $(TRGT)objdump

MCU  = cortex-m4

# List all default C defines here, like -D_DEBUG=1
DDEFS =

# List all default ASM defines here, like -D_DEBUG=1
DADEFS =

# List all default directories to look for include files here
DINCDIR = $(sort $(dir $(call rwildcard,src/include/,*)))

# List the default directory to look for the libraries here
DLIBDIR =

# List all default libraries here
DLIBS = -lm

# Define project name and Ram/Flash mode here
PROJECT        = gnssdo
RUN_FROM_FLASH = 1
HEAP_SIZE      = 4k
STACK_SIZE     = 2k

#
# Define linker script file here
#
ifeq ($(RUN_FROM_FLASH), 0)
LDSCRIPT = ./prj/stm32f407ve_ram.ld
FULL_PRJ = $(PROJECT)
else
LDSCRIPT = ./prj/stm32f407ve_flash.ld
FULL_PRJ = $(PROJECT)
endif

#
# Define FPU settings here. if FPU usage is not defined,
# default to ENABLE.
#
ifeq ($(USE_HARD_FPU),)
USE_HARD_FPU = 1
endif
ifeq ($(USE_HARD_FPU), 0)
FPU =
else
FPU = -mfloat-abi=hard -mfpu=fpv4-sp-d16
endif

# when running from RAM, assume this is a debug build ...
ifeq ($(RUN_FROM_FLASH), 0)
DDEFS += -DDEBUG
endif

# List all user C define here, like -D_DEBUG=1
UDEFS =

# Define ASM defines here
UADEFS =

# List C source files here
SRC = $(call rwildcard,src/,*.c)

# List C++ source files here
CXXSRC = $(call rwildcard,src/,*.cpp)

# List ASM source files here
ASRC = $(call rwildcard,src/,*.s)

# List all user directories here
UINCDIR = src/freertos/include \
          src/freertos/portable/GCC/ARM_CM4F

# List the user directory to look for the libraries here
ULIBDIR =

# List all user libraries here
ULIBS =

# Define optimisation level here
ifeq ($(RUN_FROM_FLASH), 0)
OPT = -O0 -g3
ASOPT = -g
else
OPT = -O3 -falign-functions=16 -fno-inline -fomit-frame-pointer -flto
ASOPT =
endif

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
OBJS    = $(addsuffix .o, $(ASRC) $(SRC) $(CXXSRC))
LIST    = $(addsuffix .lss, $(ASRC) $(SRC) $(CXXSRC))
DEP     = $(addsuffix .d, $(ASRC) $(SRC) $(CXXSRC))
LIBS    = $(DLIBS) $(ULIBS)
MCFLAGS = -mcpu=$(MCU) -mthumb $(FPU)

ASFLAGS  = -c
ASFLAGS += $(ASOPT)

CPFLAGS  = $(MCFLAGS) $(OPT) -Wall -Wstrict-prototypes -fverbose-asm
CPFLAGS += -ffunction-sections -fdata-sections
CPFLAGS += $(DEFS)
CPFLAGS += -MD -MP -MF $(@:.o=.d)

CXXFLAGS = -c $(MCFLAGS) $(OPT) $(DEFS) -std=c++17 -fno-rtti -fno-exceptions
CXXFLAGS+= -fno-threadsafe-statics -fno-use-cxa-atexit

LDFLAGS  = $(MCFLAGS) -T$(LDSCRIPT) -Xlinker --defsym=__HEAP_SIZE=$(HEAP_SIZE) -Xlinker --defsym=__STACK_SIZE=$(STACK_SIZE)
LDFLAGS += -Wl,-Map=lst/$(FULL_PRJ).map,--cref,--gc-sections,--no-warn-mismatch $(LIBDIR)

.PHONY: all
all: $(OBJS) bin/$(FULL_PRJ).elf bin/$(FULL_PRJ).hex bin/$(FULL_PRJ).s19 \
bin/$(FULL_PRJ).bin lst/$(FULL_PRJ).lss $(LDSCRIPT) Makefile
	@$(SZ) --format=Berkeley -d bin/$(FULL_PRJ).elf

%.c.o : %.c $(LDSCRIPT) Makefile
	@echo "CC      $<"
	@$(CC) -c $(CPFLAGS) $(INCDIR) $< -o $@
	@$(DUMP) -S -d $@ > $(addsuffix .lss, $<)

%.cpp.o : %.cpp $(LDSCRIPT) Makefile
	@echo "CXX     $<"
	@$(CXX) -c $(CXXFLAGS) $(INCDIR) $< -o $@
	@$(DUMP) -S -d $@ > $(addsuffix .lss, $<)

%.s.o : %.s $(LDSCRIPT) Makefile
	@echo "AS      $<"
	@$(AS) $(ASFLAGS) $< -o $@
	@$(DUMP) -S -d $@ > $(addsuffix .lss, $<)

bin/$(FULL_PRJ).elf: $(OBJS) $(LDSCRIPT) Makefile
	@echo "LD      $@"
	@$(CXX) $(OBJS) $(LDFLAGS) $(LIBS) -o $@

bin/$(FULL_PRJ).hex: bin/$(FULL_PRJ).elf $(LDSCRIPT) Makefile
	@echo "OBJCOPY $@"
	@$(CP) -O ihex $< $@

bin/$(FULL_PRJ).s19: bin/$(FULL_PRJ).elf $(LDSCRIPT) Makefile
	@echo "OBJCOPY $@"
	@$(CP) -O srec $< $@

bin/$(FULL_PRJ).bin:  bin/$(FULL_PRJ).elf $(LDSCRIPT) Makefile
	@echo "OBJCOPY $@"
	@$(CP) -O binary $< $@

lst/$(FULL_PRJ).lss: bin/$(FULL_PRJ).elf $(LDSCRIPT) Makefile
	@echo "OBJDUMP $(FULL_PRJ).elf"
	@$(DUMP) -S -d bin/$(FULL_PRJ).elf > $@

.PHONY: doc
doc:
	@doxygen
	@if [ -e doc/latex ]; then cd doc/latex; $(MAKE); fi;

.PHONY: clean
clean:
	@rm -rfv $(OBJS) $(LIST) $(DEP)
	@rm -rfv bin/$(FULL_PRJ).elf bin/$(FULL_PRJ).hex bin/$(FULL_PRJ).bin bin/$(FULL_PRJ).s19
	@rm -rfv lst/$(FULL_PRJ).lss lst/$(FULL_PRJ).map

-include $(DEP)

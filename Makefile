################################################################################
# Company:
#
# Project:        GPS frequency standard
#
# Description:    makefile
#
# Filename:       Makefile
#
# Author:         Tobias Plüss <tpluess@ieee.org>
#
# Creation-Date:  03.03.2020
################################################################################

# uncomment if build is too slow.
#MAKEFLAGS := --jobs=8
#MAKEFLAGS += --output-sync=target

find=$(wildcard $1$2) $(foreach d,$(wildcard $1*),$(call find,$d/,$2))

MCU  = cortex-m4

#
# List all defines here, like -D_DEBUG=1
#
DEFS =

#
# List the user directory to look for the libraries here
#
LIBDIR =

#
# List used libraries here
#
LIBS = -lm

#
# Define Ram/Flash mode here and optionally a project name
#
PROJECT        =
RUN_FROM_FLASH = 0
HEAP_SIZE      = 4k
STACK_SIZE     = 2k
USE_HARD_FPU   = 1

#
# manually add c, assembly or c++ source files if necessary.
# for instance: SRC = somefolder/file.c OR $(call find,somefolder/,*.c)
#
SRC    = $(call find,3rdparty/freertos/,*.c)
CXXSRC =
ASRC   =

#
# List all include directories here (besides src/include)
#
INCDIR = 3rdparty/freertos/include \
         3rdparty/freertos/portable/GCC/ARM_CM4F \
         3rdparty/cmsis

MEMORYMAP = ./prj/stm32f407ve.ld

#
# Define optimisation level here
#
ifeq ($(RUN_FROM_FLASH), 1)
OPT = -O3 -falign-functions=16 -fno-inline -fomit-frame-pointer -flto
else
OPT = -O0 -g3 -Wa,-g
endif

################################################################################

CC   = arm-none-eabi-gcc
CXX  = arm-none-eabi-g++
CP   = arm-none-eabi-objcopy
AS   = arm-none-eabi-gcc -x assembler-with-cpp
SZ   = arm-none-eabi-size
DUMP = arm-none-eabi-objdump

#
# automatically determine project file name based on directory name if empty
#
ifndef PROJECT
PROJECT = $(notdir $(CURDIR))
endif

#
# Define FPU settings here.
#
ifeq ($(USE_HARD_FPU), 1)
FPU = -mhard-float -mfloat-abi=hard -mfpu=fpv4-sp-d16
else
FPU =
endif

#
# define an additional symbol when code runs from ram
#
ifeq ($(RUN_FROM_FLASH), 1)
DEFS += -DRUN_FROM_FLASH
endif

#
# default directories to look for include files
#
INCDIR += $(sort $(dir $(call find,src/include/,*)))

#
# Define linker script file here depending on the ram/flash mode
#
ifeq ($(RUN_FROM_FLASH), 1)
LDSCRIPT = ./prj/flash.ld
else
LDSCRIPT = ./prj/ram.ld
endif

#
# automatically find all c, assembly and c++ source files
#
SRC    += $(call find,src/,*.c)
CXXSRC += $(call find,src/,*.cpp)
ASRC   += $(call find,src/,*.s)
ASRC   += $(call find,src/,*.S)

INC  = $(patsubst %,-I%,$(INCDIR))
LIB  = $(patsubst %,-L%,$(LIBDIR))

OBJS    = $(addsuffix .o, $(ASRC) $(SRC) $(CXXSRC))
LIST    = $(addsuffix .lss, $(ASRC) $(SRC) $(CXXSRC))
DEP     = $(addsuffix .d, $(ASRC) $(SRC) $(CXXSRC))
SU      = $(addsuffix .su, $(SRC) $(CXXSRC)) #stack usage
#CI      = $(addsuffix .su, $(SRC) $(CXXSRC)) #callgraph info

MCFLAGS = -mcpu=$(MCU) -mthumb $(FPU)

WARNFLAGS  = -Wall -Wextra -Wimplicit-fallthrough -Wshadow -Wunused
WARNFLAGS += -Wmisleading-indentation -Wswitch-default
WARNFLAGS += -Wformat=2 -Wformat-truncation -Wundef -Wpedantic
WARNFLAGS += -Wstack-usage=540

COMMONFLAGS  = -ffunction-sections -fdata-sections -fverbose-asm -fno-common
COMMONFLAGS += -fstack-usage #-fcallgraph-info

ASFLAGS  = $(MCFLAGS) $(OPT) $(DEFS)

CPFLAGS  = $(MCFLAGS) $(OPT) $(DEFS) $(WARNFLAGS) $(COMMONFLAGS)
CPFLAGS += -Wstrict-prototypes -Wmissing-prototypes
CPFLAGS += -MD -MP -MF $(@:.o=.d)

CXXFLAGS  = $(MCFLAGS) $(OPT) $(DEFS) $(WARNFLAGS) $(COMMONFLAGS)
CXXFLAGS += -std=c++20
CXXFLAGS += -fno-threadsafe-statics -fno-use-cxa-atexit -fno-rtti -fno-exceptions
CXXFLAGS += -MD -MP -MF $(@:.o=.d)

LDFLAGS  = $(MCFLAGS) $(OPT) $(DEFS) $(WARNFLAGS) $(COMMONFLAGS)
LDFLAGS += -T$(MEMORYMAP) -T$(LDSCRIPT)
LDFLAGS += -Xlinker --defsym=__HEAP_SIZE=$(HEAP_SIZE)
LDFLAGS += -Xlinker --defsym=__STACK_SIZE=$(STACK_SIZE)
LDFLAGS += -Wl,-Map=lst/$(PROJECT).map,--cref,--gc-sections,--no-warn-mismatch,--no-warn-rwx-segments $(LIB)

.PHONY: all
all: $(OBJS) bin/$(PROJECT).elf bin/$(PROJECT).hex bin/$(PROJECT).s19 \
bin/$(PROJECT).bin lst/$(PROJECT).lss $(LDSCRIPT) Makefile
	@$(SZ) --format=Berkeley -d bin/$(PROJECT).elf

%.c.o : %.c $(LDSCRIPT) Makefile
	@echo "CC      $<"
	@$(CC) -c $(CPFLAGS) $(INC) $< -o $@
	@$(DUMP) -S -d $@ > $(addsuffix .lss, $<)

%.cpp.o : %.cpp $(LDSCRIPT) Makefile
	@echo "CXX     $<"
	@$(CXX) -c $(CXXFLAGS) $(INC) $< -o $@
	@$(DUMP) -S -d $@ > $(addsuffix .lss, $<)

%.s.o : %.s $(LDSCRIPT) Makefile
	@echo "AS      $<"
	@$(AS) -c $(ASFLAGS) $< -o $@
	@$(DUMP) -S -d $@ > $(addsuffix .lss, $<)

%.S.o : %.S $(LDSCRIPT) Makefile
	@echo "AS      $<"
	@$(AS) -c $(ASFLAGS) $< -o $@
	@$(DUMP) -S -d $@ > $(addsuffix .lss, $<)

bin/$(PROJECT).elf: $(OBJS) $(LDSCRIPT) Makefile
	@echo "LD      $@"
	@$(CXX) $(OBJS) $(LDFLAGS) $(LIBS) -o $@

bin/$(PROJECT).hex: bin/$(PROJECT).elf $(LDSCRIPT) Makefile
	@echo "OBJCOPY $@"
	@$(CP) -O ihex $< $@

bin/$(PROJECT).s19: bin/$(PROJECT).elf $(LDSCRIPT) Makefile
	@echo "OBJCOPY $@"
	@$(CP) -O srec $< $@

bin/$(PROJECT).bin:  bin/$(PROJECT).elf $(LDSCRIPT) Makefile
	@echo "OBJCOPY $@"
	@$(CP) -O binary $< $@

lst/$(PROJECT).lss: bin/$(PROJECT).elf $(LDSCRIPT) Makefile
	@echo "OBJDUMP $(PROJECT).elf"
	@$(DUMP) -S -d bin/$(PROJECT).elf > $@

.PHONY: doc
doc:
	@doxygen
	@if [ -e doc/latex ]; then cd doc/latex; $(MAKE); fi;

.PHONY: clean
clean:
	@rm -rfv $(OBJS) $(LIST) $(DEP) $(SU)
	@rm -rfv bin/$(PROJECT).elf bin/$(PROJECT).hex bin/$(PROJECT).bin bin/$(PROJECT).s19
	@rm -rfv lst/$(PROJECT).lss lst/$(PROJECT).map

-include $(DEP)

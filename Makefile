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

MAKEFLAGS := --jobs=8
MAKEFLAGS += --output-sync=target

rwildcard=$(wildcard $1$2) $(foreach d,$(wildcard $1*),$(call rwildcard,$d/,$2))

CC   = arm-none-eabi-gcc
CXX  = arm-none-eabi-g++
CP   = arm-none-eabi-objcopy
AS   = arm-none-eabi-gcc -x assembler-with-cpp
SZ   = arm-none-eabi-size
GDB  = arm-none-eabi-gdb
DUMP = arm-none-eabi-objdump

MCU  = cortex-m4

# List all defines here, like -D_DEBUG=1
DEFS =

# List all default directories to look for include files here
DINCDIR = $(sort $(dir $(call rwildcard,src/include/,*)))

# List the default directory to look for the libraries here
DLIBDIR =

# List all default libraries here
DLIBS = -lm

# Define project name and Ram/Flash mode here
PROJECT        = gnssdo
RUN_FROM_FLASH = 0
HEAP_SIZE      = 4k
STACK_SIZE     = 2k
USE_HARD_FPU   = 1

#
# Define linker script file here
#
ifeq ($(RUN_FROM_FLASH), 0)
LDSCRIPT = ./prj/stm32f407ve_ram.ld
else
LDSCRIPT = ./prj/stm32f407ve_flash.ld
endif

#
# Define FPU settings here.
#
ifeq ($(USE_HARD_FPU), 0)
FPU =
else
FPU = -mfloat-abi=hard -mfpu=fpv4-sp-d16
endif

# when running from RAM, assume this is a debug build ...
ifeq ($(RUN_FROM_FLASH), 0)
# DEFS += -DDEBUG
endif

# List C source files here
SRC = $(call rwildcard,src/,*.c)

# List C++ source files here
CXXSRC = $(call rwildcard,src/,*.cpp)

# List ASM source files here
ASRC = $(call rwildcard,src/,*.s)
ASRC += $(call rwildcard,src/,*.S)

# List all user directories here
UINCDIR = src/freertos/include \
          src/freertos/portable/GCC/ARM_CM4F

# List the user directory to look for the libraries here
ULIBDIR =

# List all user libraries here
ULIBS =

# Define optimisation level here
ifeq ($(RUN_FROM_FLASH), 0)
OPT = -O0 -g3 -Wa,-g
else
OPT = -O3 -falign-functions=16 -fno-inline -fomit-frame-pointer -flto
endif

#
# End of user defines
##############################################################################################

INCDIR  = $(patsubst %,-I%,$(DINCDIR) $(UINCDIR))
LIBDIR  = $(patsubst %,-L%,$(DLIBDIR) $(ULIBDIR))

OBJS    = $(addsuffix .o, $(ASRC) $(SRC) $(CXXSRC))
LIST    = $(addsuffix .lss, $(ASRC) $(SRC) $(CXXSRC))
DEP     = $(addsuffix .d, $(ASRC) $(SRC) $(CXXSRC))
LIBS    = $(DLIBS) $(ULIBS)
MCFLAGS = -mcpu=$(MCU) -mthumb $(FPU)

ASFLAGS  = $(MCFLAGS) $(OPT) $(DEFS)

CPFLAGS  = $(MCFLAGS) $(OPT) $(DEFS) -Wall -Wstrict-prototypes -Wextra -fverbose-asm
CPFLAGS += -ffunction-sections -fdata-sections
CPFLAGS += -MD -MP -MF $(@:.o=.d)

CXXFLAGS = $(MCFLAGS) $(OPT) $(DEFS) -Wall -Wextra -std=c++17 -fverbose-asm
CXXFLAGS+= -fno-threadsafe-statics -fno-use-cxa-atexit -fno-rtti -fno-exceptions
CXXFLAGS += -MD -MP -MF $(@:.o=.d)

LDFLAGS  = $(MCFLAGS) -T$(LDSCRIPT)
LDFLAGS += -Xlinker --defsym=__HEAP_SIZE=$(HEAP_SIZE)
LDFLAGS += -Xlinker --defsym=__STACK_SIZE=$(STACK_SIZE)
LDFLAGS += -Wl,-Map=lst/$(PROJECT).map,--cref,--gc-sections,--no-warn-mismatch $(LIBDIR)

.PHONY: all
all: $(OBJS) bin/$(PROJECT).elf bin/$(PROJECT).hex bin/$(PROJECT).s19 \
bin/$(PROJECT).bin lst/$(PROJECT).lss $(LDSCRIPT) Makefile
	@$(SZ) --format=Berkeley -d bin/$(PROJECT).elf

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
	@rm -rfv $(OBJS) $(LIST) $(DEP)
	@rm -rfv bin/$(PROJECT).elf bin/$(PROJECT).hex bin/$(PROJECT).bin bin/$(PROJECT).s19
	@rm -rfv lst/$(PROJECT).lss lst/$(PROJECT).map

-include $(DEP)

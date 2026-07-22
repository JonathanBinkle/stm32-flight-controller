# >>> TODO: adjust if necessary
PROJECT		= blinky
DEVICE		= stm32f411ceu6
FLASH_BASE	= 0x08000000
SRC_DIR		= src
INCLUDE_DIR	= include
OPENCM3_DIR	= lib/libopencm3
OPT 		= -O0
CSTD 		?= -std=c11
# <<<

### NOTE: you shouldn't need to adjust anything below. ###

include $(OPENCM3_DIR)/mk/genlink-config.mk

# General
BUILD_DIR	:= $(SRC_DIR)/bin
Q			:= @
NULL		:= 2>/dev/null
CFILES		:= $(shell find $(SRC_DIR) -name '*.c')
VPATH 		:= $(SRC_DIR) $(INCLUDE_DIR)

# GNU ARM toolchain
PREFIX	?= arm-none-eabi-
CC		= $(PREFIX)gcc
LD		= $(PREFIX)gcc
OBJCOPY	= $(PREFIX)objcopy
OBJDUMP	= $(PREFIX)objdump

# Header files
OPENCM3_INC	= $(OPENCM3_DIR)/include
NEWLIB_INC  = /usr/include/newlib
INCLUDES    += -I$(INCLUDE_DIR) -I$(OPENCM3_INC) -isystem $(NEWLIB_INC)

# Object files
OBJS = $(CFILES:%.c=$(BUILD_DIR)/%.o)
GENERATED_BINS = $(BUILD_DIR)/$(PROJECT).elf $(BUILD_DIR)/$(PROJECT).bin

# Preprocessor flags
TGT_CPPFLAGS += -MD
TGT_CPPFLAGS += -Wall -Wundef $(INCLUDES)
TGT_CPPFLAGS += $(INCLUDES) $(OPENCM3_DEFS)

# Compiler flags
TGT_CFLAGS += $(OPT) $(CSTD) -ggdb3
TGT_CFLAGS += $(ARCH_FLAGS)
TGT_CFLAGS += -fno-common
TGT_CFLAGS += -ffunction-sections -fdata-sections
TGT_CFLAGS += -Wextra -Wshadow -Wno-unused-variable -Wimplicit-function-declaration
TGT_CFLAGS += -Wredundant-decls -Wstrict-prototypes -Wmissing-prototypes

# Linker flags
LDSCRIPT	:= $(SRC_DIR)/$(LDSCRIPT)
TGT_LDFLAGS += -T$(LDSCRIPT) -L$(OPENCM3_DIR)/lib -nostartfiles
TGT_LDFLAGS += $(ARCH_FLAGS)
TGT_LDFLAGS += -specs=nano.specs
TGT_LDFLAGS += -Wl,--gc-sections

# Linker script generator fills this in for us
ifeq (,$(DEVICE))
	LDLIBS += -l$(OPENCM3_LIB)
endif
LDLIBS += -Wl,--start-group -lc -lm -lgcc -lnosys -Wl,--end-group

# Use linker script generator and clean it up it afterwards
ifeq (,$(DEVICE))
$(LDSCRIPT):
ifeq (,$(wildcard $(LDSCRIPT)))
    $(error Unable to find specified linker script: $(LDSCRIPT))
endif
else
GENERATED_BINS += $(LDSCRIPT)
endif

# Build rules
all: $(BUILD_DIR)/$(PROJECT).elf $(BUILD_DIR)/$(PROJECT).bin

lsp:
	@bear -- make clean all

flash:
	@st-flash --connect-under-reset --reset write $(BUILD_DIR)/$(PROJECT).bin $(FLASH_BASE)

debug:
	@st-util

$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(Q)$(CC) $(TGT_CFLAGS) $(CFLAGS) $(TGT_CPPFLAGS) $(CPPFLAGS) -o $@ -c $<

$(BUILD_DIR)/$(PROJECT).elf: $(OBJS) $(LDSCRIPT) $(LIBDEPS)
	$(Q)$(LD) $(TGT_LDFLAGS) $(LDFLAGS) $(OBJS) $(LDLIBS) -o $@

%.bin: %.elf
	$(Q)$(OBJCOPY) -O binary  $< $@

clean:
	rm -rf $(BUILD_DIR) $(GENERATED_BINS)

.PHONY: all build flash debug clean
-include $(OBJS:.o=.d)
include $(OPENCM3_DIR)/mk/genlink-rules.mk

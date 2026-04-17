ROOT    := $(shell pwd)
BUILD   := $(ROOT)/build
LD_SCRIPT := $(ROOT)/STM32F103CBTX_FLASH.ld
ELF     := $(BUILD)/firmware.elf

CC      := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
SIZE    := arm-none-eabi-size

CPU     := -mcpu=cortex-m3 -mthumb -mfloat-abi=soft
DEFS    := -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB
INCS    := -I$(ROOT)/Core/Inc \
           -I$(ROOT)/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy \
           -I$(ROOT)/Drivers/STM32F1xx_HAL_Driver/Inc \
           -I$(ROOT)/Drivers/CMSIS/Device/ST/STM32F1xx/Include \
           -I$(ROOT)/Drivers/CMSIS/Include

CFLAGS  := $(CPU) -std=gnu11 -g3 $(DEFS) $(INCS) -O0 -ffunction-sections -fdata-sections -Wall --specs=nano.specs
ASFLAGS := $(CPU) -g3 $(DEFS) -c -x assembler-with-cpp --specs=nano.specs
LDFLAGS := $(CPU) -T$(LD_SCRIPT) --specs=nosys.specs --specs=nano.specs \
           -Wl,-Map=$(BUILD)/firmware.map -Wl,--gc-sections -static \
           -Wl,--start-group -lc -lm -Wl,--end-group

C_SRCS  := $(wildcard Core/Src/*.c) \
           $(wildcard Drivers/STM32F1xx_HAL_Driver/Src/*.c)
ASM_SRCS := $(wildcard Core/Startup/*.s)

OBJS    := $(patsubst %.c,$(BUILD)/%.o,$(C_SRCS)) \
           $(patsubst %.s,$(BUILD)/%.o,$(ASM_SRCS))

.PHONY: all flash clean

all: $(ELF)
	@$(SIZE) $<

$(ELF): $(OBJS)
	@mkdir -p $(dir $@)
	@echo "  LD  $@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	@echo "  CC  $<"
	@$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/%.o: %.s
	@mkdir -p $(dir $@)
	@echo "  ASM $<"
	@$(CC) $(ASFLAGS) -c $< -o $@

flash: $(ELF)
	@openocd -f interface/cmsis-dap.cfg -f target/stm32f1x.cfg -c "program $< verify reset exit"

clean:
	@rm -rf $(BUILD)

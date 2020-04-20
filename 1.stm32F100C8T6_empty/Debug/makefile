################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
stm32F100C8T6_empty.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
stm32F100C8T6_empty.list \

OBJCOPY_HEX += \
stm32F100C8T6_empty.hex \

OBJCOPY_BIN += \
stm32F100C8T6_empty.bin \


# All Target
all: stm32F100C8T6_empty.elf secondary-outputs

# Tool invocations
stm32F100C8T6_empty.elf: $(OBJS) $(USER_OBJS) D:\stm32exOnPix\source\ code\template\1.stm32F100C8T6_empty\STM32F100C8TX_FLASH.ld
	arm-none-eabi-gcc -o "stm32F100C8T6_empty.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"D:\stm32exOnPix\source code\template\1.stm32F100C8T6_empty\STM32F100C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="stm32F100C8T6_empty.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

stm32F100C8T6_empty.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "stm32F100C8T6_empty.list"
	@echo 'Finished building: $@'
	@echo ' '

stm32F100C8T6_empty.hex: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "stm32F100C8T6_empty.hex"
	@echo 'Finished building: $@'
	@echo ' '

stm32F100C8T6_empty.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "stm32F100C8T6_empty.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

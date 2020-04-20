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
LED_stm32F100C8T6.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
LED_stm32F100C8T6.list \

OBJCOPY_HEX += \
LED_stm32F100C8T6.hex \

OBJCOPY_BIN += \
LED_stm32F100C8T6.bin \


# All Target
all: LED_stm32F100C8T6.elf secondary-outputs

# Tool invocations
LED_stm32F100C8T6.elf: $(OBJS) $(USER_OBJS) D:\stm32exOnPix\source\ code\template\2.LED_stm32F100C8T6\STM32F100C8TX_FLASH.ld
	arm-none-eabi-gcc -o "LED_stm32F100C8T6.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"D:\stm32exOnPix\source code\template\2.LED_stm32F100C8T6\STM32F100C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="LED_stm32F100C8T6.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

LED_stm32F100C8T6.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "LED_stm32F100C8T6.list"
	@echo 'Finished building: $@'
	@echo ' '

LED_stm32F100C8T6.hex: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "LED_stm32F100C8T6.hex"
	@echo 'Finished building: $@'
	@echo ' '

LED_stm32F100C8T6.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "LED_stm32F100C8T6.bin"
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

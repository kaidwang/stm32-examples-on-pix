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
Timer_stm32F100.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Timer_stm32F100.list \

OBJCOPY_HEX += \
Timer_stm32F100.hex \

OBJCOPY_BIN += \
Timer_stm32F100.bin \


# All Target
all: Timer_stm32F100.elf secondary-outputs

# Tool invocations
Timer_stm32F100.elf: $(OBJS) $(USER_OBJS) D:\stm32exOnPix\source\ code\template\3.Timer_stm32F100\STM32F100C8TX_FLASH.ld
	arm-none-eabi-gcc -o "Timer_stm32F100.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"D:\stm32exOnPix\source code\template\3.Timer_stm32F100\STM32F100C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Timer_stm32F100.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Timer_stm32F100.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Timer_stm32F100.list"
	@echo 'Finished building: $@'
	@echo ' '

Timer_stm32F100.hex: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "Timer_stm32F100.hex"
	@echo 'Finished building: $@'
	@echo ' '

Timer_stm32F100.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Timer_stm32F100.bin"
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

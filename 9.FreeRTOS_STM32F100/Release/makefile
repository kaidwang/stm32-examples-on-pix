################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/subdir.mk
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
FreeRTOS_STM32F100.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
FreeRTOS_STM32F100.list \

OBJCOPY_HEX += \
FreeRTOS_STM32F100.hex \

OBJCOPY_BIN += \
FreeRTOS_STM32F100.bin \


# All Target
all: FreeRTOS_STM32F100.elf secondary-outputs

# Tool invocations
FreeRTOS_STM32F100.elf: $(OBJS) $(USER_OBJS) D:\stm32exOnPix\source\ code\template\9.FreeRTOS_STM32F100\STM32F100C8TX_FLASH.ld
	arm-none-eabi-gcc -o "FreeRTOS_STM32F100.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"D:\stm32exOnPix\source code\template\9.FreeRTOS_STM32F100\STM32F100C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="FreeRTOS_STM32F100.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

FreeRTOS_STM32F100.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "FreeRTOS_STM32F100.list"
	@echo 'Finished building: $@'
	@echo ' '

FreeRTOS_STM32F100.hex: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "FreeRTOS_STM32F100.hex"
	@echo 'Finished building: $@'
	@echo ' '

FreeRTOS_STM32F100.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "FreeRTOS_STM32F100.bin"
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

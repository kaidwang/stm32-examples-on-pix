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
Timer_PWM_Input_STM32F100.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Timer_PWM_Input_STM32F100.list \


# All Target
all: Timer_PWM_Input_STM32F100.elf secondary-outputs

# Tool invocations
Timer_PWM_Input_STM32F100.elf: $(OBJS) $(USER_OBJS) D:\Lesson\ plan\source\ code\template\Timer_PWM_Input_STM32F100\STM32F100C8TX_FLASH.ld
	arm-none-eabi-gcc -o "Timer_PWM_Input_STM32F100.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"D:\Lesson plan\source code\template\Timer_PWM_Input_STM32F100\STM32F100C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Timer_PWM_Input_STM32F100.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Timer_PWM_Input_STM32F100.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Timer_PWM_Input_STM32F100.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

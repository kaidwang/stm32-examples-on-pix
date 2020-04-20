################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
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
SPI_MPU6000_STM32F427.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
SPI_MPU6000_STM32F427.list \

OBJCOPY_HEX += \
SPI_MPU6000_STM32F427.hex \

OBJCOPY_BIN += \
SPI_MPU6000_STM32F427.bin \


# All Target
all: SPI_MPU6000_STM32F427.elf secondary-outputs

# Tool invocations
SPI_MPU6000_STM32F427.elf: $(OBJS) $(USER_OBJS) D:\stm32exOnPix\source\ code\template\8.SPI_MPU6000_STM32F427\STM32F427VITX_FLASH.ld
	arm-none-eabi-gcc -o "SPI_MPU6000_STM32F427.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"D:\stm32exOnPix\source code\template\8.SPI_MPU6000_STM32F427\STM32F427VITX_FLASH.ld" --specs=nosys.specs -Wl,-Map="SPI_MPU6000_STM32F427.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

SPI_MPU6000_STM32F427.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "SPI_MPU6000_STM32F427.list"
	@echo 'Finished building: $@'
	@echo ' '

SPI_MPU6000_STM32F427.hex: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "SPI_MPU6000_STM32F427.hex"
	@echo 'Finished building: $@'
	@echo ' '

SPI_MPU6000_STM32F427.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "SPI_MPU6000_STM32F427.bin"
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

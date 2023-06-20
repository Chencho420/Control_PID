################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/EEPROM/eeprom_emul.c \
../Drivers/EEPROM/flash_interface.c \
../Drivers/EEPROM/stm32g4xx_ll_crc.c 

OBJS += \
./Drivers/EEPROM/eeprom_emul.o \
./Drivers/EEPROM/flash_interface.o \
./Drivers/EEPROM/stm32g4xx_ll_crc.o 

C_DEPS += \
./Drivers/EEPROM/eeprom_emul.d \
./Drivers/EEPROM/flash_interface.d \
./Drivers/EEPROM/stm32g4xx_ll_crc.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/EEPROM/%.o Drivers/EEPROM/%.su Drivers/EEPROM/%.cyclo: ../Drivers/EEPROM/%.c Drivers/EEPROM/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I"/home/gabriel/STM32CubeIDE/workspace_1.12.0/PID VERSION 3B/Drivers/SSD1306_driver" -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I"/home/gabriel/STM32CubeIDE/workspace_1.12.0/PID VERSION 3B/Drivers/EEPROM" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-EEPROM

clean-Drivers-2f-EEPROM:
	-$(RM) ./Drivers/EEPROM/eeprom_emul.cyclo ./Drivers/EEPROM/eeprom_emul.d ./Drivers/EEPROM/eeprom_emul.o ./Drivers/EEPROM/eeprom_emul.su ./Drivers/EEPROM/flash_interface.cyclo ./Drivers/EEPROM/flash_interface.d ./Drivers/EEPROM/flash_interface.o ./Drivers/EEPROM/flash_interface.su ./Drivers/EEPROM/stm32g4xx_ll_crc.cyclo ./Drivers/EEPROM/stm32g4xx_ll_crc.d ./Drivers/EEPROM/stm32g4xx_ll_crc.o ./Drivers/EEPROM/stm32g4xx_ll_crc.su

.PHONY: clean-Drivers-2f-EEPROM


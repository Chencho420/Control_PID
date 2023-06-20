################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/SSD1306_driver/ssd1306.c \
../Drivers/SSD1306_driver/ssd1306_fonts.c 

OBJS += \
./Drivers/SSD1306_driver/ssd1306.o \
./Drivers/SSD1306_driver/ssd1306_fonts.o 

C_DEPS += \
./Drivers/SSD1306_driver/ssd1306.d \
./Drivers/SSD1306_driver/ssd1306_fonts.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/SSD1306_driver/%.o Drivers/SSD1306_driver/%.su Drivers/SSD1306_driver/%.cyclo: ../Drivers/SSD1306_driver/%.c Drivers/SSD1306_driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I"/home/gabriel/STM32CubeIDE/workspace_1.12.0/PID VERSION 3B/Drivers/SSD1306_driver" -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I"/home/gabriel/STM32CubeIDE/workspace_1.12.0/PID VERSION 3B/Drivers/EEPROM" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-SSD1306_driver

clean-Drivers-2f-SSD1306_driver:
	-$(RM) ./Drivers/SSD1306_driver/ssd1306.cyclo ./Drivers/SSD1306_driver/ssd1306.d ./Drivers/SSD1306_driver/ssd1306.o ./Drivers/SSD1306_driver/ssd1306.su ./Drivers/SSD1306_driver/ssd1306_fonts.cyclo ./Drivers/SSD1306_driver/ssd1306_fonts.d ./Drivers/SSD1306_driver/ssd1306_fonts.o ./Drivers/SSD1306_driver/ssd1306_fonts.su

.PHONY: clean-Drivers-2f-SSD1306_driver


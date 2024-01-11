################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/ili9341.c \
../Core/Inc/lcd_io_gpio8.c \
../Core/Inc/stm32_adafruit_lcd.c \
../Core/Inc/structPackets.c 

OBJS += \
./Core/Inc/ili9341.o \
./Core/Inc/lcd_io_gpio8.o \
./Core/Inc/stm32_adafruit_lcd.o \
./Core/Inc/structPackets.o 

C_DEPS += \
./Core/Inc/ili9341.d \
./Core/Inc/lcd_io_gpio8.d \
./Core/Inc/stm32_adafruit_lcd.d \
./Core/Inc/structPackets.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/%.o Core/Inc/%.su Core/Inc/%.cyclo: ../Core/Inc/%.c Core/Inc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc

clean-Core-2f-Inc:
	-$(RM) ./Core/Inc/ili9341.cyclo ./Core/Inc/ili9341.d ./Core/Inc/ili9341.o ./Core/Inc/ili9341.su ./Core/Inc/lcd_io_gpio8.cyclo ./Core/Inc/lcd_io_gpio8.d ./Core/Inc/lcd_io_gpio8.o ./Core/Inc/lcd_io_gpio8.su ./Core/Inc/stm32_adafruit_lcd.cyclo ./Core/Inc/stm32_adafruit_lcd.d ./Core/Inc/stm32_adafruit_lcd.o ./Core/Inc/stm32_adafruit_lcd.su ./Core/Inc/structPackets.cyclo ./Core/Inc/structPackets.d ./Core/Inc/structPackets.o ./Core/Inc/structPackets.su

.PHONY: clean-Core-2f-Inc


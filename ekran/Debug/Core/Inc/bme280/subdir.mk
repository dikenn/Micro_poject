################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/bme280/bme280.c 

OBJS += \
./Core/Inc/bme280/bme280.o 

C_DEPS += \
./Core/Inc/bme280/bme280.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/bme280/%.o Core/Inc/bme280/%.su Core/Inc/bme280/%.cyclo: ../Core/Inc/bme280/%.c Core/Inc/bme280/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-bme280

clean-Core-2f-Inc-2f-bme280:
	-$(RM) ./Core/Inc/bme280/bme280.cyclo ./Core/Inc/bme280/bme280.d ./Core/Inc/bme280/bme280.o ./Core/Inc/bme280/bme280.su

.PHONY: clean-Core-2f-Inc-2f-bme280


################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/exc7200/exc7200.c 

OBJS += \
./Drivers/BSP/Components/exc7200/exc7200.o 

C_DEPS += \
./Drivers/BSP/Components/exc7200/exc7200.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/exc7200/%.o Drivers/BSP/Components/exc7200/%.su Drivers/BSP/Components/exc7200/%.cyclo: ../Drivers/BSP/Components/exc7200/%.c Drivers/BSP/Components/exc7200/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../Core/Inc -I../Drivers/BSP/STM32F429I-Discovery -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Components-2f-exc7200

clean-Drivers-2f-BSP-2f-Components-2f-exc7200:
	-$(RM) ./Drivers/BSP/Components/exc7200/exc7200.cyclo ./Drivers/BSP/Components/exc7200/exc7200.d ./Drivers/BSP/Components/exc7200/exc7200.o ./Drivers/BSP/Components/exc7200/exc7200.su

.PHONY: clean-Drivers-2f-BSP-2f-Components-2f-exc7200


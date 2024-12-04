################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/NUCLEO-WB15CC/nucleo_wb15cc.c 

OBJS += \
./Drivers/BSP/NUCLEO-WB15CC/nucleo_wb15cc.o 

C_DEPS += \
./Drivers/BSP/NUCLEO-WB15CC/nucleo_wb15cc.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/NUCLEO-WB15CC/%.o Drivers/BSP/NUCLEO-WB15CC/%.su Drivers/BSP/NUCLEO-WB15CC/%.cyclo: ../Drivers/BSP/NUCLEO-WB15CC/%.c Drivers/BSP/NUCLEO-WB15CC/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_NUCLEO_64 -DUSE_HAL_DRIVER -DSTM32WB15xx -c -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/BSP/NUCLEO-WB15CC -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-NUCLEO-2d-WB15CC

clean-Drivers-2f-BSP-2f-NUCLEO-2d-WB15CC:
	-$(RM) ./Drivers/BSP/NUCLEO-WB15CC/nucleo_wb15cc.cyclo ./Drivers/BSP/NUCLEO-WB15CC/nucleo_wb15cc.d ./Drivers/BSP/NUCLEO-WB15CC/nucleo_wb15cc.o ./Drivers/BSP/NUCLEO-WB15CC/nucleo_wb15cc.su

.PHONY: clean-Drivers-2f-BSP-2f-NUCLEO-2d-WB15CC


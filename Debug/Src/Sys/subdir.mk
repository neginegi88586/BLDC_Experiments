################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/Sys/system_stm32f4xx.c 

OBJS += \
./Src/Sys/system_stm32f4xx.o 

C_DEPS += \
./Src/Sys/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/Sys/%.o Src/Sys/%.su Src/Sys/%.cyclo: ../Src/Sys/%.c Src/Sys/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F405xx -DSTM32 -DSTM32F4 -DSTM32F405RGTx -c -I"C:/Users/idune/STM32CubeIDE/workspace_1.19.0/BLDC_Experiments/Inc" -I"C:/Users/idune/STM32CubeIDE/workspace_1.19.0/BLDC_Experiments/Inc/Sys" -I"C:/Users/idune/STM32CubeIDE/workspace_1.19.0/BLDC_Experiments/Src" -I"C:/Users/idune/STM32CubeIDE/workspace_1.19.0/BLDC_Experiments/Src/Sys" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src-2f-Sys

clean-Src-2f-Sys:
	-$(RM) ./Src/Sys/system_stm32f4xx.cyclo ./Src/Sys/system_stm32f4xx.d ./Src/Sys/system_stm32f4xx.o ./Src/Sys/system_stm32f4xx.su

.PHONY: clean-Src-2f-Sys


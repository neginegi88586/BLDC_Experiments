################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/app.c \
../Src/bemf_pll.c \
../Src/encoder.c \
../Src/firmware.c \
../Src/foc.c \
../Src/main.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/app.o \
./Src/bemf_pll.o \
./Src/encoder.o \
./Src/firmware.o \
./Src/foc.o \
./Src/main.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/app.d \
./Src/bemf_pll.d \
./Src/encoder.d \
./Src/firmware.d \
./Src/foc.d \
./Src/main.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F405xx -DSTM32 -DSTM32F4 -DSTM32F405RGTx -c -I"C:/Users/idune/STM32CubeIDE/workspace_1.19.0/BLDC_Experiments/Inc" -I"C:/Users/idune/STM32CubeIDE/workspace_1.19.0/BLDC_Experiments/Inc/Sys" -I"C:/Users/idune/STM32CubeIDE/workspace_1.19.0/BLDC_Experiments/Src" -I"C:/Users/idune/STM32CubeIDE/workspace_1.19.0/BLDC_Experiments/Src/Sys" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/app.cyclo ./Src/app.d ./Src/app.o ./Src/app.su ./Src/bemf_pll.cyclo ./Src/bemf_pll.d ./Src/bemf_pll.o ./Src/bemf_pll.su ./Src/encoder.cyclo ./Src/encoder.d ./Src/encoder.o ./Src/encoder.su ./Src/firmware.cyclo ./Src/firmware.d ./Src/firmware.o ./Src/firmware.su ./Src/foc.cyclo ./Src/foc.d ./Src/foc.o ./Src/foc.su ./Src/main.cyclo ./Src/main.d ./Src/main.o ./Src/main.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su

.PHONY: clean-Src


################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Lib/MATLAB/state_feedback_controller.c \
../Core/Lib/MATLAB/state_feedback_controller_data.c 

C_DEPS += \
./Core/Lib/MATLAB/state_feedback_controller.d \
./Core/Lib/MATLAB/state_feedback_controller_data.d 

OBJS += \
./Core/Lib/MATLAB/state_feedback_controller.o \
./Core/Lib/MATLAB/state_feedback_controller_data.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Lib/MATLAB/%.o Core/Lib/MATLAB/%.su Core/Lib/MATLAB/%.cyclo: ../Core/Lib/MATLAB/%.c Core/Lib/MATLAB/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303x8 -c -I../Core/Inc -I"C:/Program Files/MATLAB/R2024b/simulink/include" -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/nopla/dev/InvertedPendulum/Adorobo-Inverted-Pendulum/Core/Lib/MATLAB" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Lib-2f-MATLAB

clean-Core-2f-Lib-2f-MATLAB:
	-$(RM) ./Core/Lib/MATLAB/state_feedback_controller.cyclo ./Core/Lib/MATLAB/state_feedback_controller.d ./Core/Lib/MATLAB/state_feedback_controller.o ./Core/Lib/MATLAB/state_feedback_controller.su ./Core/Lib/MATLAB/state_feedback_controller_data.cyclo ./Core/Lib/MATLAB/state_feedback_controller_data.d ./Core/Lib/MATLAB/state_feedback_controller_data.o ./Core/Lib/MATLAB/state_feedback_controller_data.su

.PHONY: clean-Core-2f-Lib-2f-MATLAB


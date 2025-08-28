################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/WMM_COF.c \
../Core/Src/button.c \
../Core/Src/control_loop.c \
../Core/Src/exti.c \
../Core/Src/gps.c \
../Core/Src/leds.c \
../Core/Src/low_power_idle.c \
../Core/Src/mag.c \
../Core/Src/main.c \
../Core/Src/stepper.c \
../Core/Src/stm32l4xx_hal_msp.c \
../Core/Src/stm32l4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32l4xx.c \
../Core/Src/timers.c \
../Core/Src/warming_up.c \
../Core/Src/wmm.c 

OBJS += \
./Core/Src/WMM_COF.o \
./Core/Src/button.o \
./Core/Src/control_loop.o \
./Core/Src/exti.o \
./Core/Src/gps.o \
./Core/Src/leds.o \
./Core/Src/low_power_idle.o \
./Core/Src/mag.o \
./Core/Src/main.o \
./Core/Src/stepper.o \
./Core/Src/stm32l4xx_hal_msp.o \
./Core/Src/stm32l4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32l4xx.o \
./Core/Src/timers.o \
./Core/Src/warming_up.o \
./Core/Src/wmm.o 

C_DEPS += \
./Core/Src/WMM_COF.d \
./Core/Src/button.d \
./Core/Src/control_loop.d \
./Core/Src/exti.d \
./Core/Src/gps.d \
./Core/Src/leds.d \
./Core/Src/low_power_idle.d \
./Core/Src/mag.d \
./Core/Src/main.d \
./Core/Src/stepper.d \
./Core/Src/stm32l4xx_hal_msp.d \
./Core/Src/stm32l4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32l4xx.d \
./Core/Src/timers.d \
./Core/Src/warming_up.d \
./Core/Src/wmm.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/WMM_COF.cyclo ./Core/Src/WMM_COF.d ./Core/Src/WMM_COF.o ./Core/Src/WMM_COF.su ./Core/Src/button.cyclo ./Core/Src/button.d ./Core/Src/button.o ./Core/Src/button.su ./Core/Src/control_loop.cyclo ./Core/Src/control_loop.d ./Core/Src/control_loop.o ./Core/Src/control_loop.su ./Core/Src/exti.cyclo ./Core/Src/exti.d ./Core/Src/exti.o ./Core/Src/exti.su ./Core/Src/gps.cyclo ./Core/Src/gps.d ./Core/Src/gps.o ./Core/Src/gps.su ./Core/Src/leds.cyclo ./Core/Src/leds.d ./Core/Src/leds.o ./Core/Src/leds.su ./Core/Src/low_power_idle.cyclo ./Core/Src/low_power_idle.d ./Core/Src/low_power_idle.o ./Core/Src/low_power_idle.su ./Core/Src/mag.cyclo ./Core/Src/mag.d ./Core/Src/mag.o ./Core/Src/mag.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stepper.cyclo ./Core/Src/stepper.d ./Core/Src/stepper.o ./Core/Src/stepper.su ./Core/Src/stm32l4xx_hal_msp.cyclo ./Core/Src/stm32l4xx_hal_msp.d ./Core/Src/stm32l4xx_hal_msp.o ./Core/Src/stm32l4xx_hal_msp.su ./Core/Src/stm32l4xx_it.cyclo ./Core/Src/stm32l4xx_it.d ./Core/Src/stm32l4xx_it.o ./Core/Src/stm32l4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32l4xx.cyclo ./Core/Src/system_stm32l4xx.d ./Core/Src/system_stm32l4xx.o ./Core/Src/system_stm32l4xx.su ./Core/Src/timers.cyclo ./Core/Src/timers.d ./Core/Src/timers.o ./Core/Src/timers.su ./Core/Src/warming_up.cyclo ./Core/Src/warming_up.d ./Core/Src/warming_up.o ./Core/Src/warming_up.su ./Core/Src/wmm.cyclo ./Core/Src/wmm.d ./Core/Src/wmm.o ./Core/Src/wmm.su

.PHONY: clean-Core-2f-Src


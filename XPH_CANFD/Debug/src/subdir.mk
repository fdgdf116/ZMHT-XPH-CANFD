################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
LD_SRCS += \
../src/lscript.ld 

C_SRCS += \
../src/axi_canfd_drv.c \
../src/axis_fifo_drv.c \
../src/canfd_sys.c \
../src/helloworld.c \
../src/platform.c \
../src/ringbuffer.c 

OBJS += \
./src/axi_canfd_drv.o \
./src/axis_fifo_drv.o \
./src/canfd_sys.o \
./src/helloworld.o \
./src/platform.o \
./src/ringbuffer.o 

C_DEPS += \
./src/axi_canfd_drv.d \
./src/axis_fifo_drv.d \
./src/canfd_sys.d \
./src/helloworld.d \
./src/platform.d \
./src/ringbuffer.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MicroBlaze gcc compiler'
	mb-gcc -Wall -O0 -g3 -c -fmessage-length=0 -MT"$@" -I../../XPH_CANFD_bsp/micro_microblaze_0/include -mlittle-endian -mxl-barrel-shift -mxl-pattern-compare -mno-xl-soft-div -mcpu=v11.0 -mno-xl-soft-mul -mxl-multiply-high -mhard-float -mxl-float-convert -mxl-float-sqrt -Wl,--no-relax -ffunction-sections -fdata-sections -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



cmd_src/drivers/src/pca9685.o := arm-none-eabi-gcc -Wp,-MD,src/drivers/src/.pca9685.o.d    -I/home/kiko/Code/crazyflie-firmware/src/drivers/src -Isrc/drivers/src -D__firmware__ -fno-exceptions -Wall -Wmissing-braces -fno-strict-aliasing -ffunction-sections -fdata-sections -Wdouble-promotion -std=gnu11 -DCRAZYFLIE_FW   -I/home/kiko/Code/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include   -I/home/kiko/Code/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include   -I/home/kiko/Code/crazyflie-firmware/vendor/libdw1000/inc   -I/home/kiko/Code/crazyflie-firmware/vendor/FreeRTOS/include   -I/home/kiko/Code/crazyflie-firmware/vendor/FreeRTOS/portable/GCC/ARM_CM4F   -I/home/kiko/Code/crazyflie-firmware/src/config   -I/home/kiko/Code/crazyflie-firmware/src/platform/interface   -I/home/kiko/Code/crazyflie-firmware/src/deck/interface   -I/home/kiko/Code/crazyflie-firmware/src/deck/drivers/interface   -I/home/kiko/Code/crazyflie-firmware/src/drivers/interface   -I/home/kiko/Code/crazyflie-firmware/src/drivers/bosch/interface   -I/home/kiko/Code/crazyflie-firmware/src/drivers/esp32/interface   -I/home/kiko/Code/crazyflie-firmware/src/hal/interface   -I/home/kiko/Code/crazyflie-firmware/src/modules/interface   -I/home/kiko/Code/crazyflie-firmware/src/modules/interface/kalman_core   -I/home/kiko/Code/crazyflie-firmware/src/modules/interface/lighthouse   -I/home/kiko/Code/crazyflie-firmware/src/modules/interface/outlierfilter   -I/home/kiko/Code/crazyflie-firmware/src/modules/interface/cpx   -I/home/kiko/Code/crazyflie-firmware/src/modules/interface/p2pDTR   -I/home/kiko/Code/crazyflie-firmware/src/modules/interface/controller   -I/home/kiko/Code/crazyflie-firmware/src/modules/interface/estimator   -I/home/kiko/Code/crazyflie-firmware/src/utils/interface   -I/home/kiko/Code/crazyflie-firmware/src/utils/interface/kve   -I/home/kiko/Code/crazyflie-firmware/src/utils/interface/lighthouse   -I/home/kiko/Code/crazyflie-firmware/src/utils/interface/tdoa   -I/home/kiko/Code/crazyflie-firmware/src/lib/FatFS   -I/home/kiko/Code/crazyflie-firmware/src/lib/CMSIS/STM32F4xx/Include   -I/home/kiko/Code/crazyflie-firmware/src/lib/STM32_USB_Device_Library/Core/inc   -I/home/kiko/Code/crazyflie-firmware/src/lib/STM32_USB_OTG_Driver/inc   -I/home/kiko/Code/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc   -I/home/kiko/Code/crazyflie-firmware/src/lib/vl53l1   -I/home/kiko/Code/crazyflie-firmware/src/lib/vl53l1/core/inc   -I/home/kiko/Code/crazyflie-firmware/examples/multi_robot_system/build/include/generated -fno-delete-null-pointer-checks -Wno-unused-but-set-variable -Wno-unused-const-variable -fomit-frame-pointer -fno-var-tracking-assignments -Wno-pointer-sign -fno-strict-overflow -fconserve-stack -Werror=implicit-int -Werror=date-time -DCC_HAVE_ASM_GOTO -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3 -fno-math-errno -DARM_MATH_CM4 -D__FPU_PRESENT=1 -mfp16-format=ieee -Wno-array-bounds -Wno-stringop-overread -Wno-stringop-overflow -DSTM32F4XX -DSTM32F40_41xxx -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER -Os -Werror   -c -o src/drivers/src/pca9685.o /home/kiko/Code/crazyflie-firmware/src/drivers/src/pca9685.c

source_src/drivers/src/pca9685.o := /home/kiko/Code/crazyflie-firmware/src/drivers/src/pca9685.c

deps_src/drivers/src/pca9685.o := \
  /home/kiko/Code/crazyflie-firmware/src/drivers/interface/i2cdev.h \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include/stdint.h \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include/stdbool.h \
  /home/kiko/Code/crazyflie-firmware/src/drivers/interface/i2c_drv.h \
  /home/kiko/Code/crazyflie-firmware/vendor/FreeRTOS/include/FreeRTOS.h \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include/stddef.h \
  /home/kiko/Code/crazyflie-firmware/src/config/FreeRTOSConfig.h \
    $(wildcard include/config/h.h) \
    $(wildcard include/config/debug/queue/monitor.h) \
  /home/kiko/Code/crazyflie-firmware/src/config/config.h \
    $(wildcard include/config/h/.h) \
    $(wildcard include/config/block/address.h) \
  /home/kiko/Code/crazyflie-firmware/src/drivers/interface/nrf24l01.h \
  /home/kiko/Code/crazyflie-firmware/src/drivers/interface/nRF24L01reg.h \
  /home/kiko/Code/crazyflie-firmware/src/config/trace.h \
  /home/kiko/Code/crazyflie-firmware/src/hal/interface/usec_time.h \
  /home/kiko/Code/crazyflie-firmware/src/utils/interface/cfassert.h \
  /home/kiko/Code/crazyflie-firmware/vendor/FreeRTOS/include/projdefs.h \
  /home/kiko/Code/crazyflie-firmware/vendor/FreeRTOS/include/portable.h \
  /home/kiko/Code/crazyflie-firmware/vendor/FreeRTOS/include/deprecated_definitions.h \
  /home/kiko/Code/crazyflie-firmware/vendor/FreeRTOS/portable/GCC/ARM_CM4F/portmacro.h \
  /home/kiko/Code/crazyflie-firmware/vendor/FreeRTOS/include/mpu_wrappers.h \
  /home/kiko/Code/crazyflie-firmware/vendor/FreeRTOS/include/semphr.h \
  /home/kiko/Code/crazyflie-firmware/vendor/FreeRTOS/include/queue.h \
  /home/kiko/Code/crazyflie-firmware/vendor/FreeRTOS/include/task.h \
  /home/kiko/Code/crazyflie-firmware/vendor/FreeRTOS/include/list.h \
  /home/kiko/Code/crazyflie-firmware/vendor/FreeRTOS/include/queue.h \
  /home/kiko/Code/crazyflie-firmware/src/config/stm32fxxx.h \
  /home/kiko/Code/crazyflie-firmware/src/lib/CMSIS/STM32F4xx/Include/stm32f4xx.h \
  /home/kiko/Code/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/core_cm4.h \
  /home/kiko/Code/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/cmsis_version.h \
  /home/kiko/Code/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/cmsis_compiler.h \
  /home/kiko/Code/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/cmsis_gcc.h \
  /home/kiko/Code/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/mpu_armv7.h \
  /home/kiko/Code/crazyflie-firmware/src/lib/CMSIS/STM32F4xx/Include/system_stm32f4xx.h \
  /home/kiko/Code/crazyflie-firmware/src/config/stm32f4xx_conf.h \
  /home/kiko/Code/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_adc.h \
  /home/kiko/Code/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_crc.h \
  /home/kiko/Code/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_dbgmcu.h \
  /home/kiko/Code/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_dma.h \
    $(wildcard include/config/it.h) \
  /home/kiko/Code/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_exti.h \
  /home/kiko/Code/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_flash.h \
  /home/kiko/Code/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_gpio.h \
  /home/kiko/Code/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_i2c.h \
  /home/kiko/Code/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_iwdg.h \
  /home/kiko/Code/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_pwr.h \
  /home/kiko/Code/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_rcc.h \
  /home/kiko/Code/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_rtc.h \
  /home/kiko/Code/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_sdio.h \
  /home/kiko/Code/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_spi.h \
  /home/kiko/Code/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_syscfg.h \
  /home/kiko/Code/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_tim.h \
  /home/kiko/Code/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_usart.h \
  /home/kiko/Code/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_wwdg.h \
  /home/kiko/Code/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_misc.h \
  /home/kiko/Code/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_cryp.h \
  /home/kiko/Code/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_hash.h \
  /home/kiko/Code/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_rng.h \
  /home/kiko/Code/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_can.h \
  /home/kiko/Code/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_dac.h \
  /home/kiko/Code/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_dcmi.h \
  /home/kiko/Code/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_fsmc.h \
  /home/kiko/Code/crazyflie-firmware/src/drivers/interface/pca9685.h \
  /usr/include/newlib/math.h \
  /usr/include/newlib/sys/reent.h \
  /usr/include/newlib/_ansi.h \
  /usr/include/newlib/newlib.h \
  /usr/include/newlib/_newlib_version.h \
  /usr/include/newlib/sys/config.h \
    $(wildcard include/config/h//.h) \
  /usr/include/newlib/machine/ieeefp.h \
  /usr/include/newlib/sys/features.h \
  /usr/include/newlib/sys/_types.h \
  /usr/include/newlib/machine/_types.h \
  /usr/include/newlib/machine/_default_types.h \
  /usr/include/newlib/sys/lock.h \
  /usr/include/newlib/sys/cdefs.h \
  /usr/include/newlib/_ansi.h \
  /home/kiko/Code/crazyflie-firmware/vendor/FreeRTOS/include/task.h \
  /home/kiko/Code/crazyflie-firmware/src/config/config.h \
  /home/kiko/Code/crazyflie-firmware/src/modules/interface/static_mem.h \

src/drivers/src/pca9685.o: $(deps_src/drivers/src/pca9685.o)

$(deps_src/drivers/src/pca9685.o):
cmd_src/modules/src/crtp_commander_high_level.o := arm-none-eabi-gcc -Wp,-MD,src/modules/src/.crtp_commander_high_level.o.d    -I/home/kiko/Code/crazyflie-firmware/src/modules/src -Isrc/modules/src -D__firmware__ -fno-exceptions -Wall -Wmissing-braces -fno-strict-aliasing -ffunction-sections -fdata-sections -Wdouble-promotion -std=gnu11 -DCRAZYFLIE_FW   -I/home/kiko/Code/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include   -I/home/kiko/Code/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include   -I/home/kiko/Code/crazyflie-firmware/vendor/libdw1000/inc   -I/home/kiko/Code/crazyflie-firmware/vendor/FreeRTOS/include   -I/home/kiko/Code/crazyflie-firmware/vendor/FreeRTOS/portable/GCC/ARM_CM4F   -I/home/kiko/Code/crazyflie-firmware/src/config   -I/home/kiko/Code/crazyflie-firmware/src/platform/interface   -I/home/kiko/Code/crazyflie-firmware/src/deck/interface   -I/home/kiko/Code/crazyflie-firmware/src/deck/drivers/interface   -I/home/kiko/Code/crazyflie-firmware/src/drivers/interface   -I/home/kiko/Code/crazyflie-firmware/src/drivers/bosch/interface   -I/home/kiko/Code/crazyflie-firmware/src/drivers/esp32/interface   -I/home/kiko/Code/crazyflie-firmware/src/hal/interface   -I/home/kiko/Code/crazyflie-firmware/src/modules/interface   -I/home/kiko/Code/crazyflie-firmware/src/modules/interface/kalman_core   -I/home/kiko/Code/crazyflie-firmware/src/modules/interface/lighthouse   -I/home/kiko/Code/crazyflie-firmware/src/modules/interface/outlierfilter   -I/home/kiko/Code/crazyflie-firmware/src/modules/interface/cpx   -I/home/kiko/Code/crazyflie-firmware/src/modules/interface/p2pDTR   -I/home/kiko/Code/crazyflie-firmware/src/modules/interface/controller   -I/home/kiko/Code/crazyflie-firmware/src/modules/interface/estimator   -I/home/kiko/Code/crazyflie-firmware/src/utils/interface   -I/home/kiko/Code/crazyflie-firmware/src/utils/interface/kve   -I/home/kiko/Code/crazyflie-firmware/src/utils/interface/lighthouse   -I/home/kiko/Code/crazyflie-firmware/src/utils/interface/tdoa   -I/home/kiko/Code/crazyflie-firmware/src/lib/FatFS   -I/home/kiko/Code/crazyflie-firmware/src/lib/CMSIS/STM32F4xx/Include   -I/home/kiko/Code/crazyflie-firmware/src/lib/STM32_USB_Device_Library/Core/inc   -I/home/kiko/Code/crazyflie-firmware/src/lib/STM32_USB_OTG_Driver/inc   -I/home/kiko/Code/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc   -I/home/kiko/Code/crazyflie-firmware/src/lib/vl53l1   -I/home/kiko/Code/crazyflie-firmware/src/lib/vl53l1/core/inc   -I/home/kiko/Code/crazyflie-firmware/examples/multi_robot_system/build/include/generated -fno-delete-null-pointer-checks -Wno-unused-but-set-variable -Wno-unused-const-variable -fomit-frame-pointer -fno-var-tracking-assignments -Wno-pointer-sign -fno-strict-overflow -fconserve-stack -Werror=implicit-int -Werror=date-time -DCC_HAVE_ASM_GOTO -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3 -fno-math-errno -DARM_MATH_CM4 -D__FPU_PRESENT=1 -mfp16-format=ieee -Wno-array-bounds -Wno-stringop-overread -Wno-stringop-overflow -DSTM32F4XX -DSTM32F40_41xxx -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER -Os -Werror   -c -o src/modules/src/crtp_commander_high_level.o /home/kiko/Code/crazyflie-firmware/src/modules/src/crtp_commander_high_level.c

source_src/modules/src/crtp_commander_high_level.o := /home/kiko/Code/crazyflie-firmware/src/modules/src/crtp_commander_high_level.c

deps_src/modules/src/crtp_commander_high_level.o := \
  /usr/include/newlib/string.h \
  /usr/include/newlib/_ansi.h \
  /usr/include/newlib/newlib.h \
  /usr/include/newlib/_newlib_version.h \
  /usr/include/newlib/sys/config.h \
    $(wildcard include/config/h//.h) \
  /usr/include/newlib/machine/ieeefp.h \
  /usr/include/newlib/sys/features.h \
  /usr/include/newlib/sys/reent.h \
  /usr/include/newlib/_ansi.h \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include/stddef.h \
  /usr/include/newlib/sys/_types.h \
  /usr/include/newlib/machine/_types.h \
  /usr/include/newlib/machine/_default_types.h \
  /usr/include/newlib/sys/lock.h \
  /usr/include/newlib/sys/cdefs.h \
  /usr/include/newlib/sys/_locale.h \
  /usr/include/newlib/strings.h \
  /usr/include/newlib/sys/string.h \
  /usr/include/newlib/errno.h \
  /usr/include/newlib/sys/errno.h \
  /usr/include/newlib/math.h \
  /home/kiko/Code/crazyflie-firmware/vendor/FreeRTOS/include/FreeRTOS.h \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include/stdint.h \
  /home/kiko/Code/crazyflie-firmware/src/config/FreeRTOSConfig.h \
    $(wildcard include/config/h.h) \
    $(wildcard include/config/debug/queue/monitor.h) \
  /home/kiko/Code/crazyflie-firmware/src/config/config.h \
    $(wildcard include/config/h/.h) \
    $(wildcard include/config/block/address.h) \
  /home/kiko/Code/crazyflie-firmware/src/drivers/interface/nrf24l01.h \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include/stdbool.h \
  /home/kiko/Code/crazyflie-firmware/src/drivers/interface/nRF24L01reg.h \
  /home/kiko/Code/crazyflie-firmware/src/config/trace.h \
  /home/kiko/Code/crazyflie-firmware/src/hal/interface/usec_time.h \
  /home/kiko/Code/crazyflie-firmware/src/utils/interface/cfassert.h \
  /home/kiko/Code/crazyflie-firmware/vendor/FreeRTOS/include/projdefs.h \
  /home/kiko/Code/crazyflie-firmware/vendor/FreeRTOS/include/portable.h \
  /home/kiko/Code/crazyflie-firmware/vendor/FreeRTOS/include/deprecated_definitions.h \
  /home/kiko/Code/crazyflie-firmware/vendor/FreeRTOS/portable/GCC/ARM_CM4F/portmacro.h \
  /home/kiko/Code/crazyflie-firmware/vendor/FreeRTOS/include/mpu_wrappers.h \
  /home/kiko/Code/crazyflie-firmware/vendor/FreeRTOS/include/task.h \
  /home/kiko/Code/crazyflie-firmware/vendor/FreeRTOS/include/list.h \
  /home/kiko/Code/crazyflie-firmware/vendor/FreeRTOS/include/semphr.h \
  /home/kiko/Code/crazyflie-firmware/vendor/FreeRTOS/include/queue.h \
  /home/kiko/Code/crazyflie-firmware/vendor/FreeRTOS/include/task.h \
  /home/kiko/Code/crazyflie-firmware/src/modules/interface/crtp.h \
  /home/kiko/Code/crazyflie-firmware/src/modules/interface/crtp_commander_high_level.h \
  /home/kiko/Code/crazyflie-firmware/src/modules/interface/math3d.h \
  /home/kiko/Code/crazyflie-firmware/src/modules/interface/stabilizer_types.h \
  /home/kiko/Code/crazyflie-firmware/src/hal/interface/imu_types.h \
  /home/kiko/Code/crazyflie-firmware/src/utils/interface/lighthouse/lighthouse_types.h \
  /home/kiko/Code/crazyflie-firmware/src/modules/interface/planner.h \
  /home/kiko/Code/crazyflie-firmware/src/modules/interface/pptraj.h \
  /home/kiko/Code/crazyflie-firmware/src/modules/interface/pptraj_compressed.h \
  /usr/include/newlib/stdio.h \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include/stdarg.h \
  /usr/include/newlib/sys/types.h \
  /usr/include/newlib/sys/_stdint.h \
  /usr/include/newlib/machine/endian.h \
  /usr/include/newlib/machine/_endian.h \
  /usr/include/newlib/sys/select.h \
  /usr/include/newlib/sys/_sigset.h \
  /usr/include/newlib/sys/_timeval.h \
  /usr/include/newlib/sys/timespec.h \
  /usr/include/newlib/sys/_timespec.h \
  /usr/include/newlib/sys/_pthreadtypes.h \
  /usr/include/newlib/sys/sched.h \
  /usr/include/newlib/machine/types.h \
  /usr/include/newlib/sys/stdio.h \
  /home/kiko/Code/crazyflie-firmware/src/modules/interface/log.h \
    $(wildcard include/config/debug/log/enable.h) \
  /home/kiko/Code/crazyflie-firmware/src/modules/interface/param.h \
  /home/kiko/Code/crazyflie-firmware/src/modules/interface/param_logic.h \
  /home/kiko/Code/crazyflie-firmware/src/modules/interface/static_mem.h \
  /home/kiko/Code/crazyflie-firmware/src/modules/interface/mem.h \
  /home/kiko/Code/crazyflie-firmware/src/modules/interface/commander.h \
  /home/kiko/Code/crazyflie-firmware/src/config/config.h \
  /home/kiko/Code/crazyflie-firmware/src/modules/interface/stabilizer_types.h \
  /home/kiko/Code/crazyflie-firmware/src/modules/interface/stabilizer.h \
  /home/kiko/Code/crazyflie-firmware/src/modules/interface/estimator/estimator.h \
    $(wildcard include/config/estimator/kalman/enable.h) \
    $(wildcard include/config/estimator/ukf/enable.h) \
    $(wildcard include/config/estimator/oot.h) \
  /home/kiko/Code/crazyflie-firmware/src/modules/interface/controller/controller.h \
    $(wildcard include/config/controller/oot.h) \
  /home/kiko/Code/crazyflie-firmware/src/modules/interface/app.h \

src/modules/src/crtp_commander_high_level.o: $(deps_src/modules/src/crtp_commander_high_level.o)

$(deps_src/modules/src/crtp_commander_high_level.o):

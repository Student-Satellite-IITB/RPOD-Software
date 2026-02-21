# platform/stm32/nucleo_h7a3ziq/board.cmake
set(STM32_STARTUP_ASM   "startup_stm32h7a3xxq.s")
set(STM32_LINKER_SCRIPT "STM32H7A3XX_FLASH.ld")

set(STM32_CPU_FLAGS
    -mcpu=cortex-m7
    -mthumb
    -mfpu=fpv5-d16
    -mfloat-abi=hard
)
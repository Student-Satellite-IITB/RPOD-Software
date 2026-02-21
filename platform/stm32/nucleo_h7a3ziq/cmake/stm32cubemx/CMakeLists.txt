cmake_minimum_required(VERSION 3.22)

# ---- patch_cubemx.sh (manual patch; rerun after CubeMX regen)
get_filename_component(CUBEMX_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
# ---- end patch

# Enable CMake support for ASM and C languages
enable_language(C ASM)
# STM32CubeMX generated symbols (macros)
set(MX_Defines_Syms 
	USE_PWR_DIRECT_SMPS_SUPPLY 
	USE_NUCLEO_64 
	USE_HAL_DRIVER 
	STM32H7A3xxQ 
	STM32_THREAD_SAFE_STRATEGY=2
    $<$<CONFIG:Debug>:DEBUG>
)

# STM32CubeMX generated include paths
set(MX_Include_Dirs
    ${CUBEMX_ROOT}/Core/Inc
    ${CUBEMX_ROOT}/Drivers/STM32H7xx_HAL_Driver/Inc
    ${CUBEMX_ROOT}/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
    ${CUBEMX_ROOT}/Drivers/BSP/STM32H7xx_Nucleo
    ${CUBEMX_ROOT}/Drivers/CMSIS/Device/ST/STM32H7xx/Include
    ${CUBEMX_ROOT}/Drivers/CMSIS/Include
)

# STM32CubeMX generated application sources
set(MX_Application_Src
    ${CUBEMX_ROOT}/Core/Src/main.c
    ${CUBEMX_ROOT}/Core/Src/stm32h7xx_it.c
    ${CUBEMX_ROOT}/Core/Src/stm32h7xx_hal_msp.c
    ${CUBEMX_ROOT}/Core/Src/stm32h7xx_hal_timebase_tim.c
    ${CUBEMX_ROOT}/Core/Src/sysmem.c
    ${CUBEMX_ROOT}/Core/Src/syscalls.c
    ${CUBEMX_ROOT}/startup_stm32h7a3xxq.s
)

# STM32 HAL/LL Drivers
set(STM32_Drivers_Src
    ${CUBEMX_ROOT}/Core/Src/system_stm32h7xx.c
    ${CUBEMX_ROOT}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim.c
    ${CUBEMX_ROOT}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c
    ${CUBEMX_ROOT}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c
    ${CUBEMX_ROOT}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c
    ${CUBEMX_ROOT}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c
    ${CUBEMX_ROOT}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c
    ${CUBEMX_ROOT}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c
    ${CUBEMX_ROOT}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c
    ${CUBEMX_ROOT}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_hsem.c
    ${CUBEMX_ROOT}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c
    ${CUBEMX_ROOT}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c
    ${CUBEMX_ROOT}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_mdma.c
    ${CUBEMX_ROOT}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c
    ${CUBEMX_ROOT}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c
    ${CUBEMX_ROOT}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c
    ${CUBEMX_ROOT}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c.c
    ${CUBEMX_ROOT}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c_ex.c
    ${CUBEMX_ROOT}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_exti.c
    ${CUBEMX_ROOT}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_usart.c
    ${CUBEMX_ROOT}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_usart_ex.c
    ${CUBEMX_ROOT}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart_ex.c
    ${CUBEMX_ROOT}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.c
    ${CUBEMX_ROOT}/Drivers/BSP/STM32H7xx_Nucleo/stm32h7xx_nucleo.c
)

# Drivers Midllewares



# Link directories setup
set(MX_LINK_DIRS

)
# Project static libraries
set(MX_LINK_LIBS 
    STM32_Drivers
    
)
# Interface library for includes and symbols
add_library(stm32cubemx INTERFACE)
target_include_directories(stm32cubemx INTERFACE ${MX_Include_Dirs})
target_compile_definitions(stm32cubemx INTERFACE ${MX_Defines_Syms})

# Create STM32_Drivers static library
add_library(STM32_Drivers OBJECT)
target_sources(STM32_Drivers PRIVATE ${STM32_Drivers_Src})
target_link_libraries(STM32_Drivers PUBLIC stm32cubemx)


# Add STM32CubeMX generated application sources to the project
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${MX_Application_Src})

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE ${MX_LINK_DIRS})

# Add libraries to the project
target_link_libraries(${CMAKE_PROJECT_NAME} ${MX_LINK_LIBS})

# Add the map file to the list of files to be removed with 'clean' target
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES ADDITIONAL_CLEAN_FILES ${CMAKE_PROJECT_NAME}.map)

# Validate that STM32CubeMX code is compatible with C standard
if((CMAKE_C_STANDARD EQUAL 90) OR (CMAKE_C_STANDARD EQUAL 99))
    message(ERROR "Generated code requires C11 or higher")
endif()

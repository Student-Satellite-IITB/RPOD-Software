# cmake/stm32-platform.cmake
# STM32 build: CubeMX stm32cubemx + repo-owned entry + rcu_main composition

set(RPOD_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include)
set(EIGEN_DIR        ${CMAKE_SOURCE_DIR}/lib/eigen)

if(NOT DEFINED STM32_BOARD)
    message(FATAL_ERROR "STM32_BOARD not set (e.g. -DSTM32_BOARD=nucleo_h7a3ziq)")
endif()

set(STM32_CUBEMX_DIR ${CMAKE_SOURCE_DIR}/platform/stm32/${STM32_BOARD})
set(BOARD_CMAKE       ${STM32_CUBEMX_DIR}/board.cmake)
set(CUBEMX_CORE_INC   ${STM32_CUBEMX_DIR}/Core/Inc)

function(rpod_require_file path what)
  if(NOT EXISTS "${path}")
    message(FATAL_ERROR "${what} not found: ${path}")
  endif()
endfunction()


# ---- Patch CubeMX (optional toggle) ----
option(STM32_APPLY_CUBEMX_PATCH "Apply patch_cubemx.sh at configure time" ON)
if(STM32_APPLY_CUBEMX_PATCH)
  execute_process(
    COMMAND bash ${CMAKE_SOURCE_DIR}/platform/stm32/patch_cubemx.sh ${STM32_CUBEMX_DIR}
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    RESULT_VARIABLE PATCH_RES
  )
  if(NOT PATCH_RES EQUAL 0)
    message(FATAL_ERROR "CubeMX patch failed for ${STM32_CUBEMX_DIR}")
  endif()
endif()


# ---- Board config ----
rpod_require_file("${BOARD_CMAKE}" "board.cmake")
include("${BOARD_CMAKE}")

# board.cmake must define:
# - STM32_STARTUP_ASM   (e.g. "startup_stm32h7a3xxq.s")
# - STM32_LINKER_SCRIPT (e.g. "STM32H7A3XX_FLASH.ld")
# - STM32_CPU_FLAGS     (list of flags)

rpod_require_file("${STM32_CUBEMX_DIR}/${STM32_STARTUP_ASM}" "Startup ASM")
rpod_require_file("${STM32_CUBEMX_DIR}/${STM32_LINKER_SCRIPT}" "Linker script")


# ---- Firmware target name ----
set(FW_TARGET ${STM32_BOARD})
# IMPORTANT: stm32cubemx uses ${CMAKE_PROJECT_NAME} when attaching sources/libs
set(CMAKE_PROJECT_NAME ${FW_TARGET})
# Create the executable that stm32cubemx will populate
add_executable(${FW_TARGET})

# ---- Pull in CubeMX stm32cubemx ----
add_subdirectory(
  ${STM32_CUBEMX_DIR}/cmake/stm32cubemx
  ${CMAKE_BINARY_DIR}/stm32cubemx_build
)

# ---- MCU flags applied to CubeMX compilation ----
target_compile_options(stm32cubemx INTERFACE
  ${STM32_CPU_FLAGS}
  -Wall -Wextra -Wpedantic
  -fdata-sections -ffunction-sections
  $<$<CONFIG:Debug>:-O0;-g3>
  $<$<CONFIG:Release>:-Os;-g0>
  $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti;-fno-exceptions;-fno-threadsafe-statics>
)

# Startup asm preprocessing
set_source_files_properties(
  "${STM32_CUBEMX_DIR}/${STM32_STARTUP_ASM}"
  PROPERTIES COMPILE_OPTIONS "-x;assembler-with-cpp"
)

# ---- Link flags ----
target_link_options(${FW_TARGET} PRIVATE
  ${STM32_CPU_FLAGS}
  -T "${STM32_CUBEMX_DIR}/${STM32_LINKER_SCRIPT}"
  --specs=nano.specs
  -Wl,-Map=${FW_TARGET}.map
  -Wl,--gc-sections
  -Wl,--print-memory-usage
  -Wl,--start-group -lc -lm -Wl,--end-group
)


# ---- Repo-owned entry + RCU composition ----
add_library(rpod_rcu STATIC
  ${CMAKE_SOURCE_DIR}/src/platform/stm32/entry.cpp
  ${CMAKE_SOURCE_DIR}/src/system/rcu/rcu_main.cpp
  ${CMAKE_SOURCE_DIR}/src/apps/vbn/RNAVFilter.cpp
  # Temporary baremetal RTOS implementation
  ${CMAKE_SOURCE_DIR}/src/os/baremetal/rtos.cpp
  # ${CMAKE_SOURCE_DIR}/src/os/freertos/rtos.cpp
)

target_compile_options(rpod_rcu PRIVATE
  ${STM32_CPU_FLAGS}
  -Wall -Wextra -Wpedantic
  -fdata-sections -ffunction-sections
  $<$<CONFIG:Debug>:-O0;-g3>
  $<$<CONFIG:Release>:-Os;-g0>
  $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti;-fno-exceptions;-fno-threadsafe-statics>
)

target_include_directories(rpod_rcu PUBLIC
  ${RPOD_INCLUDE_DIR}
  ${EIGEN_DIR}
  ${CUBEMX_CORE_INC}
)

target_link_libraries(rpod_rcu PUBLIC stm32cubemx)

## Firmware links CubeMX + RCU lib (plain signature)
target_link_libraries(${FW_TARGET} stm32cubemx rpod_rcu)

# Optional: Can be used to add ifdefs in the code for conditional compilation based on target platform
# target_compile_definitions(rpod_rcu PUBLIC TARGET_STM32=1)
# STM32 Deployment

This directory contains STM32 platform integration for RPOD-Software.

We use STM32CubeMX **only** to generate HAL + startup/linker artifacts (and later middleware like FreeRTOS).
All builds are driven from the repo CMake (not CubeIDE).

## Directory Layout

```bash
platform/stm32/
|
├── README.md
├── nucleo_h7a3ziq
│   ├── CMakeLists.txt
│   ├── CMakePresets.json
│   ├── Core
│   │   ├── Inc
│   │   └── Src
│   ├── Drivers
│   │   ├── BSP
│   │   ├── CMSIS
│   │   └── STM32H7xx_HAL_Driver
│   ├── STM32H7A3XX_FLASH.ld # Linker script (board-specific)
│   ├── board.cmake # Board metadata used by repo CMake (manual file)
│   ├── cmake
│   │   ├── gcc-arm-none-eabi.cmake
│   │   └── stm32cubemx # CubeMX-generated CMake for sources/defines/includes
│   ├── newlib_lock_glue.c
│   ├── nucleo_h7a3ziq.ioc
│   ├── startup_stm32h7a3xxq.s # Startup assembly (board-specific)
│   └── stm32_lock.h
└── patch_cubemx.sh # Patch script to be applied after CubeMX regen

```

### Boards currently supported
- `nucleo_h7a3ziq` (STM32H7A3ZIQ Nucleo)

## CubeMX workflow

1) Create / edit the CubeMX project on Windows inside:
   `platform/stm32/<board>/`

2) Use **Generate Code** in CubeMX.

3) Immediately after every CubeMX regen, run the patch:

```bash
./platform/stm32/patch_cubemx.sh platform/stm32/<board>
```
Example:

```bash
./platform/stm32/patch_cubemx.sh platform/stm32/nucleo_h7a3ziq
```

This fixes CubeMX-generated cmake/stm32cubemx/CMakeLists.txt so it works even when the CubeMX project is nested inside a higher level application repository.

### Editing policy

- Do not edit files under platform/stm32/<board>/ manually.
- Exceptions:
    - board.cmake (repo-owned)
    - Core/Src/main.c USER CODE blocks only (for calling RPOD entry hooks)
- Any change to CubeMX CMake must happen through patch_cubemx.sh (so regen is safe).
---

## Board metadata: `board.cmake`

Each board directory contains a small repo-owned file `board.cmake` used by `cmake/stm32-platform.cmake`. It must define:

```cmake
# Example board.cmake
set(STM32_STARTUP_ASM   "startup_stm32h7a3xxq.s")
set(STM32_LINKER_SCRIPT "STM32H7A3XX_FLASH.ld")

set(STM32_CPU_FLAGS
  -mcpu=cortex-m7
  -mthumb
  -mfpu=fpv5-d16
  -mfloat-abi=hard
)
```

This keeps board-specific filenames out of the top-level build logic.

## Entry hooks from CubeMX `main.c`

CubeMX owns `main()` (`Core/Src/main.c`). We call repo-owned hooks from USER CODE blocks:

- `rcu_init()` once after peripheral init and COM init
- `rcu_loop()` inside the main while-loop
- (optional) `rcu_start()` if/when we move to an RTOS start pattern


These are declared in:
- `include/platform/stm32/entry.h`

and implemented by the repo in:

- `src/platform/stm32/entry.cpp`
- `src/system/rcu_main.cpp` (RCU application logic)

## Building (STM32)

```bash
cmake -S . -B build/stm32-debug -G Ninja \
  -DTARGET_PLATFORM=stm32 \
  -DSTM32_BOARD=nucleo_h7a3ziq \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_TOOLCHAIN_FILE=cmake/stm32-toolchain.cmake

cmake --build build/stm32-debug
```
Output ELF: `build/stm32-debug/nucleo_h7a3ziq.elf`\
Flash using STM32CubeProgrammer (Windows) via ST-LINK.
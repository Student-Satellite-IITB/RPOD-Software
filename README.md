# RPOD-Software
This repository contains the flight software for RPOD module. It is a work in progress!
## Instructions

### Installation
```bash
git clone https://github.com/Student-Satellite-IITB/RPOD-Software.git
cd RPOD-Software
```
### Building the code
```bash
mkdir build
cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/linux-toolchain.cmake
cmake --build . --target <MyExecutable>
```
This creates the executable. \
Ensure there is file called simulated-image.png in test/ folder \
Now run the executable
```bash
./<MyExecutable>
```
Check the annotated image in the test directory.

## STM32-BUILD

```bash
cmake -S . -B build/stm32-debug -G Ninja \
  -DTARGET_PLATFORM=stm32 \
  -DSTM32_BOARD=nucleo_h7a3ziq \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_TOOLCHAIN_FILE=cmake/stm32-toolchain.cmake
```
```bash
cmake --build build/stm32-debug
```
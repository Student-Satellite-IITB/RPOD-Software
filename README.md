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
make
```
This creates the executable. \
Ensure there is file called simulated-image.png in test/ folder \
Now run the executable
```bash
./rpod_test_pipeline
```
Check the annotated image in the test directory.
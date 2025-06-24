cmake_minimum_required(VERSION 3.10)
project(RPOD_FeaturePipeline)

# Enable compile_commands.json generation for VS Code
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set C++ version
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


if(NOT DEFINED TARGET_PLATFORM)
    set(TARGET_PLATFORM "linux")
endif()

if(TARGET_PLATFORM STREQUAL "linux")
    find_package(OpenCV REQUIRED)
endif()

# Include directories
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/include
)

# ==== Source Files ====
# Platform-specific sources
if(TARGET_PLATFORM STREQUAL "linux")
    set(PLATFORM_SOURCES
        src/platform/linux/FeatureExtractor.cpp
        src/os/posix/rtos.cpp
    )
elseif(TARGET_PLATFORM STREQUAL "stm32")
    set(PLATFORM_SOURCES
        src/platform/stm32/FeatureExtractor.cpp
        src/os/freertos/rtos.cpp
    )
endif()

# Common sources
set(SOURCES
    src/main.cpp
    src/vbn/FeatureDetector.cpp
    ${PLATFORM_SOURCES}
)

# ==== Executables ====
# Main application executable
add_executable(rpod_test_pipeline ${SOURCES})
# Add test executables
add_executable(vbn_featuredetection_test 
    test/vbn_featuredetection_test.cpp 
    src/vbn/FeatureDetector.cpp
    src/platform/linux/FeatureExtractor.cpp
    src/os/posix/rtos.cpp)
add_executable(rtos_task_test test/rtos_task_test.cpp src/os/posix/rtos.cpp)
add_executable(rtos_mutex_test test/rtos_mutex_test.cpp src/os/posix/rtos.cpp)

# ==== Link Libraries ====
# Platform-specific linking
if(TARGET_PLATFORM STREQUAL "linux")
    target_link_libraries(rpod_test_pipeline ${OpenCV_LIBS} pthread)
    # Link VBN test executable
    target_link_libraries(vbn_featuredetection_test ${OpenCV_LIBS} pthread)
    # Link RTOS test executables
    target_link_libraries(rtos_task_test pthread)
    target_link_libraries(rtos_mutex_test pthread)

endif()
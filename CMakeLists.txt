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

option(WITH_OPENCV "Build components requiring OpenCV" ON)

if(TARGET_PLATFORM STREQUAL "linux" AND WITH_OPENCV)
    find_package(OpenCV REQUIRED)
    include_directories(${OpenCV_INCLUDE_DIRS})
endif()

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/lib/eigen
)

# ==== Source Files ====
# Platform-specific sources
if(TARGET_PLATFORM STREQUAL "linux")
    file(GLOB PLATFORM_SOURCES
        src/platform/linux/*.cpp
        src/os/posix/rtos.cpp
    )
elseif(TARGET_PLATFORM STREQUAL "stm32")
    file(GLOB PLATFORM_SOURCES
        src/platform/stm32/*.cpp
        src/os/freertos/rtos.cpp
    )
endif()

# Common sources
file(GLOB_RECURSE SOURCES
    # src/main.cpp
    src/apps/*.cpp
    ${PLATFORM_SOURCES}
)

# ==== Executables ====
# format: add_executable(<name> <main source> <source files>)

# Only build these if OpenCV is enabled
if(TARGET_PLATFORM STREQUAL "linux")

    if(WITH_OPENCV)
        # Main application test executables
        add_executable(rpod_test_pipeline src/main.cpp ${SOURCES})
        add_executable(rpod_som src/som_main.cpp ${SOURCES})

        # Add test executables
        add_executable(vbn_batch_runner test/vbn_batch_runner.cpp ${SOURCES})

        add_executable(vbn_featuredetection_test test/vbn_featuredetection_test.cpp ${SOURCES})
        add_executable(vbn_staticposeestimation_test test/vbn_staticposeestimation_test.cpp ${SOURCES})

        # VBN pipeline test with Ground Monitor
        add_executable(vbn_pipeline_test test/vbn_pipeline_test.cpp tools/groundmonitor/GroundMonitor.cpp ${SOURCES})
        target_link_libraries(vbn_pipeline_test ${OpenCV_LIBS} pthread)
        target_include_directories(vbn_pipeline_test PRIVATE ${CMAKE_SOURCE_DIR})

    endif()
    
    add_executable(vbn_imagecapturebuild_test test/vbn_imagecapturebuild_test.cpp ${SOURCES})
    add_executable(vbn_imagecapture_test test/vbn_imagecapture_test.cpp ${SOURCES})

    # RTOS test executables
    add_executable(rtos_task_test test/rtos_task_test.cpp src/os/posix/rtos.cpp)
    add_executable(rtos_mutex_test test/rtos_mutex_test.cpp src/os/posix/rtos.cpp)
    add_executable(rtos_semaphore_test test/rtos_semaphore_test.cpp src/os/posix/rtos.cpp)
    add_executable(rtos_countingsem_test test/rtos_countingsem_test.cpp src/os/posix/rtos.cpp)
    add_executable(rtos_queue_test test/rtos_queue_test.cpp src/os/posix/rtos.cpp)

endif()

# ==== Link Libraries ====
# Platform-specific linking
if(TARGET_PLATFORM STREQUAL "linux")

    if(WITH_OPENCV)
        target_link_libraries(rpod_test_pipeline ${OpenCV_LIBS} pthread)
        target_link_libraries(rpod_som ${OpenCV_LIBS} pthread)
        # Link VBN test executable
        target_link_libraries(vbn_batch_runner ${OpenCV_LIBS} pthread)
        target_link_libraries(vbn_featuredetection_test ${OpenCV_LIBS} pthread)
        target_link_libraries(vbn_staticposeestimation_test ${OpenCV_LIBS} pthread)
    endif()

    target_link_libraries(vbn_imagecapturebuild_test pthread)
    target_link_libraries(vbn_imagecapture_test pthread)

    # Link RTOS test executables
    target_link_libraries(rtos_task_test pthread)
    target_link_libraries(rtos_mutex_test pthread)
    target_link_libraries(rtos_semaphore_test pthread)
    target_link_libraries(rtos_countingsem_test pthread)
    target_link_libraries(rtos_queue_test pthread)

endif()
# cmake/linux-platform.cmake
# Linux-native build: RPOD core + linux platform + test executables

option(WITH_OPENCV "Build components requiring OpenCV (linux only)" ON)

set(RPOD_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include)
set(EIGEN_DIR        ${CMAKE_SOURCE_DIR}/lib/eigen)

# OpenCV only when enabled
if(WITH_OPENCV)
    find_package(OpenCV REQUIRED)
endif()

# A single place for include dirs + pthread + any common compile flags
add_library(rpod_linux_defaults INTERFACE)
target_include_directories(rpod_linux_defaults INTERFACE
  ${RPOD_INCLUDE_DIR}
  ${EIGEN_DIR}
)
target_link_libraries(rpod_linux_defaults INTERFACE pthread)

# ---------------- Libraries ----------------
file(GLOB_RECURSE MODULE_SOURCES
  ${CMAKE_SOURCE_DIR}/src/apps/*.cpp
)

add_library(rpod_modules STATIC ${MODULE_SOURCES})
target_link_libraries(rpod_modules PUBLIC rpod_linux_defaults)

file(GLOB PLATFORM_LINUX_SOURCES
  ${CMAKE_SOURCE_DIR}/src/platform/linux/*.cpp
  ${CMAKE_SOURCE_DIR}/src/os/posix/rtos.cpp
)

add_library(rpod_platform_linux STATIC ${PLATFORM_LINUX_SOURCES})
target_link_libraries(rpod_platform_linux PUBLIC rpod_linux_defaults)

# ---------------- Executables ----------------
if(WITH_OPENCV)
    add_executable(rpod_test_pipeline ${CMAKE_SOURCE_DIR}/src/main.cpp)
    target_link_libraries(rpod_test_pipeline rpod_linux_defaults rpod_modules rpod_platform_linux ${OpenCV_LIBS})

    add_executable(vbn_batch_runner ${CMAKE_SOURCE_DIR}/test/vbn_batch_runner.cpp)
    target_link_libraries(vbn_batch_runner  rpod_linux_defaults rpod_modules rpod_platform_linux ${OpenCV_LIBS})

    add_executable(vbn_featuredetection_test ${CMAKE_SOURCE_DIR}/test/vbn_featuredetection_test.cpp)
    target_link_libraries(vbn_featuredetection_test rpod_linux_defaults rpod_modules rpod_platform_linux ${OpenCV_LIBS})

    add_executable(vbn_staticposeestimation_test ${CMAKE_SOURCE_DIR}/test/vbn_staticposeestimation_test.cpp)
    target_link_libraries(vbn_staticposeestimation_test rpod_linux_defaults rpod_modules rpod_platform_linux ${OpenCV_LIBS})

    add_executable(vbn_pipeline_test
        ${CMAKE_SOURCE_DIR}/test/vbn_pipeline_test.cpp
        ${CMAKE_SOURCE_DIR}/tools/groundmonitor/GroundMonitor.cpp
    )
    target_include_directories(vbn_pipeline_test PRIVATE ${CMAKE_SOURCE_DIR})
    target_link_libraries(vbn_pipeline_test rpod_linux_defaults rpod_modules rpod_platform_linux ${OpenCV_LIBS})
    

    add_executable(rnav_pipeline_test
        ${CMAKE_SOURCE_DIR}/test/rnav_pipeline_test.cpp
        ${CMAKE_SOURCE_DIR}/tools/groundmonitor/GroundMonitor.cpp
    )
    target_include_directories(rnav_pipeline_test PRIVATE ${CMAKE_SOURCE_DIR})
    target_link_libraries(rnav_pipeline_test rpod_linux_defaults rpod_modules rpod_platform_linux ${OpenCV_LIBS})
    

    add_executable(vbn_ledcharacterisation_test
        ${CMAKE_SOURCE_DIR}/test/vbn_ledcharacterisation_test.cpp
        ${CMAKE_SOURCE_DIR}/tools/groundmonitor/GroundMonitor.cpp
    )
    target_include_directories(vbn_ledcharacterisation_test PRIVATE ${CMAKE_SOURCE_DIR})
    target_link_libraries(vbn_ledcharacterisation_test rpod_linux_defaults rpod_modules rpod_platform_linux ${OpenCV_LIBS})

endif()

add_executable(vbn_imagecapturebuild_test ${CMAKE_SOURCE_DIR}/test/vbn_imagecapturebuild_test.cpp)
target_link_libraries(vbn_imagecapturebuild_test rpod_linux_defaults rpod_modules rpod_platform_linux)

add_executable(vbn_imagecapture_test ${CMAKE_SOURCE_DIR}/test/vbn_imagecapture_test.cpp)
target_link_libraries(vbn_imagecapture_test rpod_linux_defaults rpod_modules rpod_platform_linux)

add_executable(rnav_rnavfilter_test ${CMAKE_SOURCE_DIR}/test/rnav_rnavfilter_test.cpp)
target_link_libraries(rnav_rnavfilter_test rpod_linux_defaults rpod_modules rpod_platform_linux)

# RTOS unit tests (posix)


add_executable(rtos_task_test 
    ${CMAKE_SOURCE_DIR}/test/rtos_task_test.cpp 
    ${CMAKE_SOURCE_DIR}/src/os/posix/rtos.cpp
)
target_link_libraries(rtos_task_test rpod_linux_defaults)

add_executable(rtos_mutex_test 
    ${CMAKE_SOURCE_DIR}/test/rtos_mutex_test.cpp 
    ${CMAKE_SOURCE_DIR}/src/os/posix/rtos.cpp
)
target_link_libraries(rtos_mutex_test rpod_linux_defaults)

add_executable(rtos_semaphore_test
    ${CMAKE_SOURCE_DIR}/test/rtos_semaphore_test.cpp
    ${CMAKE_SOURCE_DIR}/src/os/posix/rtos.cpp
)
target_link_libraries(rtos_semaphore_test rpod_linux_defaults)

add_executable(rtos_countingsem_test
    ${CMAKE_SOURCE_DIR}/test/rtos_countingsem_test.cpp
    ${CMAKE_SOURCE_DIR}/src/os/posix/rtos.cpp
)
target_link_libraries(rtos_countingsem_test rpod_linux_defaults)

add_executable(rtos_queue_test
    ${CMAKE_SOURCE_DIR}/test/rtos_queue_test.cpp
    ${CMAKE_SOURCE_DIR}/src/os/posix/rtos.cpp
)
target_link_libraries(rtos_queue_test rpod_linux_defaults)
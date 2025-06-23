cmake_minimum_required(VERSION 3.10)
project(RPOD_FeaturePipeline)

# Enable compile_commands.json generation for VS Code
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set C++ version
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/include
)

# Define source files
set(SOURCES
    src/main.cpp
    src/vbn/FeatureDetector.cpp
    src/platform/linux/FeatureExtractor.cpp
)

# Create the executable
add_executable(rpod_test_pipeline ${SOURCES})

# Link OpenCV libraries
target_link_libraries(rpod_test_pipeline ${OpenCV_LIBS})

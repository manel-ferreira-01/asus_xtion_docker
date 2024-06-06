cmake_minimum_required(VERSION 3.10)
project(OpenNI2_OpenCV)

# Set the path to OpenNI2
set(OPENNI2_DIR "" CACHE PATH "Path to the OpenNI2 directory")

if(NOT OPENNI2_DIR)
    message(FATAL_ERROR "Please set the OPENNI2_DIR variable to the path to your OpenNI2 installation.")
endif()

# Include directories
include_directories(${OPENNI2_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Link directories
link_directories(${OPENNI2_DIR}/lib)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Add executable
add_executable(OpenNI2_OpenCV src/main.cpp src/pointcloudviz.cpp)

# Link libraries
target_link_libraries(OpenNI2_OpenCV ${OpenCV_LIBS} OpenNI2)

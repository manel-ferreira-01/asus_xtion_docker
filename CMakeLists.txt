cmake_minimum_required(VERSION 3.10)
project(OpenNI2_OpenCV)

# Set the path to OpenNI2
#set(OPENNI2_DIR "${CMAKE_CURRENT_SOURCE_DIR}/libs")
#set(OPENNI2_DIR "/home/manuelf/Downloads/OpenNI-Linux-x64-2.2/Include")
set(OPENNI2_DIR "/usr/include/openni2")


# print openni2_dir
message(STATUS "OPENNI2_DIR: ${OPENNI2_DIR}")

if(NOT OPENNI2_DIR)
    message(FATAL_ERROR "Please set the OPENNI2_DIR variable to the path to your OpenNI2 installation.")
endif()

# Include directories
include_directories(${OPENNI2_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(/usr/include/openni2)

# Link directories
#link_directories(/home/manuelf/Downloads/OpenNI-Linux-x64-2.2/Redist)

# Find OpenCV
find_package(OpenCV REQUIRED)

# add pcl library
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# Add executable
add_executable(OpenNI2_OpenCV src/main.cpp src/pointcloudviz.cpp)

# Link libraries
target_link_libraries(OpenNI2_OpenCV ${OpenCV_LIBS}
     ${PCL_LIBRARIES}
     OpenNI2)

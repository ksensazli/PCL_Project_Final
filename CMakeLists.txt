cmake_minimum_required(VERSION 3.24)
project(PCL_Project_v1)

set(CMAKE_CXX_STANDARD 11)

find_package(PCL 1.10 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(PCL_Project_v1 main.cpp CommonProcesses.cpp CommonProcesses.h Segmentation.cpp Segmentation.h Region_Growing.cpp Region_Growing.h RANSAC.cpp RANSAC.h)
target_link_libraries (PCL_Project_v1 ${PCL_LIBRARIES})

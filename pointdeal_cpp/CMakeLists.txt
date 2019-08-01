cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pointdeal_cpp)

find_package(PCL 1.7 )
add_definitions(-std=c++11)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (mesh_sampling mesh_sampling.cpp)
target_link_libraries (mesh_sampling ${PCL_LIBRARIES})

add_executable (normalization normalization.cpp)
target_link_libraries (normalization ${PCL_LIBRARIES})

add_executable (resampling resampling.cpp)
target_link_libraries (resampling ${PCL_LIBRARIES})
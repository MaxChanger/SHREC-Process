cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cloud_icp)

find_package(PCL 1.7 )
add_definitions(-std=c++11)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (cloud_icp cloud_icp.cpp)
target_link_libraries (cloud_icp ${PCL_LIBRARIES})

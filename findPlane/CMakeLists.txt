cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cloud_findPlane)

find_package(PCL 1.7 )
add_definitions(-std=c++11)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (cloud_findPlane cloud_findPlane.cpp)
target_link_libraries (cloud_findPlane ${PCL_LIBRARIES})

add_executable (cloud_findPlane2 cloud_findPlane2.cpp)
target_link_libraries (cloud_findPlane2 ${PCL_LIBRARIES})

add_executable (viewRT viewRT.cpp)
target_link_libraries (viewRT ${PCL_LIBRARIES})

add_executable (render render.cpp)
target_link_libraries (render ${PCL_LIBRARIES})

add_executable (viewManyRender viewManyRender.cpp)
target_link_libraries (viewManyRender ${PCL_LIBRARIES})

add_executable (viewOne viewOne.cpp)
target_link_libraries (viewOne ${PCL_LIBRARIES})

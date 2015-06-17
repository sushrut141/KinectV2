cmake_minimum_required(VERSION 2.8)

project(PCD)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LINK_DIRS})

add_executable(pcd_viewer pcd_viewer.cpp)
target_link_libraries(pcd_viewer ${PCL_LIBRARIES})

cmake_minimum_required(VERSION 2.8)

project(plyviewer)

find_package(PCL 1.7)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})


add_executable(plyviewer plyparse.cpp)
target_link_libraries(plyviewer ${PCL_LIBRARIES})

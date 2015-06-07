cmake_minimum_required(VERSION 2.8)

project(PCViewer)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(PCV_DIR ${PCViewer_SOURCE_DIR})

find_package(PCL 1.7)
if(PCL_FOUND)
	link_directories(${PCL_LIBRARY_DIRS})
	add_definitions(${PCL_DEFINITIONS})
	include_directories(${PCL_INCLUDE_DIRS})
else(PCL_FOUND)
	message("PointCloud Lib Found")
endif(PCL_FOUND)
	

find_package(OpenCV 2)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(freenect2)
include_directories(${freenect2_INCLUDE_DIRS})
link_directories(${freeect2_LIBRARY_DIRS})

include_directories("${PCV_DIR}/include")



set(SOURCES
	${PCV_DIR}/include/PCViewer/ImgLoader.h
	${PCV_DIR}/include/PCViewer/PCViewer.h
	${PCV_DIR}/src/ImgLoader.cpp
	${PCV_DIR}/src/PCViewer.cpp)

set(LINK
	boost_system
	${PCL_LIBS}
	${OpenCV_LIBS})
	
add_library(pclviewer SHARED ${SOURCES})
target_link_libraries(pclviewer ${LINK})

install(TARGETS pclviewer DESTINATION "${PCV_DIR}/lib")
	


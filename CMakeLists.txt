cmake_minimum_required(VERSION 2.8)

project(CALIB)

set(IMGL ${CALIB_SOURCE_DIR})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(VTK REQUIRED)
find_package(PCL 1.7 REQUIRED COMPONENTS io common visualization)
if(PCL_FOUND)
	include_directories(${PCL_INCLUDE_DIRS})
	link_directories(${PCL_LIBRARY_DIRS})
	add_definitions(${PCL_DEFINITIONS})
else(PCL_FOUND)
	message("PCL not found")
endif(PCL_FOUND)


find_package(freenect2)

if(freenect2_FOUND)
	include_directories(${freenect2_INCLUDE_DIRS})
	link_directories(${freenectt2_LIBRARY_DIRS})
endif(freenect2_FOUND)


find_package(OpenCV 2)
include_directories(${OpenCV_INCLUDE_DIRECTORIES})

#PCViewer
include_directories("${IMGL}/PCViewer/include")
link_directories("${IMGL}/PCViewer/lib/")

#kinect2Grabber
include_directories("${IMGL}/Kinect2Grabber/include")
link_directories("${IMGL}/Kinect2Grabber/lib/")

set(PCVL "${IMGL}/PCViewer/lib/libpclviewer.so")
set(GRABBER "${IMGL}/Kinect2Grabber/lib/libkinectgrabber.so")


include_directories("${IMGL}/Kinect2Grabber/libusb/include/libusb-1.0/")
link_directories("${GRABBER}/Kinect2Grabber/libusb/lib/")



set(LINK
	boost_system
	${PCVL}
	${GRABBER}
	${freenect2_LIBRARIES}
	usb-1.0
	turbojpeg
	${PCL_LIBS}
	pcl_visualization
	pcl_io
	pcl_common
	${VTK_LIBRARIES}
	${OpenCV_LIBS})

add_executable(calib main.cpp)
target_link_libraries(calib ${LINK})

install(TARGETS calib DESTINATION "${IMGL}/bin")



	

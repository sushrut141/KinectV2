cmake_minimum_required(VERSION 2.8)

project(grabber)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(GRABBER ${grabber_SOURCE_DIR})

find_package(freenect2)

if(freenect2_FOUND)
	include_directories(${freenect2_INCLUDE_DIRS})
	link_directories(${freenectt2_LIBRARY_DIRS})
endif(freenect2_FOUND)

find_package(OpenCV 2)
include_directories(${OpenCV_INCLUDE_DIRS})

find_library(KINGRAB NAME kinectgrabber PATH "${GRABBER}/Kinect2Grabber/lib/")
if(KINGRAB)
	message("Found kinect grabber")
endif(KINGRAB)

find_library(USB NAME usb PATH "${GRABBER}/Kinect2Grabber/lib/")
if(USB)
	message("Found patched USB")
endif(USB)

include_directories("${GRABBER}/Kinect2Grabber/include")
link_directories("${GRABBER}/Kinect2Grabber/bin")

include_directories("${GRABBER}/libusb/include/libusb-1.0/")
link_directories("${GRABBER}/Kinect2Grabber/libusb/lib/")

set(LINK
	${KINGRAB}
	${freenect2_LIBRARIES}
	usb-1.0
	turbojpeg
	${OpenCV_LIBRARIES}
	)

add_executable(grabber grabber.cpp)
target_link_libraries(grabber ${LINK})

install(TARGETS grabber DESTINATION "${GRABBER}/bin")

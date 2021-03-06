cmake_minimum_required(VERSION 2.8)

project(grabber)

set(GRABBER ${CMAKE_CURRENT_SOURCE_DIR})

message(${GRABBER})

find_package(freenect2)

if(freenect2_FOUND)
	include_directories(${freenect2_INCLUDE_DIRS})
	link_directories(${freenect2_LIBRARY_DIRS})
endif(freenect2_FOUND)

find_package(OpenCV 2)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(PCL 1.7)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LINK_DIRS})

find_package(OpenMP)
include_directories(${OpenMP_INCLUDE_DIRS})
link_directories(${OpenMP_LINK_DIRS})


include_directories("${GRABBER}/Kinect2Grabber/include")
link_directories("${GRABBER}/Kinect2Grabber/lib")

include_directories("${GRABBER}/libusb/include/libusb-1.0/")
link_directories("${GRABBER}/Kinect2Grabber/libusb/lib/")

set(LINK
	kinectgrabber
	${PCL_LIBRARIES}
	${freenect2_LIBRARIES}
	usb-1.0
	turbojpeg
	${OpenCV_LIBRARIES}
	${OpenMP_LIBRARIES}
	)

add_executable(grabber grabber.cpp)
target_link_libraries(grabber ${LINK})

add_executable(ortho orthoplane.cpp)
target_link_libraries(ortho ${LINK})

add_executable(transform transform_cloud.cpp)
target_link_libraries(transform ${LINK})

install(TARGETS grabber DESTINATION "${GRABBER}/bin")

cmake_minimum_required(VERSION 2.8)

project(Kinect2Grabber)

set(MY_DIR ${Kinect2Grabber_SOURCE_DIR})

find_package(freenect2)
if(freenect2_FOUND)
	include_directories(${freenect2_INCLUDE_DIRS})
	message("${freenect2_INCLUDE_DIRS}")
	link_directories(${freenect2_LIBRARY_DIRS})
	message("${freenect2_LIBRARY_DIRS}")
else(freenect2_FOUND)
	message(" libfreenect was not found")
endif(freenect2_FOUND)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LINK_DIRS})

include_directories("${MY_DIR}/libusb/include/libusb-1.0/")
link_directories("${MY_DIR}/libusb/lib/")


include_directories("${MY_DIR}/include")

set(FREENECT2 "/usr/local/lib/libfreenect2.so")


set(SOURCES 
	"${MY_DIR}/src/Kinect2Grabber.cpp"
	${MY_DIR}/include/Kinect2Grabber/Kinect2Grabber.h
)

set(LINK
	usb-1.0
	turbojpeg
	${FREENECT2}
	${PCL_LIBRARIES}
	${OpenCV_LIB}
)

add_Library(kinectgrabber SHARED ${SOURCES})
target_link_libraries(kinectgrabber ${LINK})

install(TARGETS kinectgrabber DESTINATION "${MY_DIR}/lib")






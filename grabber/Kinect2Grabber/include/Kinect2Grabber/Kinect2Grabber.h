#ifndef KINECT_GRABBER_H_
#define KINECT_GRABBER_H_

#include <string>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <opencv2/opencv.hpp>
#include <iostream>




namespace kinect
{
	struct FrameData
	{
		cv::Mat color_;
		cv::Mat IR_;
		cv::Mat depth_;

	};


	class Kinect2Grabber
	{
	public:
		
		Kinect2Grabber();

		~Kinect2Grabber();

		FrameData&  getFrameData();

	private:

		std::string deviceSerial_;
		libfreenect2::Freenect2 freenect2_;

		libfreenect2::SyncMultiFrameListener* listener;
		libfreenect2::Freenect2Device* dev_;
		libfreenect2::PacketPipeline* pipeline_; 
		libfreenect2::FrameMap frames_;

		FrameData data_;
	};
}

#endif
#include <Kinect2Grabber/Kinect2Grabber.h>



namespace kinect
{

Kinect2Grabber::Kinect2Grabber() : registered(new unsigned char[512*424*3]),cloud_(new pcl::PointCloud<pcl::PointXYZRGB>)
{
	
	if(freenect2_.enumerateDevices()==0)
	{
		std::cout<<"No devices connected"<<std::endl;
		exit(EXIT_FAILURE);
	}
	deviceSerial_ = freenect2_.getDefaultDeviceSerialNumber();

	std::cout<<"Found device with [DEVICE SERIAL] : "<<deviceSerial_<<std::endl;
	
	//difference between pipeline methods...vs without pipeline?
	std::cout<<"Opening device interface..."<<std::endl;

	pipeline_ = new libfreenect2::OpenCLPacketPipeline();

	dev_ = freenect2_.openDevice(deviceSerial_,pipeline_);


	if(dev_!=NULL)
		std::cout<<"Device object created... "<<std::endl;

	listener = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Depth | libfreenect2::Frame::Ir);
	
	dev_->setColorFrameListener(listener);
	dev_->setIrAndDepthFrameListener(listener);

	dev_->start();


	registration = new libfreenect2::Registration(dev_->getIrCameraParams(), dev_->getColorCameraParams());

	ir_camera_ = dev_->getIrCameraParams();

	std::cout<<"Camera centre : "<<ir_camera_.cx<<" "<<ir_camera_.cy<<std::endl;
	std::cout<<"Focal parameters : "<<ir_camera_.fx<<" "<<ir_camera_.fy<<std::endl;

}

Kinect2Grabber::~Kinect2Grabber()
{

	listener->release(frames_);

	dev_->stop();
	dev_->close();

	delete registration;

	delete[] registered;

	//delete frame listener object
	delete listener;

	
	std::cout<<"Kinect Grabber object destroyed"<<std::endl;
}


void Kinect2Grabber::captureFrameData() 
{

	listener->waitForNewFrame(frames_);

	std::cout<<"Requesting Frame data"<<std::endl;

	libfreenect2::Frame *rgb = frames_[libfreenect2::Frame::Color];
	libfreenect2::Frame *depth = frames_[libfreenect2::Frame::Depth];
	libfreenect2::Frame *ir = frames_[libfreenect2::Frame::Ir];

	data_.color_ = cv::Mat(rgb->height,rgb->width,CV_8UC3,rgb->data);
	data_.depth_ = cv::Mat(depth->height,depth->width,CV_32FC1,depth->data)/4500;
	data_.IR_	 = cv::Mat(ir->height,ir->width,CV_32FC1,ir->data)/20000.0f;

}


void Kinect2Grabber::captureRegisteredImage()
{
	listener->waitForNewFrame(frames_);

	std::cout<<"Requesting Frame data"<<std::endl;

	rgb = frames_[libfreenect2::Frame::Color];
	depth = frames_[libfreenect2::Frame::Depth];
	ir = frames_[libfreenect2::Frame::Ir];

	//std::cout<<"depth value is "<<(float)depth->data[2]<<std::endl;

	// if (!registered) registered = new unsigned char[depth->height*depth->width*rgb->bytes_per_pixel];

    registration->apply(rgb,depth,registered);

    data_.color_ = cv::Mat(depth->height,depth->width,CV_8UC3,registered);
    data_.depth_ = cv::Mat(depth->height,depth->width,CV_32FC1,depth->data)/4500;
    data_.IR_	 = cv::Mat(ir->height,ir->width,CV_32FC1,ir->data)/20000.0f;

}


const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& Kinect2Grabber::getPointCloud()
{
	captureRegisteredImage();

	int size_r = data_.color_.rows;
	int size_c = data_.color_.cols;
	int index = 0;
	
	cloud_->width = size_c;
	cloud_->height = size_r;

	cloud_->points.resize(size_r*size_c);

	#pragma omp parallel for
	for(int i=0;i<size_r;i++)
	{
		for(int j=0;j<size_c;j++,index++)
		{
			float depth_pos  = data_.depth_.at<float>(i,j);
			float pos_x = (float)((j-ir_camera_.cx)*depth_pos)/ir_camera_.fx;
			float pos_y = (float)((i-ir_camera_.cy)*depth_pos)/ir_camera_.fy;
	
			cloud_->points[index].x = pos_x;
			cloud_->points[index].y = pos_y;
			cloud_->points[index].z = depth_pos;
	
			const cv::Vec3b tmp = data_.color_.at<cv::Vec3b>(i,j);

			cloud_->points[index].b = tmp.val[0];
			cloud_->points[index].g = tmp.val[1];
			cloud_->points[index].r = tmp.val[2];

		}
	}

	return cloud_;

}


const FrameData& Kinect2Grabber::getFrameData()
{
	captureFrameData();

	return data_;
}


const FrameData& Kinect2Grabber::getRegisteredImage()
{
	captureRegisteredImage();

	return data_;
}

const libfreenect2::Freenect2Device::IrCameraParams& Kinect2Grabber::getIrParams()
{
	return ir_camera_;
}


}
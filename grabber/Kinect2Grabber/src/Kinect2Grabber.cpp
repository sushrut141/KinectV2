#include <Kinect2Grabber/Kinect2Grabber.h>



namespace kinect
{

Kinect2Grabber::Kinect2Grabber() : registered(new unsigned char[512*424*3])
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


const FrameData& Kinect2Grabber::getFrameData() 
{

	listener->waitForNewFrame(frames_);

	std::cout<<"Requesting Frame data"<<std::endl;

	libfreenect2::Frame *rgb = frames_[libfreenect2::Frame::Color];
	libfreenect2::Frame *depth = frames_[libfreenect2::Frame::Depth];
	libfreenect2::Frame *ir = frames_[libfreenect2::Frame::Ir];

	data_.color_ = cv::Mat(rgb->height,rgb->width,CV_8UC3,rgb->data);
	data_.depth_ = cv::Mat(depth->height,depth->width,CV_32FC1,depth->data)/4500;
	data_.IR_	 = cv::Mat(ir->height,ir->width,CV_32FC1,ir->data)/20000.0f;

	return data_;
}


const FrameData& Kinect2Grabber::getRegisteredImage()
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

    return data_;

}



const libfreenect2::Freenect2Device::IrCameraParams& Kinect2Grabber::getIrParams()
{
	return ir_camera_;
}


}
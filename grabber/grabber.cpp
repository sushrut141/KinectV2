#include <Kinect2Grabber/Kinect2Grabber.h>
#include <signal.h>

bool stop = false;

void sigint_handler(int s)
{
  stop = true;
}

int main(int argc,char**argv)
{
	signal(SIGINT,sigint_handler);

	kinect::Kinect2Grabber obj;

	kinect::FrameData data;

	cv::namedWindow( "RGB",CV_WINDOW_AUTOSIZE );
	cv::namedWindow( "Depth",CV_WINDOW_AUTOSIZE );
	cv::namedWindow( "Ir",CV_WINDOW_AUTOSIZE );
	
	while(!stop)
	{
		data = obj.getFrameData();
		cv::imshow("RGB",data.color_);
		cv::imshow("Depth",data.depth_);
		cv::imshow("Ir",data.IR_);
		cv::waitKey(1);
	}

	return 0;

}
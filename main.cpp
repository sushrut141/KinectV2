/*
This example shows how to use the Kinect2Grabber class to get image data and Point Cloud 
using The Kinect V2
*/


#include <Kinect2Grabber/Kinect2Grabber.h>
#include <iostream>

#define COLOR_DATA 1
#define REGISTERED_DATA 0
#define SHOW_CLOUD 0

typedef pcl::visualization::PCLVisualizer Viewer;

bool running = true;

void sigint_handle(int s)
{
	running = false;
}


void showImage(const kinect::FrameData& data)
{
	cv::imshow("color",data.color_);
	cv::imshow("depth",data.depth_);
	cv::imshow("IR",data.IR_);
}


int main(int argc,char**argv)
{
	signal(SIGINT,sigint_handle);

	kinect::Kinect2Grabber grabber;

	kinect::FrameData data;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

	cv::namedWindow("color",CV_WINDOW_AUTOSIZE);
	cv::namedWindow("depth",CV_WINDOW_AUTOSIZE);
	cv::namedWindow("IR",CV_WINDOW_AUTOSIZE);


	//Initialize PCL Visualizer to view Point Cloud
	boost::shared_ptr<Viewer> viewer(new Viewer("kinect Cloud"));
	viewer->addPointCloud<PointT>(cloud_,"Kinect Cloud");
	viewer->setBackgroundColor (0, 0, 0);
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3, "Kinect Cloud");
	viewer->initCameraParameters ();



	while(running)
	{
		#ifdef COLOR_DATA
		//get Color,Ir and Depth frame data in cv::Mat contatiners
		data = grabber.getFrameData();
		showImage(data);
		cv::waitKey(1000);
		#endif

		#ifdef REGISTERED_DATA
		//get the registered image,Ir and Depth frame
		data = grabber.getRegisteredImage();
		showImage(data);
		cv::waitKey(1000);
		#endif

		#ifdef SHOW_CLOUD
		cloud = grabber.getPointCloud();
		//show the point cloud
		viewer->updatePointCloud(cloud,"Kinect Cloud");
		viewer->spinOnce (100);
		#endif

	}

	return 0;



}

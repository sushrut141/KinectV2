#include <Kinect2Grabber/Kinect2Grabber.h>
#include <PCViewer/PCViewer.h>
#include <signal.h>
#include <memory>

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>

bool stop = false;
bool first = true;

void sigint_handler(int s)
{
  stop = true;
}


int main(int argc, char**argv)
{
	signal(SIGINT,sigint_handler);
	std::string rgb(argv[1]);
	std::string ir(argv[2]);

	kinect::PCViewer drawPoints(rgb,ir);
	
	kinect::Kinect2Grabber grabber;


	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

	pcl::visualization::PCLVisualizer* viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
  

	cv::namedWindow("IR",CV_WINDOW_AUTOSIZE);

	while(!stop)
	{
		kinect::FrameData data = grabber.getFrameData();

		if((!data.color_.empty()) && (!data.depth_.empty())){
		cloud = drawPoints.getPointCloud(data.color_,data.depth_);
	 	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "Kinect Cloud");

	 	if(first){
	 		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1, "Kinect Cloud");
    		viewer->addCoordinateSystem (1.0);
    		viewer->initCameraParameters ();
    		first = false;
	 	}

		//std::cout<<"points are "<<cloud->points[0].x<<std::endl<<cloud->points[0].y<<std::endl<<cloud->points[0].z<<std::endl;
		viewer->updatePointCloud (cloud, "Kinect Cloud");
    	viewer->spinOnce (1000);

		//cv::imshow("IR",data.IR_);
		//cv::waitKey(1);
		}
	}

	return 0;

}









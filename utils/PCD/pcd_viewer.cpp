#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>



int main(int argc,char**argv)
{
	std::string pcd(argv[1]);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZRGB>);

	if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcd,*cloud_)==-1)
		std::cout<<"Failed to load pcd file"<<std::endl;


	pcl::visualization::PCLVisualizer* viewer(new pcl::visualization::PCLVisualizer ("Kinect Cloud"));
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud_,rgb,"Kinect Cloud");
	viewer->setBackgroundColor (0, 0, 0);
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3, "Kinect Cloud");
	viewer->addCoordinateSystem (3.0);

	viewer->updatePointCloud(cloud_,"Kinect Cloud");
	viewer->spinOnce(100000);

	return 0;


}
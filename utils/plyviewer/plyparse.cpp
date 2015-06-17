#include <fstream>
#include <string>
#include <iostream>
#include <sstream>
#include <vector>


//PCL
#include <pcl/common/common_headers.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

//print pcd file
bool print = false;

struct Point 
{
	Point(float x,float y,float z,unsigned char r,unsigned char g,unsigned char b):
	x(x),y(y),z(z),r(r),g(g),b(b){}
	float x,y,z;
	float r,g,b;
	void print()
	{
		std::cout<<"Vertex"<<" "<<x<<" "<<y<<" "<<z<<std::endl;
		std::cout<<"color"<<" "<<r<<" "<<g<<" "<<b<<std::endl;
	}
	void nornalizeColor()
	{
		r = r/255;g = g/255;b = b/255;
	}
};

int main(int argc,char**argv)
{

	pcl::PLYWriter writer;
	if(argc!=2){
		std::cout<<"Enter path to .ply file to run"<<std::endl;
		exit(EXIT_FAILURE);
	}
	
	long long int num,vertex_num;
	std::ifstream inFile(argv[1]);

	std::vector<Point> points;

	std::string line,words;

	while(std::getline(inFile,line))
	{
		std::istringstream streamer(line);
		streamer>>words;
		if(words.compare("ply")==0)
			continue;
		if(words.compare("format")==0)
			continue;
		if(words.compare("comment")==0)
			continue;
		if(words.compare("element")==0)
		{
			streamer >> words >>num;
			if(words.compare("vertex")==0){
				vertex_num = num;
				continue;
			}
		}
		if(words.compare("end_header")==0)
			break;
	}

	std::cout<<"Number of vertices are "<<vertex_num<<std::endl;
	for(long long int i =0;i < vertex_num; i++)
	{
		float x,y,z;
		unsigned int r,g,b;
		std::getline(inFile,line);
		std::istringstream values(line);
		values >> x >> y >> z;
		values >> r >> g >> b;
		points.push_back(Point(x,y,z,r,g,b));

	}
	/*
	for(long long int i=0;i<vertex_num;i++){
		points.at(i).nornalizeColor();
		points.at(i).print();
	}
	*/

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());

	cloud->points.resize(vertex_num);

	for(long long int i=0;i<vertex_num;i++)
	{
		//vertex
		cloud->points[i].x = points[i].x;
		cloud->points[i].y = points[i].y;
		cloud->points[i].z = points[i].z;
		//color
		cloud->points[i].r = points[i].r;
		cloud->points[i].g = points[i].g;
		cloud->points[i].b = points[i].b;
	}

	//downsample
	pcl::VoxelGrid<pcl::PointXYZRGB>  sor;
  	sor.setInputCloud (cloud);
  	sor.setLeafSize (0.01f, 0.01f, 0.01f);
  	sor.filter (*cloud_filtered);

  	writer.write("../acad.ply",*cloud_filtered);

  	std::cout<<"Number of points ater filtering"<<cloud_filtered->width*cloud_filtered->height<<std::endl;

  	pcl::visualization::PCLVisualizer* viewer_f (new pcl::visualization::PCLVisualizer ("Kinect Cloud_Filter"));
  	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_f(cloud_filtered);
  	viewer_f->addPointCloud<pcl::PointXYZRGB>(cloud_filtered,rgb_f,"Kinect Cloud_Filter");
    viewer_f->setBackgroundColor (0, 0, 0);
    viewer_f->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3, "Kinect Cloud_Filter");
	viewer_f->addCoordinateSystem (1.0);
	viewer_f->initCameraParameters ();

	pcl::visualization::PCLVisualizer* viewer (new pcl::visualization::PCLVisualizer ("Kinect Cloud"));
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud,rgb,"Kinect Cloud");
    viewer->setBackgroundColor (0, 0, 0);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3, "Kinect Cloud");
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();

    viewer->updatePointCloud(cloud, "Kinect Cloud");
	viewer->spinOnce (100000);
	viewer_f->updatePointCloud(cloud_filtered, "Kinect Cloud");
	viewer_f->spinOnce (100000);

	if(print){
				cloud_->height = 1;
				cloud_->width = cloud_->points.size();
				pcl::io::savePCDFileASCII ("test_new.pcd",*cloud_);
				print = false;
			}

	delete viewer;


}
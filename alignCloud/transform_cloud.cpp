#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

typedef pcl::PointXYZRGB PointT;


float getNormals(Eigen::Vector3f& a,Eigen::Vector3f& b,Eigen::Vector3f& c,std::string path)
{
	std::ifstream file(path.c_str());
	std::string line;

	std::string words;

	float value;

	float normals[3][3];

	std::getline(file,line);
	std::istringstream streamer(line);

	streamer>>words;

	if(words.compare("VALUE")==0){
		std::string nextline;
		std::getline(file,nextline);
		std::istringstream valuestream(nextline);
		valuestream>>value;
	}
	std::string nextline;
	std::getline(file,nextline);
	std::istringstream newstream(nextline);
	newstream>>words;
	std::cout<<words<<std::endl;
	if(words.compare("NORMALS")==0)
	{
		for(int i=0;i<3;i++)
		{
			for(int j=0;j<3;j++)
			{
				std::getline(file,line);
				std::istringstream normalstream(line);

				normalstream>>normals[i][j];
			}
		}
	}

	a = Eigen::Vector3f(normals[0][0],normals[0][1],normals[0][2]);
	b = Eigen::Vector3f(normals[1][0],normals[1][1],normals[1][2]);
	c = Eigen::Vector3f(normals[2][0],normals[2][1],normals[2][2]);

	return value;

}

int main(int argc,char**argv)
{
	std::string cloud1(argv[1]);
	std::string cloud2(argv[2]);

	std::string normal1(argv[3]);
	std::string normal2(argv[4]);

	pcl::PointCloud<PointT>::Ptr cloud_1(new pcl::PointCloud<PointT>());

	if(pcl::io::loadPCDFile<PointT>(cloud1,*cloud_1)==-1)
		std::cout<<"Failed to load PCD file"<<std::endl;

	pcl::PointCloud<PointT>::Ptr cloud_2(new pcl::PointCloud<PointT>());

	if(pcl::io::loadPCDFile<PointT>(cloud2,*cloud_2)==-1)
		std::cout<<"Failed to load PCD file"<<std::endl;

	pcl::PointCloud<PointT>::Ptr cloud_sum(new pcl::PointCloud<PointT>());

	pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>());

	Eigen::Vector3f normal_11,normal_12,normal_13;
	Eigen::Vector3f normal_21,normal_22,normal_23;

	float value_1 = getNormals(normal_11,normal_12,normal_13,normal1);
	float value_2 = getNormals(normal_21,normal_22,normal_23,normal2);

	std::cout<<"Values of normals are"<<std::endl;
	std::cout<<normal_11<<std::endl;
	std::cout<<normal_12<<std::endl;
	std::cout<<normal_13<<std::endl;

	Eigen::MatrixXf rotation = Eigen::MatrixXf::Random(3,3);

	rotation.row(0) = normal_11;
	rotation.row(1) = normal_12;
	rotation.row(2) = normal_13;

	std::cout<<"Normal matrix is "<<rotation<<std::endl;

	Eigen::Vector3f out_1(normal_21[0],normal_22[0],normal_23[0]);

	Eigen::Vector3f a = rotation.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(out_1);

	Eigen::Vector3f out_2(normal_21[1],normal_22[1],normal_23[1]);

	Eigen::Vector3f b = rotation.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(out_2);

	Eigen::Vector3f out_3(normal_21[2],normal_22[2],normal_23[2]);

	Eigen::Vector3f c = rotation.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(out_3);

	Eigen::Matrix3f rotate;

	rotate.row(0) = a;
	rotate.row(1) = b;
	rotate.row(2) = c;

	std::cout<<"Rotation matrix is "<<rotate<<std::endl;


	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
	transform.block<3,3>(0,0) = rotate;

	std::cout<<"Transformation Matrix is "<<transform<<std::endl;

	pcl::transformPointCloud(*cloud_1,*transformed_cloud,transform);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("Kinect Cloud"));

	pcl::io::savePCDFileASCII ("transformed_cloud.pcd",*transformed_cloud);

	cloud_sum = transformed_cloud;
	*cloud_sum+=*cloud_2;


	/*
	int v1,v2;

	viewer->createViewPort(0.0,0.0,0.5,1.0,v1);

	viewer->addPointCloud<PointT>(cloud_2,"Kinect Cloud",v1);

	viewer->setBackgroundColor (0, 0, 0,v1);
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3, "Kinect Cloud");


	viewer->createViewPort(0.5,0.0,1.0,1.0,v2);

	viewer->addPointCloud<PointT>(transformed_cloud,"Kinect Cloud",v2);

	viewer->setBackgroundColor (0, 0, 0,v2);

	viewer->initCameraParameters ();

	//viewer->updatePointCloud(transformed_cloud,"Kinect Cloud");

	viewer->spinOnce(100000);
	*/
	
	viewer->addPointCloud<PointT>(cloud_sum,"Kinect Cloud");
	viewer->setBackgroundColor (0, 0, 0);
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3, "Kinect Cloud");

	

	viewer->updatePointCloud(cloud_sum,"Kinect Cloud");

	viewer->spinOnce(100000);
	

}
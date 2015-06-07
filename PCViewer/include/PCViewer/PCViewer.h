#ifndef POINT_CLOUD_LOADER_H_
#define POINT_CLOUD_LOADER_H_

#include <PCViewer/ImgLoader.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

namespace kinect
{

class PCViewer : public ImgLoader
{
public:

	PCViewer(std::string rgbPath,std::string irPath):ImgLoader(rgbPath,irPath)
	{
		//data_ = getFrameData();
		calibCamera();
		computeTransforms();
		//init();
		std::cout<<"Calibration and transform computation complete"<<std::endl;
	}

	~PCViewer();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr
	getPointCloud(const cv::Mat& colorImg, const cv::Mat& depthImg);



private:

	void computeTransforms();

	//void init();

	cv::Mat data_;

	Eigen::Matrix4d world2rgb_;
	Eigen::Matrix4d depth2world_;

};

}

#endif
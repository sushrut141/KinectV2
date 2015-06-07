#include <PCViewer/PCViewer.h>
#include <iostream>
#include <stdint.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


namespace kinect
{
	PCViewer::~PCViewer()
	{
		std::cout<<depth2world_<<std::endl;
		std::cout<<"PCViewer object destroyed"<<std::endl;
	}


	void PCViewer::computeTransforms()
	{
		
		Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > erotation((double*)(rotation_.data));
		Eigen::Matrix3d rotation = erotation;
		Eigen::Map<Eigen::Vector3d> etranslation ((double*)(translation_.data));
		Eigen::Vector3d translation = etranslation;
		Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > edepth_matrix((double*)(depth_camera_matrix_.data));
		Eigen::Matrix3d depth_matrix = edepth_matrix;
		Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > ergb_matrix((double*)(rgb_camera_matrix_.data));
		Eigen::Matrix3d rgb_matrix = ergb_matrix;

		Eigen::Matrix4d rototranslation = Eigen::Matrix4d::Zero();
		Eigen::Matrix4d odepth_matrix = Eigen::Matrix4d::Zero();
		Eigen::Matrix4d orgb_matrix = Eigen::Matrix4d::Zero(); 

		rototranslation.block<3,3>(0,0) = rotation;
		rototranslation.block<3,1>(0,3) = translation;
		rototranslation(3,3) = 1;

		odepth_matrix.block<3,3>(0,0) = depth_matrix;
		odepth_matrix(3,3) = 1;

		orgb_matrix.block<3,3>(0,0) = rgb_matrix;
		orgb_matrix(3,3) = 1;

		world2rgb_ = orgb_matrix * rototranslation;
		depth2world_ = odepth_matrix.inverse();
	}

	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr
	PCViewer::getPointCloud(const cv::Mat& colorImg, const cv::Mat& depthImg) 
	{
		int i;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZRGB>());

		cv::Mat depth =  depthImg;
		cv::Mat _color_ =  colorImg;

		cv::Mat color(depth.rows,depth.cols,CV_8UC3,0);

		cv::resize(_color_,color,depth.size(),cv::INTER_CUBIC);

		cloud_->width = depth.cols;
      	cloud_->height = depth.rows;

		cloud_->points.resize(depth.rows*depth.cols);

		for(int y = 0; y < depth.rows; ++y)
		{	

			const uint16_t *itD = depth.ptr<uint16_t>(y);

			for(size_t x = 0; x < (size_t)depth.cols; ++x, ++itD,++i )
			{
				const float depth_value = *itD / 1000.0f;
				// Check for invalid measurements
				if(isnan(depth_value) || *itD >= 10000)
				{
					continue;
				}

				Eigen::Vector4d psd(x, y, 1.0, 1.0/depth_value);
				Eigen::Vector4d psddiv = psd * depth_value;

				Eigen::Vector4d pworld = depth2world_ * psddiv;

				Eigen::Vector4d rgb_img_homo = world2rgb_ *  pworld;
				
				Eigen::Vector4d rgb_img = rgb_img_homo / rgb_img_homo.z();

				if(rgb_img.x() > 0 && rgb_img.x() < color.cols && rgb_img.y() > 0 && rgb_img.y() < color.rows){
					
					cloud_->points[i].z = depth_value;
					cloud_->points[i].x = pworld.x() ;
					cloud_->points[i].y = pworld.y() ;
					const cv::Vec3b tmp = color.at<cv::Vec3b>(rgb_img.y(), rgb_img.x());
					cloud_->points[i].b = tmp.val[0];
					cloud_->points[i].g = tmp.val[1];
					cloud_->points[i].r = tmp.val[2];
				}
			}
		}
		
		return cloud_;
	}
	


}
#include <Kinect2Grabber/Kinect2Grabber.h>
#include <signal.h>

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>


//Segmentation
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>



bool print = true;
bool stop = false;
bool segment = true;

void sigint_handler(int s)
{
  stop = true;
}

int key;

int main(int argc,char**argv)
{
	signal(SIGINT,sigint_handler);

	int i=0;

	kinect::Kinect2Grabber obj;

	kinect::FrameData data;

	//std::vector<int> compress;
	//compress.push_back(CV_IMWRITE_JPEG_QUALITY );
	//compress.push_back(100);
	//compress.push_back(CV_IMWRITE_PNG_COMPRESSION);
	//compress.push_back(2);

	cv::namedWindow( "registered",CV_WINDOW_AUTOSIZE );
	//cv::namedWindow( "Depth",CV_WINDOW_AUTOSIZE );
	//cv::namedWindow( "Ir",CV_WINDOW_AUTOSIZE );
	

	data = obj.getRegisteredImage();
	//data = obj.getFrameData();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZRGB>());

	cloud_->points.resize(data.depth_.rows*data.depth_.cols);

	pcl::visualization::PCLVisualizer* viewer(new pcl::visualization::PCLVisualizer ("Kinect Cloud"));
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud_,rgb,"Kinect Cloud");
	viewer->setBackgroundColor (0, 0, 0);
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1, "Kinect Cloud");
	//viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();

	//segment
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.01);

	seg.setInputCloud (cloud_);
	

	while(key!=99)
	{
			data = obj.getRegisteredImage();
			//data = obj.getFrameData();


			cv::imshow("registered",data.color_);
			//cv::imshow("Depth",data.depth_);
			//cv::imshow("Ir",data.IR_);

			key = cv::waitKey(10);

			libfreenect2::Freenect2Device::IrCameraParams params = obj.getIrParams();
	
			std::cout<<"Camera centre : "<<params.cx<<" "<<params.cy<<std::endl;
			std::cout<<"Focal parameters : "<<params.fx<<" "<<params.fy<<std::endl;
			
			int size_r = data.color_.rows;
			int size_c = data.color_.cols;
	
			int index = 0;

			#pragma omp parallel for
			for(int i=0;i<size_r;i++)
			{
				for(int j=0;j<size_c;j++,index++)
				{
					float depth_pos  = data.depth_.at<float>(i,j);
					float pos_x = (float)((j-params.cx)*depth_pos)/params.fx;
					float pos_y = (float)((i-params.cy)*depth_pos)/params.fy;
	
					cloud_->points[index].x = pos_x;
					cloud_->points[index].y = pos_y;
					cloud_->points[index].z = depth_pos;
	
					const cv::Vec3b tmp = data.color_.at<cv::Vec3b>(i,j);
	
					cloud_->points[index].b = tmp.val[0];
					cloud_->points[index].g = tmp.val[1];
					cloud_->points[index].r = tmp.val[2];
	
				}
			}


			if(segment){
	  			seg.segment (*inliers, *coefficients);
	  			for(unsigned int i=0;i<inliers->indices.size();i++)
				{
					cloud_->points[inliers->indices[i]].r = 255;

					cloud_->points[inliers->indices[i]].g = 0;

					cloud_->points[inliers->indices[i]].b = 0;
				}
			}
	
			viewer->updatePointCloud(cloud_, "Kinect Cloud");
			viewer->spinOnce (100);

			if(print&&(key==99)){
				cloud_->height = 1;
				cloud_->width = cloud_->points.size();
				pcl::io::savePCDFileASCII ("test_new.pcd",*cloud_);
				print = false;
			}


	}

	

	return 0;

}
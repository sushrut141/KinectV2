#include <Kinect2Grabber/Kinect2Grabber.h>
#include <signal.h>


#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/voxel_grid.h>


/*
//Segmentation
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
*/

//Multi plane
#include <time.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/planar_polygon_fusion.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/plane_coefficient_comparator.h>
#include <pcl/segmentation/euclidean_plane_coefficient_comparator.h>
#include <pcl/segmentation/rgb_plane_coefficient_comparator.h>
#include <pcl/segmentation/edge_aware_plane_comparator.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/surface/convex_hull.h>

typedef pcl::PointXYZRGB PointT;

bool contours_only = true;

bool print = true;
bool stop = false;
bool segment = true;

void sigint_handler(int s)
{
  stop = true;
}

int key;
time_t start,end;
double seconds = 0.0;

int main(int argc,char**argv)
{
	signal(SIGINT,sigint_handler);


	srand(time(NULL));

	int r_ = rand()%255;int g_ = rand()%255;int b_ = rand()%255;

	int i=0;

	kinect::Kinect2Grabber obj;

	kinect::FrameData data;

	//std::vector<int> compress;
	//compress.push_back(CV_IMWRITE_JPEG_QUALITY );
	//compress.push_back(100);
	//compress.push_back(CV_IMWRITE_PNG_COMPRESSION);
	//compress.push_back(2);

	//cv::namedWindow( "registered",CV_WINDOW_AUTOSIZE );
	//cv::namedWindow( "Depth",CV_WINDOW_AUTOSIZE );
	//cv::namedWindow( "Ir",CV_WINDOW_AUTOSIZE );
	

	data = obj.getRegisteredImage();
	//data = obj.getFrameData();

	pcl::PointCloud<PointT>::Ptr cloud_;
	pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);

	pcl::PointCloud<PointT>::Ptr cloud_segment(new pcl::PointCloud<PointT>());


	pcl::visualization::PCLVisualizer* viewer(new pcl::visualization::PCLVisualizer ("Kinect Cloud"));

	cloud_ = obj.getPointCloud();

	//pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud_);
	viewer->addPointCloud<PointT>(cloud_,"Kinect Cloud");
	//pcl::visualization::PointCloudColorHandlerCustom <PointT> color_handle(cloud_segment,r_,g_,b_);
	//viewer->addPointCloud<PointT>(cloud_segment,"Kinect Cloud");
	viewer->setBackgroundColor (0, 0, 0);
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3, "Kinect Cloud");
	//viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();

	/*
	//segment
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<PointT> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.01);

	seg.setInputCloud (cloud_);
	*/

	time(&start);
	while(seconds<100)
	{
			//data = obj.getRegisteredImage();

			cloud_ = obj.getPointCloud();

			/*
			if(segment){
	  			seg.segment (*inliers, *coefficients);
	  			for(unsigned int i=0;i<inliers->indices.size();i++)
				{
					cloud_->points[inliers->indices[i]].r = 255;

					cloud_->points[inliers->indices[i]].g = 0;

					cloud_->points[inliers->indices[i]].b = 0;
				}
			}
		*/
	
			///////////////////////////////////////////////
			//Multi Plane Segmentation
			///////////////////////////////////////////////
			pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
 			pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;
	
	   		 pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
	  		
	  	 	ne.setInputCloud (cloud_);
	  		ne.compute (*normal_cloud);
	  		float* distance_map = ne.getDistanceMap ();
	
	  		boost::shared_ptr<pcl::EdgeAwarePlaneComparator<PointT,pcl::Normal> > eapc(new pcl::EdgeAwarePlaneComparator<PointT, pcl::Normal> ());
	  		eapc->setDistanceMap (distance_map);
	  		eapc->setDistanceThreshold (0.01f, false);
	
	  		std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
	  		std::vector<pcl::ModelCoefficients> model_coefficients;
	  		std::vector<pcl::PointIndices> inlier_indices;  
	  		pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
	  		std::vector<pcl::PointIndices> label_indices;
	  		std::vector<pcl::PointIndices> boundary_indices;
	  		mps.setInputNormals (normal_cloud);
	  		mps.setInputCloud (cloud_);
	
	  		mps.segmentAndRefine (regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);
		
	  		//////////////////view contours of segmented planes////////////////////////////
  			if(contours_only)
			{
			std::vector<PointT,Eigen::aligned_allocator<PointT> > c_points;

			int index_s=0;

			unsigned int size_region = regions.size();

			long int cloud_size;

			for(unsigned int i=0;i<size_region;i++)
			{
				cloud_size+=regions[i].getContour().size();	
			}

			cloud_segment->points.resize(cloud_size);

			for(unsigned int i=0;i<regions.size();i++)
			{

				c_points = regions[i].getContour();

				r_ = rand()%255;g_ = rand()%255;b_ = rand()%255;

				for(unsigned int j=0;j<c_points.size();j++,index_s++)
				{
					cloud_segment->points[index_s].x = c_points[j].x;
					cloud_segment->points[index_s].y = c_points[j].y;
					cloud_segment->points[index_s].z = c_points[j].z;

					cloud_segment->points[index_s].r = r_;
					cloud_segment->points[index_s].g = g_;
					cloud_segment->points[index_s].b = b_;
				}
			}

		}

			//////////////////////////////////////////////////////////////////////////



			///////////////////////////////////////////////////////////////////////////////
			//Using boundary indices
			///////////////////////////////////////////////////////////////////////////////
		else{
			
			for(unsigned int i=0;i<boundary_indices.size();i++)
			{

				r_ = rand()%255;g_ = rand()%255;b_ = rand()%255;
	
				for(unsigned int j=0;j<boundary_indices[i].indices.size();j++)
				{
					cloud_->points[boundary_indices[i].indices[j]].r = r_;

					cloud_->points[boundary_indices[i].indices[j]].g = g_;

					cloud_->points[boundary_indices[i].indices[j]].b = b_;

				}
			}
			
			/*
			for(unsigned int i=0;i<label_indices.size();i++)
			{
				r_ = rand()%255;g_ = rand()%255;b_ = rand()%255;

				for(unsigned int j=0;j<label_indices[i].indices.size();j++)
				{
					cloud_->points[label_indices[i].indices[j]].r = r_;

					cloud_->points[label_indices[i].indices[j]].g = g_;

					cloud_->points[label_indices[i].indices[j]].b = b_;


				}
			}
			*/
		}

			//viewer->updatePointCloud(cloud_segment,"Kinect Cloud");
			viewer->updatePointCloud(cloud_segment,"Kinect Cloud");

			viewer->spinOnce (100);

			if(print&&(key==99)){
				cloud_->height = 1;
				cloud_->width = cloud_->points.size();
				pcl::io::savePCDFileASCII ("test_new.pcd",*cloud_);
				print = false;
			}

			time(&end);

			seconds = difftime(end,start);


	}

	

	return 0;

}
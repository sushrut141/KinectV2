#include <Kinect2Grabber/Kinect2Grabber.h>
#include <fstream>
#include <map>
#include <time.h>

#include <pcl/visualization/pcl_visualizer.h>
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
#include <pcl/io/pcd_io.h>

typedef pcl::PointXYZRGB PointT;


struct Indices
{

	int indices[3];


	Indices(int x,int y,int z)
	{
		
		indices[0] = x;
		indices[1] = y;
		indices[2] = z;
	}

};




int main(int argc,char**argv)
{
	kinect::Kinect2Grabber obj;
	kinect::FrameData img;

	std::map<double,Indices,std::less<int>,Eigen::aligned_allocator<std::pair<double,Indices> > > planes;

	std::ofstream file("out.txt");

	pcl::PointCloud<PointT>::Ptr cloud_1;
	pcl::PointCloud<PointT>::Ptr cloud_2(new pcl::PointCloud<PointT>());

	img = obj.getRegisteredImage();

	cv::namedWindow("registered",CV_WINDOW_AUTOSIZE);

	cv::imshow("registered",img.color_);

	int key = cv::waitKey(10000);

	pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
	pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;

	//visualization
	pcl::visualization::PCLVisualizer* viewer(new pcl::visualization::PCLVisualizer ("planes"));
	viewer->setBackgroundColor (0, 0, 0);
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3, "planes");
	viewer->initCameraParameters ();

	pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>());

	if(key==99)
	{

			cloud_1 = obj.getPointCloud();
	
			ne.setInputCloud(cloud_1);
			ne.compute(*normal_cloud);
			float* distanceMap = ne.getDistanceMap();
	
			boost::shared_ptr<pcl::EdgeAwarePlaneComparator<PointT,pcl::Normal> > eapc(new pcl::EdgeAwarePlaneComparator<PointT, pcl::Normal> ());
			eapc->setDistanceMap (distanceMap);
			eapc->setDistanceThreshold (0.01f, false);
	
			std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
			std::vector<pcl::ModelCoefficients> model_coefficients;
			std::vector<pcl::PointIndices> inlier_indices;  
			pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
			std::vector<pcl::PointIndices> label_indices;
			std::vector<pcl::PointIndices> boundary_indices;
			mps.setInputNormals (normal_cloud);
			mps.setInputCloud (cloud_1);
	
			mps.segmentAndRefine (regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);
	
			//find orthogonal planes
			for(unsigned int i=0;i<regions.size();i++){
				for(unsigned int j=i+1;j<regions.size();j++){
					for(unsigned int k=j+1;k<regions.size();k++){
	
						Eigen::Vector3f normal_1 = regions[i].getCoefficients().segment(0,3);
						Eigen::Vector3f normal_2 = regions[j].getCoefficients().segment(0,3);
						Eigen::Vector3f normal_3 = regions[k].getCoefficients().segment(0,3);
						
						normal_1.normalize();
						normal_2.normalize();
						normal_3.normalize();

						double a = normal_1.dot(normal_2);
						double b = normal_2.dot(normal_3);
						double c = normal_3.dot(normal_1);

						double dot = a + b + c;

						planes.insert(std::pair<double,Indices>(dot,Indices(i,j,k)));

	
					}
				}
			}
			/*
			for(std::map<double,Indices>::iterator i = planes.begin();i!=planes.end();i++)
			{
				file<<"VALUE"<<std::endl;
				file<<i->first<<std::endl;
				file<<"NORMALS"<<std::endl;
				file<<i->second.normal1<<std::endl;
				file<<i->second.normal2<<std::endl;
				file<<i->second.normal3<<std::endl;
			}
			*/
			file<<"VALUE"<<std::endl;
			file<<planes.begin()->first<<std::endl;
			file<<"NORMALS"<<std::endl;

			file<<regions[planes.begin()->second.indices[0]].getCoefficients().segment(0,3)<<std::endl;
			file<<regions[planes.begin()->second.indices[1]].getCoefficients().segment(0,3)<<std::endl;
			file<<regions[planes.begin()->second.indices[2]].getCoefficients().segment(0,3)<<std::endl;

			Indices outline = planes.begin()->second;

			long long int size;

			for(int i=0;i<3;i++){
				size+=regions[outline.indices[i]].getContour().size();
			}

			cloud_2->points.resize(size);

			long long int cloud_index=0;

			srand(time(NULL));

			for(int i=0;i<3;i++)
			{

				int r_ = rand()%255, g_ = rand()%255, b_ = rand()%255;

				pcl::PointCloud<PointT>::VectorType contours = regions[outline.indices[i]].getContour();

				for(unsigned int j=0;j<contours.size();j++,cloud_index++)
				{
					cloud_2->points[cloud_index].x = contours[j].x;
					cloud_2->points[cloud_index].y = contours[j].y;
					cloud_2->points[cloud_index].z = contours[j].z;

					cloud_2->points[cloud_index].r = r_;
					cloud_2->points[cloud_index].g = g_;
					cloud_2->points[cloud_index].b = b_;
				}

			}

			viewer->addPointCloud<PointT>(cloud_2,"planes");
	}
	pcl::io::savePCDFileASCII ("cloud_1.pcd",*cloud_1);

	viewer->updatePointCloud(cloud_2,"planes");
	viewer->spinOnce(10000);

}


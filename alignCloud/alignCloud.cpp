#include <iostream>
#include <time.h>
#include <Kinect2Grabber/Kinect2Granner.h>

//PCL
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

typedef std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >  RegionVec;
typedef pcL::PointCloud<pcl::PointXYZRGB> ColorCloud;
typedef pcL::PointCloud<pcl::PointXYZRGB>::Ptr ColorCloudPtr;

typedef Eigen::Vector3f vec3;

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


void segmentPlanes(RegionVec& regions, ColorCloudPtr cloud)
{
	pcL::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>());

	ne.setInputCloud(cloud_1);
	ne.compute(*normal_cloud);
	float* distanceMap = ne.getDistanceMap();

	boost::shared_ptr<pcl::EdgeAwarePlaneComparator<PointT,pcl::Normal> > eapc(new pcl::EdgeAwarePlaneComparator<PointT, pcl::Normal> ());
	eapc->setDistanceMap (distanceMap);
	eapc->setDistanceThreshold (0.01f, false);

	std::vector<pcl::ModelCoefficients> model_coefficients;
	std::vector<pcl::PointIndices> inlier_indices;  
	pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
	std::vector<pcl::PointIndices> label_indices;
	std::vector<pcl::PointIndices> boundary_indices;
	mps.setInputNormals (normal_cloud);
	mps.setInputCloud (cloud_1);

	mps.segmentAndRefine (regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);

}

void findOrthoPlanes(const RegionVec& regions, std::map<double,Indices>& planes)
{
	for(unsigned int i=0;i<regions.size();i++)
	{
		for(unsigned int j=i+1;j<regions.size();j++)
		{
			for(unsigned int k=j+1;k<regions.size();k++)
			{

				vec3 normal_1 = regions[i].getCoefficients().segment(0,3);
				vec3 normal_2 = regions[j].getCoefficients().segment(0,3);
				vec3 normal_3 = regions[k].getCoefficients().segment(0,3);
				
				normal_1.normalize();
				normal_2.normalize();
				normal_3.normalize();

				double a = normal_1.dot(normal_2);
				double b = normal_2.dot(normal_3);
				double c = normal_3.dot(normal_1);

				double dot = a + b + c;
				if(dot<0.5)
					planes.insert(std::pair<double,Indices>(dot,Indices(i,j,k)));
			}
		}
	}

}



const Eigen::Matrix3f getRotation(Eigen::MatrixXf rotation,vec3 a,vec3 b, vec3 c)
{
	std::map<double,Eigen::Matrix3f> rotationMap;
	std::vector<vec3> out;
	out.push_back(a);
	out.push_back(b);
	out.push_back(c);

	for(int i=0;i<3;i++)
	{
		for(int j=0;j<3;j++)
		{
			for(int k=0;k<3;k++)
			{
				if(i!=j&&i!=k&&j!=k)
				{
					vec3 normal_21 = out.at(i);
					vec3 normal_22 = out.at(j);
					vec3 normal_23 = out.at(k);

					vec3 out_1(normal_21[0],normal_22[0],normal_23[0]);
		
					vec3 a_ = rotation.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(out_1);
		
					vec3 out_2(normal_21[1],normal_22[1],normal_23[1]);
		
					vec3 b_ = rotation.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(out_2);
		
					vec3 out_3(normal_21[2],normal_22[2],normal_23[2]);
		
					vec3 c_ = rotation.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(out_3);

					a_.normalize();
					b_.normalize();
					c_.normalize();

					Eigen::Matrix3f temp;
					temp.row(0) = a_;
					temp.row(1) = b_;
					temp.row(2) = c_;

					/*vec3 col_1 = temp.col(0);
					vec3 col_2 = temp.col(1);
					vec3 col_3 = temp.col(2);

					double dot1 = col_1.dot(col_2);
					double dot2 = col_2.dot(col_3);
					double dot3 = col_3.dot(col_1);

					double sum = dot1 + dot2 + dot3;*/

					Eigen::Matrix3f transpose  = temp;
					transpose.transpose();

					Eigen::Matrix3f isIdentity = transpose*temp;

					double sum = isIdentity(0,0) + isIdentity(1,1)   + isIdentity(2,2);

					double det = temp.determinant();   

					if(det>0&&det<1&&sum<3.2){	
						rotationMap.insert(std::pair<double,Eigen::Matrix3f>(sum,temp));
						std::cout<<"The Identity Matrix Product : "<<std::endl;
						std::cout<<"///////////////////"<<std::endl;
						std::cout<<isIdentity<<std::endl;
						std::cout<<"///////////////////"<<std::endl;
						std::cout<<"The value of determinant  is "<<det<<std::endl;
						std::cout<<"///////////////////"<<std::endl;
					}


				}
			}
		}
	}

	return rotationMap.rbegin()->second;
}

int main(int argc,char**argv)
{
	time_t start,end;

	ColorCloudPtr cloud_1(new ColorCloud);

	ColorCloudPtr cloud_2(new ColorCloud);

	ColorCloudPtr transformed_cloud(new ColorCloud);

	ColorCloudPtr cloud_sum(new ColorCloud);

	kinect::Kinect2Grabber obj;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("Kinect Cloud"));

	RegionVec regions_1,regions_2;

	time(start);

	std::map<double,Indices> planes_1,planes_2;

	ColorCloudPtr cloud_1 = obj.getPointCloud();

	segmentPlanes(regions_1,cloud_1);

	findOrthoPlanes(regions_1,planes_1);

	*cloud_sum = *cloud_1;


	while(seconds<100)
	{

		cloud_2 = obj.getPointCloud();

		segmentPlanes(regions_2,cloud_2);

		if(region_2.size()<3)
			continue;

		findOrthoPlanes(regions_2,planes_2);

		//check if three ortho planes found
		if(planes_2.size()<3)
			continue;

		//do....iterate over all sets of ortho planes while finding transform
/*
		for(unsigned int i=0;i<planes_1.size();i++)
		{
			vec3 n1 = regions_1[planes_1.at(i).second[0]].getCoefficients().segment(0,3);
			vec3 n2 = regions_1[planes_1.at(i).second[1]].getCoefficients().segment(0,3);
			vec3 n3 = regions_1[planes_1.at(i).second[2]].getCoefficients().segment(0,3);

			for(unsigned int j=0;j<planes_2.size();j++)
			{
				vec3 n_1 = regions_2[planes_2.at(i).second[0]].getCoefficients().segment(0,3);
				vec3 n_2 = regions_2[planes_2.at(i).second[1]].getCoefficients().segment(0,3);
				vec3 n_3 = regions_2[planes_2.at(i).second[2]].getCoefficients().segment(0,3);

				

			}


		}
*/

		//finding transform using only the most orthogonal planes

		Eigen::MatrixXf rotation;

		vec3 n_1 = regions_2[planes_2.begin()->second[0]].getCoefficients().segment(0,3);
		vec3 n_2 = regions_2[planes_2.begin()->second[1]].getCoefficients().segment(0,3);
		vec3 n_3 = regions_2[planes_2.begin()->second[2]].getCoefficients().segment(0,3);


		rotation.row(0) = regions_1[planes_1.begin()->second[0]].getCoefficients().segment(0,3);
		rotation.row(1) = regions_1[planes_1.begin()->second[1]].getCoefficients().segment(0,3);
		rotation.row(2) = regions_1[planes_1.begin()->second[2]].getCoefficients().segment(0,3);


		Eigen::Matrix3f rotation_mat = getRotation(rotation,n_1,n_2,n_3);

		Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
	
		transform.block<3,3>(0,0) = rotation_mat;

		pcl::transformPointCloud(*cloud_sum,*transformed_cloud,transform);

		//pcl::io::savePCDFileASCII ("transformed_cloud.pcd",*transformed_cloud);

		//cannot add two point clouds of different size
		*cloud_sum = *cloud_2 + *transformed_cloud;
		
		viewer->addPointCloud<PointT>(cloud_sum,"Kinect Cloud");
		viewer->setBackgroundColor (0, 0, 0);
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3, "Kinect Cloud");

		viewer->updatePointCloud(cloud_sum,"Kinect Cloud");

		viewer->spinOnce(10000);

		*cloud_1 = *cloud_2;

		regions_1 = regions_2;

		planes_1 = planes_2;


	}


}
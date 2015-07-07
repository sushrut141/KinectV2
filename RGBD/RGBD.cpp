#include <Kinect2Grabber/Kinect2Grabber.h>
#include <time.h>
#include <fstream>

//OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>

//PCL
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/planar_polygon_fusion.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/io/pcd_io.h>



typedef pcl::PointXYZRGB PointT;

//feature points
std::vector<cv::KeyPoint> keypoints_1,keypoints_2;

cv::Mat m_desc1,m_desc2;

cv::FlannBasedMatcher matcher;

//feature matches between images
std::vector<cv::DMatch> matches;

//pcl correspondences matcher
boost::shared_ptr<std::vector<pcl::Correspondence,Eigen::aligned_allocator<pcl::Correspondence> > > 
correspondences(new std::vector<pcl::Correspondence,Eigen::aligned_allocator<pcl::Correspondence> >);

pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> corrsRejectorSAC;


pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;





void computeFeatures(const cv::Mat& m_img1,const cv::Mat& m_img2)
{

	cv::SurfFeatureDetector detector(400);

	cv::SurfDescriptorExtractor extractor;

	detector.detect(m_img1,keypoints_1);
	detector.detect(m_img2,keypoints_2);

	if(keypoints_1.size()<4||keypoints_2.size()<4)
	{
		std::cout<<"no keypoints detected"<<std::endl;
		throw;
	}

	extractor.compute(m_img1,keypoints_1,m_desc1);
	extractor.compute(m_img2,keypoints_2,m_desc2);

	matcher.match(m_desc1,m_desc2,matches);
	if(matches.size()<4){
		std::cout<<"Not enough keypoint matches"<<std::endl;
		throw;
	}
}


int outlierRemovalHomography(
	  std::vector<char>& matchesMask,
      const double ransacReprojThreshold,
      std::vector<cv::Point2f>& points1,
      std::vector<cv::Point2f>& points2)
{
    cv::Mat H12;
    int numberInliers=0;

    if(matches.size()>=4) //If there are enough matches to perform findHomography, do it
    {
        std::vector<int> queryIdxs( matches.size() ), trainIdxs( matches.size() );
        for( size_t i = 0; i < matches.size(); i++ )
        {
            queryIdxs[i] = matches[i].queryIdx;
            trainIdxs[i] = matches[i].trainIdx;
        }

        cv::KeyPoint::convert(keypoints_1, points1, queryIdxs);
        cv::KeyPoint::convert(keypoints_2, points2, trainIdxs);

        H12 = cv::findHomography( cv::Mat(points1), cv::Mat(points2), CV_RANSAC, ransacReprojThreshold );

        matchesMask.resize( matches.size(), 0 );
        cv::Mat points1Transformed; cv::perspectiveTransform(cv::Mat(points1), points1Transformed, H12);

        //For each correspondence x_i <-> x_i', if distance(x_i',H*x_i)<threshold, then consider the correspondence as an inlier
        double maxInlierDist = ransacReprojThreshold < 0 ? 3 : ransacReprojThreshold;
        for( size_t i1 = 0; i1 < points2.size(); i1++ )
        {
            if( norm(points2[i1] - points1Transformed.at<cv::Point2f>((int)i1,0)) <= maxInlierDist ) // inlier
            {
                matchesMask[i1] = 1;
                numberInliers++;
            }
        }
    }
    else //If there are very few matches, return -1
    {
        numberInliers=-1;
    }

    return numberInliers;
}

void matchesWith3DValidData(
				const std::vector<cv::Point2f>& points2d1,
                const std::vector<cv::Point2f>& points2d2,
                const pcl::PointCloud<PointT>::Ptr& pointCloudPtr1,
				const pcl::PointCloud<PointT>::Ptr& pointCloudPtr2)
{

    static int numberValidPoints;
    static int point1X,point1Y,point2X,point2Y;
    static int pointIndex1,pointIndex2;
    static double depthPoint1,depthPoint2;

    for(int i=0;i<points2d1.size();i++)
    {
        point1X=points2d1[i].x;
        point1Y=points2d1[i].y;

        point2X=points2d2[i].x;
        point2Y=points2d2[i].y;

        pointIndex1=640*point1Y+point1X;
        pointIndex2=640*point2Y+point2X;

        if(pointIndex1<=0 ||  //Check if the idexes are invalid
           pointIndex1>=pointCloudPtr1->points.size() ||
           pointIndex2<=0 ||
           pointIndex2>=pointCloudPtr2->points.size())
        {
            (*correspondences)[i].index_query=-1;
            (*correspondences)[i].index_match=-1;
            (*correspondences)[i].distance=0;
        }
        else
        {
            depthPoint1=pointCloudPtr1->points[pointIndex1].z;
            depthPoint2=pointCloudPtr2->points[pointIndex2].z;

         
                //Check for valid (x,y,z) values
                if(pcl_isfinite (pointCloudPtr1->points[pointIndex1].x) &&
                   pcl_isfinite (pointCloudPtr1->points[pointIndex1].y) &&
                   pcl_isfinite (pointCloudPtr1->points[pointIndex1].z) &&
                   pcl_isfinite (pointCloudPtr2->points[pointIndex2].x) &&
                   pcl_isfinite (pointCloudPtr2->points[pointIndex2].y) &&
                   pcl_isfinite (pointCloudPtr2->points[pointIndex2].z))
                {
                    double distance = sqrt(pow(pointCloudPtr1->points[pointIndex1].x-
                                               pointCloudPtr2->points[pointIndex2].x,2)+
                                           pow(pointCloudPtr1->points[pointIndex1].y-
                                               pointCloudPtr2->points[pointIndex2].y,2)+
                                           pow(pointCloudPtr1->points[pointIndex1].z-
                                               pointCloudPtr2->points[pointIndex2].z,2));

                    (*correspondences)[i].index_query=pointIndex1;
                    (*correspondences)[i].index_match=pointIndex2;
                    (*correspondences)[i].distance=distance;
                }
                else
                {
                    (*correspondences)[i].index_query=-1;
                    (*correspondences)[i].index_match=-1;
                    (*correspondences)[i].distance=0;

                }
        }
    }

}


int estimateVisual3DRigidTransformation(
		const std::vector<cv::Point2f>& points2d1,
		const std::vector<cv::Point2f>& points2d2,
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointCloudPtr1,
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointCloudPtr2,
		Eigen::Matrix4f& H)
{
    correspondences->clear();
    correspondences->resize(points2d1.size());

    matchesWith3DValidData(points2d1,points2d2,pointCloudPtr1,pointCloudPtr2);


    corrsRejectorSAC.setInputSource(pointCloudPtr1);
    corrsRejectorSAC.setInputTarget(pointCloudPtr2);
    corrsRejectorSAC.setInlierThreshold(0.20);
    corrsRejectorSAC.setMaximumIterations(500);
    corrsRejectorSAC.setInputCorrespondences(correspondences);
    boost::shared_ptr<pcl::Correspondences> correspondencesRejSAC (new pcl::Correspondences);
    corrsRejectorSAC.getCorrespondences(*correspondencesRejSAC);
    H = corrsRejectorSAC.getBestTransformation();

    //Return the number of RANSAC inliers
    return correspondencesRejSAC->size();
}



void segmentPlanes(std::vector<pcl::PointIndices>& boundary_indices, pcl::PointCloud<PointT>::Ptr& cloud)
{
	pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>());

	ne.setInputCloud(cloud);
	ne.compute(*normal_cloud);
	
	std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
	std::vector<pcl::ModelCoefficients> model_coefficients;
	std::vector<pcl::PointIndices> inlier_indices;  
	pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
	std::vector<pcl::PointIndices> label_indices;
	mps.setInputNormals (normal_cloud);
	mps.setInputCloud (cloud);

	mps.segmentAndRefine (regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);

}

void colorBoundaries(const std::vector<pcl::PointIndices>& boundaries,pcl::PointCloud<PointT>::Ptr& cloud)
{
	for(unsigned int i=0;i<boundaries.size();i++)
	{
		int r_ = rand()%255;int g_ = rand()%255;int b_ = rand()%255;

		for(unsigned int j=0;j<boundaries[i].indices.size();j++)
		{
			cloud->points[boundaries[i].indices[j]].r = r_;

			cloud->points[boundaries[i].indices[j]].g = g_;

			cloud->points[boundaries[i].indices[j]].b = b_;

		}
	}
}


int main(int argc,char**argv)
{
	std::ofstream file("out.txt");
	srand(time(NULL));
	kinect::Kinect2Grabber grabber;

	kinect::FrameData frame1,frame2;

	pcl::PointCloud<PointT>::Ptr cloud_1(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_2(new pcl::PointCloud<PointT>);

	pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>);

	try
	{
	int key;

	frame1 = grabber.getFrameData();
	cv::imwrite("img1.png",frame1.color_);
	*cloud_1 = *(grabber.getPointCloud());

	cv::namedWindow("registered",CV_WINDOW_AUTOSIZE);

	cv::imshow("registered",frame1.color_);

	key = cv::waitKey(10000);

	if(key==99){
		frame2 = grabber.getFrameData();
		cv::imwrite("img2.png",frame2.color_);
		*cloud_2 = *(grabber.getPointCloud());
	}

	cv::imshow("registered",frame2.color_);

	cv::waitKey(1000);


	cv::Mat img1,img2;
	img1 = frame1.color_;
	img2 = frame2.color_;

	computeFeatures(img1,img2);

	std::vector<char> mask;

	std::vector<cv::Point2f> points1;
	std::vector<cv::Point2f> points2;

	//matched points
	std::vector<cv::Point2f> points_1;
	std::vector<cv::Point2f> points_2;

	std::vector<cv::DMatch> good_matches;

	int inliers = outlierRemovalHomography(mask,3.0,points_1,points_2);
	if(inliers==-1)
		throw;

	for(unsigned int i =0;i<mask.size();i++)
	{
		if(mask.at(i)=='1')
		{
			points_1.push_back(points1.at(i));
			points_2.push_back(points2.at(i));

		}
	}

/*	if(good_matches.empty()){
		std::cout<<"not enough inliers found"<<std::endl;
		throw;
	}
	cv::namedWindow("matches",CV_WINDOW_AUTOSIZE);

	cv::Mat img_matches;
  	cv::drawMatches( img1,keypoints_1, img2,keypoints_2,
               good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
               std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

  	cv::imshow("matches",img_matches);

  	cv::waitKey(10000);*/

	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

	int inliers_3d = estimateVisual3DRigidTransformation(points_1,points_2,cloud_1,cloud_2,transform);

	std::vector<pcl::PointIndices> boundaries_1,boundaries_2;

	//find planar regions
	segmentPlanes(boundaries_1,cloud_1);
	segmentPlanes(boundaries_2,cloud_2);

	//highlight boundaries
	colorBoundaries(boundaries_1,cloud_1);
	colorBoundaries(boundaries_2,cloud_2);

	file<<transform;

	pcl::transformPointCloud(*cloud_1,*transformed_cloud,transform);

	pcl::PointCloud<PointT>::Ptr sum_cloud(new pcl::PointCloud<PointT>);

	sum_cloud = transformed_cloud;

	*sum_cloud+=*cloud_2;

	pcl::io::savePCDFileASCII ("cloud_1.pcd",*cloud_1);
	pcl::io::savePCDFileASCII ("cloud_2.pcd",*cloud_2);
	pcl::io::savePCDFileASCII ("cloud_sum.pcd",*sum_cloud);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("Kinect Cloud"));
	viewer->addPointCloud<PointT>(sum_cloud,"Kinect Cloud");
	viewer->setBackgroundColor (0, 0, 0);
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3, "Kinect Cloud");

	viewer->updatePointCloud(sum_cloud,"Kinect Cloud");

	viewer->spinOnce(100000);
}
catch(...)
{
	std::cout<<"Program throws error"<<std::endl;
	return 0;
}


}
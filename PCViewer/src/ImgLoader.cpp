#include <PCViewer/ImgLoader.h>
#include <iostream>
#include <dirent.h>


namespace kinect
{
	ImgLoader::ImgLoader(std::string rgbPath,std::string irPath)
	{
		rgbImgPath_ = rgbPath;
		irImgPath_ = irPath;

		getImageFileNames(rgbImgPath_,rgbImgFiles_);
		getImageFileNames(irImgPath_,irImgFiles_);


	}

	ImgLoader::~ImgLoader()
	{

		std::cout<<"Object is destroyed"<<std::endl;

	}

	void ImgLoader::calibCamera()
	{
		const cv::Size board_size(6,9);
		const float square_size = 0.025;

		const cv::TermCriteria term_criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50, DBL_EPSILON);

		std::vector<std::vector<cv::Point2f> > rgbImgPoints;
		std::vector<std::vector<cv::Point2f> > irImgPoints;

		std::vector<cv::Mat> rvec,tvec;

		if(rgbImgPath_.back()!='/')
			rgbImgPath_+=std::string("/");
		if(irImgPath_.back()!='/')
			irImgPath_+=std::string("/");

		unsigned int img_num = rgbImgFiles_.size();

		for(unsigned int i=0;i<img_num;i++)
			std::cout<<"Files are "<<rgbImgFiles_.at(i)<<std::endl;

		for(unsigned int i=0; i<img_num; i++)
		{
			std::string rgb_name = rgbImgPath_ + rgbImgFiles_.at(i);
			std::string ir_name  = irImgPath_  + irImgFiles_.at(i);

			rgbImg = cv::imread(rgb_name,0);
			irImg  = cv::imread(ir_name,0);

			std::vector<cv::Point2f > camera1ImagePoints;
		    bool found1 = cv::findChessboardCorners(rgbImg, board_size, camera1ImagePoints, cv::CALIB_CB_FAST_CHECK);


		    std::vector<cv::Point2f> camera2ImagePoints;
		    bool found2 = cv::findChessboardCorners(irImg, board_size, camera2ImagePoints, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);

		    
		    if(found1){
		    	cv::cornerSubPix(rgbImg, camera1ImagePoints, cv::Size(11, 11), cv::Size(-1, -1), term_criteria);
		    	rgbImgPoints.push_back(camera1ImagePoints);
		    }

		    if(found2){
			    cv::cornerSubPix(irImg, camera2ImagePoints, cv::Size(11, 11), cv::Size(-1, -1), term_criteria);
			    irImgPoints.push_back(camera2ImagePoints);
			}
		}

		calibsize_depth_ = irImg.size();
	    calibsize_rgb_ 	 = rgbImg.size();

		std::vector<std::vector<cv::Point3f> > pointsBoard(1);
		calcBoardCornerPositions(board_size, square_size, pointsBoard[0]);

		pointsBoard.resize(img_num,pointsBoard[0]);

		double error_1 = cv::calibrateCamera(pointsBoard, rgbImgPoints, calibsize_rgb_,
											 rgb_camera_matrix_, rgb_distortion_,  rvec,  tvec);
		
		double error_2 = cv::calibrateCamera(pointsBoard, irImgPoints, calibsize_depth_, 
											 depth_camera_matrix_, depth_distortion_,  rvec,  tvec);
		

		double rms = cv::stereoCalibrate(pointsBoard, rgbImgPoints, irImgPoints,
		                rgb_camera_matrix_, rgb_distortion_,
		                depth_camera_matrix_, depth_distortion_,
		                calibsize_rgb_, rotation_, translation_, essential_, fundamental_,
		                term_criteria,
		                cv::CALIB_FIX_INTRINSIC
		                );
		std::cout<<"Camera calibration done"<<std::endl;

	}


	void ImgLoader::getImageFileNames(std::string dir,std::vector<std::string>& files)
	{
		 DIR *dp;
    	struct dirent *dirp;

    	std::cout<<"Opening dir "<<dir<<" "<<std::endl;

    	//open rgb
    	if((dp  = opendir(dir.c_str())) == NULL) {
    	    std::cout << "Error opening " << dir <<std::endl;
    	}
	
    	while ((dirp = readdir(dp)) != NULL) {
    	    if(dirp->d_name[0]!='.')
    	        files.push_back(std::string(dirp->d_name));
	    }
	    	
    	closedir(dp);
	}


	void ImgLoader::calcBoardCornerPositions(const cv::Size & board_size, const float square_size_, std::vector<cv::Point3f> & corners)
	{
	    corners.clear();
	        for( int i = 0; i < board_size.height; ++i )
	            for( int j = 0; j < board_size.width; ++j )
	                corners.push_back(cv::Point3f(float( j*square_size_ ), float( i*square_size_ ), 0));
	}


}
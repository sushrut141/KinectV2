#ifndef IMG_LOADER_H_
#define IMG_LOADER_H_

#include <string>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>


namespace kinect
{

class ImgLoader
{
public:
	ImgLoader(std::string rgbPath,std::string irPath);

	~ImgLoader();

protected:
	void calibCamera();


private:

	void calcBoardCornerPositions(const cv::Size& board_size, const float square_size_, std::vector<cv::Point3f>& corners);

	void getImageFileNames(std::string dir,std::vector<std::string>& files);


protected:

	cv::Mat rotation_, translation_, essential_, fundamental_;
	cv::Mat rgb_camera_matrix_, depth_camera_matrix_, depth_distortion_, rgb_distortion_;

	cv::Mat rgbImg,irImg;

	cv::Size calibsize_depth_, calibsize_rgb_;

private:
	
	std::vector<std::string> rgbImgFiles_;
	std::vector<std::string> irImgFiles_; 

	std::string rgbImgPath_;
	std::string irImgPath_;


};

}

#endif


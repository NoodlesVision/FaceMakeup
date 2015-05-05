#ifndef __FACE_MAKEUP__
#define __FACE_MAKEUP__

#include "SDMLocator.h"
#include "opencv2\opencv.hpp"
#include "opencv\cv.h"

#include "hpfacedetectorwrapper.h"
#include "imgwarp_mls.h"
#include "imgwarp_piecewiseaffine.h"

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

using namespace cv;
using namespace std;

class FaceMakeUp
{

public:
	FaceMakeUp();
	~FaceMakeUp();
public: 
	void loadFaceModels(string& pthModels);
	int getNumModels();
	void getRefsMasks(SDMLocator* locator, HPFaceDetectorWarpper* faceDetector, Mat& meanShape);
	
	void getLipMask(Mat& img, std::vector<cv::Point2f>& vecLMs);
	void makeupLips(Mat& img,vector<cv::Point2f>& vecImgLMs, int indModel);

	void makeupEyes(Mat& img,vector<cv::Point2f>& vecLMs, int indModel);

private: 
	std::vector <std::vector<cv::Point2f> >m_vecRefLipROILMs;
	std::vector <cv:: Mat > m_refLipROIs;
	
	std::vector <std::vector<cv::Point2f> > m_vecEyesRefROILMs;
	std::vector <cv::Mat > m_refEyesROIs;
	
	std::vector < string > m_pathModels;
	ImgWarp_MLS * m_warp;

	Mat m_imgLipMask;
	Mat m_imgLipROI;
	
	Mat m_imgEyesMask;
	Mat m_imgEyesROI;
};

#endif
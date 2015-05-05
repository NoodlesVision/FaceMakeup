#ifndef __POSE_CORRECT__
#define __POSE_CORRECT__

#include "opencv2\opencv.hpp"
#include "opencv\cv.h"

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

using namespace std;
using namespace cv;

class CPoseEstimate
{
	
public:
	CPoseEstimate();
	~CPoseEstimate();

	void load3DFace();
	void load3DFace_Eye();
	void computerPosePOIST(std::vector<Point2f>& landmarks);
	void computerPosePOISTByEye(std::vector<Point2f>& landmarks);
	void correctPose(Mat& pFrame);
	void correctPoseByEye(Mat& pFrame);
	void drawAxis(Mat& pFrame);
	void drawAxis_Eye(Mat& pFrame);
	void getPose(float& p, float& y, float& r);
	void getPose_Eye(float& p, float& y, float& r);
	void mergeFace();
public:
	vector< CvPoint3D32f > m_modelPtsPOIST;
	Mat m_rotMat;
	
	float m_pitch;
	float m_yaw;
	float m_roll;

	float meaLmksX;
	float meaLmksY;
	
	CvPOSITObject *m_positObject;
	CvMatr32f m_rotation_matrix;
	CvVect32f m_translation_vector;
	CvTermCriteria m_criteria;

	
	
	vector< CvPoint3D32f > m_modelPtsPOIST_Eye;
	Mat m_rotMat_Eye;
	
	float m_pitch_Eye;
	float m_yaw_Eye;
	float m_roll_Eye;

	float meaLmksX_Eye;
	float meaLmksY_Eye;
	
	CvPOSITObject *m_positObject_Eye;
	CvMatr32f m_rotation_matrix_Eye;
	CvVect32f m_translation_vector_Eye;
	CvTermCriteria m_criteria_Eye;

	Mat m_faceMask;
	Mat m_eyeMask;

	Rect m_leftEye_mask;
	Rect m_rightEye_mask;

	Mat m_correct;
	Mat m_correct_eye;
	Mat m_correct_final;
};

#endif
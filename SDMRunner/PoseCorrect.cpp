#include "PoseCorrect.h"


double model3dAll[320][320][3];

double model_49[49][3] = {
{-55.4203338623047,	-82.6790466308594,	61.1132392883301},
{-45.5692253112793,	-98.4211883544922,	64.8120117187500},
{-34.7885284423828,	-105.959381103516,	65.0586395263672},
{-24.4476337432861,	-110.638717651367,	63.5711669921875},
{-14.3771553039551,	-112.276359558105,	61.4060516357422},
{11.6301584243774,	-111.405639648438,	62.4921112060547},
{21.7039909362793,	-110.074790954590,	65.6361770629883},
{34.1593513488770,	-104.322738647461,	67.2592926025391},
{45.2833251953125,	-93.7277908325195,	66.3257598876953},
{56.0607147216797,	-70.6840744018555,	61.1257743835449},
{-0.381882011890411,	-110.447166442871,	48.5439338684082},
{-1.38499450683594,	-119.805107116699,	35.0852890014648},
{-1.38499450683594,	-126.509101867676,	23.1398639678955},
{-1.38499450683594,	-137.019577026367,	10.5875854492188},
{-12.3490858078003,	-113.586593627930,	0.477435529232025},
{-6.31677961349487,	-118.696006774902,	-1.36035811901093},
{-0.399197280406952,	-118.971603393555,	-2.33775830268860},
{5.56237840652466,	-115.680725097656,	-1.45222997665405},
{11.6026706695557,	-112.446601867676,	-0.551718592643738},
{-39.0101127624512,	-89.7556304931641,	45.8344039916992},
{-32.4472198486328,	-94.5481109619141,	50.7096672058106},
{-24.1688404083252,	-94.4388885498047,	51.7532806396484},
{-18.0284042358398,	-92.1822509765625,	45.6816024780273},
{-25.2106838226318,	-94.3058547973633,	43.4760856628418},
{-31.3653163909912,	-95.3374023437500,	43.4153213500977},
{16.3626499176025,	-90.3282394409180,	44.7543678283691},
{23.3715114593506,	-96.4614105224609,	49.5382347106934},
{31.6546459197998,	-95.9850234985352,	50.6045913696289},
{39.5506668090820,	-87.5450897216797,	44.9239692687988},
{31.7264556884766,	-94.8802413940430,	42.4075164794922},
{23.4386577606201,	-95.0840530395508,	42.3959236145020},
{-23.7462558746338,	-103.895309448242,	-22.1397361755371},
{-17.3444252014160,	-113.227745056152,	-16.4889354705811},
{-9.28938674926758,	-117.863014221191,	-14.2303762435913},
{-0.399441987276077,	-119.092079162598,	-16.1315784454346},
{7.51848649978638,	-117.259140014648,	-14.2647218704224},
{15.5840816497803,	-112.874992370605,	-18.5067939758301},
{21.9577903747559,	-104.642257690430,	-25.1313438415527},
{15.5699234008789,	-113.285018920898,	-29.4509716033936},
{7.54606628417969,	-115.750511169434,	-32.2126464843750},
{-1.38499450683594,	-110.130569458008,	-33.7415161132813},
{-9.36198997497559,	-113.395126342773,	-31.4356021881104},
{-17.4460258483887,	-110.101562500000,	-27.7213973999023},
{-9.35895061492920,	-113.582191467285,	-21.4510593414307},
{-0.388509154319763,	-113.709754943848,	-22.4384746551514},
{8.60704326629639,	-112.371444702148,	-22.5363407135010},
{8.60704326629639,	-112.371444702148,	-22.5363407135010},
{-0.388509154319763,	-113.709754943848,	-22.4384746551514},
{-10.3598794937134,	-113.353294372559,	-21.4673328399658}
};

CPoseEstimate::CPoseEstimate()
{
	m_pitch = 180.;
	m_yaw = 180.;
	m_roll = 180.;	
	
	m_eyeMask = Mat(320, 320, CV_32FC1, Scalar(0)); 
	m_leftEye_mask = Rect(Point(117, 111), Point(151, 147));
	m_rightEye_mask = Rect(Point(172, 111), Point(206, 147));

	load3DFace();
	m_positObject = cvCreatePOSITObject( &m_modelPtsPOIST[0], static_cast<int>(m_modelPtsPOIST.size()));
	
	m_rotation_matrix = new float[9];
	m_translation_vector = new float[3];
	m_criteria = cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 200, 1.0e-4f);

	
	load3DFace_Eye();
	m_positObject_Eye = cvCreatePOSITObject( &m_modelPtsPOIST_Eye[0], static_cast<int>(m_modelPtsPOIST_Eye.size()));
	
	m_rotation_matrix_Eye = new float[9];
	m_translation_vector_Eye = new float[3];
	m_criteria_Eye = cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 200, 1.0e-4f);

	
}
CPoseEstimate::~CPoseEstimate()
{
	m_rotMat.release();	
	cvReleasePOSITObject(&m_positObject);
	delete[] m_rotation_matrix;
	delete[] m_translation_vector;

	
	m_rotMat_Eye.release();
	cvReleasePOSITObject(&m_positObject_Eye);
	delete[] m_rotation_matrix_Eye;
	delete[] m_translation_vector_Eye;
}

void CPoseEstimate::load3DFace_Eye()
{
	float xOffset = -55.4203338623047;  
    float yOffset = -82.6790466308594;  
    float zOffset =	61.1132392883301;  

	for(int i = 0; i < 13; i++)
	{
		Point3f pt;
		pt.x = model_49[i][0] - xOffset;
		pt.y = model_49[i][1] - yOffset;
		pt.z = model_49[i][2] - zOffset;
		m_modelPtsPOIST_Eye.push_back(pt);
	}

	for(int i = 19; i < 31; i++)
	{
		Point3f pt;
		pt.x = model_49[i][0] - xOffset;
		pt.y = model_49[i][1] - yOffset;
		pt.z = model_49[i][2] - zOffset;
		m_modelPtsPOIST_Eye.push_back(pt);
	}
}


void CPoseEstimate::load3DFace()
{
	float xOffset = -55.4203338623047;  
    float yOffset = -82.6790466308594;  
    float zOffset =	61.1132392883301;  

	for(int i = 0; i < 49; i++)
	{
		Point3f pt;
		pt.x = model_49[i][0] - xOffset;
		pt.y = model_49[i][1] - yOffset;
		pt.z = model_49[i][2] - zOffset;
		m_modelPtsPOIST.push_back(pt);
	}

	
	FILE *fp = NULL;	
	fp = fopen("3d.bin", "rb");
	if (fp == NULL)	
		return;	
	for(int m = 0; m < 320; m++)
	{
		for(int n = 0; n < 320; n++)
		{
			for(int d = 0; d < 3; d++)
			{
				fread((char*)&model3dAll[m][n][d], sizeof(double), 1, fp);				
			}
		}
	}
	fclose(fp);

	m_faceMask = imread("reference_320_320.png", 0);
	/*for ( int p=0; p< 320; p++ )  
	{  
		for ( int q=0; q< 320; q++ )  
		{
			for(int r = 0; r < 49; r++)
			{
				CvPoint2D32f point2D = cvPoint2D32f( 0.0, 0.0 ); 		
				if(abs(model3dAll[p][q][0] - model_49[r][0]) < 0.1 && abs(model3dAll[p][q][1] - model_49[r][1]) < 0.1 && abs(model3dAll[p][q][2] - model_49[r][2]) < 0.1)
				{
					circle(m_faceMask, Point( q,  p), 2, CV_RGB(255,255,255), -1);
				}
				
			} 

		}
		
	} 

	imshow("m_faceMask", m_faceMask);
	waitKey(0);*/


	Mat tmpMask = Mat(320, 320, CV_8U, Scalar(0));
	for ( int p=0; p< 320; p++ )  
	{  
		for ( int q=0; q< 320; q++ )  
		{
			if (m_leftEye_mask.contains(Point(p, q)) || m_rightEye_mask.contains(Point(p, q))) 
			{
					tmpMask.at<uchar>(q,p) = 255;
			}
		}
	}
	GaussianBlur(tmpMask, tmpMask, Size(11, 11), 10);
	
	for ( int p=0; p< 320; p++ )  
	{  
		for ( int q=0; q< 320; q++ )  
		{
			
			m_eyeMask.at<float>(q,p) = 1.0*(tmpMask.at<uchar>(q,p)) / 255.0;
			
		}
	}
	tmpMask.release();
		
}



void CPoseEstimate::computerPosePOIST(std::vector<Point2f>& landmarks)
{
	vector <CvPoint2D32f> srcImagePointsPOIST;	
	double t = (double)cvGetTickCount();
	if(landmarks.size() == 49)
	{
		//printf("landmarks0 is %f %f\n", landmarks[0].x, landmarks[0].y);
		//printf("landmarks48is %f %f\n", landmarks[48].x, landmarks[48].y);
		double meanX = 0;
		double meanY = 0;
		for(int i = 0; i < landmarks.size(); i++)
		{
			meanX += landmarks[i].x;
			meanY += landmarks[i].y;
		}
		meanX =  meanX / landmarks.size();
		meanY = meanY / landmarks.size();
	
		meaLmksX = meanX;
		meaLmksY = meanY;
	
		for(int i = 0; i < landmarks.size(); i++)
		{
			srcImagePointsPOIST.push_back(Point2d(landmarks[i].x - meanX , landmarks[i].y- meanY ));
		}
	}
	else if(landmarks.size() == 68)
	{
		//printf("landmarks0 is %f %f\n", landmarks[17].x, landmarks[17].y);
		double meanX = 0;
		double meanY = 0;
		for(int i = 17; i < landmarks.size(); i++)
		{
			if(i != 60 && i != 64)
			{
				meanX += landmarks[i].x;
				meanY += landmarks[i].y;
			}
		}
		meanX =  meanX / 49;
		meanY = meanY / 49;
	
		meaLmksX = meanX;
		meaLmksY = meanY;
	
		for(int i = 17; i < landmarks.size(); i++)
		{
			if(i != 60 && i != 64)
				srcImagePointsPOIST.push_back(Point2d(landmarks[i].x - meanX , landmarks[i].y- meanY ));
		}
	}
	
	
	if(m_modelPtsPOIST.size() != srcImagePointsPOIST.size())
	{
		return;
	}
	
	Vec3d eavPOSIT;
	cvPOSIT( m_positObject, &srcImagePointsPOIST[0], 492.3077, m_criteria, m_rotation_matrix, m_translation_vector );
	t = (double)cvGetTickCount() - t;
	double tt = t/((double)cvGetTickFrequency()*1000.);
	//printf( "pose estimate time = %g ms\n",tt);
	Mat rotM(3, 3, CV_64FC1);
	rotM.at<double>(0, 0) = m_rotation_matrix[0];
	rotM.at<double>(0, 1) = m_rotation_matrix[1];
	rotM.at<double>(0, 2) = m_rotation_matrix[2];
	rotM.at<double>(1, 0) = m_rotation_matrix[3];
	rotM.at<double>(1, 1) = m_rotation_matrix[4];
	rotM.at<double>(1, 2) = m_rotation_matrix[5];
	rotM.at<double>(2, 0) = m_rotation_matrix[6];
	rotM.at<double>(2, 1) = m_rotation_matrix[7];
	rotM.at<double>(2, 2) = m_rotation_matrix[8];
	rotM.copyTo(m_rotMat);
	Mat _tmp,_tmp1,_tmp2,_tmp3,_tmp4,_tmp5;
	double pm[12] = {m_rotation_matrix[0],m_rotation_matrix[1],m_rotation_matrix[2],0,
							  m_rotation_matrix[3],m_rotation_matrix[4],m_rotation_matrix[5],0,
							  m_rotation_matrix[6],m_rotation_matrix[7],m_rotation_matrix[8],0};
	decomposeProjectionMatrix(Mat(3,4,CV_64FC1,pm),_tmp,_tmp1,_tmp2,_tmp3,_tmp4,_tmp5,eavPOSIT);
	
	m_pitch = (float)((int)(eavPOSIT[0] * 10)) / 10;
	m_yaw = (float)((int)(eavPOSIT[1] * 10)) / 10;
	m_roll = (float)((int)(eavPOSIT[2] * 10)) / 10;
}

void CPoseEstimate::computerPosePOISTByEye(std::vector<Point2f>& landmarks)
{
	vector <CvPoint2D32f> srcImagePointsPOIST;	
	if(landmarks.size() == 49)
	{
		//printf("landmarks0 is %f %f\n", landmarks[0].x, landmarks[0].y);
		double meanX = 0;
		double meanY = 0;
		
		for(int i = 0; i < 13; i++)
		{
			meanX += landmarks[i].x;
			meanY += landmarks[i].y;
		}
		for(int i = 19; i < 31; i++)
		{
			meanX += landmarks[i].x;
			meanY += landmarks[i].y;
		}		
		
		meanX =  meanX / 25;
		meanY = meanY / 25;
	
		meaLmksX_Eye = meanX;
		meaLmksY_Eye = meanY;
	
		for(int i = 0; i < 13; i++)
		{
			srcImagePointsPOIST.push_back(Point2d(landmarks[i].x - meanX , landmarks[i].y- meanY ));
		}
		for(int i = 19; i < 31; i++)
		{
			srcImagePointsPOIST.push_back(Point2d(landmarks[i].x - meanX , landmarks[i].y- meanY ));
		}

	}
	else if(landmarks.size() == 68)
	{
		printf("landmarks0 is %f %f\n", landmarks[17].x, landmarks[17].y);
		
		double meanX = 0;
		double meanY = 0;
		for(int i = 17; i < 30; i++)
		{
			
			meanX += landmarks[i].x;
			meanY += landmarks[i].y;
			
		}
		for(int i = 36; i < 48; i++)
		{
			
			meanX += landmarks[i].x;
			meanY += landmarks[i].y;
			
		}

		meanX =  meanX / 25;
		meanY = meanY / 25;
	
		meaLmksX_Eye = meanX;
		meaLmksY_Eye = meanY;
	
		for(int i = 17; i < 30; i++)
		{
			srcImagePointsPOIST.push_back(Point2d(landmarks[i].x - meanX , landmarks[i].y- meanY ));
		}

		for(int i = 36; i < 48; i++)
		{
			srcImagePointsPOIST.push_back(Point2d(landmarks[i].x - meanX , landmarks[i].y- meanY ));
		}
	}
	
	
	if(m_modelPtsPOIST_Eye.size() != srcImagePointsPOIST.size())
	{
		return;
	}
	
	Vec3d eavPOSIT;
	cvPOSIT( m_positObject_Eye, &srcImagePointsPOIST[0], 492.3077, m_criteria_Eye, m_rotation_matrix_Eye, m_translation_vector_Eye );
	Mat rotM(3, 3, CV_64FC1);
	rotM.at<double>(0, 0) = m_rotation_matrix_Eye[0];
	rotM.at<double>(0, 1) = m_rotation_matrix_Eye[1];
	rotM.at<double>(0, 2) = m_rotation_matrix_Eye[2];
	rotM.at<double>(1, 0) = m_rotation_matrix_Eye[3];
	rotM.at<double>(1, 1) = m_rotation_matrix_Eye[4];
	rotM.at<double>(1, 2) = m_rotation_matrix_Eye[5];
	rotM.at<double>(2, 0) = m_rotation_matrix_Eye[6];
	rotM.at<double>(2, 1) = m_rotation_matrix_Eye[7];
	rotM.at<double>(2, 2) = m_rotation_matrix_Eye[8];
	rotM.copyTo(m_rotMat_Eye);
	Mat _tmp,_tmp1,_tmp2,_tmp3,_tmp4,_tmp5;
	double pm[12] = {m_rotation_matrix_Eye[0],m_rotation_matrix_Eye[1],m_rotation_matrix_Eye[2],0,
							  m_rotation_matrix_Eye[3],m_rotation_matrix_Eye[4],m_rotation_matrix_Eye[5],0,
							  m_rotation_matrix_Eye[6],m_rotation_matrix_Eye[7],m_rotation_matrix_Eye[8],0};
	decomposeProjectionMatrix(Mat(3,4,CV_64FC1,pm),_tmp,_tmp1,_tmp2,_tmp3,_tmp4,_tmp5,eavPOSIT);
	
	m_pitch_Eye = (float)((int)(eavPOSIT[0] * 10)) / 10;
	m_yaw_Eye = (float)((int)(eavPOSIT[1] * 10)) / 10;
	m_roll_Eye = (float)((int)(eavPOSIT[2] * 10)) / 10;
}

void CPoseEstimate::getPose(float& p, float& y, float& r)
{
	p = m_pitch;
	y = m_yaw;
	r = m_roll;
}

void CPoseEstimate::getPose_Eye(float& p, float& y, float& r)
{
	p = m_pitch_Eye;
	y = m_yaw_Eye;
	r = m_roll_Eye;
	
}

void CPoseEstimate::correctPose(Mat& pFrame)
{
	double* _r = m_rotMat.ptr<double>();

	Mat grayImage;
	cvtColor(pFrame, grayImage, cv::COLOR_BGR2GRAY);
	m_correct = Mat(320, 320, CV_8U, Scalar(0)); 
	Mat face = Mat(480, 640, CV_8U, Scalar(0)); 

	float xOffset = -55.4203338623047;  
    float yOffset = -82.6790466308594;  
    float zOffset =	61.1132392883301;  
	
	vector <CvPoint2D32f>  projectPoints;
	double t = (double)cvGetTickCount();
	for(int i = 0; i < 320; i++)
	{
		for(int j = 0; j < 320; j++)
		{
			if(m_faceMask.at<uchar>(j,i) != 255)
			{
				CvPoint3D32f point3D;  		
				point3D.x = _r[0] * (model3dAll[j][i][0] - xOffset) +   _r[1] * (model3dAll[j][i][1] - yOffset) +  _r[2] *(model3dAll[j][i][2] - 61.1132392883301) + m_translation_vector[0]; 
				point3D.y = _r[3] * (model3dAll[j][i][0] - xOffset) +   _r[4] * (model3dAll[j][i][1] - yOffset) +  _r[5] * (model3dAll[j][i][2] - 61.1132392883301) + m_translation_vector[1];
				point3D.z = _r[6] * (model3dAll[j][i][0] - xOffset) +   _r[7] *(model3dAll[j][i][1] - yOffset) +  _r[8] * (model3dAll[j][i][2] - 61.1132392883301) + m_translation_vector[2];
				
				CvPoint2D32f point2D = cvPoint2D32f( 0.0, 0.0 ); 
				if ( point3D.z != 0 )  
				{  
					point2D.x = 492.3077* point3D.x / point3D.z + meaLmksX;   
					point2D.y = 492.3077 * point3D.y / point3D.z + meaLmksY;      
				} 
				projectPoints.push_back( point2D );
				m_correct.at<uchar>(j, i) = grayImage.at<uchar>(point2D.y, point2D.x);
			}
			
			
		}
	}
	t = (double)cvGetTickCount() - t;
	double tt = t/((double)cvGetTickFrequency()*1000.);
	//printf( "face mask time = %g ms\n",tt);
	
	for ( int p=0; p< projectPoints.size(); p++ )  
	{  
		circle(face, Point2f(projectPoints[p].x, projectPoints[p].y), 1, CV_RGB(255,255,255), -1);
	} 

	imshow("model3DPro", face);
	//imshow("correct", m_correct);
}

void CPoseEstimate::correctPoseByEye(Mat& pFrame)
{
	double* _r = m_rotMat_Eye.ptr<double>();

	Mat grayImage;
	cvtColor(pFrame, grayImage, cv::COLOR_BGR2GRAY);
	m_correct_eye = Mat(320, 320, CV_8U, Scalar(0)); 
	Mat face_Eye = Mat(480, 640, CV_8U, Scalar(0)); 

	float xOffset = -55.4203338623047;  
    float yOffset = -82.6790466308594;  
    float zOffset =	61.1132392883301;  
	
	vector <CvPoint2D32f>  projectPoints;
	
	for(int i = 0; i < 320; i++)
	{
		for(int j = 0; j < 320; j++)
		{
			if(m_faceMask.at<uchar>(j,i) != 255)
			{
				CvPoint3D32f point3D;  		
				point3D.x = _r[0] * (model3dAll[j][i][0] - xOffset) +   _r[1] * (model3dAll[j][i][1] - yOffset) +  _r[2] *(model3dAll[j][i][2] - 61.1132392883301) + m_translation_vector_Eye[0]; 
				point3D.y = _r[3] * (model3dAll[j][i][0] - xOffset) +   _r[4] * (model3dAll[j][i][1] - yOffset) +  _r[5] * (model3dAll[j][i][2] - 61.1132392883301) + m_translation_vector_Eye[1];
				point3D.z = _r[6] * (model3dAll[j][i][0] - xOffset) +   _r[7] *(model3dAll[j][i][1] - yOffset) +  _r[8] * (model3dAll[j][i][2] - 61.1132392883301) + m_translation_vector_Eye[2];
				
				CvPoint2D32f point2D = cvPoint2D32f( 0.0, 0.0 ); 
				if ( point3D.z != 0 )  
				{  
					point2D.x = 492.3077* point3D.x / point3D.z + meaLmksX_Eye;   
					point2D.y = 492.3077 * point3D.y / point3D.z + meaLmksY_Eye;      
				} 
				projectPoints.push_back( point2D );
				m_correct_eye.at<uchar>(j, i) = grayImage.at<uchar>(point2D.y, point2D.x);
			}
			
			
		}
	}
	
	for ( int p=0; p< projectPoints.size(); p++ )  
	{  
		circle(face_Eye, Point2f(projectPoints[p].x, projectPoints[p].y), 1, CV_RGB(255,255,255), -1);
	} 

	//imshow("model3DPro_Eye", face_Eye);
	//imshow("correct_Eye", m_correct_eye);
}

void CPoseEstimate::mergeFace()
{
	imshow("m_correct_ori", m_correct);	
	if(!m_correct.empty() && !m_correct_eye.empty())
	{
		Mat m_correct_flip = Mat(320, 320, CV_8U, Scalar(0));	
		if(m_yaw >= 0)
		{
			flip(m_correct, m_correct_flip, 1);
			//imshow("correct", m_correct);
			//imshow("m_correct_flip", m_correct_flip);			
			for(int i = 0; i < 320; i++)
			{
				for(int j = 0; j < 320; j++)
				{
						if(i < 160)
						{
							m_correct.at<uchar>(j,i) = m_correct_flip.at<uchar>(j,i);
						}
				}
			}
			//imshow("correctMerge", m_correct);
		}
		else
		{
			flip(m_correct, m_correct_flip, 1);
			//imshow("correct", m_correct);
			//imshow("m_correct_flip", m_correct_flip);
			for(int i = 0; i < 320; i++)
			{
				for(int j = 0; j < 320; j++)
				{
					if(i >= 160)
						m_correct.at<uchar>(j,i) = m_correct_flip.at<uchar>(j,i);
				}
			}
			//imshow("correctMerge", m_correct);
		}

		m_correct_final = Mat(320, 320, CV_8U, Scalar(0));	
		//m_correct.copyTo(m_correct_final);
		for(int y = 0; y < 320; y++)
		{
			for(int x = 0; x < 320; x++)
			{						
				m_correct_final.at<uchar>(y,x) = int(m_correct.at<uchar>(y,x)*(1-m_eyeMask.at<float>(y,x)) + m_correct_eye.at<uchar>(y,x)*m_eyeMask.at<float>(y,x) + 0.5);
							
			}
		}
		m_correct_flip.release();
	}
}
void CPoseEstimate::drawAxis(Mat& pFrame)
{	
		
	double* _r = m_rotMat.ptr<double>();	
	vector< Point3f > axis(4);
	axis[0].x = 50;
	axis[0].y = 0;
	axis[0].z = 0;

	axis[1].x = 0;
	axis[1].y = -50;
	axis[1].z = 0;

	axis[2].x = 0;
	axis[2].y = 0;
	axis[2].z = 50;

	axis[3].x = 0;
	axis[3].y = 0;
	axis[3].z = 0;

	
	vector <CvPoint3D32f>  ptsAxis;
	for ( size_t  p=0; p< axis.size(); p++ )  
	{  
		CvPoint3D32f point3D;  
		
		point3D.x = _r[0] * axis[p].x +   _r[1] * axis[p].y +  _r[2] * axis[p].z + 50; 
		point3D.y = _r[3] * axis[p].x +   _r[4] * axis[p].y +  _r[5] * axis[p].z + 50;
		point3D.z = _r[6] * axis[p].x +   _r[7] * axis[p].y +  _r[8] * axis[p].z + 50;
				
		ptsAxis.push_back(point3D);
	} 
	line(pFrame, Point(ptsAxis[3].x, ptsAxis[3].y), Point(ptsAxis[0].x, ptsAxis[0].y), CV_RGB(255,0,0), 2);
	line(pFrame, Point(ptsAxis[3].x, ptsAxis[3].y), Point(ptsAxis[1].x, ptsAxis[1].y), CV_RGB(0,255,0), 2);
	line(pFrame, Point(ptsAxis[3].x, ptsAxis[3].y), Point(ptsAxis[2].x, ptsAxis[2].y), CV_RGB(0,0,255), 2);
	
	vector <CvPoint2D32f>  ptsModels;
	for ( size_t  p=0;  p < m_modelPtsPOIST.size(); p++ )  
	{  
		CvPoint3D32f point3D;  
		
		point3D.x = _r[0] * m_modelPtsPOIST[p].x +   _r[1] * m_modelPtsPOIST[p].y +  _r[2] * m_modelPtsPOIST[p].z + m_translation_vector[0]; 
		point3D.y = _r[3] * m_modelPtsPOIST[p].x +   _r[4] * m_modelPtsPOIST[p].y +  _r[5] * m_modelPtsPOIST[p].z + m_translation_vector[1];
		point3D.z = _r[6] * m_modelPtsPOIST[p].x +   _r[7] * m_modelPtsPOIST[p].y +  _r[8] * m_modelPtsPOIST[p].z + m_translation_vector[2];
				
		CvPoint2D32f point2D = cvPoint2D32f( 0.0, 0.0 ); 
		if ( point3D.z != 0 )  
        {  
            point2D.x = 492.3077 * point3D.x / point3D.z + meaLmksX;   
            point2D.y = 492.3077 * point3D.y / point3D.z + meaLmksY;      
        } 
		ptsModels.push_back( point2D );  
		//circle(pFrame, point2D, 1, CV_RGB(255,255,255), -1);
	} 
	//printf("landmarks0 pro is %f %f\n", ptsModels[0].x, ptsModels[0].y);
	//printf("landmarks48 pro is %f %f\n", ptsModels[48].x, ptsModels[48].y);
	
	
	stringstream ss;
	ss << m_pitch;
	string txt = "Pitch: " + ss.str();
	putText(pFrame, txt,  Point(100, 20), 1, 1.3, Scalar(0,255,0), 2);
	stringstream ss1;
	ss1 << m_yaw;
	string txt1 = "Yaw: " + ss1.str();
	putText(pFrame, txt1,  Point(100, 40), 1, 1.3, Scalar(0,255,0), 2);
	stringstream ss2;
	ss2 << m_roll;
	string txt2 = "Roll: " + ss2.str();
	putText(pFrame, txt2,  Point(100, 60), 1, 1.3, Scalar(0,255,0), 2);

}

void CPoseEstimate::drawAxis_Eye(Mat& pFrame)
{	
	//for ( int p=0; p< 320; p++ )  
	//{  
	//	for ( int q=0; q< 320; q++ )  
	//	{
	//		for(int r = 0; r < 49; r++)
	//		{
	//			CvPoint2D32f point2D = cvPoint2D32f( 0.0, 0.0 ); 		
	//			if(abs(model3dAll[p][q][0] - model_49[r][0]) < 0.1 && abs(model3dAll[p][q][1] - model_49[r][1]) < 0.1 && abs(model3dAll[p][q][2] - model_49[r][2]) < 0.1)
	//			{					
	//				circle(m_correct, Point( q,  p), 2, CV_RGB(255,255,255), -1);
	//				//printf("%d %d \n",q,p);
	//			}
	//			
	//		} 

	//	}
	//	
	//} 

	//for ( int p=0; p< 320; p++ )  
	//{  
	//	for ( int q=0; q< 320; q++ )  
	//	{
	//		for(int r = 0; r < 49; r++)
	//		{
	//			CvPoint2D32f point2D = cvPoint2D32f( 0.0, 0.0 ); 		
	//			if(abs(model3dAll[p][q][0] - model_49[r][0]) < 0.1 && abs(model3dAll[p][q][1] - model_49[r][1]) < 0.1 && abs(model3dAll[p][q][2] - model_49[r][2]) < 0.1)
	//			{					
	//				circle(m_correct_eye, Point( q,  p), 2, CV_RGB(255,255,255), -1);
	//			}
	//			
	//		} 

	//		/*for(int r = 19; r < 31; r++)
	//		{
	//			CvPoint2D32f point2D = cvPoint2D32f( 0.0, 0.0 ); 		
	//			if(abs(model3dAll[p][q][0] - model_49[r][0]) < 0.1 && abs(model3dAll[p][q][1] - model_49[r][1]) < 0.1 && abs(model3dAll[p][q][2] - model_49[r][2]) < 0.1)
	//			{					
	//				circle(m_correct_eye, Point( q,  p), 2, CV_RGB(255,255,255), -1);
	//			}
	//			
	//		} */

	//	}
	//	
	//} 
	
	double* _r = m_rotMat_Eye.ptr<double>();	
	vector< Point3f > axis(4);
	axis[0].x = 50;
	axis[0].y = 0;
	axis[0].z = 0;

	axis[1].x = 0;
	axis[1].y = -50;
	axis[1].z = 0;

	axis[2].x = 0;
	axis[2].y = 0;
	axis[2].z = 50;

	axis[3].x = 0;
	axis[3].y = 0;
	axis[3].z = 0;

	
	vector <CvPoint3D32f>  ptsAxis;
	for ( size_t  p=0; p< axis.size(); p++ )  
	{  
		CvPoint3D32f point3D;  
		
		point3D.x = _r[0] * axis[p].x +   _r[1] * axis[p].y +  _r[2] * axis[p].z + 50; 
		point3D.y = _r[3] * axis[p].x +   _r[4] * axis[p].y +  _r[5] * axis[p].z + 50;
		point3D.z = _r[6] * axis[p].x +   _r[7] * axis[p].y +  _r[8] * axis[p].z + 50;
				
		ptsAxis.push_back(point3D);
	} 
	line(pFrame, Point(ptsAxis[3].x, ptsAxis[3].y), Point(ptsAxis[0].x, ptsAxis[0].y), CV_RGB(255,0,0), 2);
	line(pFrame, Point(ptsAxis[3].x, ptsAxis[3].y), Point(ptsAxis[1].x, ptsAxis[1].y), CV_RGB(0,255,0), 2);
	line(pFrame, Point(ptsAxis[3].x, ptsAxis[3].y), Point(ptsAxis[2].x, ptsAxis[2].y), CV_RGB(0,0,255), 2);
	
	vector <CvPoint2D32f>  ptsModels;
	for ( size_t  p=0;  p < m_modelPtsPOIST_Eye.size(); p++ )  
	{  
		CvPoint3D32f point3D;  
		
		point3D.x = _r[0] * m_modelPtsPOIST_Eye[p].x +   _r[1] * m_modelPtsPOIST_Eye[p].y +  _r[2] * m_modelPtsPOIST_Eye[p].z + m_translation_vector_Eye[0]; 
		point3D.y = _r[3] * m_modelPtsPOIST_Eye[p].x +   _r[4] * m_modelPtsPOIST_Eye[p].y +  _r[5] * m_modelPtsPOIST_Eye[p].z + m_translation_vector_Eye[1];
		point3D.z = _r[6] * m_modelPtsPOIST_Eye[p].x +   _r[7] * m_modelPtsPOIST_Eye[p].y +  _r[8] * m_modelPtsPOIST_Eye[p].z + m_translation_vector_Eye[2];
				
		CvPoint2D32f point2D = cvPoint2D32f( 0.0, 0.0 ); 
		if ( point3D.z != 0 )  
        {  
            point2D.x = 492.3077 * point3D.x / point3D.z + meaLmksX_Eye;   
            point2D.y = 492.3077 * point3D.y / point3D.z + meaLmksY_Eye;      
        } 
		ptsModels.push_back( point2D );  
	} 	
	//printf("landmarks0 pro is %f %f\n", ptsModels[0].x, ptsModels[0].y);
	
	
	stringstream ss;
	ss << m_pitch_Eye;
	string txt = "Pitch: " + ss.str();
	putText(pFrame, txt,  Point(100, 20), 1, 1.3, Scalar(0,255,0), 2);
	stringstream ss1;
	ss1 << m_yaw_Eye;
	string txt1 = "Yaw: " + ss1.str();
	putText(pFrame, txt1,  Point(100, 40), 1, 1.3, Scalar(0,255,0), 2);
	stringstream ss2;
	ss2 << m_roll_Eye;
	string txt2 = "Roll: " + ss2.str();
	putText(pFrame, txt2,  Point(100, 60), 1, 1.3, Scalar(0,255,0), 2);

}

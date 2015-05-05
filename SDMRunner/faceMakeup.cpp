#include "faceMakeup.h"

#define SQR(x) x*x	
void WarpPoint2Mat(cv::Mat& res, vector< cv::Point2f > &Points)
{
	int nPoints = Points.size();
	Mat shape(nPoints*2, 1, CV_32F);
	
	for (int i = 0; i < nPoints ; i++)
    {
        shape.at<float>(i, 0) = Points[i].x;
        shape.at<float>(i + nPoints, 0) = Points[i].y;
		
	}
	shape.copyTo(res);
}



void WarpFL2DB(cv::Mat& src, cv::Mat& res)
{
	int nPoints = src.rows / 2;
	Mat shape(nPoints*2, 1, CV_64F);
	
	for (int i = 0; i < nPoints ; i++)
    {
        shape.at<double>(i, 0) = src.at<float>(i, 0);
        shape.at<double>(i + nPoints, 0) = src.at<float>(i + nPoints, 0);
		
	}
	//Mat tmp(shape);
	shape.copyTo(res);
}


void Mat2WarpPoint(cv::Mat& res, vector< cv::Point > &Points)
{
	int nPoints = res.rows / 2;
	if(Points.size() != nPoints)		
		Points.resize(nPoints);
	for (int i = 0; i < nPoints ; i++)
    {
		Points[i].x = int (res.at<float>(i, 0) + 0.5);
        Points[i].y = int (res.at<float>(i + nPoints, 0) + 0.5);
	}
}


void calcSimT(cv::Mat &src,cv::Mat &dst,
	      float &a,float &b,float &tx,float &ty)
{
  assert((src.type() == CV_32F) && (dst.type() == CV_32F) && 
	 (src.rows == dst.rows) && (src.cols == dst.cols) && (src.cols == 1));
  int i,n = src.rows/2;
  cv::Mat H(4,4,CV_32F,cv::Scalar(0));
  cv::Mat g(4,1,CV_32F,cv::Scalar(0));
  cv::Mat p(4,1,CV_32F);
  cv::MatIterator_<float> ptr1x = src.begin<float>();
  cv::MatIterator_<float> ptr1y = src.begin<float>()+n;
  cv::MatIterator_<float> ptr2x = dst.begin<float>();
  cv::MatIterator_<float> ptr2y = dst.begin<float>()+n;
  for(i = 0; i < n; i++,++ptr1x,++ptr1y,++ptr2x,++ptr2y){
    H.at<float>(0,0) += SQR(*ptr1x) + SQR(*ptr1y);
    H.at<float>(0,2) += *ptr1x; H.at<float>(0,3) += *ptr1y;
    g.at<float>(0,0) += (*ptr1x)*(*ptr2x) + (*ptr1y)*(*ptr2y);
    g.at<float>(1,0) += (*ptr1x)*(*ptr2y) - (*ptr1y)*(*ptr2x);
    g.at<float>(2,0) += *ptr2x; g.at<float>(3,0) += *ptr2y;
  }
  H.at<float>(1,1) = H.at<float>(0,0); H.at<float>(1,2) = H.at<float>(2,1) = -1.0*(H.at<float>(3,0) = H.at<float>(0,3));
  H.at<float>(1,3) = H.at<float>(3,1) = H.at<float>(2,0) = H.at<float>(0,2); H.at<float>(2,2) = H.at<float>(3,3) = n;
  cv::solve(H,g,p,CV_CHOLESKY);
  a = p.at<float>(0,0); b = p.at<float>(1,0); tx = p.at<float>(2,0); ty = p.at<float>(3,0); return;
}


void simT(cv::Mat &s,float a,float b,float tx,float ty)
{
  assert((s.type() == CV_32F) && (s.cols == 1));
  int i,n = s.rows/2; float x,y; 
  cv::MatIterator_<float> xp = s.begin<float>(),yp = s.begin<float>()+n;
 
  for(i = 0; i < n; i++,++xp,++yp)
  {
    x = *xp; 
	y = *yp; 
	*xp = a*x - b*y + tx; 
	*yp = b*x + a*y + ty;    
  }
  
  return;
}

void Dilate(Mat& img, Mat& dst)
{
 
  Mat element = getStructuringElement(MORPH_ELLIPSE,Size(3, 3),Point( 0, 0));
  dilate(img, dst, element, Point(-1, -1), 1);
 // imshow( "Erosion Demo", dst);
}

void Erode(Mat& img, Mat& dst)
{
 
  Mat element = getStructuringElement(MORPH_ELLIPSE,Size(2, 2),Point( 0, 0));
  erode(img, dst, element, Point(-1, -1), 1);
  //imshow( "Erosion Demo", dst);
}


FaceMakeUp::FaceMakeUp()
{
	m_warp = new ImgWarp_PieceWiseAffine;
	((ImgWarp_PieceWiseAffine *) m_warp)->backGroundFillAlg = ImgWarp_PieceWiseAffine::BGMLS;	

	m_imgLipMask = Mat(480, 640, CV_8U, Scalar(0));
	m_imgEyesMask = Mat(480, 640, CV_8U, Scalar(0));
}
FaceMakeUp::~FaceMakeUp()
{
	delete m_warp;
	m_imgLipMask.release();
	m_imgEyesMask.release();
}

void FaceMakeUp::loadFaceModels(string& pthModels)
{
	ifstream ifsInputPth;
	ifsInputPth.open(pthModels.c_str());
	if(!ifsInputPth.is_open())
	{
		std::cout << "inputName file open error" << endl;
		return ;
	}
	int nLinesInputName = 0;
	std::string strLine;
	while(getline(ifsInputPth,strLine))
	{ 		
		std::string temp(strLine);
		m_pathModels.push_back(temp);
	}  
	ifsInputPth.close();

}

int FaceMakeUp::getNumModels()
{
	return m_refLipROIs.size();
}
void FaceMakeUp::getRefsMasks(SDMLocator* locator, HPFaceDetectorWarpper* faceDetector, Mat& meanShape)
{
	if(m_pathModels.size() <= 0)
	{
		printf("no models\n");
		return;
	}
	Mat meanShapeCpy;
	meanShape.copyTo(meanShapeCpy);
	for(int i = 0; i < m_pathModels.size(); i++)
	{
		Mat ref = cv::imread(m_pathModels[i]);
		Mat refGray;
		cvtColor(ref, refGray, cv::COLOR_BGR2GRAY);
		FaceArray faces;
		faceDetector->DetectFaces(refGray.data, refGray.cols, refGray.rows, faces);
		Rect rFace;
		rFace.x = max(0, faces[0].centerX - faces[0].radius);
		rFace.y = max(0, faces[0].centerY - faces[0].radius);
		if(rFace.x + 2*faces[0].radius > refGray.cols)
			rFace.width = refGray.cols - rFace.x;
		else
			rFace.width = 2*faces[0].radius;
		if(rFace.y + 2*faces[0].radius > refGray.rows)
			rFace.height = refGray.rows- rFace.y;
		else
			rFace.height = 2*faces[0].radius;

		if(!locator->detect49(meanShape, refGray, rFace))
		{
			printf("no landmarks found!\n");
			return;
		}
		int getNumLandmarks = meanShape.rows /2;
		std::vector<cv::Point2f> vecRefLMs;
		for (int j = 0; j < getNumLandmarks; ++j) 
		{
			vecRefLMs.push_back(Point2f(meanShape.at<float>(j, 0), meanShape.at<float>(j + getNumLandmarks, 0)));
			//cv::circle(ref, Point2f(meanShape.at<float>(j, 0), meanShape.at<float>(j + getNumLandmarks, 0)), 1, Scalar(0.0f, 255.0f, 0.0f), -1);
		}

		vector <Point2f> lipOutline;
		for(int i = 0; i < 12; i++)
		{
			cv::Point2f p1;
			p1 = Point2f(vecRefLMs[31 + i].x, vecRefLMs[31 + i].y);
			lipOutline.push_back(p1);
		}
		Rect rLip = boundingRect(lipOutline);
		//rectangle( ref, rLip.tl(), rLip.br(), CV_RGB(255,255,255), 2, 8, 0 ); 
		Mat refLipROI(ref, rLip);
		Mat tempRefLip;
		refLipROI.copyTo(tempRefLip);
		m_refLipROIs.push_back(tempRefLip);
		//circle(refLipROI, Point(6,6), 3, CV_RGB(255,255,255), 2, 8, 0 ); 
			
		
		std::vector<cv::Point2f> vecRefROILMs;
		for(int i = 0; i < 18; i++)
		{
			cv::Point2f p1;
			p1 = Point2f(vecRefLMs[31 + i].x - rLip.x, vecRefLMs[31 + i].y - rLip.y);
			vecRefROILMs.push_back(p1);
			//circle(refROI, p1, 1, CV_RGB(255,255,255), -1, 8, 0 ); 
		}
		m_vecRefLipROILMs.push_back(vecRefROILMs);
		//imshow("ref", ref);
		//waitKey(0);



		vector<Point2f> eyesOutline;
		for(int i = 0; i < 31; i++ )
		{
			Point2f pt;
			pt.x = vecRefLMs[i].x;
			pt.y = vecRefLMs[i].y;
			eyesOutline.push_back(pt);
		}
		/*for(int i = 19; i < 31; i++)
		{
			Point2f pt;
			pt.x = vecRefLMs[i].x;
			pt.y = vecRefLMs[i].y;
			eyesOutline.push_back(pt);
		}	*/
		Rect rEyes = boundingRect(eyesOutline);
		Mat refEyesROITmp(ref, rEyes);
		Mat refEyesROI;
		refEyesROITmp.copyTo(refEyesROI);
		m_refEyesROIs.push_back(refEyesROI);
		
		std::vector<cv::Point2f> vecEyesRefROILMs;
		for(int i = 0; i < 31; i++)
		{
			Point2f pt;
			pt.x = eyesOutline[i].x - rEyes.x;
			pt.y = eyesOutline[i].y - rEyes.y;
			vecEyesRefROILMs.push_back(pt);
			//circle(refEyesROI, pt, 1, CV_RGB(255,255,255), -1, 8, 0 );
		}
		m_vecEyesRefROILMs.push_back(vecEyesRefROILMs);
		//imshow("refEyesROI", refEyesROI);
		//waitKey(0);
				
		meanShapeCpy.copyTo(meanShape);
	}

}

void FaceMakeUp::getLipMask(Mat& img, std::vector<cv::Point2f>& vecLMs)
{

	vector <Point2f> lipOutline;
	for(int i = 0; i < 12; i++)
	{
		cv::Point2f p1;
		p1 = Point2f(vecLMs[31 + i].x, vecLMs[31 + i].y);
		lipOutline.push_back(p1);
	}
	Rect rMask= boundingRect(lipOutline);
	if(rMask.x < 0)
		rMask.x = 0;
	if(rMask.y < 0)
		rMask.y = 0;
	if(rMask.x + rMask.width > img.cols)
		rMask.width = img.cols - rMask.x;
	if(rMask.y + rMask.height > img.rows)
		rMask.height = img.rows - rMask.y;

	//Mat lipMask = Mat(480, 640, CV_8U, Scalar(0));
	//Mat lipMask = Mat(240, 320, CV_8U, Scalar(0));
	m_imgLipMask.setTo(0);	
	Point2f ptsUpperLip[10];
	Point2f ptsLowerLip[10];
	
	ptsUpperLip[0] = Point2f(vecLMs[31].x, vecLMs[31].y);
	ptsUpperLip[1] = Point2f(vecLMs[32].x, vecLMs[32].y);
	ptsUpperLip[2] = Point2f(vecLMs[33].x, vecLMs[33].y);
	ptsUpperLip[3] = Point2f(vecLMs[34].x, vecLMs[34].y);
	ptsUpperLip[4] = Point2f(vecLMs[35].x, vecLMs[35].y);
	ptsUpperLip[5] = Point2f(vecLMs[36].x, vecLMs[36].y);
	ptsUpperLip[6] = Point2f(vecLMs[37].x, vecLMs[37].y);
	ptsUpperLip[7] = Point2f(vecLMs[45].x, vecLMs[45].y);
	ptsUpperLip[8] = Point2f(vecLMs[44].x, vecLMs[44].y);
	ptsUpperLip[9] = Point2f(vecLMs[43].x, vecLMs[43].y);


	ptsLowerLip[0] = vecLMs[37];
	ptsLowerLip[1] = vecLMs[38];
	ptsLowerLip[2] = vecLMs[39];
	ptsLowerLip[3] = vecLMs[40];
	ptsLowerLip[4] = vecLMs[41];
	ptsLowerLip[5] = vecLMs[42];
	ptsLowerLip[6] = vecLMs[31];
	ptsLowerLip[7] = vecLMs[48];
	ptsLowerLip[8] = vecLMs[47];
	ptsLowerLip[9] = vecLMs[46];

	vector<Point2f> ptsUpperLipVec;
	for(int i = 0; i < 10; i++)
	{
		ptsUpperLipVec.push_back(ptsUpperLip[i]);
	}

	vector<Point2f> ptsLowerLipVec;
	for(int i = 0; i < 10; i++)
	{
		ptsLowerLipVec.push_back(ptsLowerLip[i]);
	}
	
	//double t = (double)cvGetTickCount();
	
	
	Rect rLip = boundingRect(ptsUpperLipVec);
	int x = rLip.x;
	int y = rLip.y;
	int wd = rLip.width;
	int ht = rLip.height;
	
	//Mat_<Vec3b> _I = img;
	for(int i = y; i < y + ht; i++)
	{
		for(int j = x; j < x + wd; j++)
		{
			if(pointPolygonTest(ptsUpperLipVec, Point2f(j, i), false) != -1)
			{
				m_imgLipMask.at<uchar>(i, j) = 255;
			}
			
		}

	}

	rLip = boundingRect(ptsLowerLipVec);
	x = rLip.x;
	y = rLip.y;
	wd = rLip.width;
	ht = rLip.height;
	for(int i = y; i < y + ht; i++)
	{
		for(int j = x; j < x + wd; j++)
		{
			
			if(pointPolygonTest(ptsLowerLipVec, Point2f(j, i), false) != -1)
			{
				m_imgLipMask.at<uchar>(i, j) = 255;
				
			}
			
		}

	}
	
	//t = (double)cvGetTickCount() - t;
	//double tt = t/((double)cvGetTickFrequency()*1000.);
	//printf( "lip mask time = %g ms\n", tt);
	//rectangle( lipMask, rLip.tl(), rLip.br(), CV_RGB(255,255,255), 2, 8, 0 ); 
	//imshow("lipMask", lipMask);
	
	Mat mskROI(m_imgLipMask, rMask);
	mskROI.copyTo(m_imgLipROI);
	//Dilate(maskROI, maskROI);
	//Erode(maskROI, maskROI);
	int ww = 2*(rMask.width/2/3)-1;
	int hh =  2*(rMask.height/2/3)-1;
	if(ww > 0 && hh > 0)
		GaussianBlur(m_imgLipROI, m_imgLipROI, Size(ww, hh), 0, 0 /*BORDER_REPLICATE*/);
	//GaussianBlur(maskROI, maskROI, Size(7, 7), 5, 5, BORDER_REPLICATE);
	//printf("%d %d\n", 2*(rMask.width/2/5)-1, 2*(rMask.height/2/5)-1);
	//imshow("m_imgLipROI", m_imgLipROI);
}

void FaceMakeUp::makeupLips(Mat& img,vector<cv::Point2f>& vecImgLMs, int indModel)
{
	if(vecImgLMs.size() != 49)
		return;

	vector <Point2f> lipOutline;
	for(int i = 0; i < 12; i++)
	{
		cv::Point2f p1;
		p1 = Point2f(vecImgLMs[31 + i].x, vecImgLMs[31 + i].y);
		lipOutline.push_back(p1);
	}
	Rect rLip = boundingRect(lipOutline);
	if(rLip.x < 0)
		rLip.x = 0;
	if(rLip.y < 0)
		rLip.y = 0;
	if(rLip.x + rLip.width > img.cols)
		rLip.width = img.cols - rLip.x;
	if(rLip.y + rLip.height > img.rows)
		rLip.height = img.rows - rLip.y;

	Mat imgLipROI(img, rLip);
	Mat imgLipROICpy;
	imgLipROI.copyTo(imgLipROICpy);
	
	vector <Point2f> lipROIOutline;
	vector <Point> lipIntROIOutline;
	vector<cv::Point> vecIntRefLipLMs;
	for(int i = 0; i < 18; i++)
	{
		cv::Point2f p1;
		p1 = Point2f(vecImgLMs[31 + i].x - rLip.x, vecImgLMs[31 + i].y - rLip.y);
		lipROIOutline.push_back(p1);
		//circle(imgLipROI, p1, 1, CV_RGB(255,255,255), -1, 8, 0 ); 
		lipIntROIOutline.push_back(Point(p1.x, p1.y));
		vecIntRefLipLMs.push_back(Point(int(m_vecRefLipROILMs[indModel][i].x+0.5), int(m_vecRefLipROILMs[indModel][i].y+0.5)));
	}
	
	float a1,b1,tx1,ty1;
	Mat refShape, imgShape;
	WarpPoint2Mat(refShape, m_vecRefLipROILMs[indModel]);
	WarpPoint2Mat(imgShape, lipROIOutline);
	calcSimT(refShape, imgShape, a1,b1,tx1,ty1);//cal sim transform
	simT(refShape, a1,b1,tx1,ty1);

	vector< cv::Point > refNewPoints;
	Mat2WarpPoint(refShape, refNewPoints);

	m_warp->setTargetSize(rLip.width, rLip.height);
    m_warp->setSize(m_refLipROIs[indModel].cols, m_refLipROIs[indModel].rows);
    m_warp->setSrcPoints(vecIntRefLipLMs);
    m_warp->setDstPoints(refNewPoints);
    m_warp->alpha = 1;
    m_warp->gridSize = 5;
	m_warp->calcDelta(); 
	Mat dstImg = m_warp->genNewImg(m_refLipROIs[indModel], 1);
	
	
	m_warp->setTargetSize(rLip.width, rLip.height);
    m_warp->setSize(rLip.width, rLip.height);
    m_warp->setSrcPoints(refNewPoints);
    m_warp->setDstPoints(lipIntROIOutline);
    m_warp->alpha = 1;
    m_warp->gridSize = 5;
	m_warp->calcDelta(); 
	Mat dstImg1 = m_warp->genNewImg(dstImg, 1);
	
	//imshow("imgLipROI", imgLipROI);
	//imshow("dstImg", dstImg);
	//imshow("dstImg1", dstImg1);

	Mat refLabColorSpace;
	std::vector<cv::Mat> refLab;
	cvtColor(dstImg1, refLabColorSpace, CV_BGR2Lab);
	split(refLabColorSpace, refLab);
	
	/*imshow("L", refLab[0]);
	imshow("A", refLab[1]);
	imshow("B", refLab[2]);
	imshow("refLabColorSpace", refLabColorSpace);*/


	Mat imgLabColorSpace;
	std::vector<cv::Mat> imgLab;
	cvtColor(imgLipROICpy, imgLabColorSpace, CV_BGR2Lab);
	split(imgLabColorSpace, imgLab);
	
	/*imshow("imgL", imgLab[0]);
	imshow("imgA", imgLab[1]);
	imshow("imgB", imgLab[2]);
	imshow("imgLabColorSpace", imgLabColorSpace);*/

	int wd = imgLabColorSpace.cols;
	int ht = imgLabColorSpace.rows;
	for(int i = 0; i < ht; i++)
	{
		for(int j = 0; j < wd; j++)
		{
			if(m_imgLipROI.at<uchar>(i, j) != 0)
			{
				imgLab[0].at<uchar>(i,j) = int(imgLab[0].at<uchar>(i,j)*0.7 + refLab[0].at<uchar>(i, j)*0.3 + 0.5);
				imgLab[1].at<uchar>(i,j) = int(imgLab[1].at<uchar>(i,j)*0.1 + refLab[1].at<uchar>(i, j)*0.9 + 0.5);
				imgLab[2].at<uchar>(i,j) = int(imgLab[2].at<uchar>(i,j)*0.1 + refLab[2].at<uchar>(i, j)*0.9 + 0.5);
			}
		}
	}

	merge(imgLab, imgLabColorSpace);
	cvtColor(imgLabColorSpace, imgLipROICpy, CV_Lab2BGR);
	//imgLipROICpy.copyTo(imgLipROI);
	Mat_<Vec3b> img_LipROICpy = imgLipROICpy;
	Mat_<Vec3b> img_LipROI = imgLipROI;
	for(int i = 0; i < ht; i++)
	{
		for(int j = 0; j < wd; j++)
		{
			if(m_imgLipROI.at<uchar>(i, j) != 0)
			{
				img_LipROI(i, j)[0]  = int(img_LipROICpy(i, j)[0]*(1.0*(m_imgLipROI.at<uchar>(i, j))/255) + img_LipROI(i, j)[0]*(1 - (1.0*m_imgLipROI.at<uchar>(i, j))/255) + 0.5);	
				img_LipROI(i, j)[1]  = int(img_LipROICpy(i, j)[1]*(1.0*(m_imgLipROI.at<uchar>(i, j))/255) + img_LipROI(i, j)[1]*(1 - (1.0*m_imgLipROI.at<uchar>(i, j))/255) + 0.5);	
				img_LipROI(i, j)[2]  = int(img_LipROICpy(i, j)[2]*(1.0*(m_imgLipROI.at<uchar>(i, j))/255) + img_LipROI(i, j)[2]*(1 - (1.0*m_imgLipROI.at<uchar>(i, j))/255) + 0.5);	
			}
		}
	}
		
}

void FaceMakeUp::makeupEyes(Mat& img,vector<cv::Point2f>& vecLMs, int indModel)
{
	//Mat eyesMask = Mat(480, 640, CV_8U, Scalar(0));
	//Mat eyesMask = Mat(240, 320, CV_8U, Scalar(0));
	m_imgEyesMask.setTo(0);

	vector<Point2f> ptsLeftEyeVec;
	vector<Point2f> ptsRightEyeVec;
	vector<Point2f> eyesImgOutline;
	for(int i = 0; i < 31; i++ )
	{
		Point2f pt;
		pt.x = vecLMs[i].x;
		pt.y = vecLMs[i].y;
		eyesImgOutline.push_back(pt);
	}
	
	Rect rEyes = boundingRect(eyesImgOutline);
	rEyes.x = max(0, rEyes.x);
	rEyes.y = max(0, rEyes.y);
	if(rEyes.x + rEyes.width > m_imgEyesMask.cols)
		rEyes.width = m_imgEyesMask.cols - rEyes.x;
	if(rEyes.y + rEyes.height > m_imgEyesMask.rows)
		rEyes.height = m_imgEyesMask.rows - rEyes.y;

	//rectangle(img, rEyes, Scalar(255,255,255));
	Mat mskROI(m_imgEyesMask, rEyes);
	mskROI.copyTo(m_imgEyesROI);

	vector<Point> eyesIntImgROILMs;
	vector<Point2f> eyesImgROILMs;
	vector<Point> eyeIntRefROILMs;
	for(int i = 0; i < 31; i++ )
	{
		Point pt;
		pt.x = int(vecLMs[i].x - rEyes.x + 0.5);
		pt.y = int(vecLMs[i].y - rEyes.y + 0.5);
		eyesIntImgROILMs.push_back(pt);

		Point2f pt1;
		pt1.x = vecLMs[i].x - rEyes.x;
		pt1.y = vecLMs[i].y - rEyes.y;
		eyesImgROILMs.push_back(pt1);
		
		Point pt2;
		pt2.x = int(m_vecEyesRefROILMs[indModel][i].x + 0.5);
		pt2.y = int(m_vecEyesRefROILMs[indModel][i].y + 0.5);
		eyeIntRefROILMs.push_back(pt2);
	}
	
	Point2f leftEye;
	Point2f rightEye;
	float xSum = 0;
	float ySum = 0;
	for(int i = 0; i < 6; i++)
	{
		xSum = xSum + (vecLMs[i + 19].x - rEyes.x);
		ySum = ySum + (vecLMs[i + 19].y - rEyes.y);
		Point2f pt(vecLMs[i + 19].x - rEyes.x, vecLMs[i + 19].y - rEyes.y);
		ptsLeftEyeVec.push_back(pt);	
	}
	leftEye.x = xSum / 6;
	leftEye.y = ySum / 6;

	xSum = 0;
	ySum = 0;
	for(int i = 0; i < 6; i++)
	{
		xSum = xSum + (vecLMs[i + 25].x - rEyes.x);
		ySum = ySum + (vecLMs[i + 25].y - rEyes.y);
		Point2f pt(vecLMs[i + 25].x - rEyes.x, vecLMs[i + 25].y - rEyes.y);
		ptsRightEyeVec.push_back(pt);		
	}
	rightEye.x = xSum / 6;
	rightEye.y = ySum / 6;
	
	int leftRadius = int (0.65*sqrt((leftEye.x - vecLMs[0].x + rEyes.x)*(leftEye.x - vecLMs[0].x + rEyes.x) + (leftEye.y - vecLMs[0].y + rEyes.y)*(leftEye.y - vecLMs[0].y + rEyes.y)) + 0.5);
	int rightRadius = int (0.65*sqrt((rightEye.x - vecLMs[9].x + rEyes.x)*(rightEye.x - vecLMs[9].x + rEyes.x) + (rightEye.y - vecLMs[9].y + rEyes.y)*(rightEye.y - vecLMs[9].y + rEyes.y)) + 0.5);

	//ellipse();
	circle(m_imgEyesROI, leftEye, leftRadius, Scalar(255,255,255), -1);
	circle(m_imgEyesROI, rightEye, rightRadius, Scalar(255,255,255), -1);
	int ww = 2*(rEyes.width/2/3)-1;
	int hh =  2*(rEyes.height/2/3)-1;
	if(ww > 0 && hh > 0)
		GaussianBlur(m_imgEyesROI, m_imgEyesROI, Size(2*(rEyes.width/2/3)-1, 2*(rEyes.height/2/3)-1), 0, 0, BORDER_REPLICATE);
	//GaussianBlur(maskEyesROI, maskEyesROI, Size(21,21), 21, 21, BORDER_REPLICATE);
	
	
	Mat eyeImgROI;
	Mat imgROI(img, rEyes);
	imgROI.copyTo(eyeImgROI);
	//imshow("eyeImgROI", eyeImgROI);

	int wd = rEyes.width;
	int ht = rEyes.height;
	for(int i = 0; i < ht; i++)
	{
		for(int j = 0; j < wd; j++)
		{
			if(pointPolygonTest(ptsLeftEyeVec, Point2f(j, i), false) != -1 || pointPolygonTest(ptsRightEyeVec, Point2f(j, i), false) != -1)
			{
				m_imgEyesROI.at<uchar>(i, j) = 0;
			}
			
		}

	}
	GaussianBlur(m_imgEyesROI, m_imgEyesROI, Size(3, 3),3);
	//imshow("maskEyesROI", m_imgEyesROI);


	float a1,b1,tx1,ty1;
	Mat refEyeShape, imgEyeShape;
	WarpPoint2Mat(refEyeShape, m_vecEyesRefROILMs[indModel]);
	WarpPoint2Mat(imgEyeShape, eyesImgROILMs);
	calcSimT(refEyeShape, imgEyeShape, a1,b1,tx1,ty1);//cal sim transform
	simT(refEyeShape, a1,b1,tx1,ty1);

	vector< cv::Point > refEyeNewShape;
	Mat2WarpPoint(refEyeShape, refEyeNewShape);

	m_warp->setTargetSize(eyeImgROI.cols, eyeImgROI.rows);
    m_warp->setSize(m_refEyesROIs[indModel].cols, m_refEyesROIs[indModel].rows);
    m_warp->setSrcPoints(eyeIntRefROILMs);
    m_warp->setDstPoints(refEyeNewShape);
    m_warp->alpha = 1;
    m_warp->gridSize = 5;
	m_warp->calcDelta(); 
	Mat dstImgEye = m_warp->genNewImg(m_refEyesROIs[indModel], 1);
	//
	//
	m_warp->setTargetSize(eyeImgROI.cols, eyeImgROI.rows);
    m_warp->setSize(eyeImgROI.cols, eyeImgROI.rows);
    m_warp->setSrcPoints(refEyeNewShape);
    m_warp->setDstPoints(eyesIntImgROILMs);
    m_warp->alpha = 1;
    m_warp->gridSize = 5;
	m_warp->calcDelta(); 
	Mat dstImgFinal = m_warp->genNewImg(dstImgEye, 1);	
	
	//imshow("dstImgEye", dstImgEye);
	//imshow("dstImgFinal", dstImgFinal);

	Mat refLabColorSpace;
	std::vector<cv::Mat> refLab;
	cvtColor(dstImgFinal, refLabColorSpace, CV_BGR2Lab);
	split(refLabColorSpace, refLab);

	Mat imgLabColorSpace;
	std::vector<cv::Mat> imgLab;
	cvtColor(eyeImgROI, imgLabColorSpace, CV_BGR2Lab);
	split(imgLabColorSpace, imgLab);

	int w = m_imgEyesROI.cols;
	int h = m_imgEyesROI.rows;
	//printf("%d %d %d %d %d %d %d %d %d %d\n", w, h, imgLeftROI.cols, imgLeftROI.rows, imgLeftEyeROI.cols,imgLeftEyeROI.rows, dstImgLeftFinal.cols, dstImgLeftFinal.rows, leftEyeMaskROI.cols, leftEyeMaskROI.rows);
	for(int i = 0; i < h; i++)
	{
		for(int j = 0; j < w; j++)
		{
			if(m_imgEyesROI.at<uchar>(i, j) != 0)
			{
				imgLab[0].at<uchar>(i,j) = int(imgLab[0].at<uchar>(i,j)*0.7 + refLab[0].at<uchar>(i, j)*0.3 + 0.5);
				imgLab[1].at<uchar>(i,j) = int(imgLab[1].at<uchar>(i,j)*0.1 + refLab[1].at<uchar>(i, j)*0.9 + 0.5);
				imgLab[2].at<uchar>(i,j) = int(imgLab[2].at<uchar>(i,j)*0.1 + refLab[2].at<uchar>(i, j)*0.9 + 0.5);
			}
		}
	}

	merge(imgLab, imgLabColorSpace);
	cvtColor(imgLabColorSpace, eyeImgROI, CV_Lab2BGR);
	//eyeImgROI.copyTo(imgROI);

	Mat_<Vec3b> img_EyeROI = eyeImgROI;
	Mat_<Vec3b> img_ROI = imgROI;
	for(int i = 0; i < ht; i++)
	{
		for(int j = 0; j < wd; j++)
		{
			if(m_imgEyesROI.at<uchar>(i, j) != 0)
			{
				uchar v0 = int(img_EyeROI(i, j)[0]*(1.0*(m_imgEyesROI.at<uchar>(i, j))/255) + img_ROI(i, j)[0]*(1 - (1.0*m_imgEyesROI.at<uchar>(i, j))/255) + 0.5);
				uchar v1 = int(img_EyeROI(i, j)[1]*(1.0*(m_imgEyesROI.at<uchar>(i, j))/255) + img_ROI(i, j)[1]*(1 - (1.0*m_imgEyesROI.at<uchar>(i, j))/255) + 0.5);
				uchar v2 = int(img_EyeROI(i, j)[2]*(1.0*(m_imgEyesROI.at<uchar>(i, j))/255) + img_ROI(i, j)[2]*(1 - (1.0*m_imgEyesROI.at<uchar>(i, j))/255) + 0.5);

				if(v0 > 255)
					v0 = 255;
				if(v1 > 255)
					v1 = 255;
				if(v2 > 255)
					v2 = 255;
				
				img_ROI(i, j)[0] = v0;	
				img_ROI(i, j)[1] = v1;	
				img_ROI(i, j)[2] = v2;	
			}
		}
	}


}
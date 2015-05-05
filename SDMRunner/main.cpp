#include "SDMLocator.h"
#include "hpfacedetectorwrapper.h"
#include "opencv2\highgui/highgui.hpp"
#include "PoseCorrect.h"
#include "faceMakeup.h"

int main()
{
	SDMLocator locator;
	double t = (double)cvGetTickCount();
	if(!locator.readBinModel("49.bin"))
	{
		printf("can not find SDM model\n");
		return -1;
	}
	t = (double)cvGetTickCount() - t;
	printf( "load model time = %g ms\n", t/((double)cvGetTickFrequency()*1000.) );
	HPFaceDetectorWarpper* faceDetector = NULL;
	faceDetector = new HPFaceDetectorWarpper(false);
	faceDetector->Init(true, true, true);
	
	CPoseEstimate* pEst = new CPoseEstimate();
	
	FaceMakeUp* pMakeup = new FaceMakeUp();
	pMakeup->loadFaceModels(string("models.txt"));
	Mat meaShape = locator.getMeanShape();
	pMakeup->getRefsMasks(&locator, faceDetector, meaShape);
	int numModels = pMakeup->getNumModels();
	
	delete faceDetector;
	faceDetector = NULL;
	
	faceDetector = new HPFaceDetectorWarpper(true);
	faceDetector->Init(true, true, true);

	bool isDetect = true;
	bool isShowInfo = false;
	float score = 0;
	int indexModel = 0;
	
	VideoCapture cap;
	cap.open(0);
	//cap.open("hwf.avi");
	
	Mat X0;
	while(1)
	{
		Mat img;// = cv::imread("ttt.jpg");// = cv::imread(inputFilename.string());
		cap >> img;
		if(img.empty())
			break;
		Mat landmarksImage = img.clone();
		Mat imgGray;
		cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);
		FaceArray faces;
		
		Rect rFace;
		if(isDetect)
		{
			// face detection
			faceDetector->DetectFaces(imgGray.data, imgGray.cols, imgGray.rows, faces);
			if(faces.size() > 0)
			{
				
				rFace.x = max(0, faces[0].centerX - faces[0].radius);
				rFace.y = max(0, faces[0].centerY - faces[0].radius);
				if(rFace.x + 2*faces[0].radius > imgGray.cols)
					rFace.width = imgGray.cols - rFace.x;
				else
					rFace.width = 2*faces[0].radius;
				if(rFace.y + 2*faces[0].radius > imgGray.rows)
					rFace.height = imgGray.rows- rFace.y;
				else
					rFace.height = 2*faces[0].radius;
		
				// fit the model
				double t = (double)cvGetTickCount();
				Mat meanShape = locator.getMeanShape();
				if(!locator.detect49(meanShape, imgGray, rFace))
				{
					break;
				}
				t = (double)cvGetTickCount() - t;
				double tt = t/((double)cvGetTickFrequency()*1000.);
				//printf( "detection time = %g ms\n", tt);
				//printf("score: %f\n", score);		
				meanShape.copyTo(X0);
				char cText[10];
				sprintf(cText, "%.2f ms", tt);
				char cTextscore[10];
				std::string str = "Tracker: ";
				str += cText;				
				cv::putText(landmarksImage, str, cv::Point(240, 20),1, 1.3, Scalar(0,255,0), 2);
	
				
				for (int j = 0; j < locator.getNumLandmarks(); ++j) 
				{
					cv::circle(landmarksImage, Point2f(X0.at<float>(j, 0), X0.at<float>(j + locator.getNumLandmarks(), 0)), 1, Scalar(0.0f, 255.0f, 0.0f), -1);
				}
				isDetect = false;			
			}
		}
		else
		{
			float p = 0.; 
			float y = 0.; 
			float r = 0.;
			std::vector<cv::Point2f> shape1;
			std::vector<cv::Point2f> shape2;
			for (int i = 0; i < locator.getNumLandmarks(); ++i) 
			{
				shape1.push_back(Point2f(X0.at<float>(i, 0), X0.at<float>(i + locator.getNumLandmarks(), 0)));
			}
			cv::Rect r1 = boundingRect(shape1);
			float dev1 = sqrt(float(r1.width*r1.width + r1.height*r1.height));
			Mat meanShape = locator.getMeanShape();
			double t = (double)cvGetTickCount();
			locator.track49(meanShape, X0, imgGray);
			t = (double)cvGetTickCount() - t;
			double tt = t/((double)cvGetTickFrequency()*1000.);
			//printf( "TRACK time = %g ms\n",tt);
			meanShape.copyTo(X0);
			//printf("score: %f\n", score);
			
			
			for (int i = 0; i < locator.getNumLandmarks(); ++i) 
			{
				shape2.push_back(Point2f(X0.at<float>(i, 0), X0.at<float>(i + locator.getNumLandmarks(), 0)));
			}
			cv::Rect r2 = boundingRect(shape2);
			float dev2 = sqrt(float(r2.width*r2.width + r2.height*r2.height));
			float scoreTest = (abs(dev2 - dev1)) / dev1;
			//printf("%f %f %f\n", dev1, dev2, scoreTest);
			score = 1 / (1+ scoreTest);

			/*double stable = 0.0;
			for(int i = 0; i < shape2.size(); i++)
			{
				stable = stable + sqrt((shape2[i].x - shape1[i].x)*(shape2[i].x - shape1[i].x) + (shape2[i].y - shape1[i].y)*(shape2[i].y - shape1[i].y));
			}
			printf("stable %f\n", stable/shape2.size());*/
			
			if(isShowInfo)
			{
				char cText[10];
				sprintf(cText, "%.2f ms", tt);

				char cTextscore[10];
				sprintf(cTextscore, "%.2f", score);
				std::string str = "Tracker: ";
				str += cText;
			
				std::string strScore = "Score: ";
				strScore += cTextscore;
			
				cv::putText(landmarksImage, str, cv::Point(240, 20), 1, 1.3, Scalar(0,255,0), 2);
				cv::putText(landmarksImage, strScore, cv::Point(240, 40), 1, 1.3, Scalar(0,255,0), 2);
			}

			if(score < 0.9)
				isDetect = true;
			else
			{
				std::vector<cv::Point2f> vecLMs;
				// draw the final result
				for (int i = 0; i < locator.getNumLandmarks(); ++i) 
				{
					vecLMs.push_back(Point2f(X0.at<float>(i, 0), X0.at<float>(i + locator.getNumLandmarks(), 0)));
					if(isShowInfo)
						cv::circle(landmarksImage, Point2f(X0.at<float>(i, 0), X0.at<float>(i + locator.getNumLandmarks(), 0)), 1, Scalar(0.0f, 255.0f, 0.0f), -1);
				}
				
				if(isShowInfo)
				{
					pEst->computerPosePOIST(vecLMs);
					pEst->correctPose(img);	
					pEst->getPose(p, y, r);
					pEst->drawAxis(landmarksImage);
				}
				t = (double)cvGetTickCount();
				pMakeup->getLipMask(img, vecLMs);
				pMakeup->makeupLips(img, vecLMs, indexModel);
				t = (double)cvGetTickCount() - t;
				double tLips = t/((double)cvGetTickFrequency()*1000.);
				//printf( "lip makeup time = %g ms\n",tt);

				t = (double)cvGetTickCount();
				pMakeup->makeupEyes(img, vecLMs, indexModel);
				t = (double)cvGetTickCount() - t;
				double tEyes = t/((double)cvGetTickFrequency()*1000.);
				//printf( "eyes makeup time = %g ms\n",tt);


				if(isShowInfo)
				{
					char cText[10];
					sprintf(cText, "%.2f ms", tLips);

					char cTextscore[10];
					sprintf(cTextscore, "%.2f ms", tEyes);
					
					std::string str = "Lips: ";
					str += cText;
			
					std::string strScore = "Eyes: ";
					strScore += cTextscore;
			
					cv::putText(landmarksImage, str, cv::Point(450, 20), 1, 1.3, Scalar(0,255,0), 2);
					cv::putText(landmarksImage, strScore, cv::Point(450, 40), 1, 1.3, Scalar(0,255,0), 2);
				}
			}
		}
		imshow("makeup", img);	
		imshow("landmarksImage", landmarksImage);
		int qq = cv::waitKey(1);
		if(qq == 'q')
			break;
		if(qq == 's')
			isShowInfo = !isShowInfo;
		if(qq == 'c')
		{
			indexModel = indexModel + 1;
			if(indexModel == numModels)
				indexModel = 0;
		}
	}
	
	delete pEst;
	delete pMakeup;
	delete faceDetector;
	locator.Release();
	return 0;
}
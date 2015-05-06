/*
 * DescriptorExtractor.hpp
 *
 *  Created on: 21.03.2014
 *      Author: Patrik Huber
 */
#pragma once

#ifndef DESCRIPTOREXTRACTOR_HPP_
#define DESCRIPTOREXTRACTOR_HPP_

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <string>
#include <iostream>
extern "C" {
	#include "hog.h"
	
}

using cv::Mat;
using std::vector;


class DescriptorExtractor
{
public:
	virtual ~DescriptorExtractor() {}

	// returns a Matrix, as many rows as points, 1 descriptor = 1 row
	// the default argument is quite ugly in an inheritance hierarchy
	virtual cv::Mat getDescriptors(const cv::Mat& image, std::vector<cv::Point2f>& locations, int windowSizeHalf=0, float c = 2.0, float s = 2.0) = 0;

	virtual cv::Mat getDescriptors(const cv::Mat& image, cv::Mat& points, int windowSizeHalf=0, float c = 2.0, float s = 2.0) = 0;

};



class VlHogDescriptorExtractor : public DescriptorExtractor
{
public:
	// Store the params as private vars? However, it might be better to store the parameters separately, to be able to share a FeatureDescriptorExtractor over multiple Sdm cascade levels

	enum class VlHogType 
	{
		DalalTriggs,
		Uoctti
	};

	// means we use the adaptive parameters depending on the regressor-level and facebox size
	VlHogDescriptorExtractor(VlHogType vlhogType) : hogType(vlhogType)
	{

	};
	
	// use the parameters given
	VlHogDescriptorExtractor(VlHogType vlhogType, int numCells, int cellSize, int numBins) : hogType(vlhogType), numCells(numCells), cellSize(cellSize), numBins(numBins)
	{
	};

	//cv::Mat getDescriptors(const cv::Mat& image, std::vector<cv::Point2f>& locations, int windowSizeHalf, float c, float s ) ;
	//cv::Mat getDescriptors(const cv::Mat& image, cv::Mat& points, int windowSizeHalf, float c, float s ); 
	
	
	void warpPix(const cv::Mat& gray, float c, float s, cv::Rect& src, Mat& dst)
	{
		int wd = src.width;
		int ht = src.height;
		float x0 = src.x + wd/2 ;
		float y0 = src.y + ht/2;
		int xl = src.x;
		int yl = src.y;
		
		int yy = gray.rows - 1;
		int xx = gray.cols - 1;
		//printf("x0 y0 is : %f %f", x0, y0);	
		//ang = ang * CV_PI/180;
		//float c = cos(ang);
		//float s = sin(ang);
		dst = Mat(ht, wd, CV_8UC1);
		for(int i = 0; i < ht; i++)
		{
			for(int j = 0; j < wd; j++)
			{
				int x1 = xl + j;
				int y1 = yl + i;
				//printf("x0 y0 is : %d %d\n", x1, y1);	
				
				float x2 = (x1 - x0)*c - (y1 - y0)*s + x0;
				float y2 = (x1 - x0)*s + (y1 - y0)*c + y0;
				int x = cvRound(x2);
				int y = cvRound(y2);
				x = MIN(MAX(0, x), xx);
				y = MIN(MAX(0, y), yy);
				//printf("%d %d %d %d\n", x1, y1, x, y);
				dst.at<uchar>(i, j) = gray.at<uchar>(y, x);
				
				//printf("%f %f %f \n",rotM.at<double>(0, 0), rotM.at<double>(0, 1), rotM.at<double>(0, 2));
			}
		}
		
	};


	// Maybe split the class in an AdaptiveVlHog and a VlHogDesc...?
	// Or better solution with less code duplication?
	cv::Mat getDescriptors(const cv::Mat& image, std::vector<cv::Point2f>& locations, int windowSizeHalf, float c, float s ) 
	{
		Mat grayImage;
		if (image.channels() == 3) 
		{
			cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);
		}
		else
		{
			grayImage = image;
		}
		VlHogVariant vlHogVariant;
		vlHogVariant = VlHogVariant::VlHogVariantUoctti;
		int patchWidthHalf = windowSizeHalf;

		
		if(c != 2.0)
		{	
			/*if(angle <= 0)
				angle  = CV_PI/2  + angle;	
			else
				angle = angle - CV_PI/2;*/
			
			//printf("angle is:  %f\n",  c);
			Mat hogDescriptors; // We'll get the dimensions later from vl_hog_get_*
			VlHog* hog = vl_hog_new(vlHogVariant, numBins, false); // VlHogVariantUoctti seems to be default in Matlab.
			int wd = image.cols;
			int ht = image.rows;
			int nSize = locations.size();
			for (int i = 0; i < nSize; ++i) 
			{
				// get the (x, y) location and w/h of the current patch
				int x = cvRound(locations[i].x);
				int y = cvRound(locations[i].y);
							
				Mat roiImg;
				if (x - patchWidthHalf < 0 || y - patchWidthHalf < 0 || x + patchWidthHalf >= wd || y + patchWidthHalf >= ht) 
				{	
					// The feature extraction location is too far near a border. We extend the image (add a black canvas)
					// and then extract from this larger image.
					int borderLeft = (x - patchWidthHalf) < 0 ? std::abs(x - patchWidthHalf) : 0; // Our x and y are center.
					int borderTop = (y - patchWidthHalf) < 0 ? std::abs(y - patchWidthHalf) : 0;
					int borderRight = (x + patchWidthHalf) >= wd ? std::abs(wd - (x + patchWidthHalf)) : 0;
					int borderBottom = (y + patchWidthHalf) >= ht ? std::abs(ht - (y + patchWidthHalf)) : 0;
					Mat extendedImage = grayImage.clone();
					cv::copyMakeBorder(extendedImage, extendedImage, borderTop, borderBottom, borderLeft, borderRight, cv::BORDER_CONSTANT, cv::Scalar(0));
					cv::Rect roi((x - patchWidthHalf) + borderLeft, (y - patchWidthHalf) + borderTop, patchWidthHalf * 2, patchWidthHalf * 2); // Rect: x y w h. x and y are top-left corner.
					warpPix(extendedImage, c, s, roi, roiImg);
					//roiImg1 = extendedImage(roi).clone(); // clone because we need a continuous memory block
				}
				else 
				{
					cv::Rect roi(x - patchWidthHalf, y - patchWidthHalf, patchWidthHalf * 2, patchWidthHalf * 2); // x y w h. Rect: x and y are top-left corner. Our x and y are center. Convert.
					// we have exactly the same window as the matlab code.
					// extract the patch and supply it to vl_hog
					warpPix(grayImage, c, s, roi, roiImg);					
					//roiImg1 = grayImage(roi).clone(); // clone because we need a continuous memory block
				}
			
				//if(i == 0)
				//{
				//	cv::imshow("roiImg", roiImg);
				//	//cv::imshow("roiImg1", roiImg1);
				//	cv::waitKey(1);
				//}
				
				roiImg.convertTo(roiImg, CV_32FC1); // because vl_hog_put_image expects a float* (values 0.f-255.f)
				//if (adaptivePatchSize) 
				{
					cv::resize(roiImg, roiImg, cv::Size(cellSize*numCells, cellSize*numCells)); // actually we shouldn't resize when the image is smaller than 30, but Zhenhua does it
					// in his Matlab code. If we don't resize, we probably have to adjust the HOG parameters.
				}
				// vl_hog_new: numOrientations=hogParameter.numBins, transposed (=col-major):false)
				//VlHog* hog = vl_hog_new(vlHogVariant, numBins, false); // VlHogVariantUoctti seems to be default in Matlab.
				vl_hog_put_image(hog, (float*)roiImg.data, roiImg.cols, roiImg.rows, 1, cellSize); // (the '1' is numChannels)
				vl_size ww = vl_hog_get_width(hog); // we could assert that ww == hh == numCells
				vl_size hh = vl_hog_get_height(hog);
				vl_size dd = vl_hog_get_dimension(hog); // assert ww=hogDim1, hh=hogDim2, dd=hogDim3
				//float* hogArray = (float*)malloc(ww*hh*dd*sizeof(float));
				Mat hogArray(1, ww*hh*dd, CV_32FC1); // safer & same result. Don't use C-style memory management.
				//vl_hog_extract(hog, hogArray); // just interpret hogArray in col-major order to get the same n x 1 vector as in matlab. (w * h * d)
				vl_hog_extract(hog, hogArray.ptr<float>(0));
				//vl_hog_delete(hog);
				Mat hogDescriptor(hh*ww*dd, 1, CV_32FC1);
				// Stack the third dimensions of the HOG descriptor of this patch one after each other in a column-vector
				for (int j = 0; j < dd; ++j) 
				{
					//Mat hogFeatures(hh, ww, CV_32FC1, hogArray + j*ww*hh);
					Mat hogFeatures(hh, ww, CV_32FC1, hogArray.ptr<float>(0) + j*ww*hh); // Creates the same array as in Matlab. I might have to check this again if hh!=ww (non-square)
					hogFeatures = hogFeatures.t(); // Necessary because the Matlab reshape() takes column-wise from the matrix while the OpenCV reshape() takes row-wise.
					hogFeatures = hogFeatures.reshape(0, hh*ww); // make it to a column-vector
					Mat currentDimSubMat = hogDescriptor.rowRange(j*ww*hh, j*ww*hh + ww*hh);
					hogFeatures.copyTo(currentDimSubMat);
				}
				//free(hogArray); // not necessary - we use a Mat.
				//features = [features; double(reshape(tmp, [], 1))];
				// B = reshape(A,m,n) returns the m-by-n matrix B whose elements are taken column-wise from A
				// Matlab (& Eigen, OpenGL): Column-major.
				// OpenCV: Row-major.
				// (access is always (r, c).)
				//Mat currentFeaturesSubrange = hogDescriptors.rowRange(i*hogDims, i*hogDims + hogDims);
				//hogDescriptor.copyTo(currentFeaturesSubrange);
				hogDescriptor = hogDescriptor.t(); // now a row-vector
				hogDescriptors.push_back(hogDescriptor);
				roiImg.release();

			}
			vl_hog_delete(hog);
			// hogDescriptors needs to have dimensions numLandmarks x hogFeaturesDimension, where hogFeaturesDimension is e.g. 3*3*16=144
			return hogDescriptors;
		}
		else
		{	
			/*if(angle <= 0)
				angle  = CV_PI/2  + angle;	
			else
				angle = angle - CV_PI/2;*/
			
			//printf("angle is:  %f\n",  c);
			Mat hogDescriptors; // We'll get the dimensions later from vl_hog_get_*
			VlHog* hog = vl_hog_new(vlHogVariant, numBins, false); // VlHogVariantUoctti seems to be default in Matlab.
			int wd = image.cols;
			int ht = image.rows;
			int nSize = locations.size();
			for (int i = 0; i < nSize; ++i) 
			{
				// get the (x, y) location and w/h of the current patch
				int x = cvRound(locations[i].x);
				int y = cvRound(locations[i].y);
							
				Mat roiImg;
				if (x - patchWidthHalf < 0 || y - patchWidthHalf < 0 || x + patchWidthHalf >= wd || y + patchWidthHalf >= ht) 
				{	
					// The feature extraction location is too far near a border. We extend the image (add a black canvas)
					// and then extract from this larger image.
					int borderLeft = (x - patchWidthHalf) < 0 ? std::abs(x - patchWidthHalf) : 0; // Our x and y are center.
					int borderTop = (y - patchWidthHalf) < 0 ? std::abs(y - patchWidthHalf) : 0;
					int borderRight = (x + patchWidthHalf) >= wd ? std::abs(wd - (x + patchWidthHalf)) : 0;
					int borderBottom = (y + patchWidthHalf) >= ht ? std::abs(ht - (y + patchWidthHalf)) : 0;
					Mat extendedImage = grayImage.clone();
					cv::copyMakeBorder(extendedImage, extendedImage, borderTop, borderBottom, borderLeft, borderRight, cv::BORDER_CONSTANT, cv::Scalar(0));
					cv::Rect roi((x - patchWidthHalf) + borderLeft, (y - patchWidthHalf) + borderTop, patchWidthHalf * 2, patchWidthHalf * 2); // Rect: x y w h. x and y are top-left corner.
					//warpPix(extendedImage, c, s, roi, roiImg);
					roiImg = extendedImage(roi).clone(); // clone because we need a continuous memory block
				}
				else 
				{
					cv::Rect roi(x - patchWidthHalf, y - patchWidthHalf, patchWidthHalf * 2, patchWidthHalf * 2); // x y w h. Rect: x and y are top-left corner. Our x and y are center. Convert.
					// we have exactly the same window as the matlab code.
					// extract the patch and supply it to vl_hog
					//warpPix(grayImage, c, s, roi, roiImg);					
					roiImg = grayImage(roi).clone(); // clone because we need a continuous memory block
				}
			
				//if(i == 0)
				//{
				//	cv::imshow("roiImg", roiImg);
				//	//cv::imshow("roiImg1", roiImg1);
				//	cv::waitKey(1);
				//}
				
				roiImg.convertTo(roiImg, CV_32FC1); // because vl_hog_put_image expects a float* (values 0.f-255.f)
				//if (adaptivePatchSize) 
				{
					cv::resize(roiImg, roiImg, cv::Size(cellSize*numCells, cellSize*numCells)); // actually we shouldn't resize when the image is smaller than 30, but Zhenhua does it
					// in his Matlab code. If we don't resize, we probably have to adjust the HOG parameters.
				}
				// vl_hog_new: numOrientations=hogParameter.numBins, transposed (=col-major):false)
				//VlHog* hog = vl_hog_new(vlHogVariant, numBins, false); // VlHogVariantUoctti seems to be default in Matlab.
				vl_hog_put_image(hog, (float*)roiImg.data, roiImg.cols, roiImg.rows, 1, cellSize); // (the '1' is numChannels)
				vl_size ww = vl_hog_get_width(hog); // we could assert that ww == hh == numCells
				vl_size hh = vl_hog_get_height(hog);
				vl_size dd = vl_hog_get_dimension(hog); // assert ww=hogDim1, hh=hogDim2, dd=hogDim3
				//float* hogArray = (float*)malloc(ww*hh*dd*sizeof(float));
				Mat hogArray(1, ww*hh*dd, CV_32FC1); // safer & same result. Don't use C-style memory management.
				//vl_hog_extract(hog, hogArray); // just interpret hogArray in col-major order to get the same n x 1 vector as in matlab. (w * h * d)
				vl_hog_extract(hog, hogArray.ptr<float>(0));
				//vl_hog_delete(hog);
				Mat hogDescriptor(hh*ww*dd, 1, CV_32FC1);
				// Stack the third dimensions of the HOG descriptor of this patch one after each other in a column-vector
				for (int j = 0; j < dd; ++j) 
				{
					//Mat hogFeatures(hh, ww, CV_32FC1, hogArray + j*ww*hh);
					Mat hogFeatures(hh, ww, CV_32FC1, hogArray.ptr<float>(0) + j*ww*hh); // Creates the same array as in Matlab. I might have to check this again if hh!=ww (non-square)
					hogFeatures = hogFeatures.t(); // Necessary because the Matlab reshape() takes column-wise from the matrix while the OpenCV reshape() takes row-wise.
					hogFeatures = hogFeatures.reshape(0, hh*ww); // make it to a column-vector
					Mat currentDimSubMat = hogDescriptor.rowRange(j*ww*hh, j*ww*hh + ww*hh);
					hogFeatures.copyTo(currentDimSubMat);
				}
				//free(hogArray); // not necessary - we use a Mat.
				//features = [features; double(reshape(tmp, [], 1))];
				// B = reshape(A,m,n) returns the m-by-n matrix B whose elements are taken column-wise from A
				// Matlab (& Eigen, OpenGL): Column-major.
				// OpenCV: Row-major.
				// (access is always (r, c).)
				//Mat currentFeaturesSubrange = hogDescriptors.rowRange(i*hogDims, i*hogDims + hogDims);
				//hogDescriptor.copyTo(currentFeaturesSubrange);
				hogDescriptor = hogDescriptor.t(); // now a row-vector
				hogDescriptors.push_back(hogDescriptor);
				roiImg.release();

			}
			vl_hog_delete(hog);
			// hogDescriptors needs to have dimensions numLandmarks x hogFeaturesDimension, where hogFeaturesDimension is e.g. 3*3*16=144
			return hogDescriptors;
		}		
		
	};

	// Maybe split the class in an AdaptiveVlHog and a VlHogDesc...?
	// Or better solution with less code duplication?
	cv::Mat getDescriptors(const cv::Mat& image, cv::Mat& points, int windowSizeHalf, float c, float s ) 
	{
		Mat grayImage;
		if (image.channels() == 3) 
		{
			cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);
		}
		else
		{
			grayImage = image;
		}
		VlHogVariant vlHogVariant;
		vlHogVariant = VlHogVariant::VlHogVariantUoctti;
		/*switch (hogType)
		{
		case VlHogDescriptorExtractor::VlHogType::DalalTriggs:
			vlHogVariant = VlHogVariant::VlHogVariantDalalTriggs;
			break;
		case VlHogDescriptorExtractor::VlHogType::Uoctti:
			vlHogVariant = VlHogVariant::VlHogVariantUoctti;
			break;
		default:
			break;
		}*/
		
		int patchWidthHalf = windowSizeHalf;
		//bool adaptivePatchSize = false;
		//if (windowSizeHalf > 0) 
		//{ // A windowSize was given, meaning we use adaptive. Note: Solve this more properly!
		//	adaptivePatchSize = true;
		//}
		//if (adaptivePatchSize) 
		//{
		//	// adaptive:
		//	//int NUM_CELL = 3; // number of cells in the local patch for local feature extraction, i.e.a 3x3 grid
		//	patchWidthHalf = windowSizeHalf;
		//	//cellSize = 10; // One cell is 10x10
		//	//numCells = 3; // Always 3 for adaptive, 3 * 10 = 30, i.e. always a 30x30 patch
		//	//numBins = 9; // always 4? Or 9 = default of vl_hog ML?
		//	// Q: When patch < 30, don't resize. If < 30, make sure it's even?
		//	// Q: 3 cells might not be so good when the patch is small, e.g. does a 2x2 cell make sense?
		//}
		//else 
		//{
		//	// traditional:
		//	patchWidthHalf = numCells * (cellSize / 2); // patchWidthHalf: Zhenhua's 'numNeighbours'. cellSize: has nothing to do with HOG. It's rather the number of HOG cells we want.
		//}
		
		if(c != 2.0)
		{	
			/*if(angle <= 0)
				angle  = CV_PI/2  + angle;	
			else
				angle = angle - CV_PI/2;*/
			
			//printf("angle is:  %f\n",  c);
			Mat hogDescriptors; // We'll get the dimensions later from vl_hog_get_*
			VlHog* hog = vl_hog_new(vlHogVariant, numBins, false); // VlHogVariantUoctti seems to be default in Matlab.
			int wd = image.cols;
			int ht = image.rows;
			int roiSz = cellSize*numCells;
			Mat roiImg;
			int nSize = points.rows / 2;
			for (int i = 0; i < nSize; ++i) 
			{
				// get the (x, y) location and w/h of the current patch
				int x = cvRound(points.at<float>(i));
				int y = cvRound(points.at<float>(i+nSize));
							
				
				if (x - patchWidthHalf < 0 || y - patchWidthHalf < 0 || x + patchWidthHalf >= wd || y + patchWidthHalf >= ht) 
				{	
					// The feature extraction location is too far near a border. We extend the image (add a black canvas)
					// and then extract from this larger image.
					int borderLeft = (x - patchWidthHalf) < 0 ? std::abs(x - patchWidthHalf) : 0; // Our x and y are center.
					int borderTop = (y - patchWidthHalf) < 0 ? std::abs(y - patchWidthHalf) : 0;
					int borderRight = (x + patchWidthHalf) >= wd ? std::abs(wd - (x + patchWidthHalf)) : 0;
					int borderBottom = (y + patchWidthHalf) >= ht ? std::abs(ht - (y + patchWidthHalf)) : 0;
					Mat extendedImage = grayImage.clone();
					cv::copyMakeBorder(extendedImage, extendedImage, borderTop, borderBottom, borderLeft, borderRight, cv::BORDER_CONSTANT, cv::Scalar(0));
					cv::Rect roi((x - patchWidthHalf) + borderLeft, (y - patchWidthHalf) + borderTop, patchWidthHalf * 2, patchWidthHalf * 2); // Rect: x y w h. x and y are top-left corner.
					warpPix(extendedImage, c, s, roi, roiImg);
					//roiImg1 = extendedImage(roi).clone(); // clone because we need a continuous memory block
				}
				else 
				{
					cv::Rect roi(x - patchWidthHalf, y - patchWidthHalf, patchWidthHalf * 2, patchWidthHalf * 2); // x y w h. Rect: x and y are top-left corner. Our x and y are center. Convert.
					// we have exactly the same window as the matlab code.
					// extract the patch and supply it to vl_hog
					warpPix(grayImage, c, s, roi, roiImg);					
					//roiImg1 = grayImage(roi).clone(); // clone because we need a continuous memory block
				}
			
				//if(i == 0)
				//{
				//	cv::imshow("roiImg", roiImg);
				//	//cv::imshow("roiImg1", roiImg1);
				//	cv::waitKey(1);
				//}
				
				roiImg.convertTo(roiImg, CV_32FC1); // because vl_hog_put_image expects a float* (values 0.f-255.f)
				//if (adaptivePatchSize) 
				{
					cv::resize(roiImg, roiImg, cv::Size(roiSz, roiSz)); // actually we shouldn't resize when the image is smaller than 30, but Zhenhua does it
					// in his Matlab code. If we don't resize, we probably have to adjust the HOG parameters.
				}
				// vl_hog_new: numOrientations=hogParameter.numBins, transposed (=col-major):false)
				//VlHog* hog = vl_hog_new(vlHogVariant, numBins, false); // VlHogVariantUoctti seems to be default in Matlab.
				vl_hog_put_image(hog, (float*)roiImg.data, roiImg.cols, roiImg.rows, 1, cellSize); // (the '1' is numChannels)
				vl_size ww = vl_hog_get_width(hog); // we could assert that ww == hh == numCells
				vl_size hh = vl_hog_get_height(hog);
				vl_size dd = vl_hog_get_dimension(hog); // assert ww=hogDim1, hh=hogDim2, dd=hogDim3
				//float* hogArray = (float*)malloc(ww*hh*dd*sizeof(float));
				int multipyRes = ww*hh*dd;
				Mat hogArray(1, multipyRes, CV_32FC1); // safer & same result. Don't use C-style memory management.
				//vl_hog_extract(hog, hogArray); // just interpret hogArray in col-major order to get the same n x 1 vector as in matlab. (w * h * d)
				vl_hog_extract(hog, hogArray.ptr<float>(0));
				//vl_hog_delete(hog);
				Mat hogDescriptor(multipyRes, 1, CV_32FC1);
				// Stack the third dimensions of the HOG descriptor of this patch one after each other in a column-vector
				int multipy_Res = ww*hh;
				for (int j = 0; j < dd; ++j) 
				{
					//Mat hogFeatures(hh, ww, CV_32FC1, hogArray + j*ww*hh);
					Mat hogFeatures(hh, ww, CV_32FC1, hogArray.ptr<float>(0) + j*multipy_Res); // Creates the same array as in Matlab. I might have to check this again if hh!=ww (non-square)
					hogFeatures = hogFeatures.t(); // Necessary because the Matlab reshape() takes column-wise from the matrix while the OpenCV reshape() takes row-wise.
					hogFeatures = hogFeatures.reshape(0, multipy_Res); // make it to a column-vector
					Mat currentDimSubMat = hogDescriptor.rowRange(j*multipy_Res, j*multipy_Res + multipy_Res);
					hogFeatures.copyTo(currentDimSubMat);
				}
				//free(hogArray); // not necessary - we use a Mat.
				//features = [features; double(reshape(tmp, [], 1))];
				// B = reshape(A,m,n) returns the m-by-n matrix B whose elements are taken column-wise from A
				// Matlab (& Eigen, OpenGL): Column-major.
				// OpenCV: Row-major.
				// (access is always (r, c).)
				//Mat currentFeaturesSubrange = hogDescriptors.rowRange(i*hogDims, i*hogDims + hogDims);
				//hogDescriptor.copyTo(currentFeaturesSubrange);
				hogDescriptor = hogDescriptor.t(); // now a row-vector
				hogDescriptors.push_back(hogDescriptor);
				
			}
			roiImg.release();
			vl_hog_delete(hog);
			// hogDescriptors needs to have dimensions numLandmarks x hogFeaturesDimension, where hogFeaturesDimension is e.g. 3*3*16=144
			return hogDescriptors;
		}
		else
		{	
			/*if(angle <= 0)
				angle  = CV_PI/2  + angle;	
			else
				angle = angle - CV_PI/2;*/
			
			//printf("angle is:  %f\n",  c);
			Mat hogDescriptors; // We'll get the dimensions later from vl_hog_get_*
			VlHog* hog = vl_hog_new(vlHogVariant, numBins, false); // VlHogVariantUoctti seems to be default in Matlab.
			int wd = image.cols;
			int ht = image.rows;
			Mat roiImg;
			int nSize = points.rows / 2;
			for (int i = 0; i < nSize; ++i) 
			{
				// get the (x, y) location and w/h of the current patch
				int x = cvRound(points.at<float>(i));
				int y = cvRound(points.at<float>(i+nSize));
							
				if (x - patchWidthHalf < 0 || y - patchWidthHalf < 0 || x + patchWidthHalf >= wd || y + patchWidthHalf >= ht) 
				{	
					// The feature extraction location is too far near a border. We extend the image (add a black canvas)
					// and then extract from this larger image.
					int borderLeft = (x - patchWidthHalf) < 0 ? std::abs(x - patchWidthHalf) : 0; // Our x and y are center.
					int borderTop = (y - patchWidthHalf) < 0 ? std::abs(y - patchWidthHalf) : 0;
					int borderRight = (x + patchWidthHalf) >= wd ? std::abs(wd - (x + patchWidthHalf)) : 0;
					int borderBottom = (y + patchWidthHalf) >= ht ? std::abs(ht - (y + patchWidthHalf)) : 0;
					Mat extendedImage = grayImage.clone();
					cv::copyMakeBorder(extendedImage, extendedImage, borderTop, borderBottom, borderLeft, borderRight, cv::BORDER_CONSTANT, cv::Scalar(0));
					cv::Rect roi((x - patchWidthHalf) + borderLeft, (y - patchWidthHalf) + borderTop, patchWidthHalf * 2, patchWidthHalf * 2); // Rect: x y w h. x and y are top-left corner.
					//warpPix(extendedImage, c, s, roi, roiImg);
					roiImg = extendedImage(roi).clone(); // clone because we need a continuous memory block
				}
				else 
				{
					cv::Rect roi(x - patchWidthHalf, y - patchWidthHalf, patchWidthHalf * 2, patchWidthHalf * 2); // x y w h. Rect: x and y are top-left corner. Our x and y are center. Convert.
					// we have exactly the same window as the matlab code.
					// extract the patch and supply it to vl_hog
					//warpPix(grayImage, c, s, roi, roiImg);					
					roiImg = grayImage(roi).clone(); // clone because we need a continuous memory block
				}
			
				//if(i == 0)
				//{
				//	cv::imshow("roiImg", roiImg);
				//	//cv::imshow("roiImg1", roiImg1);
				//	cv::waitKey(1);
				//}
				
				roiImg.convertTo(roiImg, CV_32FC1); // because vl_hog_put_image expects a float* (values 0.f-255.f)
				//if (adaptivePatchSize) 
				{
					cv::resize(roiImg, roiImg, cv::Size(cellSize*numCells, cellSize*numCells)); // actually we shouldn't resize when the image is smaller than 30, but Zhenhua does it
					// in his Matlab code. If we don't resize, we probably have to adjust the HOG parameters.
				}
				// vl_hog_new: numOrientations=hogParameter.numBins, transposed (=col-major):false)
				//VlHog* hog = vl_hog_new(vlHogVariant, numBins, false); // VlHogVariantUoctti seems to be default in Matlab.
				vl_hog_put_image(hog, (float*)roiImg.data, roiImg.cols, roiImg.rows, 1, cellSize); // (the '1' is numChannels)
				vl_size ww = vl_hog_get_width(hog); // we could assert that ww == hh == numCells
				vl_size hh = vl_hog_get_height(hog);
				vl_size dd = vl_hog_get_dimension(hog); // assert ww=hogDim1, hh=hogDim2, dd=hogDim3
				//float* hogArray = (float*)malloc(ww*hh*dd*sizeof(float));
				Mat hogArray(1, ww*hh*dd, CV_32FC1); // safer & same result. Don't use C-style memory management.
				//vl_hog_extract(hog, hogArray); // just interpret hogArray in col-major order to get the same n x 1 vector as in matlab. (w * h * d)
				vl_hog_extract(hog, hogArray.ptr<float>(0));
				//vl_hog_delete(hog);
				Mat hogDescriptor(hh*ww*dd, 1, CV_32FC1);
				// Stack the third dimensions of the HOG descriptor of this patch one after each other in a column-vector
				for (int j = 0; j < dd; ++j) 
				{
					//Mat hogFeatures(hh, ww, CV_32FC1, hogArray + j*ww*hh);
					Mat hogFeatures(hh, ww, CV_32FC1, hogArray.ptr<float>(0) + j*ww*hh); // Creates the same array as in Matlab. I might have to check this again if hh!=ww (non-square)
					hogFeatures = hogFeatures.t(); // Necessary because the Matlab reshape() takes column-wise from the matrix while the OpenCV reshape() takes row-wise.
					hogFeatures = hogFeatures.reshape(0, hh*ww); // make it to a column-vector
					Mat currentDimSubMat = hogDescriptor.rowRange(j*ww*hh, j*ww*hh + ww*hh);
					hogFeatures.copyTo(currentDimSubMat);
				}
				//free(hogArray); // not necessary - we use a Mat.
				//features = [features; double(reshape(tmp, [], 1))];
				// B = reshape(A,m,n) returns the m-by-n matrix B whose elements are taken column-wise from A
				// Matlab (& Eigen, OpenGL): Column-major.
				// OpenCV: Row-major.
				// (access is always (r, c).)
				//Mat currentFeaturesSubrange = hogDescriptors.rowRange(i*hogDims, i*hogDims + hogDims);
				//hogDescriptor.copyTo(currentFeaturesSubrange);
				hogDescriptor = hogDescriptor.t(); // now a row-vector
				hogDescriptors.push_back(hogDescriptor);
				
			}
			roiImg.release();
			vl_hog_delete(hog);
			// hogDescriptors needs to have dimensions numLandmarks x hogFeaturesDimension, where hogFeaturesDimension is e.g. 3*3*16=144
			return hogDescriptors;
		}		
		
	};
	

private:
	VlHogType hogType;
	int numCells;
	int cellSize;
	int numBins;
};


#endif /* DESCRIPTOREXTRACTOR_HPP_ */

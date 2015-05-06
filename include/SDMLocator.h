#ifndef __H_H_SDM_LOCATOR_H_H__
#define __H_H_SDM_LOCATOR_H_H__

#include "DescriptorExtractor.h"
#include <memory>

using namespace std;
using namespace cv;
using cv::Mat;
using cv::Scalar;
using std::vector;
using std::string;

class SDMLocator
{
public:
	
	SDMLocator();

public:
	
	int getNumLandmarks() const;
	int getNumCascadeSteps() const;
	cv::Mat getMeanShape() const;
	cv::Mat getRegressorData(int cascadeLevel);
	std::shared_ptr<DescriptorExtractor> getDescriptorExtractor(int cascadeLevel);
	
	struct HogParameter // Todo remove?
	{
		int numCells;
		int cellSize;
		int numBins;
	};
	HogParameter getHogParameters(int cascadeLevel) 
	{
		return hogParameters[cascadeLevel];
	}

public:
	bool detect49(cv::Mat& meanShape, cv::Mat& image, cv::Rect faceBox) ;
	void track49(cv::Mat& meanShape, cv::Mat& modelShape,  cv::Mat& image) ;
	
	void writeBinModel(string filename);
	bool readBinModel(string filename);

	void Release();

private:
	cv::Mat meanLandmarks; // 1 x numLandmarks*2. First all the x-coordinates, then all the y-coordinates.
	std::vector<cv::Mat> regressorData; // Holds the training data, one cv::Mat for each cascade level. Every Mat is (numFeatureDim+1) x numLandmarks*2 (for x & y)

	std::vector<HogParameter> hogParameters;
	std::vector<std::shared_ptr<DescriptorExtractor>> descriptorExtractors;
	
};

#endif
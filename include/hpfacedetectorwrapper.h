/* 
   Description: HP Video Face Detector Wrapper
   Author:      Yongqiang.Mou(mu-yong.qiang@hp.com)
   Contributor: Qian.Lin(qian.lin@hp.com), Xianwang.Wang(xianwang.wang@hp.com), 
                Peng.Wu(peng.wu@hp.com), Min.Xu(mxu@hp.com), Yangbo.Zhou(zhou.yangbo@hp.com) 
   Date:        2013.09.22
*/


#ifndef __H_H_LIB_HP_FACE_DETECTOR_WARPPER_H_H__
#define __H_H_LIB_HP_FACE_DETECTOR_WARPPER_H_H__

#include <cstdio>
#include <exception>
#include <vector>

#include "face_detect_lib.h"

using namespace std;

struct EyePoint
{
	int			xleft;
	int			yleft;
	int			xright;
	int			yright;
	float		confidence;
};

struct FapRect
{
	int left;
	int top;
	int right;
	int bottom;       // left, top, right and bottom co-ordinates of a rectangle.
	FapRect() {left=top=right=bottom=0;};
	FapRect(int l,int t,int r,int b)
	{
		if (l>r || t>b)
		{
			//throw std::exception::exception();
			throw std::exception();
		}
		left    = l;
		top     = t;
		right   = r;
		bottom  = b;
	};
};


struct Face
{
	FapRect rect;
	int centerX;
	int centerY;
	int radius;
	int inPlaneRot;
	int horzOopRot;
	int vertOopRot;
	int confidence;
	int rotationState;
};
typedef std::vector<Face> FaceArray;


class HPFaceDetectorWarpper
{
public:
	HPFaceDetectorWarpper(bool bMode = true);
	~HPFaceDetectorWarpper();
	int Init(bool bHalfProfile = true, bool bProfile = false, bool bRot = true);
	int DetectFaces(unsigned char* ucImgData, const int nImgWidth, const int nImgHeight, FaceArray& faceArray);
private:
	void* m_fdp;
	bool m_bVideoMode;
};
#endif
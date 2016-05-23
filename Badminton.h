#ifndef BADMINTON_H_
#define BADMINTON_H_

#include "stdafx.h"
#include <opencv2/opencv.hpp>
#include <Kinect.h>

using namespace cv;

class Badminton
{
public:
	Badminton();
	void m_getObjectColorSpacePoint(RGBQUAD* pColorBuffer, int iColorWidth, int iColorHeight, ColorPoint* boundingCenterPoint, ColorPoint* boundingRange, float* minRect_Angle, ColorPoint* A, ColorPoint* B, ColorPoint* C);
	void m_getObjectInfraredSpacePoint(UINT16* pInfraredBuffer, int iInfraredWidth, int iInfraredHeight, ColorPoint* infraredSpacePoint, ColorPoint* range);
private:
	Mat m_CloseRect;
	//CvPoint m_ObjectColorPoint;
};
#endif
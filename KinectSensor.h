#ifndef KINECTSENSOR_H_
#define KINECTSENSOR_H_
#include <Kinect.h>
#include <math.h>
#include <vector>
using namespace std;

template<typename T>
void vectorpolyfit(const vector<typename T>& x, const vector<typename T>& y, int Point_Number, int Fit_N, double *ks);

class KinectSensor
{
public:
	static const int     cColorWidth  = 1920;
	static const int     cColorHeight = 1080;
	static const int     cInfraredWidth  = 512;
	static const int     cInfraredHeight = 424;
	KinectSensor();
	~KinectSensor();
	HRESULT              m_hInitKinect();
	HRESULT              m_hGetDepthFrameReader(HRESULT hr);
	HRESULT              m_hGetColorFrameReader(HRESULT hr);
	HRESULT              m_hGetInfraredFrameReader(HRESULT hr);
	void Update();
	void m_doMapColorToCamera(UINT16* pDepthFrameData, UINT nDepthDataPointCount);
	void m_doMapInfraredToCamera(UINT16* pDepthFrameData, UINT nDepthDataPointCount);
	void m_getColorFrameToCameraSpacePoint(RGBQUAD* pColorBuffer, int iColorWidth, int iColorHeight, UINT16* pDepthFrameData, UINT nDepthDataPointCount);
	void m_getInfraredFrameToCameraSpacePoint(UINT16* pInfraredBuffer, int iInfraredWidth, int iInfraredHeight, UINT16* pDepthFrameData, UINT nDepthDataPointCount);
private:
	IKinectSensor*       m_pKinectSensor;
	IColorFrameReader*   m_pColorFrameReader;
	IDepthFrameReader*   m_pDepthFrameReader;
	IInfraredFrameReader* m_pInfraredFrameReader;
	RGBQUAD*             m_pColorRGBX;
};
#endif
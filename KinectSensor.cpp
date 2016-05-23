#include "stdafx.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <vector>
#include <math.h>
#include "KinectSensor.h"
#include "Badminton.h"
#include <windows.h>
#include <winbase.h>
#include "RobotControl.h"

#define PI 3.1415926

using namespace cv;
using namespace std;

extern bool MouseButtonFlag;

int dataMutationFlag = 0;
RNG rng(12345);
//////////////////////////////////////////////////////
int Angle_of_Elevation_Kinect = 20;//Kinect的仰角
int Drift_Angle_Kinect = 0;//Kinect的偏航角
//////////////////////////////////////////////////////
int Point_Index = -1;
vector <float> Trajectory_P_X;//实际物理空间的轨迹点X坐标
vector <float> Trajectory_P_Y;//实际物理空间的轨迹点Y坐标
vector <float> Trajectory_P_Z;//实际物理空间的轨迹点Z坐标
int start_calc_T = 0;//开始计算预测落点的时刻点
int end_calc_T = 0;//结束计算预测落点的时刻点
int spend_calc_T = 0;//算法计算预测落点所花费的时间
int Badminton_Orientation = 0;//球迹方向判断的变量，定义：-1为我方打向对方的球，0为初值，1为对方打向我方的球


KinectSensor::KinectSensor()
{
	m_pKinectSensor       = nullptr;
	m_pColorFrameReader   = nullptr;
	m_pDepthFrameReader   = nullptr;
	m_pInfraredFrameReader = nullptr;
	m_pColorRGBX          = new RGBQUAD[cColorHeight*cColorWidth];
	/////////////////////////////////////////////////////
	Trajectory_P_X.reserve(10);//给长度不变，容量被置为10
	Trajectory_P_Y.reserve(10);//给长度不变，容量被置为10
	Trajectory_P_Z.reserve(10);//给长度不变，容量被置为10
	/////////////////////////////////////////////////////
}

KinectSensor::~KinectSensor()
{
	if (m_pColorRGBX) {
		delete[] m_pColorRGBX;
		m_pColorRGBX = nullptr;
	}

	SafeRelease(m_pInfraredFrameReader);
	SafeRelease(m_pDepthFrameReader);
	SafeRelease(m_pColorFrameReader);

	if (m_pKinectSensor) {
		m_pKinectSensor->Close();
	}

	SafeRelease(m_pKinectSensor);
}

/*
 * 获得并打开Kinect传感器
 * 同时将状态输出、供下一步打开深度数据的读头
 * one (m_pKinectSensor)
 */
HRESULT KinectSensor::m_hInitKinect()
{
	HRESULT hr;
	hr = GetDefaultKinectSensor(&m_pKinectSensor);

	if (FAILED(hr)) {
		return hr;
	}

	if (m_pKinectSensor) {
		hr = m_pKinectSensor->Open();
	}

	if (!m_pKinectSensor || FAILED(hr)) {
		return E_FAIL;
	}

	return hr;
}

/*
 * 如果成功打开Kinect传感器的话就打开深度数据的读头(成员变量m_pDepthFrameReader)
 * 然后将状态输出、供下一步打开彩色数据的读头
 * one (m_pDepthFrameReader)
 */
HRESULT KinectSensor::m_hGetDepthFrameReader(HRESULT hr)
{
	IDepthFrameSource* pDepthFrameSource = nullptr;

	if (SUCCEEDED(hr)) {
		hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
	}

	if (SUCCEEDED(hr)) {
		hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
	}

	SafeRelease(pDepthFrameSource);

	if (FAILED(hr)) {
		return E_FAIL;
	}

	return hr;
}

/*
 * 如果成功打开深度数据的读头,那么就打开红外数据的读头(成员变量m_pInfraredFrameReader)
 * 并将状态输出
 * one (m_pInfraredFrameReader)
 */
HRESULT KinectSensor::m_hGetInfraredFrameReader(HRESULT hr)
{
	IInfraredFrameSource* pInfraredFrameSource = nullptr;

	if (SUCCEEDED(hr)) {
		hr = m_pKinectSensor->get_InfraredFrameSource(&pInfraredFrameSource);
	}

	if (SUCCEEDED(hr)) {
		hr = pInfraredFrameSource->OpenReader(&m_pInfraredFrameReader);
	}

	SafeRelease(pInfraredFrameSource);

	if (FAILED(hr)) {
		return E_FAIL;
	}

	return hr;
}

/*
 * 如果成功打开深度数据的读头,那么就打开彩色数据的读头(成员变量m_pColorFrameReader)
 * 并将状态输出
 * one (m_pColorFrameReader)
 */
HRESULT KinectSensor::m_hGetColorFrameReader(HRESULT hr)
{
	IColorFrameSource* pColorFrameSource = nullptr;

	if (SUCCEEDED(hr)) {
		hr = m_pKinectSensor->get_ColorFrameSource(&pColorFrameSource);
	}

	if (SUCCEEDED(hr)) {
		hr = pColorFrameSource->OpenReader(&m_pColorFrameReader);
	}

	SafeRelease(pColorFrameSource);

	if (FAILED(hr)) {
		return E_FAIL;
	}

	return hr;
}

/*
 * 如果深度数据的读头(m_pDepthFrameReader)打开的话，执行此函数；否则退出
 * 该函数实现获取数据 iDepthWidth, iDepthHeight, nDepthBufferSize, pDepthBuffer （AccessUnderlyingBuffer方式）
 * 同时计算出 nDepthDataPointCount (512*424)
 * 如果打开彩色数据读头(m_pColorFrameReader)的话，获取 pDepthFrameData (CopyFrameDataToArray方式，数据供彩色图处理用)
 * 然后运行函数 doMapColorToCamera (pDepthFrameData, nDepthDataPointCount) (为在彩色图下处理数据做准备)
 * 否则，运行成员函数 m_getDepthFrameToCameraSpacePoint (在深度图下进行剩下所有工作)
 * one (pDepthBuffer)
 */
void KinectSensor::Update()
{
	if (!m_pDepthFrameReader) {
		return;
	}

	IDepthFrame* pDepthFrame = nullptr;
	HRESULT hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);

	if (SUCCEEDED(hr))
	{
		int iDepthWidth           = 0;
		int iDepthHeight          = 0;
		UINT nDepthBufferSize     = 0;
		UINT nDepthDataPointCount = 0;
		UINT16* pDepthBuffer      = nullptr;

		IFrameDescription* pDepthFrameDescription = nullptr;
		hr = pDepthFrame->get_FrameDescription(&pDepthFrameDescription);

		if (SUCCEEDED(hr)) {
			hr = pDepthFrameDescription->get_Width(&iDepthWidth);
		}

		if (SUCCEEDED(hr)) {
			hr = pDepthFrameDescription->get_Height(&iDepthHeight);
		}

		if (SUCCEEDED(hr)) {
			hr = pDepthFrame->AccessUnderlyingBuffer(&nDepthBufferSize, &pDepthBuffer);
		}

		if (SUCCEEDED(hr)) {
			nDepthDataPointCount = iDepthWidth*iDepthHeight;
			UINT16* pDepthFrameData = new UINT16[nDepthDataPointCount];
			hr = pDepthFrame->CopyFrameDataToArray(nDepthDataPointCount, pDepthFrameData);

			/*
			* 如果彩色数据的读头成功打开的话就用彩色图进行处理
			* 反之，则用红外数据进行处理
			* 这样便于在红外图与彩色图之间进行选择
			*/
			if (SUCCEEDED(hr)) {
				if (m_pColorFrameReader)
					m_doMapColorToCamera(pDepthFrameData, nDepthDataPointCount);
				else
					m_doMapInfraredToCamera(pDepthFrameData, nDepthDataPointCount);
			}
				delete[] pDepthFrameData;
		}

		SafeRelease(pDepthFrameDescription);
		SafeRelease(pDepthFrame);
	}
}

/*
 * 如果彩色数据的读头 (m_pColorFrameReader)打开的话，就执行此函数；
 * 否则，执行 m_getDepthFrameToCameraSpacePoint (pDepthBuffer, iDepthWidth, iDepthHeight) 函数
 * 该函数实现获取数据 iColorWidth, iColorHeight;
 * 同时对彩色帧数据进行判断，如果数据是RGBA格式的话，获取 nColorBufferSize, pColorBuffer (AccessRawUnderlyingBuffer方式)
 * 如果是其他格式的话，同样获取 nColorBufferSize, pColorBuffer (CopyConvertedFrameDataToArray方式，同时将数据转成RGBA格式)
 * 以上一切OK的话，运行成员函数 m_getColorFrameToCameraSpacePoint(在彩色图下进行剩下所有工作)
 * one (pColorBuffer)
 */
void KinectSensor::m_doMapColorToCamera(UINT16* pDepthFrameData, UINT nDepthDataPointCount)
{
	IColorFrame* pColorFrame = nullptr;
	HRESULT hr = m_pColorFrameReader->AcquireLatestFrame(&pColorFrame);

	if (SUCCEEDED(hr))
	{
		int iColorWidth                   = 0;
		int iColorHeight                  = 0;
		UINT nColorBufferSize             = 0;
		RGBQUAD* pColorBuffer             = nullptr;
		ColorImageFormat colorImageFormat = ColorImageFormat_None;

		/* 检查彩色图片帧频率(光线弱时15Hz)， 为后面数据分析准备，该方法只有彩色帧有，深度帧频率是固定的(30Hz) */
		/**
		IColorCameraSettings* pColorCameraSettings = nullptr;
		hr = pColorFrame->get_ColorCameraSettings(&pColorCameraSettings);
		TIMESPAN frameInterval;
		hr = pColorCameraSettings->get_FrameInterval(&frameInterval);
		if (SUCCEEDED(hr)) {
			printf("the colorframe interval is %d\n", frameInterval);
		}
		SafeRelease(pColorCameraSettings);
		**/

		/* 获得每帧彩色图片的宽和高，方便后面检查是否获取到全部数据 */
		IFrameDescription* pColorFrameDescription = nullptr;
		hr = pColorFrame->get_FrameDescription(&pColorFrameDescription);

		if (SUCCEEDED(hr)) {
			hr = pColorFrameDescription->get_Width(&iColorWidth);
		}

		if (SUCCEEDED(hr)) {
			hr = pColorFrameDescription->get_Height(&iColorHeight);
		}

		if (SUCCEEDED(hr)) {
			hr = pColorFrame->get_RawColorImageFormat(&colorImageFormat);
		}

		if (SUCCEEDED(hr))
		{
			if (colorImageFormat == ColorImageFormat_Bgra) {
				hr = pColorFrame->AccessRawUnderlyingBuffer(&nColorBufferSize, reinterpret_cast<BYTE**>(&pColorBuffer));
			}
			else if (m_pColorRGBX) {
				pColorBuffer     = m_pColorRGBX;
				nColorBufferSize = cColorWidth*cColorHeight*sizeof(RGBQUAD);
				hr = pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE*>(pColorBuffer), ColorImageFormat_Bgra);
			}
			else {
				hr = E_FAIL;
			}
		}

		if (SUCCEEDED(hr)) {
			m_getColorFrameToCameraSpacePoint(pColorBuffer, iColorWidth, iColorHeight, pDepthFrameData, nDepthDataPointCount);
		}

		SafeRelease(pColorFrameDescription);
	}
	SafeRelease(pColorFrame);
}

void KinectSensor::m_doMapInfraredToCamera(UINT16* pDepthFrameData, UINT nDepthDataPointCount)
{

	IInfraredFrame* pInfraredFrame = nullptr;
	HRESULT hr = m_pInfraredFrameReader->AcquireLatestFrame(&pInfraredFrame);

	if (SUCCEEDED(hr))
	{
		int iInfraredWidth = 0;
		int iInfraredHeight = 0;
		UINT nInfraredFrameDataSize = 0;
		UINT16* pInfraredBuffer = nullptr;

		/* 获得每帧彩色图片的宽和高，方便后面检查是否获取到全部数据 */
		IFrameDescription* pInfraredFrameDescription = nullptr;
		hr = pInfraredFrame->get_FrameDescription(&pInfraredFrameDescription);

		if (SUCCEEDED(hr)) {
			hr = pInfraredFrameDescription->get_Width(&iInfraredWidth);
		}

		if (SUCCEEDED(hr)) {
			hr = pInfraredFrameDescription->get_Height(&iInfraredHeight);
		}

		if (SUCCEEDED(hr)) {
			hr = pInfraredFrame->AccessUnderlyingBuffer(&nInfraredFrameDataSize, &pInfraredBuffer);
		}

		if (SUCCEEDED(hr)) {
			m_getInfraredFrameToCameraSpacePoint(pInfraredBuffer, iInfraredWidth, iInfraredHeight, pDepthFrameData, nDepthDataPointCount);
		}

		SafeRelease(pInfraredFrameDescription);
	}

	SafeRelease(pInfraredFrame);

}

/*
 * 该函数基于 Kinect 彩色图进行数据处理，得到羽毛球落点
 * Ⅰ:如果成功得到彩色图像数据 pColorBuffer，且获得的数据 iColorWidth 及 iColorHeight 正确的话
 * 进入函数 m_getObjectColorSpacePoint (badminton对象下)，获得目标点在彩色图中的坐标位置 (1920 * 1080 坐标系下)
 * Ⅱ:如果 Ⅰ成功，那么进行坐标转换，将整个彩色图像整帧数据(1920 * 1080个)转换到 Kinect 摄像机空间 (以Kinect为坐标原点)
 * Ⅲ:结合Ⅰ和 Ⅱ得到目标点相对于Kinect摄像机空间的位置
 * Ⅳ:剔除无效点 (数值为无穷大的点)，然后将剔除后的点进行场地坐标转换，并将转换后的结果发送出去
 */
void KinectSensor::m_getColorFrameToCameraSpacePoint(RGBQUAD* pColorBuffer, int iColorWidth, int iColorHeight, UINT16* pDepthFrameData, UINT nDepthDataPointCount)
{
	if (pColorBuffer && (iColorWidth == cColorWidth) && (iColorHeight == cColorHeight) && (nDepthDataPointCount == cInfraredWidth*cInfraredHeight))
	{
		/*
		 * 自定义一个结构体 ColorPoint (因为Kinect SDK里面的结构体 ColorSpacePoint 其成员X Y 均为float型，当将minRect.center赋给它时会出错、蛋疼)
		 * 将结构体 colorSpacePoint 初始化为 {-1，-1}，便于后面进行判断
		 */
		ColorPoint boundingCenterPoint = { -1, -1 }; 
		ColorPoint boundingRange = { 0, 0 };
		ColorPoint A = { 0, 0 };
		ColorPoint B = { 0, 0 };
		ColorPoint C = { 0, 0 };
		//////////////////////////////////////////////////////////////////////////////////
		float badminton_Angle = 0;
		//////////////////////////////////////////////////////////////////////////////////
		/* Ⅰ:获得目标点在彩色图中的坐标 */
		Badminton badminton;
		badminton.m_getObjectColorSpacePoint(pColorBuffer, iColorWidth, iColorHeight, &boundingCenterPoint, &boundingRange, &badminton_Angle, &A, &B, &C);
		////////////////////////////////////////////////////////////////////
		Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));		
		////////////////////////////////////////////////////////////////////
		if ((boundingCenterPoint.x != -1) && (boundingCenterPoint.y != -1))
		{
			/* Ⅱ:将彩色图整帧数据从彩色图空间转换到 Kinect 摄像机空间 */
			CameraSpacePoint* pCameraSpacePoints = new CameraSpacePoint[iColorWidth*iColorHeight];
			ICoordinateMapper* pCoodinateMapper  = nullptr;

			m_pKinectSensor->get_CoordinateMapper(&pCoodinateMapper);
			pCoodinateMapper->MapColorFrameToCameraSpace(nDepthDataPointCount, pDepthFrameData, iColorWidth*iColorHeight, pCameraSpacePoints);
			SafeRelease(pCoodinateMapper);

			CameraSpacePoint objectCameraSpacePoint = { 0.0, 0.0, 0.0 };
			int iObjectPixel = 0;
			bool flag = true;
			for (int x = (boundingCenterPoint.x - boundingRange.x); flag && (x <= (boundingCenterPoint.x + boundingRange.x)); x++)
			{
				for (int y = (boundingCenterPoint.y - boundingRange.y); flag && (y <= (boundingCenterPoint.y + boundingRange.y)); y++)
				{
					if (is_PointinTriangle(A, B, C, x, y))
					{
						//printf("it's ok\n");
						iObjectPixel = (iColorWidth)*y + x;
						objectCameraSpacePoint = pCameraSpacePoints[iObjectPixel];
						if (isfinite(objectCameraSpacePoint.Z)) {
							flag = false;
						}
							
					}
				}
			}
			//////////////////////////////////////////////////////
			delete[] pCameraSpacePoints;
			/*
			 * 1、将无效(无限大)的数据点剔除：判断追踪获取的数据是否有效(数值有限大)，
			 * 2、将发生数据突变的数据点剔除：根据该点与前一点(有效点) Z 方向坐标值之差的绝对值是否在一个设置的阈值 fMutationThreshold 内来判断是否突变
			 * 3、判断羽球飞行的方向：根据该点与与前一点(有效非突变点) X 方向坐标值之差的正负性来判断：为正，则对方发球；为负，则为我方回球；
			 * 注意：该判断方法前提是进入视野的第一个羽毛球数据是正确有效的，我们通过按鼠标触发来开始这一进程
			 */
			//float fMutationThreshold = 2.0;


			if (isfinite(objectCameraSpacePoint.Z) && (MouseButtonFlag == true) && (objectCameraSpacePoint.Z > 0.3) && (objectCameraSpacePoint.Z < 7.5))
			{
				//printf("the badminton_Angle is %f\n", badminton_Angle);
				printf("the camerapoint is %f  %f  %f\n", objectCameraSpacePoint.X, objectCameraSpacePoint.Y, objectCameraSpacePoint.Z);
				fstream file; //txt文件的存储路径为 D : \ZXE - Files\file
				file.open("D://ZXE-Files//file//file.txt", ios::app);
				file << "camerapoint " << objectCameraSpacePoint.X << " " << objectCameraSpacePoint.Y << " " << objectCameraSpacePoint.Z << endl;
				file.close();
				//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				CameraSpacePoint badmintonSpacePoint = { 0.0, 0.0, 0.0 };//用来存储预测落点X,Y,Z
				CameraSpacePoint badmintonCourtSpacePoint = { 0.0, 0.0, 0.0 };//场地坐标系下的球迹点坐标 int x,y,z
				///////////////////////////////////////////////////////
				Point_Index++;
				
				//实际物理空间的X,Y,Z
				Trajectory_P_X.resize(Point_Index + 10);
				Trajectory_P_Y.resize(Point_Index + 10);
				Trajectory_P_Z.resize(Point_Index + 10);

				//实际物理空间的X,Y,Z
				Trajectory_P_X[Point_Index] = objectCameraSpacePoint.X*cos(Drift_Angle_Kinect*PI/180) + objectCameraSpacePoint.Z*sin(Drift_Angle_Kinect*PI/180);
				Trajectory_P_Y[Point_Index] = objectCameraSpacePoint.Y*cos(Angle_of_Elevation_Kinect*PI/180) + objectCameraSpacePoint.Z*sin(Angle_of_Elevation_Kinect*PI/180);
				Trajectory_P_Z[Point_Index] = -objectCameraSpacePoint.X*sin(Drift_Angle_Kinect*PI/180) - objectCameraSpacePoint.Y*sin(Angle_of_Elevation_Kinect*PI/180) + objectCameraSpacePoint.Z*cos(Angle_of_Elevation_Kinect*PI/180)*cos(Drift_Angle_Kinect*PI/180);
								
					if (Point_Index == 0)
					{
						//得到开始检测到羽毛球时的系统时间
						SYSTEMTIME start_systemtime;
						GetLocalTime(&start_systemtime);
						start_calc_T = start_systemtime.wMinute * 60 * 1000 + start_systemtime.wSecond * 1000 + start_systemtime.wMilliseconds * 1;
						if (boundingCenterPoint.x > 980)
						{
							Badminton_Orientation = 1;//如果等于1说明是对方打过来的球
						}
						else
						{
							Badminton_Orientation = -1;//如果等于-1说明是我方打过去的球
						}
					}
					if ((Point_Index >= 2))// && (Badminton_Orientation == 1)
					{
						double Fac[3] = { 0 };//c,b,a
						double fac[2] = { 0 };//b,k													
						//实际物理空间的X,Y,Z的拟合
						//通过vectorpolyfit拟合得到的二次多项式方程形式为：  y = c + b * x^1 + a * x^2 
						vectorpolyfit(Trajectory_P_X, Trajectory_P_Y, Point_Index + 1, 2, Fac);
						//通过vectorpolyfit拟合得到的一次方程形式为：  z = kx + b
						vectorpolyfit(Trajectory_P_X, Trajectory_P_Z, Point_Index + 1, 1, fac);
						printf("Fac[0] Fac[1] Fac[2] is %g  %g  %g\n", Fac[0], Fac[1], Fac[2]);
						printf("fac[0] fac[1] is %g  %g\n", fac[0], fac[1]);
						//预判拟合轨迹的落点							
						//对于二次多项式轨迹方程，利用求根公式就可以求其y=f(x)中的x值：x1=(-b+sqrt(pow(b,2)-4*a*c))/(2*a)  x2=(-b-sqrt(pow(b,2)-4*a*c))/(2*a)
						badmintonSpacePoint.Y = -1.45;
						badmintonSpacePoint.X = (-Fac[1] - sqrt(pow(Fac[1], 2) - 4 * Fac[2] *(Fac[0] - badmintonSpacePoint.Y))) / (2 * Fac[2]);//两个解取大者
						//通过Kinect坐标系的XOZ平面的斜率来求，也即是一个一次多项式的拟合
						badmintonSpacePoint.Z = fac[1] * badmintonSpacePoint.X + fac[0];
						printf("X  Y  Z  is %f  %f  %f\n", badmintonSpacePoint.X, badmintonSpacePoint.Y, badmintonSpacePoint.Z);
						//预测落点输出   txt文件的存储路径为 D:\ZXE-Files\file			
						file.open("D://ZXE-Files//file//file_forecast.txt", ios::app);
						file << "badmintonpoint " << badmintonSpacePoint.X << " " << badmintonSpacePoint.Y << " " << badmintonSpacePoint.Z << endl;
						file.close();
						///////////////////////////////////////
						//进行坐标转换，先按方案一的Kinect摆放位置进行坐标转换
						transferCoordinate(badmintonSpacePoint, &badmintonCourtSpacePoint);
						printf("Court_X  Court_Y  Court_Z  is %f  %f  %f\n", badmintonCourtSpacePoint.X, badmintonCourtSpacePoint.Y, badmintonCourtSpacePoint.Z);
						//预测落点输出   txt文件的存储路径为 D:\ZXE-Files\file			
						file.open("D://ZXE-Files//file//file_forecast_court.txt", ios::app);
						file << "Court_badmintonpoint " << badmintonCourtSpacePoint.X << " " << badmintonCourtSpacePoint.Y << " " << badmintonCourtSpacePoint.Z << endl;
						file.close();
						//进行串口发送
							
						if ((Point_Index == 2) && (badmintonCourtSpacePoint.Y > 0) && (badmintonCourtSpacePoint.X > -3050) && (badmintonCourtSpacePoint.X < 3050))
						{
							//只有落球点在场地范围内才进行发送，否则不发送，防止机器人跑出场地
							transmitData(badmintonCourtSpacePoint);
						}
							
						///////////////////////////////////////

						//得到可以开始预测羽毛球落点时的系统时间
						SYSTEMTIME end_systemtime;
						GetLocalTime(&end_systemtime);
						end_calc_T = end_systemtime.wMinute * 60 * 1000 + end_systemtime.wSecond * 1000 + end_systemtime.wMilliseconds * 1;
							
						spend_calc_T = end_calc_T - start_calc_T;//算法处理计算所花费的时间
						printf("spend_calc_T  is  %2d ms\n", spend_calc_T);							
					}			
			}
		}
	}
}

void KinectSensor::m_getInfraredFrameToCameraSpacePoint(UINT16* pInfraredBuffer, int iInfraredWidth, int iInfraredHeight, UINT16* pDepthFrameData, UINT nDepthDataPointCount)
{
	if (pInfraredBuffer && (iInfraredWidth == cInfraredWidth) && (iInfraredHeight == cInfraredHeight) && (nDepthDataPointCount == cInfraredWidth*cInfraredHeight))
	{
	}
}
template<typename T>
void vectorpolyfit(const vector<typename T>& x, const vector<typename T>& y, int Point_Number, int Fit_N, double *ks)
{
	double *X, *Y;
	Y = new double[Point_Number];
	X = new double[Point_Number];
	for (int i = 0; i < Point_Number; i++)
	{
		X[i] = x[i];
		Y[i] = y[i];
	}
	Matrix_PolyFit(X, Y, Point_Number, Fit_N, ks);
	delete[]Y;
	delete[]X;
}
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
int Angle_of_Elevation_Kinect = 20;//Kinect������
int Drift_Angle_Kinect = 0;//Kinect��ƫ����
//////////////////////////////////////////////////////
int Point_Index = -1;
vector <float> Trajectory_P_X;//ʵ������ռ�Ĺ켣��X����
vector <float> Trajectory_P_Y;//ʵ������ռ�Ĺ켣��Y����
vector <float> Trajectory_P_Z;//ʵ������ռ�Ĺ켣��Z����
int start_calc_T = 0;//��ʼ����Ԥ������ʱ�̵�
int end_calc_T = 0;//��������Ԥ������ʱ�̵�
int spend_calc_T = 0;//�㷨����Ԥ����������ѵ�ʱ��
int Badminton_Orientation = 0;//�򼣷����жϵı��������壺-1Ϊ�ҷ�����Է�����0Ϊ��ֵ��1Ϊ�Է������ҷ�����


KinectSensor::KinectSensor()
{
	m_pKinectSensor       = nullptr;
	m_pColorFrameReader   = nullptr;
	m_pDepthFrameReader   = nullptr;
	m_pInfraredFrameReader = nullptr;
	m_pColorRGBX          = new RGBQUAD[cColorHeight*cColorWidth];
	/////////////////////////////////////////////////////
	Trajectory_P_X.reserve(10);//�����Ȳ��䣬��������Ϊ10
	Trajectory_P_Y.reserve(10);//�����Ȳ��䣬��������Ϊ10
	Trajectory_P_Z.reserve(10);//�����Ȳ��䣬��������Ϊ10
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
 * ��ò���Kinect������
 * ͬʱ��״̬���������һ����������ݵĶ�ͷ
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
 * ����ɹ���Kinect�������Ļ��ʹ�������ݵĶ�ͷ(��Ա����m_pDepthFrameReader)
 * Ȼ��״̬���������һ���򿪲�ɫ���ݵĶ�ͷ
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
 * ����ɹ���������ݵĶ�ͷ,��ô�ʹ򿪺������ݵĶ�ͷ(��Ա����m_pInfraredFrameReader)
 * ����״̬���
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
 * ����ɹ���������ݵĶ�ͷ,��ô�ʹ򿪲�ɫ���ݵĶ�ͷ(��Ա����m_pColorFrameReader)
 * ����״̬���
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
 * ���������ݵĶ�ͷ(m_pDepthFrameReader)�򿪵Ļ���ִ�д˺����������˳�
 * �ú���ʵ�ֻ�ȡ���� iDepthWidth, iDepthHeight, nDepthBufferSize, pDepthBuffer ��AccessUnderlyingBuffer��ʽ��
 * ͬʱ����� nDepthDataPointCount (512*424)
 * ����򿪲�ɫ���ݶ�ͷ(m_pColorFrameReader)�Ļ�����ȡ pDepthFrameData (CopyFrameDataToArray��ʽ�����ݹ���ɫͼ������)
 * Ȼ�����к��� doMapColorToCamera (pDepthFrameData, nDepthDataPointCount) (Ϊ�ڲ�ɫͼ�´���������׼��)
 * �������г�Ա���� m_getDepthFrameToCameraSpacePoint (�����ͼ�½���ʣ�����й���)
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
			* �����ɫ���ݵĶ�ͷ�ɹ��򿪵Ļ����ò�ɫͼ���д���
			* ��֮�����ú������ݽ��д���
			* ���������ں���ͼ���ɫͼ֮�����ѡ��
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
 * �����ɫ���ݵĶ�ͷ (m_pColorFrameReader)�򿪵Ļ�����ִ�д˺�����
 * ����ִ�� m_getDepthFrameToCameraSpacePoint (pDepthBuffer, iDepthWidth, iDepthHeight) ����
 * �ú���ʵ�ֻ�ȡ���� iColorWidth, iColorHeight;
 * ͬʱ�Բ�ɫ֡���ݽ����жϣ����������RGBA��ʽ�Ļ�����ȡ nColorBufferSize, pColorBuffer (AccessRawUnderlyingBuffer��ʽ)
 * �����������ʽ�Ļ���ͬ����ȡ nColorBufferSize, pColorBuffer (CopyConvertedFrameDataToArray��ʽ��ͬʱ������ת��RGBA��ʽ)
 * ����һ��OK�Ļ������г�Ա���� m_getColorFrameToCameraSpacePoint(�ڲ�ɫͼ�½���ʣ�����й���)
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

		/* ����ɫͼƬ֡Ƶ��(������ʱ15Hz)�� Ϊ�������ݷ���׼�����÷���ֻ�в�ɫ֡�У����֡Ƶ���ǹ̶���(30Hz) */
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

		/* ���ÿ֡��ɫͼƬ�Ŀ�͸ߣ�����������Ƿ��ȡ��ȫ������ */
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

		/* ���ÿ֡��ɫͼƬ�Ŀ�͸ߣ�����������Ƿ��ȡ��ȫ������ */
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
 * �ú������� Kinect ��ɫͼ�������ݴ����õ���ë�����
 * ��:����ɹ��õ���ɫͼ������ pColorBuffer���һ�õ����� iColorWidth �� iColorHeight ��ȷ�Ļ�
 * ���뺯�� m_getObjectColorSpacePoint (badminton������)�����Ŀ����ڲ�ɫͼ�е�����λ�� (1920 * 1080 ����ϵ��)
 * ��:��� ��ɹ�����ô��������ת������������ɫͼ����֡����(1920 * 1080��)ת���� Kinect ������ռ� (��KinectΪ����ԭ��)
 * ��:��Ϣ�� ��õ�Ŀ��������Kinect������ռ��λ��
 * ��:�޳���Ч�� (��ֵΪ�����ĵ�)��Ȼ���޳���ĵ���г�������ת��������ת����Ľ�����ͳ�ȥ
 */
void KinectSensor::m_getColorFrameToCameraSpacePoint(RGBQUAD* pColorBuffer, int iColorWidth, int iColorHeight, UINT16* pDepthFrameData, UINT nDepthDataPointCount)
{
	if (pColorBuffer && (iColorWidth == cColorWidth) && (iColorHeight == cColorHeight) && (nDepthDataPointCount == cInfraredWidth*cInfraredHeight))
	{
		/*
		 * �Զ���һ���ṹ�� ColorPoint (��ΪKinect SDK����Ľṹ�� ColorSpacePoint ���ԱX Y ��Ϊfloat�ͣ�����minRect.center������ʱ���������)
		 * ���ṹ�� colorSpacePoint ��ʼ��Ϊ {-1��-1}�����ں�������ж�
		 */
		ColorPoint boundingCenterPoint = { -1, -1 }; 
		ColorPoint boundingRange = { 0, 0 };
		ColorPoint A = { 0, 0 };
		ColorPoint B = { 0, 0 };
		ColorPoint C = { 0, 0 };
		//////////////////////////////////////////////////////////////////////////////////
		float badminton_Angle = 0;
		//////////////////////////////////////////////////////////////////////////////////
		/* ��:���Ŀ����ڲ�ɫͼ�е����� */
		Badminton badminton;
		badminton.m_getObjectColorSpacePoint(pColorBuffer, iColorWidth, iColorHeight, &boundingCenterPoint, &boundingRange, &badminton_Angle, &A, &B, &C);
		////////////////////////////////////////////////////////////////////
		Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));		
		////////////////////////////////////////////////////////////////////
		if ((boundingCenterPoint.x != -1) && (boundingCenterPoint.y != -1))
		{
			/* ��:����ɫͼ��֡���ݴӲ�ɫͼ�ռ�ת���� Kinect ������ռ� */
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
			 * 1������Ч(���޴�)�����ݵ��޳����ж�׷�ٻ�ȡ�������Ƿ���Ч(��ֵ���޴�)��
			 * 2������������ͻ������ݵ��޳������ݸõ���ǰһ��(��Ч��) Z ��������ֵ֮��ľ���ֵ�Ƿ���һ�����õ���ֵ fMutationThreshold �����ж��Ƿ�ͻ��
			 * 3���ж�������еķ��򣺸��ݸõ�����ǰһ��(��Ч��ͻ���) X ��������ֵ֮������������жϣ�Ϊ������Է�����Ϊ������Ϊ�ҷ�����
			 * ע�⣺���жϷ���ǰ���ǽ�����Ұ�ĵ�һ����ë����������ȷ��Ч�ģ�����ͨ������괥������ʼ��һ����
			 */
			//float fMutationThreshold = 2.0;


			if (isfinite(objectCameraSpacePoint.Z) && (MouseButtonFlag == true) && (objectCameraSpacePoint.Z > 0.3) && (objectCameraSpacePoint.Z < 7.5))
			{
				//printf("the badminton_Angle is %f\n", badminton_Angle);
				printf("the camerapoint is %f  %f  %f\n", objectCameraSpacePoint.X, objectCameraSpacePoint.Y, objectCameraSpacePoint.Z);
				fstream file; //txt�ļ��Ĵ洢·��Ϊ D : \ZXE - Files\file
				file.open("D://ZXE-Files//file//file.txt", ios::app);
				file << "camerapoint " << objectCameraSpacePoint.X << " " << objectCameraSpacePoint.Y << " " << objectCameraSpacePoint.Z << endl;
				file.close();
				//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				CameraSpacePoint badmintonSpacePoint = { 0.0, 0.0, 0.0 };//�����洢Ԥ�����X,Y,Z
				CameraSpacePoint badmintonCourtSpacePoint = { 0.0, 0.0, 0.0 };//��������ϵ�µ��򼣵����� int x,y,z
				///////////////////////////////////////////////////////
				Point_Index++;
				
				//ʵ������ռ��X,Y,Z
				Trajectory_P_X.resize(Point_Index + 10);
				Trajectory_P_Y.resize(Point_Index + 10);
				Trajectory_P_Z.resize(Point_Index + 10);

				//ʵ������ռ��X,Y,Z
				Trajectory_P_X[Point_Index] = objectCameraSpacePoint.X*cos(Drift_Angle_Kinect*PI/180) + objectCameraSpacePoint.Z*sin(Drift_Angle_Kinect*PI/180);
				Trajectory_P_Y[Point_Index] = objectCameraSpacePoint.Y*cos(Angle_of_Elevation_Kinect*PI/180) + objectCameraSpacePoint.Z*sin(Angle_of_Elevation_Kinect*PI/180);
				Trajectory_P_Z[Point_Index] = -objectCameraSpacePoint.X*sin(Drift_Angle_Kinect*PI/180) - objectCameraSpacePoint.Y*sin(Angle_of_Elevation_Kinect*PI/180) + objectCameraSpacePoint.Z*cos(Angle_of_Elevation_Kinect*PI/180)*cos(Drift_Angle_Kinect*PI/180);
								
					if (Point_Index == 0)
					{
						//�õ���ʼ��⵽��ë��ʱ��ϵͳʱ��
						SYSTEMTIME start_systemtime;
						GetLocalTime(&start_systemtime);
						start_calc_T = start_systemtime.wMinute * 60 * 1000 + start_systemtime.wSecond * 1000 + start_systemtime.wMilliseconds * 1;
						if (boundingCenterPoint.x > 980)
						{
							Badminton_Orientation = 1;//�������1˵���ǶԷ����������
						}
						else
						{
							Badminton_Orientation = -1;//�������-1˵�����ҷ����ȥ����
						}
					}
					if ((Point_Index >= 2))// && (Badminton_Orientation == 1)
					{
						double Fac[3] = { 0 };//c,b,a
						double fac[2] = { 0 };//b,k													
						//ʵ������ռ��X,Y,Z�����
						//ͨ��vectorpolyfit��ϵõ��Ķ��ζ���ʽ������ʽΪ��  y = c + b * x^1 + a * x^2 
						vectorpolyfit(Trajectory_P_X, Trajectory_P_Y, Point_Index + 1, 2, Fac);
						//ͨ��vectorpolyfit��ϵõ���һ�η�����ʽΪ��  z = kx + b
						vectorpolyfit(Trajectory_P_X, Trajectory_P_Z, Point_Index + 1, 1, fac);
						printf("Fac[0] Fac[1] Fac[2] is %g  %g  %g\n", Fac[0], Fac[1], Fac[2]);
						printf("fac[0] fac[1] is %g  %g\n", fac[0], fac[1]);
						//Ԥ����Ϲ켣�����							
						//���ڶ��ζ���ʽ�켣���̣����������ʽ�Ϳ�������y=f(x)�е�xֵ��x1=(-b+sqrt(pow(b,2)-4*a*c))/(2*a)  x2=(-b-sqrt(pow(b,2)-4*a*c))/(2*a)
						badmintonSpacePoint.Y = -1.45;
						badmintonSpacePoint.X = (-Fac[1] - sqrt(pow(Fac[1], 2) - 4 * Fac[2] *(Fac[0] - badmintonSpacePoint.Y))) / (2 * Fac[2]);//������ȡ����
						//ͨ��Kinect����ϵ��XOZƽ���б������Ҳ����һ��һ�ζ���ʽ�����
						badmintonSpacePoint.Z = fac[1] * badmintonSpacePoint.X + fac[0];
						printf("X  Y  Z  is %f  %f  %f\n", badmintonSpacePoint.X, badmintonSpacePoint.Y, badmintonSpacePoint.Z);
						//Ԥ��������   txt�ļ��Ĵ洢·��Ϊ D:\ZXE-Files\file			
						file.open("D://ZXE-Files//file//file_forecast.txt", ios::app);
						file << "badmintonpoint " << badmintonSpacePoint.X << " " << badmintonSpacePoint.Y << " " << badmintonSpacePoint.Z << endl;
						file.close();
						///////////////////////////////////////
						//��������ת�����Ȱ�����һ��Kinect�ڷ�λ�ý�������ת��
						transferCoordinate(badmintonSpacePoint, &badmintonCourtSpacePoint);
						printf("Court_X  Court_Y  Court_Z  is %f  %f  %f\n", badmintonCourtSpacePoint.X, badmintonCourtSpacePoint.Y, badmintonCourtSpacePoint.Z);
						//Ԥ��������   txt�ļ��Ĵ洢·��Ϊ D:\ZXE-Files\file			
						file.open("D://ZXE-Files//file//file_forecast_court.txt", ios::app);
						file << "Court_badmintonpoint " << badmintonCourtSpacePoint.X << " " << badmintonCourtSpacePoint.Y << " " << badmintonCourtSpacePoint.Z << endl;
						file.close();
						//���д��ڷ���
							
						if ((Point_Index == 2) && (badmintonCourtSpacePoint.Y > 0) && (badmintonCourtSpacePoint.X > -3050) && (badmintonCourtSpacePoint.X < 3050))
						{
							//ֻ��������ڳ��ط�Χ�ڲŽ��з��ͣ����򲻷��ͣ���ֹ�������ܳ�����
							transmitData(badmintonCourtSpacePoint);
						}
							
						///////////////////////////////////////

						//�õ����Կ�ʼԤ����ë�����ʱ��ϵͳʱ��
						SYSTEMTIME end_systemtime;
						GetLocalTime(&end_systemtime);
						end_calc_T = end_systemtime.wMinute * 60 * 1000 + end_systemtime.wSecond * 1000 + end_systemtime.wMilliseconds * 1;
							
						spend_calc_T = end_calc_T - start_calc_T;//�㷨������������ѵ�ʱ��
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
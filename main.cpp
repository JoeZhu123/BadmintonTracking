#include <opencv2/highgui/highgui.hpp>
#include "KinectSensor.h"

using namespace cv;

bool MouseButtonFlag = false;
bool Trajectory_End = false;
extern int Point_Index;
extern vector <float> Trajectory_P_X;//实际物理空间的轨迹点X坐标
extern vector <float> Trajectory_P_Y;//实际物理空间的轨迹点Y坐标
extern vector <float> Trajectory_P_Z;//实际物理空间的轨迹点Z坐标

void mouseHander(int mouseEvent, int x, int y, int flags, void *param); //控制鼠标响应

int main(int argc, char** argv)
{
	KinectSensor kinect;
	HRESULT result;
	result = kinect.m_hInitKinect();
	result = kinect.m_hGetDepthFrameReader(result);
	result = kinect.m_hGetInfraredFrameReader(result);
	result = kinect.m_hGetColorFrameReader(result);

	cvNamedWindow("backColorImage", CV_WINDOW_AUTOSIZE);
	cvSetMouseCallback("backColorImage", mouseHander, NULL);

	while (true)
	{
		if (Trajectory_End == true)//((boundingCenterPoint.x / 3 < 150) || (boundingCenterPoint.y / 3 > 250)) || 
		{
			//某条轨迹结束时，清除该轨迹点坐标数据
			Point_Index = -1;
			//实际物理空间的X,Y,Z
			Trajectory_P_X.clear();
			Trajectory_P_Y.clear();
			Trajectory_P_Z.clear();
			printf("------------------------------Trajectory End----------------------------\n");
			Trajectory_End = false;
		}
		kinect.Update();
		if (waitKey(30) >= 0) {
			cvSetMouseCallback("backColorImage", NULL, NULL);
			break;
		}
	}

	return 0;
}

/*
 * opencv鼠标响应函数，设定实现的功能为：
 * 在指定的窗口鼠标左击时，MouseButtonFlag 为 true，实现数据发送
 * 鼠标右击时，MouseButtonFlag 为 false，停止数据发送
 * 默认情况下 MouseButtonFlag 为 false，即不发送数据
 */
void mouseHander(int mouseEvent, int x, int y, int flag, void *param)
{
	switch (mouseEvent) {
	case CV_EVENT_LBUTTONDOWN:
		MouseButtonFlag = true;
		break;
	case CV_EVENT_RBUTTONDOWN:
		MouseButtonFlag = false;
		break;
	case CV_EVENT_MBUTTONDOWN:
		Trajectory_End = true;
		break;
	}
}
#include <opencv2/highgui/highgui.hpp>
#include "KinectSensor.h"

using namespace cv;

bool MouseButtonFlag = false;
bool Trajectory_End = false;
extern int Point_Index;
extern vector <float> Trajectory_P_X;//ʵ������ռ�Ĺ켣��X����
extern vector <float> Trajectory_P_Y;//ʵ������ռ�Ĺ켣��Y����
extern vector <float> Trajectory_P_Z;//ʵ������ռ�Ĺ켣��Z����

void mouseHander(int mouseEvent, int x, int y, int flags, void *param); //���������Ӧ

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
			//ĳ���켣����ʱ������ù켣����������
			Point_Index = -1;
			//ʵ������ռ��X,Y,Z
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
 * opencv�����Ӧ�������趨ʵ�ֵĹ���Ϊ��
 * ��ָ���Ĵ���������ʱ��MouseButtonFlag Ϊ true��ʵ�����ݷ���
 * ����һ�ʱ��MouseButtonFlag Ϊ false��ֹͣ���ݷ���
 * Ĭ������� MouseButtonFlag Ϊ false��������������
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
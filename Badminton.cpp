#include "stdafx.h"
#include "Badminton.h"
#include <fstream>
#define PI 3.1415926

using namespace std;
Mat g_ForeColorImage;
Mat g_SourceColorImage;
Mat g_ForeInfraredImage;
Mat g_SourceInfraredImage;
vector<vector<Point>> g_Contours;
vector<Vec4i> g_Hierarchy;
BackgroundSubtractorMOG2 bgSubtractor(20, 16, false);
RNG rng1(12345);

Mat src;
Mat src_gray;
int thresh     = 100;
int max_thresh = 255;

Badminton::Badminton()
{
	m_CloseRect = getStructuringElement(MORPH_RECT, Size(3, 3));
}

void Badminton::m_getObjectColorSpacePoint(RGBQUAD* pColorBuffer, int iColorWidth, int iColorHeight, ColorPoint* boundingCenterPoint, ColorPoint* boundingRange, float* minRect_Angle, ColorPoint* A, ColorPoint* B, ColorPoint* C)
{
	Mat ColorImage(iColorHeight, iColorWidth, CV_8UC4, pColorBuffer);
	Mat cloneColorImage = ColorImage.clone();

	/*
	 * ����OpenCV�Բ�ɫͼ����д�������Ϊ�����š����񡢻Ҷȴ���������֡���ʴ���͡�������
	 */
	int xRatio = 3, yRatio = 3;                                                                      //��������
	resize(cloneColorImage, g_SourceColorImage, Size(iColorWidth / xRatio, iColorHeight / yRatio));  //��ɫͼ����
	flip(g_SourceColorImage, g_SourceColorImage, 1);                                                 //��ɫͼƬ����任
	imshow("colorImage", g_SourceColorImage);
	cvtColor(g_SourceColorImage, g_SourceColorImage, CV_RGB2GRAY);                                   //��ɫͼƬ�Ҷȴ���
	bgSubtractor(g_SourceColorImage, g_ForeColorImage, 0.1);                                         //��˹��ϱ���ǰ���ָ�
	morphologyEx(g_ForeColorImage, g_ForeColorImage, MORPH_OPEN, m_CloseRect);                      //��̬ѧ�����㣬�����ͺ�ʴ
	findContours(g_ForeColorImage, g_Contours, g_Hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);  //������

	/* ������������Ӿ��Σ�Ϊ��������ë����нǶȡ���ԲȦ��Ŀ����׼�� */
	Mat backColorImage = Mat::zeros(g_ForeColorImage.size(), CV_8UC3);                               //��ʼ�������
	vector<RotatedRect> minRect(g_Contours.size());
	vector<Moments> mu(g_Contours.size());                                                           //�����
	vector<Point2f> mc(g_Contours.size());                                                           //�������ľ�

	/* �ų��������� */
	for (int idx = 0; idx < g_Contours.size(); idx++)                                                //�ų�����������ѡ��Ŀ������
	{
		//double dMinCircumference      = 0;                                                           //�޶������ܳ�
		//double dMaxCircumference      = 100;
		//double dMinWidthHeightRatio   = 0.5;                                                         //�޶�������߱�
		//double dMaxWidthHeightRatio   = 10.0;
		//double dMinArea               = 5.0;                                                         //�޶��������
		//double dMaxArea               = 180.0;

		//double dContoursCircumference = arcLength(g_Contours[idx], true);
		//double dContoursArea          = fabs(contourArea(g_Contours[idx]));
		Rect ContoursRect             = boundingRect(g_Contours[idx]);

		//if ((dContoursArea>dMinArea) && (dContoursArea<dMaxArea) && (dContoursCircumference<dMaxCircumference) && (dContoursCircumference>dMinCircumference) && ((ContoursRect.width / ContoursRect.height)>dMinWidthHeightRatio) && ((ContoursRect.width / ContoursRect.height) < dMaxWidthHeightRatio))
		//{
		Scalar setShowColor = Scalar(rng1.uniform(0, 255), rng1.uniform(0, 255), rng1.uniform(0, 255));
		minRect[idx]        = minAreaRect(Mat(g_Contours[idx]));
		Point2f rect_points[4];
		minRect[idx].points(rect_points);
		if (idx == 0)  //ȡ findContours ������õĵ�һ��������ΪĿ����ë���������ų���ӰӰ��
		{
			/* ���ͼ����ǰ����ë���ڲ�ɫͼ�е����� */
			(*A).x = static_cast<int>((rect_points[0].x + rect_points[1].x) / 2);
			(*A).y = static_cast<int>((rect_points[0].y + rect_points[1].y) / 2);
			(*B).x = static_cast<int>((rect_points[0].x + rect_points[3].x) / 2);
			(*B).y = static_cast<int>((rect_points[0].y + rect_points[3].y) / 2);
			(*C).x = static_cast<int>((rect_points[1].x + rect_points[2].x) / 2);
			(*C).y = static_cast<int>((rect_points[1].y + rect_points[2].y) / 2);

			/* ��Ŀ����ͼ����������ת����ͼ����ǰ������ */
			(*A).x = (static_cast<int>((iColorWidth / xRatio - (*A).x - 1))) * xRatio;   //��ת�����ŷ�������
			(*A).y = static_cast<int>(((*A).y + 0.5)*yRatio);
			(*B).x = (static_cast<int>((iColorWidth / xRatio - (*B).x - 1))) * xRatio;   //��ת�����ŷ�������
			(*B).y = static_cast<int>(((*B).y + 0.5)*yRatio);
			(*C).x = (static_cast<int>((iColorWidth / xRatio - (*C).x - 1))) * xRatio;   //��ת�����ŷ�������
			(*C).y = static_cast<int>(((*C).y + 0.5)*yRatio);
			(*minRect_Angle) = minRect[idx].angle;
			(*boundingRange).x = (static_cast<int>(ContoursRect.width / 2 + 0.5)) * xRatio;
			(*boundingRange).y = (static_cast<int>(ContoursRect.height / 2 + 0.5)) * yRatio;
			(*boundingCenterPoint).x = (static_cast<int>((iColorWidth / xRatio) - ContoursRect.x - 1)) * xRatio - (*boundingRange).x;
			(*boundingCenterPoint).y = (static_cast<int>(ContoursRect.y + 0.5)) * yRatio + (*boundingRange).y;

			//////////////////////////////////////////////////////////////////////////////////////////////////////
		}
		/*
		 * ���ڲ�ɫͼ�·� 1/6���� 1/5 �ĵ����ε�����Ϊ�ų����ŵ�һ�ַ�ʽ
		 * �����㡢�ڲ�ɫͼ�� 200 �����ص㡢�ҷ� 310 �����ص����������ͷ������������
		 */
		//if ((yObjectPoint < (1080  * 4 / 5)) && (xObjectPoint > 0) && (xObjectPoint < (1920)))
		{
			mu[idx] = moments(g_Contours[idx], false);
			mc[idx] = Point2f(mu[idx].m10 / mu[idx].m00, mu[idx].m01 / mu[idx].m00);

			drawContours(backColorImage, g_Contours, idx, setShowColor, 3, 8, vector<Vec4i>(), 9, Point());
			circle(backColorImage, mc[idx], 20, setShowColor, 1, 8, 0);

			for (int j = 0; j < 4; j++) {
				line(backColorImage, rect_points[j], rect_points[(j + 1) % 4], setShowColor, 1, 8);
			}

		}
	//}
	}

	if (!cloneColorImage.empty()) {
			//imshow("g_ForeColorImage", g_ForeColorImage);
			imshow("backColorImage", backColorImage);
		}
	}

void Badminton::m_getObjectInfraredSpacePoint(UINT16* pInfraredBuffer, int iInfraredWidth, int iInfraredHeight, ColorPoint* infraredSpacePoint, ColorPoint* range)
{
	/*
	* ����OpenCV�����ͼ����д�������Ϊ�����񡢱�����֡���ʴ���͡�������
	*/
	Mat InfraredImage(iInfraredHeight, iInfraredWidth, CV_8UC1, pInfraredBuffer);
	Mat cloneInfraredImage = InfraredImage.clone();

	flip(cloneInfraredImage, g_SourceInfraredImage, 1);//����
	bgSubtractor(g_SourceInfraredImage, g_ForeInfraredImage, 0.1);//�������
	morphologyEx(g_ForeInfraredImage, g_ForeInfraredImage, MORPH_OPEN, m_CloseRect);//��ʴ����
	findContours(g_ForeInfraredImage, g_Contours, g_Hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);//������

	Mat backInfraredImage = Mat::zeros(g_ForeInfraredImage.size(), CV_8UC1);                         //��ʼ�������
	vector<RotatedRect> minRect(g_Contours.size());                                                  //��С��Ӿ���
	vector<Moments> mu(g_Contours.size());                                                           //�����
	vector<Point2f> mc(g_Contours.size());                                                           //�������ľ�

	/*
	* �� OpenCV �ҵ���Ŀ�껭ԲȦ������Ŀ�������ͼ�е����괫�غ���������
	*
	/* �ų��������� */
	for (int idx = 0; idx < g_Contours.size(); idx++)                                                //�ų�����������ѡ��Ŀ������
	{
		//double dMinCircumference = 0;                                                                //�޶������ܳ�
		//double dMaxCircumference = 100;
		//double dMinWidthHeightRatio = 0.5;                                                           //�޶�������߱�
		//double dMaxWidthHeightRatio = 10.0;
		//double dMinArea = 5.0;                                                                       //�޶��������
		//double dMaxArea = 180.0;

		//double dContoursCircumference = arcLength(g_Contours[idx], true);
		//double dContoursArea = fabs(contourArea(g_Contours[idx]));
		Rect ContoursRect = boundingRect(g_Contours[idx]);

		//if ((dContoursArea>dMinArea) && (dContoursArea<dMaxArea) && (dContoursCircumference<dMaxCircumference) && (dContoursCircumference>dMinCircumference) && ((ContoursRect.width / ContoursRect.height)>dMinWidthHeightRatio) && ((ContoursRect.width / ContoursRect.height) < dMaxWidthHeightRatio))
		//{
		Scalar setShowColor = Scalar(rng1.uniform(0, 255), rng1.uniform(0, 255), rng1.uniform(0, 255));
		minRect[idx] = minAreaRect(Mat(g_Contours[idx]));                                 //�����������С��Χ����

		/* ��Ŀ����ͼ����������ת����ͼ����ǰ������ */
		int xObjectPoint = static_cast<int>(iInfraredWidth - ContoursRect.width / 2 - ContoursRect.x - 1);   //��ת����
		int yObjectPoint = static_cast<int>(ContoursRect.y + ContoursRect.height / 2 + 0.5);
		/*
		* �������ͼ�·� 1/6���� 1/6 �ĵ����ε�����Ϊ�ų����ŵ�һ�ַ�ʽ
		*/
		if ((yObjectPoint < (424 * 4 / 6)) && (xObjectPoint > 512 / 6))
		{
			(*infraredSpacePoint).x = xObjectPoint;
			(*infraredSpacePoint).y = yObjectPoint;

			mu[idx] = moments(g_Contours[idx], false);
			mc[idx] = Point2f(mu[idx].m10 / mu[idx].m00, mu[idx].m01 / mu[idx].m00);

			drawContours(backInfraredImage, g_Contours, idx, setShowColor, 3, 8, vector<Vec4i>(), 9, Point());
			circle(backInfraredImage, mc[idx], 20, setShowColor, 1, 8, 0);
			Point2f rect_points[4];
			minRect[idx].points(rect_points);

			for (int j = 0; j < 4; j++) {
				line(backInfraredImage, rect_points[j], rect_points[(j + 1) % 4], setShowColor, 1, 8);
			}
		}
	}

	if (!cloneInfraredImage.empty()) {
		imshow("backInfraredImage", backInfraredImage);
	}
}
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
	 * 利用OpenCV对彩色图像进行处理，依次为：缩放、镜像、灰度处理、背景差分、腐蚀膨胀、找轮廓
	 */
	int xRatio = 3, yRatio = 3;                                                                      //缩放因子
	resize(cloneColorImage, g_SourceColorImage, Size(iColorWidth / xRatio, iColorHeight / yRatio));  //彩色图缩放
	flip(g_SourceColorImage, g_SourceColorImage, 1);                                                 //彩色图片镜像变换
	imshow("colorImage", g_SourceColorImage);
	cvtColor(g_SourceColorImage, g_SourceColorImage, CV_RGB2GRAY);                                   //彩色图片灰度处理
	bgSubtractor(g_SourceColorImage, g_ForeColorImage, 0.1);                                         //高斯混合背景前景分割
	morphologyEx(g_ForeColorImage, g_ForeColorImage, MORPH_OPEN, m_CloseRect);                      //形态学闭运算，先膨胀后腐蚀
	findContours(g_ForeColorImage, g_Contours, g_Hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);  //找轮廓

	/* 声明轮廓的外接矩形，为后面求羽毛球飞行角度、用圆圈出目标做准备 */
	Mat backColorImage = Mat::zeros(g_ForeColorImage.size(), CV_8UC3);                               //初始化零矩阵
	vector<RotatedRect> minRect(g_Contours.size());
	vector<Moments> mu(g_Contours.size());                                                           //计算矩
	vector<Point2f> mc(g_Contours.size());                                                           //计算中心矩

	/* 排除轮廓干扰 */
	for (int idx = 0; idx < g_Contours.size(); idx++)                                                //排除干扰轮廓，选定目标轮廓
	{
		//double dMinCircumference      = 0;                                                           //限定轮廓周长
		//double dMaxCircumference      = 100;
		//double dMinWidthHeightRatio   = 0.5;                                                         //限定轮廓宽高比
		//double dMaxWidthHeightRatio   = 10.0;
		//double dMinArea               = 5.0;                                                         //限定轮廓面积
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
		if (idx == 0)  //取 findContours 函数获得的第一个轮廓作为目标羽毛球轮廓，排除拖影影响
		{
			/* 获得图像处理前的羽毛球在彩色图中的坐标 */
			(*A).x = static_cast<int>((rect_points[0].x + rect_points[1].x) / 2);
			(*A).y = static_cast<int>((rect_points[0].y + rect_points[1].y) / 2);
			(*B).x = static_cast<int>((rect_points[0].x + rect_points[3].x) / 2);
			(*B).y = static_cast<int>((rect_points[0].y + rect_points[3].y) / 2);
			(*C).x = static_cast<int>((rect_points[1].x + rect_points[2].x) / 2);
			(*C).y = static_cast<int>((rect_points[1].y + rect_points[2].y) / 2);

			/* 将目标在图像处理后的坐标转换到图像处理前的坐标 */
			(*A).x = (static_cast<int>((iColorWidth / xRatio - (*A).x - 1))) * xRatio;   //反转、缩放方反处理
			(*A).y = static_cast<int>(((*A).y + 0.5)*yRatio);
			(*B).x = (static_cast<int>((iColorWidth / xRatio - (*B).x - 1))) * xRatio;   //反转、缩放方反处理
			(*B).y = static_cast<int>(((*B).y + 0.5)*yRatio);
			(*C).x = (static_cast<int>((iColorWidth / xRatio - (*C).x - 1))) * xRatio;   //反转、缩放方反处理
			(*C).y = static_cast<int>(((*C).y + 0.5)*yRatio);
			(*minRect_Angle) = minRect[idx].angle;
			(*boundingRange).x = (static_cast<int>(ContoursRect.width / 2 + 0.5)) * xRatio;
			(*boundingRange).y = (static_cast<int>(ContoursRect.height / 2 + 0.5)) * yRatio;
			(*boundingCenterPoint).x = (static_cast<int>((iColorWidth / xRatio) - ContoursRect.x - 1)) * xRatio - (*boundingRange).x;
			(*boundingCenterPoint).y = (static_cast<int>(ContoursRect.y + 0.5)) * yRatio + (*boundingRange).y;

			//////////////////////////////////////////////////////////////////////////////////////////////////////
		}
		/*
		 * 将在彩色图下方 1/6，左方 1/5 的点屏蔽掉，作为排除干扰的一种方式
		 * 经计算、在彩色图左方 200 个像素点、右方 310 个像素点是深度摄像头看不到的区域。
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
	* 利用OpenCV对深度图像进行处理，依次为：镜像、背景差分、腐蚀膨胀、找轮廓
	*/
	Mat InfraredImage(iInfraredHeight, iInfraredWidth, CV_8UC1, pInfraredBuffer);
	Mat cloneInfraredImage = InfraredImage.clone();

	flip(cloneInfraredImage, g_SourceInfraredImage, 1);//镜像
	bgSubtractor(g_SourceInfraredImage, g_ForeInfraredImage, 0.1);//背景差分
	morphologyEx(g_ForeInfraredImage, g_ForeInfraredImage, MORPH_OPEN, m_CloseRect);//腐蚀膨胀
	findContours(g_ForeInfraredImage, g_Contours, g_Hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);//找轮廓

	Mat backInfraredImage = Mat::zeros(g_ForeInfraredImage.size(), CV_8UC1);                         //初始化零矩阵
	vector<RotatedRect> minRect(g_Contours.size());                                                  //最小外接矩形
	vector<Moments> mu(g_Contours.size());                                                           //计算矩
	vector<Point2f> mc(g_Contours.size());                                                           //计算中心矩

	/*
	* 将 OpenCV 找到的目标画圆圈，并就目标在深度图中的坐标传回函数调用者
	*
	/* 排除轮廓干扰 */
	for (int idx = 0; idx < g_Contours.size(); idx++)                                                //排除干扰轮廓，选定目标轮廓
	{
		//double dMinCircumference = 0;                                                                //限定轮廓周长
		//double dMaxCircumference = 100;
		//double dMinWidthHeightRatio = 0.5;                                                           //限定轮廓宽高比
		//double dMaxWidthHeightRatio = 10.0;
		//double dMinArea = 5.0;                                                                       //限定轮廓面积
		//double dMaxArea = 180.0;

		//double dContoursCircumference = arcLength(g_Contours[idx], true);
		//double dContoursArea = fabs(contourArea(g_Contours[idx]));
		Rect ContoursRect = boundingRect(g_Contours[idx]);

		//if ((dContoursArea>dMinArea) && (dContoursArea<dMaxArea) && (dContoursCircumference<dMaxCircumference) && (dContoursCircumference>dMinCircumference) && ((ContoursRect.width / ContoursRect.height)>dMinWidthHeightRatio) && ((ContoursRect.width / ContoursRect.height) < dMaxWidthHeightRatio))
		//{
		Scalar setShowColor = Scalar(rng1.uniform(0, 255), rng1.uniform(0, 255), rng1.uniform(0, 255));
		minRect[idx] = minAreaRect(Mat(g_Contours[idx]));                                 //获得轮廓的最小包围矩形

		/* 将目标在图像处理后的坐标转换到图像处理前的坐标 */
		int xObjectPoint = static_cast<int>(iInfraredWidth - ContoursRect.width / 2 - ContoursRect.x - 1);   //反转处理
		int yObjectPoint = static_cast<int>(ContoursRect.y + ContoursRect.height / 2 + 0.5);
		/*
		* 将在深度图下方 1/6，左方 1/6 的点屏蔽掉，作为排除干扰的一种方式
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
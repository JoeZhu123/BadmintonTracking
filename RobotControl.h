#ifndef ROBOTCONTROL_H_
#define ROBOTCONTROL_H_

#include "stdafx.h"
#include <Kinect.h>
#include <math.h>
#include <vector>
using namespace std;

///////////////////////���Ԥ���ϵ����⺯��//////////////////////////////////
//�Է���K����LU�ֽ�.�ֽ�ʧ�ܷ���False.�ɹ�����True�Լ��ֽ�õ���L��U
bool Matrix_LU(double *K, int n, double *L, double *U);
bool Matrix_Inv(double *K, int n, double *InvK);//����LU�ֽⷽ������K����InvK,K[n][n]
void Matrix_Mul(double *Mul1, int Mul1_m, double *Mul2, int Mul2_n, int nm, double *Mul);
void Matrix_T(double *K, int m, int n, double *KT);//���ؾ���K��ת��KT.k[m][n]
bool Matrix_Solve(double *K, double *B, int m, int n, double *x);//Kx=B���x��K[m][n]������������С���˽�,B[m][1]
bool Matrix_PolyFit(double *x, double *y, int X_Y_Number, int Fit_N, double *ks);
//////////////////////////////////////////////////////////////////////////////////
void transmitData(CameraSpacePoint fallPoint);
void transferCoordinate(CameraSpacePoint sourcePoint, CameraSpacePoint* outPoint);
void outputData(CameraSpacePoint outPoint);
bool is_PointinTriangle(ColorPoint A, ColorPoint B, ColorPoint C, int x, int y);
//float angle(float side1, float side2, float side3);

#endif
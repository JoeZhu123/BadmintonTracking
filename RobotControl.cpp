#include "stdafx.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <vector>
#include <math.h>
#include <Kinect.h>
#include "RobotControl.h"

#define PI 3.1415926

using namespace cv;
using namespace std;


//�Է���K����LU�ֽ�.�ֽ�ʧ�ܷ���False.�ɹ�����True�Լ��ֽ�õ���L��U
bool Matrix_LU(double *K, int n, double *L, double *U)//�Է���K����LU�ֽ�.�ֽ�ʧ�ܷ���False.�ɹ�����True�Լ��ֽ�õ���L��U
{
	int i, j, a, b, c, d;
	double temp;
	for (i = 0, a = 0; i<n; i++)
	{
		for (j = 0; j<n; j++)
		{
			L[a + j] = U[a + j] = 0;
		}
		U[a + i] = 1;
		a += n;
	}
	for (j = 0, d = 0; j<n; j++)
	{
		for (i = j, b = d; i<n; i++)
		{
			temp = 0;
			a = 0, c = j;
			while (a<j)
			{
				temp += L[b + a] * U[c];
				c += n;
				a++;
			}
			L[b + j] = K[b + j] - temp;
			b += n;
		}
		i = j + 1;
		while (i<n)
		{
			temp = 0;
			a = 0, c = i;
			while (a<j)
			{
				temp += L[d + a] * U[c];
				a++;
				c += n;
			}
			if (L[d + j] == 0)
			{
				return false;
			}
			U[d + i] = (K[d + i] - temp) / L[d + j];
			i++;
		}
		d += n;
	}
	return true;
}

//����LU�ֽⷽ������K����InvK,K[n][n]
bool Matrix_Inv(double *K, int n, double *InvK)//����LU�ֽⷽ������K����InvK,K[n][n]
{
	if (1 == n)
	{
		if (K[0] == 0)
		{
			return false;
		}
		else
		{
			InvK[0] = 1 / K[0];
		}
	}
	else if (n<1)
	{
		return false;
	}
	else
	{
		int i, j, a, b;
		double *L, *U, *d, *x, *e, temp;
		a = n*n;
		L = new double[a];
		U = new double[a];
		if (Matrix_LU(K, n, L, U))
		{
			d = new double[n];
			x = new double[n];
			e = new double[n];
			for (i = 0; i<n; i++)
			{
				x[i] = d[i] = 0;
			}
			for (i = 0; i<n; i++)
			{
				for (j = 0; j<n; j++)
				{
					e[j] = 0;
				}
				e[i] = 1;
				j = 0;
				b = 0;
				while (j<n)
				{
					temp = 0;
					a = 0;
					while (a<j)
					{
						temp += d[a] * L[b + a];
						a++;
					}
					d[j] = e[j] - temp;
					d[j] /= L[b + j];
					j++;
					b += n;
				}
				j = n - 1;
				b -= n;
				while (j>-1)
				{
					temp = 0;
					a = j + 1;
					while (a<n)
					{
						temp += U[b + a] * x[a];
						a++;
					}
					x[j] = d[j] - temp;
					x[j] /= U[b + j];
					j--;
					b -= n;
				}
				for (j = 0, b = i; j<n; j++)
				{
					InvK[b] = x[j];
					b += n;
				}
			}
			delete[]d;
			delete[]x;
			delete[]e;
		}
		else
		{
			delete[]L;
			delete[]U;
			return false;
		}
		delete[]L;
		delete[]U;
	}
	return true;
}

void Matrix_Mul(double *Mul1, int Mul1_m, double *Mul2, int Mul2_n, int nm, double *Mul)
{
	//Mul1[Mul1_m][nm]*Mul2[nm][Mul2_n]=Mul������ĳ˷�
	int i, j, k, a, b, c, d;
	for (i = 0, a = 0, c = 0; i<Mul1_m; i++)
	{
		for (j = 0; j<Mul2_n; j++)
		{
			b = a + j;
			Mul[b] = 0;
			for (k = 0, d = 0; k<nm; k++)
			{
				Mul[b] += Mul1[c + k] * Mul2[d + j];
				d += Mul2_n;
			}
		}
		c += nm;
		a += Mul2_n;
	}
}

//���ؾ���K��ת��KT.k[m][n]
void Matrix_T(double *K, int m, int n, double *KT)//���ؾ���K��ת��KT.k[m][n]
{
	int i, j, a, b;
	for (i = 0, a = 0; i<m; i++)
	{
		for (j = 0, b = 0; j<n; j++)
		{
			KT[b + i] = K[a + j];
			b += m;
		}
		a += n;
	}
}

//Kx=B���x��K[m][n]������������С���˽�,B[m][1]
bool Matrix_Solve(double *K, double *B, int m, int n, double *x)//Kx=B���x��K[m][n]������������С���˽�,B[m][1]
{
	double *KT, *Kmul, *Kb, *Kinv;
	int i;
	i = n*n;
	KT = new double[m*n];
	Kmul = new double[i];
	Kinv = new double[i];
	Kb = new double[n];
	Matrix_T(K, m, n, KT);
	Matrix_Mul(KT, n, K, n, m, Kmul);
	Matrix_Mul(KT, n, B, 1, m, Kb);
	if (Matrix_Inv(Kmul, n, Kinv))
	{
		Matrix_Mul(Kinv, n, Kb, 1, n, x);
		delete[]KT;
		delete[]Kmul;
		delete[]Kinv;
		delete[]Kb;
		return true;
	}
	else
	{
		delete[]KT;
		delete[]Kmul;
		delete[]Kinv;
		delete[]Kb;
		return false;
	}
}
bool Matrix_PolyFit(double *x, double *y, int X_Y_Number, int Fit_N, double *ks)
{
	/*
	��X_Y_Number������x,y����Fit_N�ζ���ʽ���,��Ϸ��ض���ʽ��ϵ��ks
	x[X_Y_Number],y[X_Y_Number],ks[Fit_N+1]
	*/
	if (Fit_N>X_Y_Number || X_Y_Number<1)
	{
		return false;
	}
	int i, j, index, n;
	double temp, *x2, *y2;
	Fit_N++;
	y2 = new double[Fit_N];
	x2 = new double[Fit_N*Fit_N];
	for (i = 0, index = 0; i<Fit_N; i++)
	{
		y2[i] = 0;
		for (j = 0; j<Fit_N; j++)
		{
			x2[index + j] = 0;
		}
		index += Fit_N;
	}
	x2[0] = X_Y_Number;
	for (i = 0; i<Fit_N; i++)
	{
		for (j = i + 1; j<Fit_N; j++)
		{
			temp = 0;
			n = i + j;
			for (index = 0; index<X_Y_Number; index++)
			{
				temp += pow(x[index], n);
			}
			index = j;
			for (n = i; n<Fit_N; n++)
			{
				if (index >= 0)
				{
					x2[n*Fit_N + index] = temp;
				}
				index--;
			}
		}
	}
	n = Fit_N + Fit_N - 2;
	temp = 0;
	for (i = 0; i<X_Y_Number; i++)
	{
		temp += pow(x[i], n);
	}
	x2[Fit_N*Fit_N - 1] = temp;
	for (i = 0; i<Fit_N; i++)
	{
		temp = 0;
		for (j = 0; j<X_Y_Number; j++)
		{
			temp += y[j] * pow(x[j], i);
		}
		y2[i] = temp;
	}
	if (Matrix_Solve(x2, y2, Fit_N, Fit_N, ks))
	{
		delete[]y2;
		delete[]x2;
		return true;
	}
	else
	{
		delete[]y2;
		delete[]x2;
		return false;
	}
}



/*
* �ú���ͨ������ serialport ������ʵ�����ݷ��͹���
* �ú������ཫ����ǿ��ת���� int ��
* Ȼ��ʹ�������彫 int �����ݵĵ� 16 λͨ���ַ���ʽ�Ӵ��ڷ��ͳ�ȥ (���� X Y Z ��˳��)
* �������ά��������ݵ�λΪ mm
* None
*/
void transmitData(CameraSpacePoint fallPoint)
{
	union {
		float n;
		unsigned char arr[4];
	} data1, data2, data3;

	data1.n = fallPoint.X;
	data2.n = fallPoint.Y;
	data3.n = fallPoint.Z;
	unsigned char array[16] = { 0x2A, 0x2A, 0x2A, data1.arr[0], data1.arr[1], data1.arr[2], data1.arr[3], data2.arr[0], data2.arr[1], data2.arr[2], data2.arr[3], data3.arr[0], data3.arr[1], data3.arr[2], data3.arr[3], 0x23 }; //����X��Y��Z��˳��������

	CSerialPort kinectSerialPort;
	kinectSerialPort.InitPort(4, CBR_115200, 'N', 8, 1, EV_RXCHAR);
	kinectSerialPort.OpenListenThread();
	kinectSerialPort.WriteData(array, 16);
	kinectSerialPort.CloseListenTread();
	kinectSerialPort.closePort();
}

/*
* �ú���ʵ�ֽ��� Kinect Ϊԭ������������ϵ�µ���ë������(��λ mm��int ��)
* ת�����Ա�������Ϊԭ�������ϵ������(��λ mm��int ��)
*/
void transferCoordinate(CameraSpacePoint sourcePoint, CameraSpacePoint* outPoint)
{
	float m = 580, n = 280, h = 340; //��λ��mm Kinect�ڷ�λ��
	float xReferencePoint, yReferencePoint, zReferencePoint; //ReferencePoint  ��ë������ڳ���ԭ������
	xReferencePoint = 3010 + m;
	yReferencePoint = 4700 + n;
	zReferencePoint = 1110 + h;

	(*outPoint).X = (xReferencePoint - 1000 * sourcePoint.Z);
	(*outPoint).Y = (yReferencePoint - 1000 * sourcePoint.X);
	(*outPoint).Z = (zReferencePoint + 1000 * sourcePoint.Y);
}

/*
* ����������� txt �ı�����
* �ı�λ��: ���� -> file�ļ��� -> file.txt
* None
*/
void outputData(CameraSpacePoint outPoint)
{
	ofstream file;
	file.open("C://Users//rudy//Desktop//file//file.txt", ios::app);
	file << "badminton point is  " << outPoint.X << "  " << outPoint.Y << "  " << outPoint.Z << endl;
	file.close();
}

class Vector3
{
public:
	Vector3(float fx, float fy, float fz) :x(fx), y(fy), z(fz){}
	//Subtract
	Vector3 operator - (const Vector3& v) const
	{
		return Vector3(x - v.x, y - v.y, z - v.z);
	}
	//Dot product
	float Dot(const Vector3& v) const
	{
		return x*v.x + y*v.y + z*v.z;
	}
	//Cross product
	Vector3 Cross(const Vector3& v) const
	{
		return Vector3(
			y*v.z - z*v.y,
			z*v.x - x*v.z,
			x*v.y - y*v.x
			);
	}
private:
	float x, y, z;
};

bool is_PointinTriangle(ColorPoint A, ColorPoint B, ColorPoint C, int x, int y)
{

	Vector3 v0(C.x - A.x, C.y - A.y, 0);
	Vector3 v1(B.x - A.x, B.y-A.y,0);
	Vector3 v2(x - A.x, y-A.y,0);

	float dot00 = v0.Dot(v0);
	float dot01 = v0.Dot(v1);
	float dot02 = v0.Dot(v2);
	float dot11 = v1.Dot(v1);
	float dot12 = v1.Dot(v2);

	float invertDeno = 1 / (dot00 * dot11 - dot01 * dot01);
	if (isfinite(invertDeno))
	{
		float u = (dot11*dot02 - dot01*dot12)*invertDeno;

		if (u < 0 || u>1)
		{
			//printf("u < 0 || u > 1");
			return false;
		}

		float v = (dot00*dot12 - dot01*dot02)*invertDeno;
		if (v < 0 || v>1)
		{
			//printf("v < 0 || v > 1");
			return false;
		}

		return (u + v <= 1);
	}
	else
		return false;
}
/**
float angle(float side1, float side2, float side3)
{
	float ang;
	ang = acosf((powf(side1, 2) + powf(side2, 2) - powf(side3, 2)) / (2 * side1*side2)) * 180 / 3.1416;
	return ang;
}
*/
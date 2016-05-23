#ifndef _FUNCTION_H
#define _FUNCTION_H
#include <Windows.h>
#include <WinBase.h>
#include <fstream>

/* �����ṹ�� vector����ʾһ����ά���������ԱΪ float �� */
struct vector {
	float x;
	float y;
};

/* �����ṹ�� vector3D����ʾһ����ά���������ԱΪ float�� */
struct vector3D {
	float x;
	float y;
	float z;
};

void get_localtime(SYSTEMTIME* current_systemtime);                     //�õ��������ǰ��ʱ��
void export_data_to_txtfile(int data);                                  //�������ݵ�txt�ļ���
float cal_dot_product(vector A, vector B);         //�����ά����A��B�ĵ������ȻҲ��������������ģ
float cal_dot_product3D(vector3D A, vector3D B);   //������ά����A��B�ĵ��
BOOL set_console_color(WORD attributes); //���ÿ���̨�������ɫ, ����ǰ��ɫ�ͱ���ɫ
#endif
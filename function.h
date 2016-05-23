#ifndef _FUNCTION_H
#define _FUNCTION_H
#include <Windows.h>
#include <WinBase.h>
#include <fstream>

/* 声明结构体 vector，表示一个二维向量，其成员为 float 型 */
struct vector {
	float x;
	float y;
};

/* 声明结构体 vector3D，表示一个三维向量，其成员为 float型 */
struct vector3D {
	float x;
	float y;
	float z;
};

void get_localtime(SYSTEMTIME* current_systemtime);                     //得到计算机当前的时间
void export_data_to_txtfile(int data);                                  //导出数据到txt文件中
float cal_dot_product(vector A, vector B);         //计算二维向量A和B的点积，当然也可用来求向量的模
float cal_dot_product3D(vector3D A, vector3D B);   //计算三维向量A和B的点积
BOOL set_console_color(WORD attributes); //设置控制台的输出颜色, 包括前景色和背景色
#endif
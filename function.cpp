/*
 * function.h
 * 功    能：提供一些常用的函数
 * 作    者：敖小乐
 * 创建时间：2015年 5月 12日
 * 备    注：
 */
#include "function.h"
/*
 * 函数名称：get_localtime
 * 函数功能：获得计算机当前时间，包括当前年、月、日、星期、时、分、秒、毫秒
 * 输入参数：current_systemtime  : 指向SYSTEMTIME类型结构体的指针
 * 输出参数：current_systemtime
 * 返 回 值：无
 * 副 作 用：无
 * 创建时间：2015年 5月 12日
 */
void get_localtime(SYSTEMTIME* current_systemtime)
{
	GetLocalTime(current_systemtime);
}

/*
 * 函数名称：export_data_to_txtfile
 * 函数功能：将变量 data 导出到桌面文件夹folder里面的 txt 文件 filename 中
 * 输入参数：data  : 默认为 int 型，其他类型变量也适合，只是需要做适当的更改
 * 输出参数：无
 * 返 回 值：无
 * 副 作 用：将 data 输出到 txt 文件 filename 中, 默认通过追加的方式打开文件
            std::ios::app       以追加的方式打开文件
            std::ios::nocreate  不建立文件，filename不存在时打开失败
			std::ios::trunc     如果文件存在，将文件长度设为0
			std::ios::binaru    以二进制方式打开文件，缺省方式为文本方式
 * 备    注：文件夹位置、文件名可任取
 * 创建时间：2015年 5月 12日
 */
void export_data_to_txtfile(int data)
{
	std::ofstream file;
	file.open("C://Users//rudy//Desktop//folder//filename.txt", std::ios::app);
	file << data << std::endl;
	file.close();
}

/*
 * 函数名称：cal_dot_product
 * 函数功能：计算二维向量 A 和 B 的点积
 * 输入参数：A & B  ： 自定义 vector 结构体类型
 * 输出参数：无
 * 返 回 值：AB     ： float类型
 * 副 作 用：无
 * 创建时间：2015年 5月 13日
 */
float cal_dot_product(struct vector A, struct vector B)
{
	return (A.x*B.x + A.y*B.y);
}

/*
* 函数名称：cal_dot_product
* 函数功能：计算三维向量 A 和 B 的点积
* 输入参数：A & B  ： 自定义 vector3D 结构体类型
* 输出参数：无
* 返 回 值：AB     ： float类型
* 副 作 用：无
* 创建时间：2015年 5月 13日
*/
float cal_dot_product3D(struct vector3D A, struct vector3D B)
{
	return (A.x*B.x + A.y*B.y + A.z*B.z);
}

/*
 * 函数名称：set_console_color
 * 函数功能：设置控制台下字体或背景的颜色
 * 输入参数：attributes  比如：FOREGROUND_GREEN 、BACKGROUND_RED
 * 输出参数：无
 * 返 回 值：TRUE(设置成功) or FALSE(设置失败)
 * 副 作 用：设置控制台前景、背景颜色
 * 备    注：可通过颜色混合获得新的色彩，比如白色：FOREGROUND_GREEN | FOREGROUND_BLUE | FOREGROUND_RED
 * 创建时间：2015年 5月 17日
 */
BOOL set_console_color(WORD attributes)
{
	HANDLE  handle_console = GetStdHandle(STD_OUTPUT_HANDLE);
	if (handle_console == INVALID_HANDLE_VALUE)
		return FALSE;
	return SetConsoleTextAttribute(handle_console, attributes);
}
/*
 * function.h
 * ��    �ܣ��ṩһЩ���õĺ���
 * ��    �ߣ���С��
 * ����ʱ�䣺2015�� 5�� 12��
 * ��    ע��
 */
#include "function.h"
/*
 * �������ƣ�get_localtime
 * �������ܣ���ü������ǰʱ�䣬������ǰ�ꡢ�¡��ա����ڡ�ʱ���֡��롢����
 * ���������current_systemtime  : ָ��SYSTEMTIME���ͽṹ���ָ��
 * ���������current_systemtime
 * �� �� ֵ����
 * �� �� �ã���
 * ����ʱ�䣺2015�� 5�� 12��
 */
void get_localtime(SYSTEMTIME* current_systemtime)
{
	GetLocalTime(current_systemtime);
}

/*
 * �������ƣ�export_data_to_txtfile
 * �������ܣ������� data �����������ļ���folder����� txt �ļ� filename ��
 * ���������data  : Ĭ��Ϊ int �ͣ��������ͱ���Ҳ�ʺϣ�ֻ����Ҫ���ʵ��ĸ���
 * �����������
 * �� �� ֵ����
 * �� �� �ã��� data ����� txt �ļ� filename ��, Ĭ��ͨ��׷�ӵķ�ʽ���ļ�
            std::ios::app       ��׷�ӵķ�ʽ���ļ�
            std::ios::nocreate  �������ļ���filename������ʱ��ʧ��
			std::ios::trunc     ����ļ����ڣ����ļ�������Ϊ0
			std::ios::binaru    �Զ����Ʒ�ʽ���ļ���ȱʡ��ʽΪ�ı���ʽ
 * ��    ע���ļ���λ�á��ļ�������ȡ
 * ����ʱ�䣺2015�� 5�� 12��
 */
void export_data_to_txtfile(int data)
{
	std::ofstream file;
	file.open("C://Users//rudy//Desktop//folder//filename.txt", std::ios::app);
	file << data << std::endl;
	file.close();
}

/*
 * �������ƣ�cal_dot_product
 * �������ܣ������ά���� A �� B �ĵ��
 * ���������A & B  �� �Զ��� vector �ṹ������
 * �����������
 * �� �� ֵ��AB     �� float����
 * �� �� �ã���
 * ����ʱ�䣺2015�� 5�� 13��
 */
float cal_dot_product(struct vector A, struct vector B)
{
	return (A.x*B.x + A.y*B.y);
}

/*
* �������ƣ�cal_dot_product
* �������ܣ�������ά���� A �� B �ĵ��
* ���������A & B  �� �Զ��� vector3D �ṹ������
* �����������
* �� �� ֵ��AB     �� float����
* �� �� �ã���
* ����ʱ�䣺2015�� 5�� 13��
*/
float cal_dot_product3D(struct vector3D A, struct vector3D B)
{
	return (A.x*B.x + A.y*B.y + A.z*B.z);
}

/*
 * �������ƣ�set_console_color
 * �������ܣ����ÿ���̨������򱳾�����ɫ
 * ���������attributes  ���磺FOREGROUND_GREEN ��BACKGROUND_RED
 * �����������
 * �� �� ֵ��TRUE(���óɹ�) or FALSE(����ʧ��)
 * �� �� �ã����ÿ���̨ǰ����������ɫ
 * ��    ע����ͨ����ɫ��ϻ���µ�ɫ�ʣ������ɫ��FOREGROUND_GREEN | FOREGROUND_BLUE | FOREGROUND_RED
 * ����ʱ�䣺2015�� 5�� 17��
 */
BOOL set_console_color(WORD attributes)
{
	HANDLE  handle_console = GetStdHandle(STD_OUTPUT_HANDLE);
	if (handle_console == INVALID_HANDLE_VALUE)
		return FALSE;
	return SetConsoleTextAttribute(handle_console, attributes);
}
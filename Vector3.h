#ifndef _VECTOR_H
#define _VECTOR_H
class Vector3
{
public:
	Vector3(float fx, float fy, float fz);
	//Subtract
	Vector3 operator - (const Vector3& v) const;
	//Dot product
	float Dot(const Vector3& v) const;
	//Cross product
	Vector3 Cross(const Vector3& v) const;
private:
	float x, y, z;
};
#endif
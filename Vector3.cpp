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

	Vector3 v0 = C - A;
	Vector3 v1 = B - A;
	Vector3 v2 = P - A;

	float dot00 = v0.Dot(v0);
	float dot01 = v0.Dot(v1);
	float dot02 = v0.Dot(v2);
	float dot11 = v1.Dot(v1);
	float dot12 = v1.Dot(v2);

	float invertDeno = 1 / (dot00 * dot11 - dot01 * dot01);
	float u = (dot11*dot02 - dot01*dot12)*invertDeno;

	if (u < 0 || u>1)
	{
		return false;
	}

	float v = (dot00*dot12 - dot01*dot02)*invertDeno;
	if (v < 0 || v>1)
	{
		return false;
	}

	return (u + v <= 1);
}
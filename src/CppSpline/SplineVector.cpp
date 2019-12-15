#include "SplineVector.h"
#include <cmath>
#include <sstream>

SplineVector::SplineVector()
: x(0)
, y(0)
, z(0)
{

}

SplineVector::SplineVector(double _x, double _y, double _z)
: x(_x)
, y(_y)
, z(_z)
{
}

double SplineVector::length() const
{
	return sqrt(x*x + y* y + z*z);
}

bool SplineVector::operator==(const SplineVector& rhs) const
{
	if(this->x== rhs.x && this->y == rhs.y && this->z == rhs.z)
	{
		return true;
	}
	return false;
}

bool SplineVector::operator != (const SplineVector& rhs) const
{
	if(this->x== rhs.x && this->y == rhs.y && this->z == rhs.z)
	{
		return false;
	}
	return true;
}

bool SplineVector::operator<=(const SplineVector& rhs) const
{
	if(this->x <= rhs.x && this->y <= rhs.y && this->z <= rhs.z)
	{
		return true;
	}
	return false;
}

bool SplineVector::operator>=(const SplineVector& rhs) const
{
	if(this->x >= rhs.x && this->y >= rhs.y && this->z >= rhs.z)
	{
		return true;
	}
	return false;
}

SplineVector SplineVector::cross(const SplineVector& rhs) const
{
	double vx=this->y * rhs.z - this->z * rhs.y;
	double vy= - (this->x * rhs.z - this->z * rhs.x);
	double vz=this->x * rhs.y - this->y * rhs.x;

	return SplineVector(vx, vy, vz);
}

double SplineVector::dot(const SplineVector& rhs) const
{
	return this->x * rhs.x + this->y * rhs.y + this->z * rhs.z;
}

double SplineVector::lengthSq() const
{
	return this->x * this->x + this->y * this->y + this->z * this->z;
}

SplineVector SplineVector::Truncate(double max_value) const
{
	SplineVector v(x, y, z);
	double len=this->length();
	if(len == 0)
	{
		return v;
	}

	if(len > max_value)
	{
		v.x = x * max_value / len;
		v.y = y * max_value / len;
		v.z = z * max_value / len;
	}

	return v;
}

SplineVector SplineVector::normalize() const
{
	double vl = this->length();
	if(vl == 0)
	{
		return SplineVector();
	}

	double vx = this->x / vl;
	double vy = this->y / vl;
	double vz= this->z / vl;

	return SplineVector(vx, vy, vz);
}

SplineVector::SplineVector(const SplineVector& rhs)
{
	this->x=rhs.x;
	this->y=rhs.y;
	this->z=rhs.z;
}

SplineVector& SplineVector::operator=(const SplineVector& rhs)
{
	this->x=rhs.x;
	this->y=rhs.y;
	this->z=rhs.z;

	return *this;
}

SplineVector& SplineVector::operator+=(const SplineVector& rhs)
{
	this->x += rhs.x;
	this->y += rhs.y;
	this->z += rhs.z;

	return *this;
}

SplineVector& SplineVector::operator-=(const SplineVector& rhs)
{
	this->x -= rhs.x;
	this->y -= rhs.y;
	this->z -= rhs.z;

	return *this;
}

SplineVector& SplineVector::operator*=(double value)
{
	this->x *= value;
	this->y *= value;
	this->z *= value;

	return *this;
}

SplineVector& SplineVector::operator/=(double value)
{
	this->x /= value;
	this->y /= value;
	this->z /= value;

	return *this;
}

std::string SplineVector::toString() const
{
	std::ostringstream oss;
	oss << "[" << x << ", " << y << ", " << z << "]";
	return oss.str();
}

SplineVector operator+(const SplineVector& v1, const SplineVector& v2)
{
	SplineVector v=v1;
	v+=v2;
	return v;
}

SplineVector operator-(const SplineVector& v1, const SplineVector& v2)
{
	SplineVector v=v1;
	v-=v2;
	return v;
}

SplineVector operator*(const SplineVector& v1, double value)
{
	SplineVector v=v1;
	v*=value;
	return v;
}

SplineVector operator/(const SplineVector& v1, double value)
{
	SplineVector v=v1;
	v/=value;
	return v;
}

SplineVector operator*(double value, const SplineVector& v1)
{
	SplineVector v=v1;
	v*=value;
	return v;
}

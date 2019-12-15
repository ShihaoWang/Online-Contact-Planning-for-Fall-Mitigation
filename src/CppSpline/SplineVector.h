#ifndef _H__SplineVector_H
#define _H__SplineVector_H

#include <string>

class SplineVector
{
public:
	double x;
	double y;
	double z;

public:
	SplineVector();
	SplineVector(double _x, double _y, double _z);
	SplineVector(const SplineVector& rhs);
	SplineVector& operator=(const SplineVector& rhs);

public:
	SplineVector cross(const SplineVector& rhs) const;
	SplineVector normalize() const;
	double dot(const SplineVector& rhs) const;
	double lengthSq() const;

public:
	SplineVector Truncate(double max_value) const;

public:
	SplineVector& operator-=(const SplineVector& rhs);
	SplineVector& operator+=(const SplineVector& rhs);
	SplineVector& operator*=(double value);
	SplineVector& operator/=(double value);

public:
	bool operator <=(const SplineVector& rhs) const;
	bool operator >=(const SplineVector& rhs) const;
	bool operator ==(const SplineVector& rhs) const;
	bool operator !=(const SplineVector& rhs) const;

public:
	void reset() { x=0; y=0; z=0; }

public:
	std::string toString() const;

public:
	double length() const;
};

SplineVector operator+(const SplineVector& v1, const SplineVector& v2);
SplineVector operator-(const SplineVector& v1, const SplineVector& v2);
SplineVector operator*(const SplineVector& v1, double value);
SplineVector operator*(double value, const SplineVector& v1);
SplineVector operator/(const SplineVector& v1, double value);

#endif

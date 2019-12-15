#ifndef _H__CATMULL_ROM_H
#define _H__CATMULL_ROM_H

#include "Curve.h"

class CatmullRom : public Curve
{
public:
	CatmullRom();
	virtual ~CatmullRom();

protected:
	virtual void _on_way_point_added();

protected:
	SplineVector interpolate(double u, const SplineVector& P0, const SplineVector& P1, const SplineVector& P2, const SplineVector& P3);
};

#endif

#pragma once
#include "Point3D.h"
#include "TextureElement.h"
class Triangle
{
public:
	Triangle(void);
	~Triangle(void);
public:
	Point3D pt[3];
public:
	void setValue(Point3D point[3]);
	bool isAPointInTriangle(Point3D point);
	double getArea();

};

	
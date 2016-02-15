#pragma once
#include "Point3D.h"
#include "TextureElement.h"
#include <Eigen/Dense>  
#include <Eigen/LU>
using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;
class Line
{
public:
	Line(void);
	~Line(void);
	Point3D start;
	Point3D end;
	TextureElement * startElement;
	TextureElement * endElement;

public:
	bool isCross(Line l);
	bool isCrossIn3D(Line l);
	bool isCross(Point3D pt1,Point3D pt2);
	static bool isCross(Point3D pt1,Point3D pt2,Point3D pt3,Point3D pt4);
	double func(Point3D pt);
	bool operator == (Line l);
	bool isContains(Point3D point);
	bool isContains(TextureElement * te);
	bool GeometryLineInter(Vector3f x1, Vector3f x2, Vector3f y1, Vector3f y2);
	bool isCrossIn3DWithN(Line l, Point3D n);

};


#pragma once
#include "stdafx.h"
#include "Point3D.h"
class TriangleRectCross
{
public:
	TriangleRectCross();
	~TriangleRectCross();
public:
	bool isCrossed(Point3D * triPoint[3], Point3D * Rect[2]);
};


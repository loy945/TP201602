#include "StdAfx.h"
#include "TriangleRectCross.h"
#include "Line.h"
#include "Triangle.h"

TriangleRectCross::TriangleRectCross()
{
}


TriangleRectCross::~TriangleRectCross()
{
}
bool TriangleRectCross::isCrossed(Point3D * triPoint[3], Point3D * Rect[2])
{
	int i = 0, j = 0;
//判断三角形是否在正方形内，或者与正方形相交
	bool isCross = false;
	//如果三角形有一点在正方形内部，则返回true
	for (i = 0; i < 3; i++)
	{
		float x = triPoint[i]->x;
		float y = triPoint[i]->y;
		float x0 = Rect[0]->x;
		float y0 = Rect[1]->x;
		float x1 = Rect[0]->x;
		float y1 = Rect[1]->y;
		
		if ((x>x0) && (x<x1) && (y>y0) && (y<y1))
		{
			isCross = true;
			break;
		}
	}
	//如果三角形一条边与正方形相交，同样返回true
	Line * l[3];
	for (i = 0; i < 3; i++)
	{
		l[i] = new Line();
	}
	l[0]->start.setValue(*triPoint[0]);
	l[0]->end.setValue(*triPoint[1]);
	l[1]->start.setValue(*triPoint[1]);
	l[1]->end.setValue(*triPoint[2]);
	l[2]->start.setValue(*triPoint[2]);
	l[2]->end.setValue(*triPoint[0]);
	Line * lb[4];
	for (i = 0; i < 4; i++)
	{
		lb[i] = new Line();
	}
	lb[0]->start.setValue(*Rect[0]);
	lb[1]->start.setValue(Point3D(Rect[0]->x, Rect[1]->y,0));
	lb[2]->start.setValue(*Rect[1]);
	lb[3]->start.setValue(Point3D(Rect[1]->x, Rect[0]->y, 0));
	lb[0]->end.setValue(Point3D(Rect[1]->x, Rect[0]->y, 0));
	lb[1]->end.setValue(*Rect[0]);
	lb[2]->end.setValue(Point3D(Rect[0]->x, Rect[1]->y, 0));
	lb[3]->end.setValue(*Rect[1]);

	for (i = 0; i < 4; i++)
	{
		for (j = 0; j < 3; j++)
		{
			if (lb[i]->isCross(*l[j]))
			{
				isCross = true;
				break;
			}
		}
	}
	return isCross;
}
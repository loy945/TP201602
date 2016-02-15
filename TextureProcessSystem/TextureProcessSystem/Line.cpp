#pragma once
#include "stdafx.h"
#include "Line.h"
#include "VectorProjections.h"
#define MINFLOATDIF 0.000001


Line::Line(void)
{
}


Line::~Line(void)
{
}
bool Line::isCross(Line l)
{
	double k1=func(l.start);
	double k2=func(l.end);
	double k3=l.func(start);
	double k4=l.func(end);
	if((k1*k2<0)&&(k3*k4<0))
	{
		return true;//½»²æ
	}
	return false;
}
bool Line::isCrossIn3DWithN(Line l, Point3D n)
{
	Vector3f xx1;
	Vector3f xx2;
	Vector3f xy1;
	Vector3f xy2;
	if ((xx1 - xy1).norm() < MINFLOATDIF)
	{
		return false;
	}
	if ((xx1 - xy2).norm() < MINFLOATDIF)
	{
		return false;
	}
	if ((xx2 - xy1).norm() < MINFLOATDIF)
	{
		return false;
	}
	if ((xx2 - xy2).norm() < MINFLOATDIF)
	{
		return false;
	}

	Vector3f x1, x2, y1, y2;
	float n1[3], v1[3], v2[3], v3[3], v4[3], res1[3], res2[3],res3[3], res4[3];
	n1[0] = n.x;
	n1[1] = n.y;
	n1[2] = n.z;

	v1[0] = this->start.x;
	v1[1] = this->start.y;
	v1[2] = this->start.z;

	v2[0] = this->end.x;
	v2[1] = this->end.y;
	v2[2] = this->end.z;

	v3[0] = l.start.x;
	v3[1] = l.start.y;
	v3[2] = l.start.z;

	v4[0] = l.end.x;
	v4[1] = l.end.y;
	v4[2] = l.end.z;

	VectorProjections(n1, v1, res1);
	VectorProjections(n1, v2, res2);
	VectorProjections(n1, v3, res3);
	VectorProjections(n1, v4, res4);
	x1 << res1[0], res1[1], res1[2];
	x2 << res2[0], res2[1], res2[2];
	y1 << res3[0], res3[1], res3[2];
	y2 << res4[0], res4[1], res4[2];

	
	return GeometryLineInter(x1, x2, y1, y2);


}
bool Line::isCrossIn3D(Line l)
{
	Vector3f x1;
	Vector3f x2;
	Vector3f y1;
	Vector3f y2;
	x1 << this->start.x, this->start.y, this->start.z;
	x2 << this->end.x, this->end.y, this->end.z;
	y1 << l.start.x, l.start.y, l.start.z;
	y2 << l.end.x, l.end.y, l.end.z;
	return GeometryLineInter(x1, x2, y1, y2);

}

double Line::func(Point3D pt)
{
	double x1=start.x;
	double y1=start.y;
	double x2=end.x;
	double y2=end.y;
	double x=pt.x;
	double y=pt.y;
	return ((y-y1)*(x2-x1)-(x-x1)*(y2-y1));
}

bool Line::operator == (Line l)
{
	//if(((this->start==l.start)&&(this->end==l.end))||((this->start==l.end)&&(this->end==l.start)))
	if(((this->startElement->index==l.startElement->index)&&(this->endElement->index==l.endElement->index))||(this->startElement->index==l.endElement->index)&&(this->endElement->index==l.startElement->index))
	{
		return true;
	}
	return false;	
}

bool Line::isContains(Point3D point)
{
	if((this->start==point)||(this->end==point))
	{
		return true;
	}
	return false;
}

bool Line::isContains(TextureElement * te)
{
	if((this->startElement==te)||(this->endElement==te))
	{
		return true;
	}
	return false;
}
bool Line::isCross(Point3D pt1,Point3D pt2)
{
	Line l1;
	l1.start.setValue(pt1);
	l1.end.setValue(pt2);
	return this->isCross(l1);
}

bool Line::isCross(Point3D pt1, Point3D pt2, Point3D pt3, Point3D pt4)
{
	Line l1;
	l1.start.setValue(pt1);
	l1.end.setValue(pt2);
	return l1.isCross(pt3,pt4);
}
bool Line::GeometryLineInter(Vector3f x1, Vector3f x2, Vector3f y1, Vector3f y2)
{
	int i = 0;
	Matrix3f k;

	if ((x1 - y1).norm() < MINFLOATDIF)
	{
		return false;
	}
	if ((x1 - y2).norm() < MINFLOATDIF)
	{
		return false;
	}
	if ((x2 - y1).norm() < MINFLOATDIF)
	{
		return false;
	}
	if ((x2 - y2).norm() < MINFLOATDIF)
	{
		return false;
	}



	for (i = 0; i < 3; i++)
	{
		k(i, 0) = x1(i) - x2(i);
		k(i, 1) = -(y1(i) - y2(i));
		k(i, 2) = x2(i) - y2(i);
	}
	FullPivLU<Matrix3f> lu_decomp(k);
	auto rank = lu_decomp.rank();
	if (rank == 3)
	{
		return false;
	}
	else
	{
		return true;
	}
}
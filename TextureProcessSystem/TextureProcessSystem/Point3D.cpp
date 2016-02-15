#include "stdafx.h"
#include "Point3D.h"


Point3D::Point3D(void)
{
	x=y=z=0;
}


Point3D::~Point3D(void)
{
}
Point3D::Point3D(float pos[3])
{
	this->x = pos[0];
	this->y = pos[1];
	this->z = pos[2];
}
Point3D_3::Point3D_3(void)
{
	update = true;//默认数据已压入vector
	pt[0].x = -1;
	pt[0].y = -1;
	pt[0].z = -1;
}
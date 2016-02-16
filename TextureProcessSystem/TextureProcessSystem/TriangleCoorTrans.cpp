#include "StdAfx.h"
#include "TriangleCoorTrans.h"
#include "config.h"

TriangleCoorTrans::TriangleCoorTrans()
{
}


TriangleCoorTrans::~TriangleCoorTrans()
{
}

void TriangleCoorTrans::init(Point3D * * trianglePos)
{

	for (int i = 0; i < 3; i++)
	{
		m_trianglePos[i] = trianglePos[i];
	}



}
Point3D * TriangleCoorTrans::convertCoordXY2UV(Point3D * centerPosXYCoord)
{
	Vector3f l[3];
	Vector3f p;
	p[0] = centerPosXYCoord->x;
	p[1] = centerPosXYCoord->y;
	p[2] = centerPosXYCoord->z;

	for (int i = 0; i < 3; i++)
	{
		l[i][0] = m_trianglePos[i]->x;
		l[i][1] = m_trianglePos[i]->y;
		l[i][2] = m_trianglePos[i]->z;
	}
	Vector3f ap = p - l[0];
	Vector3f ab = l[1] - l[0];
	Vector3f ac = l[2] - l[0];



	//householder解方程组
	//(ab,ac)(u;v)=ap;
	Matrix3d A;
	A << ab[0], ac[0], 0, ab[1], ac[1], 0, ab[2], ac[2], 0;
	Vector3d B(ap[0],ap[1],0);

	Vector3d x = A.colPivHouseholderQr().solve(B);
	float u = x[0];
	float v = x[1];
	return &Point3D(u, v, 0);

}
Point3D * TriangleCoorTrans::convertCoordUV2XY(Point3D * centerPosUVCoord)
{
	Point3D * res;

	res = &(*m_trianglePos[0] + 
		(*m_trianglePos[1] - *m_trianglePos[0]) * centerPosUVCoord->x +
		(*m_trianglePos[2] - *m_trianglePos[0]) * centerPosUVCoord->y);
	return res;
}
Point3D * TriangleCoorTrans::convertCoordXYZ2ABC(Point3D * centerPosXYZCoord)
{
	Point3D * res=new Point3D(0,0,0);
	//读入P点坐标和三角形顶点ABC
	Vector3f l[3];
	Vector3f p;
	p[0] = centerPosXYZCoord->x;
	p[1] = centerPosXYZCoord->y;
	p[2] = centerPosXYZCoord->z;
	for (int i = 0; i < 3; i++)
	{
		l[i][0] = m_trianglePos[i]->x;
		l[i][1] = m_trianglePos[i]->y;
		l[i][2] = m_trianglePos[i]->z;
	}
	Vector3f pa = l[0] - p;
	Vector3f pb = l[1] - p;
	Vector3f pc = l[2] - p;
	Vector3f ab = l[1] - l[0];
	Vector3f ac = l[2] - l[0];
	float Sabc = 0.5*((ab.cross(ac)).norm());//S(ABC)=1/2*(ab X ac) 
	float Spbc = 0.5*((pb.cross(pc)).norm());//S(PBC)=1/2*(pb X pc) 
	float Spca = 0.5*((pc.cross(pa)).norm());//S(PCA)=1/2*(pc X pa) 
	float Spab = 0.5*((pa.cross(pb)).norm());//S(PAB)=1/2*(pa X pb) 
	//λ1=S(PBC)/S(ABC) λ2=S(PCA)/S(ABC) λ3=S(PAB)/S(ABC)	
	if (Sabc<minFloat)
	{
		return &Point3D(0, 0, 0);
	}
	if (Spbc < minFloat) Spbc = 0; if (Spca < minFloat) Spca = 0; if (Spab < minFloat) Spab = 0;
	res->x = Spbc / Sabc;
	res->y = Spca / Sabc;
	res->z = Spab / Sabc;
	return res;
}
Point3D * TriangleCoorTrans::convertCoordABC2XYZ(Point3D * centerPosABCCoord)
{
	Point3D * res = new Point3D(0, 0, 0);;
	//P=λ1*a+λ2*b+λ3*c
	res = &(
		(*m_trianglePos[0])*(centerPosABCCoord->x) +
		(*m_trianglePos[1])*(centerPosABCCoord->y) +
		(*m_trianglePos[2])*(centerPosABCCoord->z));
	return res;
}
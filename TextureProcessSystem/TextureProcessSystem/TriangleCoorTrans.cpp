#include "StdAfx.h"
#include "TriangleCoorTrans.h"


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

	//float u = ap.dot(ab) / pow(ab.norm(), 2);
	//float v = ap.dot(ac) / pow(ac.norm(), 2);
	
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
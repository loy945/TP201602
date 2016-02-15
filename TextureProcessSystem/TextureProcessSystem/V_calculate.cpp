#pragma once
#include "StdAfx.h"
#include "V_calculate.h"
#include "Line.h"
#include <math.h>
#include "Point3D.h"

CV_calculate::CV_calculate()
{
}


CV_calculate::~CV_calculate()
{
}
void CV_calculate::rotateX(float *src, float * res, float c, float s)
{
	//绕x轴旋转
	//[1 0  0 0]
	//[0 c -s 0]
	//[0 s  c 0]
	//[0 0  0 1]

	CvMat SRC, RES, R1;
	float r1[16] = 
	{ 1, 0, 0, 0,
	  0, c, -s, 0,
	  0, s, c, 0,
	  0, 0, 0, 1 };
	cvInitMatHeader(&R1, 4, 4, CV_32FC1, r1);
	cvInitMatHeader(&SRC, 4, 1, CV_32FC1, src);
	cvInitMatHeader(&RES, 4, 1, CV_32FC1, res);
	//变换
	cvMatMulAdd(&R1, &SRC, 0, &RES);
	for (int i = 0; i < 4; i++)
	{
		res[i] = cvmGet(&RES, i, 0);
		if (abs(res[i])<0.00001)
		{
			res[i] = 0;
		}
	}
	return;
}
void CV_calculate::rotateY(float *src, float * res, float c, float s)
{
	//绕y轴旋转
	//[ c 0 s 0]
	//[ 0 1 0 0]
	//[-s 0 c 0]	
	//[ 0 0 0 1]
	CvMat SRC, RES, R1;
	float r1[16] = 
	{ c, 0, s, 0, 
	  0, 1, 0, 0,
	 -s, 0, c, 0,
	  0, 0, 0, 1 };
	cvInitMatHeader(&R1, 4, 4, CV_32FC1, r1);
	cvInitMatHeader(&SRC, 4, 1, CV_32FC1, src);
	cvInitMatHeader(&RES, 4, 1, CV_32FC1, res);
	//变换
	cvMatMulAdd(&R1, &SRC, 0, &RES);
	for (int i = 0; i < 4; i++)
	{
		res[i] = cvmGet(&RES, i, 0);
		if (abs(res[i])<0.00001)
		{
			res[i] = 0;
		}
	}
	return;
}
void CV_calculate::rotateZ(float *src, float * res, float c,float s)
{
	//绕z轴旋转
	//[ c s 0 0]
	//[-s c 0 0]
	//[ 0 0 1 0]
	//[ 0 0 0 1]

	CvMat SRC, RES, R1;
	float r1[16] = 
	{ c, -s, 0, 0,
	 s, c, 0, 0, 
	  0, 0, 1, 0,
	  0, 0, 0, 1 };
	cvInitMatHeader(&R1, 4, 4, CV_32FC1, r1);
	cvInitMatHeader(&SRC, 4, 1, CV_32FC1, src);
	cvInitMatHeader(&RES, 4, 1, CV_32FC1, res);
	//变换
	cvMatMulAdd(&R1, &SRC, 0, &RES);
	for (int i = 0; i < 4; i++)
	{
		res[i] = cvmGet(&RES, i, 0);
		if (abs(res[i])<0.00001)
		{
			res[i] = 0;
		}
	}
	return;
}
void CV_calculate::move(float *src,float * res,float a, float b, float c)
{
	//平移
	//[1 0 0 a]
	//[0 1 0 b]
	//[0 0 1 c]
	//[0 0 0 1]
	CvMat SRC, RES, R1;
	float r1[16] = { 1, 0, 0, a, 0, 1, 0, b, 0, 0, 1, c, 0, 0, 0, 1 };
	cvInitMatHeader(&R1, 4, 4, CV_32FC1, r1);
	cvInitMatHeader(&SRC, 4, 1, CV_32FC1, src);
	cvInitMatHeader(&RES, 4, 1, CV_32FC1, res);
	//变换
	cvMatMulAdd(&R1, &SRC, 0, &RES);
	for (int i = 0; i < 4; i++)
	{
		res[i] = cvmGet(&RES, i, 0);
	}
	return;
	
}
void CV_calculate::rotateWith(float *src, float * res, float cosX, float sinX, float cosY, float sinY, float a, float b, float c)
{
	//先逆平移
	float res1[4], res2[4], res3[4], res4[4];
	//move(src, res1, -a, -b, -c);
	res1[0] = src[0] - a;
	res1[1] = src[1] - b;
	res1[2] = src[2] - c;
	//旋转
	rotateY(res1, res2, cosY, sinY);
	rotateX(res2, res, cosX, sinX);

	int d;
	d = 1;
	return;
}
void CV_calculate::getRotateAngle(float * src, float &cosX, float &sinX, float &cosY, float &sinY)
{

	float s1 = pow(src[0], 2) + pow(src[2], 2);
	if (s1 == 0)
	{
		cosY = 1;
		sinY = 0;
	}
	else
	{
		cosY = src[2] / sqrt(s1);
		sinY = -src[0] / sqrt(s1);
	}
	float s2 = pow(src[0], 2) + pow(src[1], 2) + pow(src[2], 2);
	cosX = sqrt(s1) / sqrt(s2);
	sinX = src[1] / sqrt(s2);	
	return;
}
float CV_calculate::getAngle2(float * v1, float * v2)
{
	float sum1 = sqrt(pow(v1[0], 2) + pow(v1[1], 2));
	float sum2 = sqrt(pow(v2[0], 2) + pow(v2[1], 2));
	float p = v1[0] * v2[0] + v1[1] * v2[1];
	if (p==0)
	{
		return 0;
	}
	return acos(p / (sum1*sum2));
}
void CV_calculate::projectToXoY(float *src, float * res)
{
	res[0] = src[0];
	res[1] = src[1];
	res[2] = 0;
}
bool CV_calculate::isCross(float * p1, float * p2, float * p3, float * p4)
{
	Point3D * pt1 = new Point3D(p1);
	Point3D * pt2 = new Point3D(p2);
	Point3D * pt3 = new Point3D(p3);
	Point3D * pt4 = new Point3D(p4);
	bool res = false;
	res=Line::isCross(*pt1, *pt2, *pt3, *pt4);

	return res;
}
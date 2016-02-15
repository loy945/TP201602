#pragma once
#include "cv.h"
#include "highgui.h"
#include <opencv2/opencv.hpp> 
class CV_calculate
{
public:
	CV_calculate();
	~CV_calculate();
	static void rotateX(float *src, float * res, float c, float s);
	static void rotateY(float *src, float * res, float c, float s);
	static void rotateZ(float *src, float * res, float c, float s);
	static void move(float *src, float * res, float a, float b, float c);
	static void rotateWith(float *src, float * res, float cosX, float sinX, float cosY, float sinY, float a, float b, float c);//Æ½ÒÆ+Ðý×ª
	static void getRotateAngle(float * src, float &cosX, float &sinX, float &cosY, float &sinY);
	static float getAngle2(float * v1, float * v2);
	static void projectToXoY(float *src, float * res);
	static bool isCross(float * p1, float * p2, float * p3, float * p4);//lin1:p1[3] p2[3] line2:p3[3] p4[3]
};


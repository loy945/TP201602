#pragma once
#include"stdafx.h"
#include "plyloader.h"
class MyParameterization
{
public:
	MyParameterization();
	~MyParameterization();
	void init(Model_PLY * ply, vector<int> faceIndexs);
	void projection(float *n, float * position, float * planeCenterPos, float res[3]);
};


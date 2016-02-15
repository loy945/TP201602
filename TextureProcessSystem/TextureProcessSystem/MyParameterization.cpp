#include "StdAfx.h"
#include "MyParameterization.h"
#include <Eigen/Dense>  
#include <Eigen/LU>
#include <Eigen/Geometry>
using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;

MyParameterization::MyParameterization()
{
}


MyParameterization::~MyParameterization()
{
}
void MyParameterization::init(Model_PLY * ply, vector<int> faceIndexs)
{
	//以中心平面为基准
	int centerIndex = faceIndexs[0];
	float center[3];
	center[0] = ply->faceArry[faceIndexs[centerIndex]].corex;
	center[1] = ply->faceArry[faceIndexs[centerIndex]].corey;
	center[2] = ply->faceArry[faceIndexs[centerIndex]].corez;
	int size = faceIndexs.size();
	float res[3];
	
	//将其他面依次投影到该平面上
	for (int j = 0; j < faceIndexs.size(); j++)
	{
		float pos[3];
		pos[0] = ply->faceArry[faceIndexs[j]].corex;
		pos[1] = ply->faceArry[faceIndexs[j]].corey;
		pos[2] = ply->faceArry[faceIndexs[j]].corez;
		projection(ply->faceArry[centerIndex].n, center, pos,res);

		MatrixXd m(4, 4);
		m(0, 0) = 3;
		m(0, 1) = -1;
		m(0, 2) = 3;
		m(0, 3) = -1;

		m(0, 0) = 3;
		m(0, 1) = -1;
		m(0, 2) = 3;
		m(0, 3) = -1;

		m(0, 0) = 3;
		m(0, 1) = -1;
		m(0, 2) = 3;
		m(0, 3) = -1;

		m(0, 0) = 3;
		m(0, 1) = -1;
		m(0, 2) = 3;
		m(0, 3) = -1;
	}

}

void MyParameterization::projection(float *n, float * position, float * planeCenterPos, float res[3])
{
	
	Vector3f nor, a;//nor是平面法向量，a是要投影到此平面上的向量
	for (int i = 0; i < 3; i++)
	{
		nor[i] = n[i];
		a[i] = position[i] - planeCenterPos[i];
	}
	//法向量单位化
	nor = nor.normalized();
	//将a投影到法向量上获得向量b
	Vector3f b;
	b = (a.dot(nor) / nor.norm())*a;
	Vector3f c = a - b;
	for (int i = 0; i < 3; i++)
	{
		res[i] = c[i] + planeCenterPos[i];
	}

}
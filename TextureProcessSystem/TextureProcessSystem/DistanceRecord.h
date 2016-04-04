#pragma once
#include "plyloader.h"
#include "TriangleFace.h"
#include "Point3D.h"
class DistanceRecord
{
public:
	DistanceRecord();
	~DistanceRecord();
public:
	Model_PLY * m_plyModel;//当前加载的模型
	vector<TriangleFace> m_triangleFaceArry;//当前模型的面片信息
public:
	void init(Model_PLY * plyModel);
	void buildNeighbourTriangleIndex(int faceIndex);
	void buildDistanceData();
	float getDistanceBetweenTwoTriangles(int index1, int index2);
	float getGeodesicDistanceBetweenTwoTriangles(int index1, int index2);
	float getGeodesicDistanceBetweenTwoTrianglesWithBEPoint(int index1, int index2,Point3D pt1,Point3D pt2);
	void initDistanceFromCenter(int index);
	bool DistanceRecord::readFromFile(const char * fileName);
	bool DistanceRecord::outputToFile(const char * fileName);
};


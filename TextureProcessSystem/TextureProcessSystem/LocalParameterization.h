#pragma once
#include"stdafx.h"
#include "VertexPair.h"
#include "Point3D.h"
#include "plyloader.h"
class Polyhedron;

//1.建立一个局部模型，顶底序号从0开始，
//2.有一个映射关系vertexPair,表示局部模型和PLY模型顶点序号对应关系
//3.按照PLY模型中的三角形关系(对应顶点序号)建立三角形面片
class LocalParameterization
{
public:
	LocalParameterization();
	~LocalParameterization();
public:
	vector<VertexPair* > * vvp;
	vector<VertexPair* > * vfp;
	void init(Model_PLY * ply, vector<int> faceIndexs);
	double localPara(Model_PLY * ply, vector<int> faceIndexs, int indexCenter, Point3D * offset, float scale);
	void face_Parameterization(Model_PLY * ply, vector<int> faceIndexs);
	void face_ParameterizationUsingCGAL(Model_PLY * ply, vector<int> faceIndexs);
	int find1by2(int index2, vector<VertexPair*> *v);
	int find2by1(int index1, vector<VertexPair*> *v);
	void updateTextureCoord(int textureIndex);//更新纹理信息
	//将m_ply网格输出成off格式
	bool writeMeshInOff(const char * fileName);
public:
	Model_PLY * m_ply;
	int m_indexCenter;
	Point3D * m_2DOffset;//指定点在三角形内的相对位置
	float m_scale;
	Polyhedron *mymesh;//展开模型网格数据
	int m_faceNums;//选中展开面片数量
	vector<bool> effectFaceVector;
};


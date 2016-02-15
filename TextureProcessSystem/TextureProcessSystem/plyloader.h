/* Demonstrates how to load PLY files
* Needs some refactoring.
*
* Model needs to be triangulated
* Use blender
*
* Just the class for loading PLY files.
*
*/

#pragma once

#include <windows.h>
#include <stdio.h>
#include <string.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <cmath>
#include <string>
#include <vector>
#include "Point3D.h"
#include "Point3fv3.h"
using namespace std;

class gl_point
{
public:
	GLfloat x;
	GLfloat y;
	GLfloat z;

	GLfloat u;
	GLfloat v;
	int pointNum;//点的序列号
	vector<int> beLongToFaceIndex;
	gl_point(){ x = y = z = 0; pointNum = -1; u = -1; v = -1;}
};
class gl_point2d
{
public:
	GLfloat x;
	GLfloat y;

	int pointNum;
	gl_point2d(){ x = y = 0; pointNum = -1; }
};

class gl_face
{
public:
	int ptnum[3];
	int facenum;//点的序列号
	float length;//射线起点到面片的距离，保留较近的面；

	float value;//θ
	float h;//新加入的参数h
	bool isSetValue;
	float vectorPosX;//向量坐标
	float vectorPosY;

	float R1[4][4],R2[4][4];//旋转矩阵保存
	float corex,corey,corez;//三角面片重心位置，也是插值向量起点  
	float pcorex,pcorey;//旋转后三角面片重心位置  
	float endx,endy,endz;//插值向量终点

	//12.11新加入面片映射成2维时,3个顶点的2维坐标索引
	int ptnum_2d[3];

	bool textureclick;
	float r,g,b;
	gl_face(){ facenum = -1;vectorPosX=0;vectorPosY=0;value=0;isSetValue=false;
	textureclick=false;r=1;g=1;b=1;
	is2DCordFixed = false; isProcessedTexCor = false; renderTime = 0; isShowColorIn3D = false; disFromCenter = 9999; isCoveredByElement = false;
	}
	//3.22 2D坐标平移拼接后固定
	bool is2DCordFixed;
	//12.4.6
	bool isProcessedTexCor;
	//展开后的2D坐标
	float _2dx[3];
	float _2dy[3];
	//2015.4.14
	//三角面片3个点纹理坐标（u,v）
	/*float _2du[3];
	float _2dv[3];*/
	//2015.4.15
	float n[3];//法线
	//2015.4.15
	float _show2dx[3];
	float _show2dy[3];
	//2015.4.16
	int renderTime;
	//3d模式是否显示颜色
	bool isShowColorIn3D;
	//2015.5.10
	vector<Point3fv3 *> texCoords;//纹理坐标数组，可以保存多组纹理坐标
	Point3fv3 texCoord;//纹理坐标
	double disFromCenter; 
	bool isCoveredByElement;
public:
	void updateTexCoord();
};

 typedef struct Templet//输入向量结构体
{
    float x;//x坐标
	float y;//y坐标
	float z;//z坐标
	float c;//输入向量对应的c
	//float s;//距离
}Templet;

class Model_PLY
{
public:
	bool Model_PLY::Load(string filename);
	void Model_PLY::writeMesh(char *filename);
	void Model_PLY::Draw();
	float* Model_PLY::calculateNormal(float *coord1, float *coord2, float *coord3);
	Model_PLY();
	~Model_PLY();

	vector<gl_face> faceArry;
	vector<gl_point> pointArry;
	vector<gl_point2d> point2DArry;

	float* Faces_Triangles;
	float* Faces_Quads;
	float* Vertex_Buffer;
	float* Normals;

	int TotalConnectedTriangles;
	int TotalConnectedQuads;
	int TotalConnectedPoints;
	int TotalFaces;

	int showPart;
};


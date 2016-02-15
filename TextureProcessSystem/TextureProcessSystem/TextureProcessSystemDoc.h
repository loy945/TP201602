
// TextureProcessSystemDoc.h : CTextureProcessSystemDoc 类的接口
//


#pragma once
#include "plyloader.h"
#include "stdafx.h"
#include "DistanceRecord.h"
class FindTextureElementPosition;

class CTextureProcessSystemDoc : public CDocument
{
protected: // 仅从序列化创建
	CTextureProcessSystemDoc();
	DECLARE_DYNCREATE(CTextureProcessSystemDoc)

// 特性
public:
	COLORREF m_BackColor;
// 操作
public:
	Model_PLY plyLoader;
	bool isChooseFace;//显示了选择的面片是，该之为true
	//2015.4.6
	int centerIndex;
public:
	vector<gl_face> selectfaceLsit;
	void calculateValue(char rbffuntion);
	int  findAllPoint();
	int findFaceByTwoPoint(int point1,int point2,int oldFaceIndex);//返回三角形索引号
	vector<int>  CTextureProcessSystemDoc::findFaceByTwoPointNewWay(int point1,int point2,int oldFaceIndex);//增加数据结构后的新方法
	void rbfcreate(char rbffunt,float *res,int *num,float *coe);
	double rbfliner(float r,float constant);//线性函数
	double rbfclbic(float r,float constant);//三次方函数
	double rbfgaussian(float r,float constant);//高斯函数
	double rbfmultiquadrics(float r,float constant);//mul函数
	double rbfthinplate(float r,float constant);//薄版样条函数
	void rotate(int i,float *de);//输入要计算的第i个三角面片编号，结果保存在矩阵de中
	void face(float *a1,float *a2,float *a3,float *res);//平面
	void R(double a,double b,float *n,float *vec,float *res,int i);//分别输入θ、α、法线、ABC点坐标 和 得新的旋转点数据A1B1C1.
	void destination(int i,float *de);//求R1 R2的逆矩阵 返回原始坐标
	void error(int i);//因为发现误差过大，所以本函数为修正误差函数，输入需要修正误差的三角面片编号i
	void node(float * fac1,float *fac2,float *fac3,float *point);//输入三个平面，得到三个平面的交点
	float rbf(float d,float e,int *size,float *c,float *coefficient,char rbffuntion);//RBF插值函数
	void crosspoint1(float * a,float *b,float *c);
	void crosspoint2(float a,float *b,float *c);
	void count_h(int i);//计算第i个三角形的h

	//12.17新加入算法
	void count_TexcoordByHuang(int a,int b,int c,int d,int e);
	void findDiffrentPoint(int a,int d,int &b,int &c1,int &c2,int e1,int e2);
	int count_Texcoord(int d,int c1,int c2);
	int  count_3FaceTexcoord(int d);
	//12.18新加入 让映射到2d的2个三角形的边拼接起来
	void alterFace2dCoord(int a,int d,int b,int c1,int c2,int e1,int e2);
	//12.27
	void resetOuterTriangleTex();//把纹理坐标在0-1之外的三角形面重置
	//2015.4.6
	void calTexCor();
	int findFaceIndex(int d,int e1,int e2);
	vector<int> toProcesseTriangleIndex;	//保存需要处理的三角形序号
	//2015.5.14
	vector<int> pocessedTriangleIndex; //保存已处理的三角形序号
	bool ispocessed(int index);
	//
	int calTexCorTimes;//绘制纹理遍历次数
	bool addIndextoProcesseTriangleIndex(int pareIndex,int index);
	void deleteIndexfromProcesseTriangleIndex(int index);
	void calTexCorByIndex(int processedindex,int steps);
	//2015.4.15
	float r,g,b;
	int count;
	vector<int> userSelectTriangleIndex;//用户指定展开起始点
	int times;//展开次数
	int userSelectingTriangleIndex;
	vector<gl_face *> markConnectedFace(double centerLimitCos,double limitCos,int steps);//中心为 userSelectingTriangleIndex号面
	double getNorError(float * n1,float * n2);
	int showGroup;
	double _limitCos;//边缘法线夹角限制
	float _centerLimitCos;//中心法线夹角限制
	int _steps;//迭代次数
	//2015.5.5
	FindTextureElementPosition *_ftep;
	//2015.5.20
	int texGenTime;
	void reset2dCoord();
	bool logTex;
	bool show_ftep;//显示文理元素
	//2015.5.21 
	float offset[2];//偏移量
	Point3D * offsetPT;
	void calVertex2D(float pos[3],int index);//计算第index个三角形上点pos，投影后的坐标
	void buildTexCoordByIndex(int index, int maxDeep, int maxNum, float radius,int textureIndex);
	void buildTexCoord(int index, vector<int>&v, int &deep, int maxDeep, int maxNum, float radius);
	double getSelectArea(vector<int>&v);
	//20150906
	DistanceRecord * dr;//测地线距离数据
	bool isdrawbg;//背景填充颜色
	bool istestV;
// 重写
public:
	virtual BOOL OnNewDocument();
	virtual void Serialize(CArchive& ar);
#ifdef SHARED_HANDLERS
	virtual void InitializeSearchContent();
	virtual void OnDrawThumbnail(CDC& dc, LPRECT lprcBounds);
#endif // SHARED_HANDLERS

// 实现
public:
	virtual ~CTextureProcessSystemDoc();

	BOOL OnOpenDocument(LPCTSTR lpszPathName);
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// 生成的消息映射函数
protected:
	DECLARE_MESSAGE_MAP()

#ifdef SHARED_HANDLERS
	// 用于为搜索处理程序设置搜索内容的 Helper 函数
	void SetSearchContent(const CString& value);
#endif // SHARED_HANDLERS
};

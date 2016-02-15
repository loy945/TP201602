#pragma once
#include "stdafx.h"
#include "TextureProcessSystemDoc.h"
#include "plyloader.h"
#include "cv.h"
#include "highgui.h"
#include <opencv2/opencv.hpp> 
// CGLBaseView 视图
#define  BUFFER_LENGTH 512
#include "var.h"
#include "CBMPLoader.h"

class CGLBaseView : public CView
{
	DECLARE_DYNCREATE(CGLBaseView)

protected:
	CGLBaseView();           // 动态创建所使用的受保护的构造函数
	virtual ~CGLBaseView();

public:
	virtual void OnDraw(CDC* pDC);      // 重写以绘制该视图
#ifdef _DEBUG
	virtual void AssertValid() const;
#ifndef _WIN32_WCE
	virtual void Dump(CDumpContext& dc) const;
#endif
#endif

protected:
	CTextureProcessSystemDoc* GetDocument(); //获取系统文档类

	//配置OpenGL环境
public:
	int w;//宽
	int h;//宽高

	HGLRC m_hRC;  //绘图 RC
	CClientDC * m_pDC; //绘图 DC
	CTextureProcessSystemDoc *m_pDoc;

	virtual void DrawScene(); //绘制场景

	BOOL SetupPixelFormat(); //绘图板设置
	void InitialOpenGL(); //初始化OpenGL
	bool initMultiTexture();
	bool isExtensionSupported(const char* string);
	void makeCheckImage(void);
	void LoadGLTextures(void);
	void drawPLY(void);
	void drawPLYwithMultiTexture();
	//2014.8.11 模型旋转变换所需变量
	bool isInit;
	//2015.3.2添加模型展开为平面变量
	bool modelShowIn3Dor2D;//true代表显示三维模型 false显示展开到二维上的模型
	void draw2DModel();
	CBMPLoader m_texture[53];      /**< 载入52幅纹理 */
	GLuint texture;
	void DrawTest();
	bool isFill;
	bool isOpenDepthTest;
	
public:
	//缩放变量
	float scale_X;
	float scale_Y;
	float scale_Z;

	//旋转变量
	float rotate_X;
	float rotate_Y;
	float rotate_Z;

	//平移变量
	float trans_X;
	float trans_Y;
	float trans_Z;

	//鼠标当前点击点
	CPoint mousepoint;

	//键盘按键 G 网格显示 P 点画模式 F填充模式
	char keyBoard;

	//选择面片 2014.8.11
public:
	bool IntersectTriangle(vector<gl_face> faceArry, vector<gl_point> pointarry);//判断线面是否相交

	GLuint selectBuff[BUFFER_LENGTH];//选择缓存
	GLint viewport[4];//视点
	GLdouble modelview[16];//模型矩阵
	GLdouble projection[16];//投影矩阵

	GLdouble nearX, nearY, nearZ;//射线起点
	GLdouble farX, farY, farZ;//射线上较远的点

	GLfloat g_color;//选中面片颜色

	vector<gl_face> selectedFace;//存放被选中的面片

protected:
	DECLARE_MESSAGE_MAP()
public:
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg void OnDestroy();
	afx_msg BOOL OnEraseBkgnd(CDC* pDC);
	afx_msg void OnSize(UINT nType, int cx, int cy);
	afx_msg virtual void OnLButtonDown(UINT nFlags, CPoint point);
	afx_msg virtual void OnLButtonUp(UINT nFlags, CPoint point);
	afx_msg virtual void OnMouseMove(UINT nFlags, CPoint point);
	afx_msg virtual BOOL OnMouseWheel(UINT nFlags, short zDelta, CPoint pt);
	virtual BOOL PreTranslateMessage(MSG* pMsg);
	afx_msg virtual void OnRButtonDown(UINT nFlags, CPoint point);
};



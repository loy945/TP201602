#pragma once
#include "GLBaseView.h"

// C_Left_Top_View 视图

class C_Left_Top_View : public CGLBaseView
{
	DECLARE_DYNCREATE(C_Left_Top_View)

protected:
	C_Left_Top_View();           // 动态创建所使用的受保护的构造函数
	virtual ~C_Left_Top_View();
#ifdef _DEBUG
	virtual void AssertValid() const;
#ifndef _WIN32_WCE
	virtual void Dump(CDumpContext& dc) const;
#endif
#endif

public:
	void DrawScene();
protected:
	DECLARE_MESSAGE_MAP()

	float Src_A[3];
	float Src_B[3];
	float Src_C[3];

	float Des_A[3];
	float Des_B[3];
	float Des_C[3];
public:

};



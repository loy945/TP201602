
// TextureProcessSystem.h : TextureProcessSystem 应用程序的主头文件
//
#pragma once

#ifndef __AFXWIN_H__
	#error "在包含此文件之前包含“stdafx.h”以生成 PCH 文件"
#endif

#include "resource.h"       // 主符号
#include "TextureProcessSystemDoc.h"
// CTextureProcessSystemApp:
// 有关此类的实现，请参阅 TextureProcessSystem.cpp
//

class CTextureProcessSystemApp : public CWinApp
{
public:
	CTextureProcessSystemApp();


// 重写
public:
	virtual BOOL InitInstance();
	virtual int ExitInstance();

// 实现
	afx_msg void OnAppAbout();
	afx_msg void OnSetValue(char rbffuntion);
	afx_msg void OnSetValueL();
	afx_msg void OnSetValueC();
	afx_msg void OnSetValueG();
	afx_msg void OnSetValueM();
	afx_msg void OnSetValueT();
	afx_msg void Check();
	afx_msg void MappingToFace();
	afx_msg void SelectFace();
	DECLARE_MESSAGE_MAP()
};

extern CTextureProcessSystemApp theApp;

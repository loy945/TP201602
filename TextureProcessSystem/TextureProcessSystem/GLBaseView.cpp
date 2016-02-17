// GLBaseView.CPropertyPage : ʵ���ļ�
//
#pragma  once
#include "stdafx.h"
#include "TextureProcessSystem.h"
#include "GLBaseView.h"
#include "plyloader.h"
#include "FindTextureElementPosition.h"
#include "CBMPLoader.h"
#include <fstream>
#include "V_calculate.h"
#include <math.h>
using namespace std;
// CGLBaseView

IMPLEMENT_DYNCREATE(CGLBaseView, CView)

CGLBaseView::CGLBaseView()
{
	rotate_X = rotate_Y = rotate_Z = 0;
	rotate_X = -90;
	rotate_Z = 90;
	trans_X = trans_Y = trans_Z = 0;
	scale_X = scale_Y = scale_Z = 1;

	isInit = false;
	modelShowIn3Dor2D = true;
	ofstream f("warnlog.txt");
	f.close();
	ofstream f1("texlog.txt");
	f1.close();
	isOpenDepthTest = false;
	
}

CGLBaseView::~CGLBaseView()
{
}

BEGIN_MESSAGE_MAP(CGLBaseView, CView)
	ON_WM_CREATE()
	ON_WM_DESTROY()
	ON_WM_ERASEBKGND()
	ON_WM_SIZE()
	ON_WM_LBUTTONDOWN()
	ON_WM_LBUTTONUP()
	ON_WM_MOUSEMOVE()
	ON_WM_MOUSEWHEEL()
	ON_WM_RBUTTONDOWN()
END_MESSAGE_MAP()


// CGLBaseView ��ͼ

void CGLBaseView::OnDraw(CDC* pDC)
{
	CTextureProcessSystemDoc* pDoc = GetDocument();

	m_pDoc = pDoc;
	// TODO:  �ڴ���ӻ��ƴ���
	wglMakeCurrent(m_pDC->m_hDC, m_hRC);//ʹ RC �뵱ǰ DC �����
	//////////////////////////////////////
	COLORREF color = pDoc->m_BackColor;
	float r = (float)GetRValue(color) / 255;
	float g = (float)GetGValue(color) / 255;
	float b = (float)GetBValue(color) / 255;
	glClearColor(r, g, b, 1.0f);//���ñ�����ɫ

	glClearDepth(1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glDepthFunc(GL_LESS);// ʹ�����
	glViewport(0, 0, w, h);  //�����ӿ�
	glLoadIdentity();
	// 	//gluLookAt(0, 0, -10, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
	// 	GLfloat light_ambient[] = { 0.0, 0.0, 0.0, 1.0 };
	// 	GLfloat light_diffuse[] = { 1.0, 1.0, 1.0, 1.0 };
	// // 	GLfloat LightPosition[] = { 0.0, 0.0, 1.0, 0.0 }; //��Դλ��
	// // 	glLightfv(GL_LIGHT1, GL_POSITION, LightPosition); //���ù�Դλ��
	// 	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
	// 	//glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glMatrixMode(GL_PROJECTION);//����ͶӰ�任״̬
	glLoadIdentity();
	glTranslatef(trans_X, trans_Y, trans_Z);
	glScalef(-scale_X*1.0f, scale_Y*1.0f, scale_Z*1.0f);
	glRotatef(rotate_X, 1.0f, 0.0f, 0.0f);
	glRotatef(rotate_Y, 0.0f, 1.0f, 0.0f);
	glRotatef(rotate_Z, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);
	glEnable(GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	glGetIntegerv(GL_VIEWPORT, viewport); // �õ��������һ�������ӿڵĲ���
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
	glGetDoublev(GL_PROJECTION_MATRIX, projection);

	////////////////////////////////////////

	//glEnable(GL_CULL_FACE);
	if (!isInit)
	{
		LoadGLTextures();
		isInit = true;
// 		float cosX, sinX, cosY, sinY;
// 		float src[4] = {1,2,3};
// 		float src1[4] = {2,3,2};
// 		float res[4];
// 		CV_calculate::getRotateAngle(src, cosX, sinX, cosY, sinY);
// 		CV_calculate::rotateWith(src, res, cosX, sinX, cosY, sinY,0,0,0);
// 		CV_calculate::rotateWith(src1, res, cosX, sinX, cosY, sinY,1,2,3);
// 		int d;
// 		d = 1;

	}
	//glCullFace(GL_BACK);	
	//DrawTest();//���Ի�ͼ
	DrawScene();  //RC ��ͼ
	//glDisable(GL_CULL_FACE);

	glLoadIdentity();

	SwapBuffers(m_pDC->m_hDC);//�� RC �����洫����ǰ�� DC �ϣ��Ӷ�
	//����Ļ����ʾ
	wglMakeCurrent(m_pDC->m_hDC, NULL);//�ͷ� RC���Ա����� DC ���л�ͼ

}


// CGLBaseView ���

#ifdef _DEBUG
void CGLBaseView::AssertValid() const
{
	CView::AssertValid();
}

#ifndef _WIN32_WCE
void CGLBaseView::Dump(CDumpContext& dc) const
{
	CView::Dump(dc);
}
#endif
#endif //_DEBUG


CTextureProcessSystemDoc * CGLBaseView::GetDocument()
{
	ASSERT(m_pDocument->IsKindOf(RUNTIME_CLASS(CTextureProcessSystemDoc)));
	return (CTextureProcessSystemDoc*)m_pDocument;
}
// CGLBaseView ��Ϣ�������

bool CGLBaseView::isExtensionSupported(const char* string)
{
	char *extension;			/**< ָ����չ�ַ�����ָ�� */
	char *end;				    /**< ���һ���ַ�ָ�� */
	int idx;

	extension = (char*)glGetString(GL_EXTENSIONS);
	if (extension == NULL)
		return false;

	/** �õ����һ���ַ� */
	end = extension + strlen(extension);

	/** ѭ�������ַ���string */
	while (extension < end)
	{
		/** �ҵ��ո�ǰ��һ���ַ��� */
		idx = strcspn(extension, " ");

		/** ����Ƿ��ҵ��ַ���string */
		if ((strlen(string) == idx) && (strncmp(string, extension, idx) == 0))
		{
			return true;
		}

		/** ��ǰ������string,������һ���ַ��� */
		extension += (idx + 1);
	}
	/** û���ҵ�,�򷵻�false */
	return false;
}

bool CGLBaseView::initMultiTexture()
{
	/** ����Ƿ�֧����չ */
	if (isExtensionSupported("GL_ARB_multitexture"))
	{

		/** ��ú���ָ���ַ */
		glActiveTextureARB = (PFNGLACTIVETEXTUREARBPROC)wglGetProcAddress("glActiveTextureARB");
		glMultiTexCoord2fARB = (PFNGLMULTITEXCOORD2FARBPROC)wglGetProcAddress("glMultiTexCoord2fARB");
		return true;
	}
	else
		return false;
}
int CGLBaseView::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
	if (CView::OnCreate(lpCreateStruct) == -1)
		return -1;

	m_pDC = new CClientDC(this);  //���� DC
	ASSERT(m_pDC != NULL);

	if (!SetupPixelFormat())    //�趨��ͼ��λͼ��ʽ�����������г�
		return -1;

	m_hRC = wglCreateContext(m_pDC->m_hDC);//���� RC

	InitialOpenGL(); //��ʼ��OpenGL
	if (!initMultiTexture())
	{
		//��֧�ֶ�������
		return -1;
	}
	LoadGLTextures();

	return 0;
}
void CGLBaseView::drawPLYwithMultiTexture()
{
	vector<gl_face> * Triangle = &(m_pDoc->plyLoader.faceArry);
	vector<gl_point> * Vertex = &(m_pDoc->plyLoader.pointArry);
	vector<gl_point2d> * Vertex2d = &(m_pDoc->plyLoader.point2DArry);
	for (int i = 0; i < m_pDoc->plyLoader.faceArry.size(); i++)
	{
		float v1x = m_pDoc->plyLoader.pointArry[Triangle->at(i).ptnum[0]].x;
		float v1y = m_pDoc->plyLoader.pointArry[Triangle->at(i).ptnum[0]].y;
		float v1z = m_pDoc->plyLoader.pointArry[Triangle->at(i).ptnum[0]].z;

		float v2x = m_pDoc->plyLoader.pointArry[Triangle->at(i).ptnum[1]].x;
		float v2y = m_pDoc->plyLoader.pointArry[Triangle->at(i).ptnum[1]].y;
		float v2z = m_pDoc->plyLoader.pointArry[Triangle->at(i).ptnum[1]].z;

		float v3x = m_pDoc->plyLoader.pointArry[Triangle->at(i).ptnum[2]].x;
		float v3y = m_pDoc->plyLoader.pointArry[Triangle->at(i).ptnum[2]].y;
		float v3z = m_pDoc->plyLoader.pointArry[Triangle->at(i).ptnum[2]].z;

		float v1u, v1v, v2u, v2v, v3u, v3v;
		if (Triangle->at(i).texCoords.size() > 0)
		{
			//�ص��������4�㣬������Ϣ
			if (Triangle->at(i).texCoords.size() > 6)
			{
				if (m_pDoc->logTex)
				{
					ofstream f("warnlog.txt", ios::app);
					f << endl;
					f << "Triangle" << i << " texCoords.size: " << Triangle->at(i).texCoords.size() << endl;
					for (int k = 0; k < Triangle->at(i).texCoords.size(); k++)
					{
						f << Triangle->at(i).texCoords[k]->cor[0][0] << "," << Triangle->at(i).texCoords[k]->cor[0][1] << endl;
						f << Triangle->at(i).texCoords[k]->cor[1][0] << "," << Triangle->at(i).texCoords[k]->cor[1][1] << endl;
						f << Triangle->at(i).texCoords[k]->cor[2][0] << "," << Triangle->at(i).texCoords[k]->cor[2][1] << endl;
					}
					f << endl;
					f.close();
				}
				continue;
			}
			
			int textureIndex = 2;
			switch (Triangle->at(i).texCoords.size())
			{
			case 6:
			{		 // ��������3,�������� 
					  glActiveTextureARB(GL_TEXTURE0_ARB + 6);
					  glEnable(GL_TEXTURE_2D);
					  textureIndex=Triangle->at(i).texCoords[5]->textureIndex;
					  glBindTexture(GL_TEXTURE_2D, m_texture[textureIndex].ID);
			}
			case 5:
			{		 // ��������3,�������� 
					  glActiveTextureARB(GL_TEXTURE0_ARB + 5);
					  glEnable(GL_TEXTURE_2D);
					  textureIndex = Triangle->at(i).texCoords[4]->textureIndex;
					  glBindTexture(GL_TEXTURE_2D, m_texture[textureIndex].ID);
			}
			case 4:
			{		 // ��������3,�������� 
					  glActiveTextureARB(GL_TEXTURE0_ARB + 4);
					  glEnable(GL_TEXTURE_2D);
					  textureIndex = Triangle->at(i).texCoords[3]->textureIndex;
					  glBindTexture(GL_TEXTURE_2D, m_texture[textureIndex].ID);
			}
			case 3:
			{		// ��������2,��������
					  glActiveTextureARB(GL_TEXTURE0_ARB + 3);
					  glEnable(GL_TEXTURE_2D);
					  textureIndex = Triangle->at(i).texCoords[2]->textureIndex;
					  glBindTexture(GL_TEXTURE_2D, m_texture[textureIndex].ID);
			}
			case 2:
			{		  //��������1,�������� 
					  glActiveTextureARB(GL_TEXTURE0_ARB + 2);
					  glEnable(GL_TEXTURE_2D);
					  textureIndex = Triangle->at(i).texCoords[1]->textureIndex;
					  glBindTexture(GL_TEXTURE_2D, m_texture[textureIndex].ID);
			}
			case 1:
			{		  //��������0,�������� 
					  glActiveTextureARB(GL_TEXTURE0_ARB + 1);
					  glEnable(GL_TEXTURE_2D);
					  textureIndex = Triangle->at(i).texCoords[0]->textureIndex;
					  glBindTexture(GL_TEXTURE_2D, m_texture[textureIndex].ID);
			}
			}
			//glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
			glColor3f(0.8, 0.8, 0.8);
			glBegin(GL_TRIANGLES);//��ʾ
			glNormal3fv(Triangle->at(i).n);
			switch (Triangle->at(i).texCoords.size())
			{
			case 6:
				glMultiTexCoord2fARB(GL_TEXTURE0_ARB + 6, Triangle->at(i).texCoords[5]->cor[0][0], Triangle->at(i).texCoords[5]->cor[0][1]);
			case 5:
				glMultiTexCoord2fARB(GL_TEXTURE0_ARB + 5, Triangle->at(i).texCoords[4]->cor[0][0], Triangle->at(i).texCoords[4]->cor[0][1]);
			case 4:
				glMultiTexCoord2fARB(GL_TEXTURE0_ARB + 4, Triangle->at(i).texCoords[3]->cor[0][0], Triangle->at(i).texCoords[3]->cor[0][1]);
			case 3:
				glMultiTexCoord2fARB(GL_TEXTURE0_ARB + 3, Triangle->at(i).texCoords[2]->cor[0][0], Triangle->at(i).texCoords[2]->cor[0][1]);
			case 2:
				glMultiTexCoord2fARB(GL_TEXTURE0_ARB + 2, Triangle->at(i).texCoords[1]->cor[0][0], Triangle->at(i).texCoords[1]->cor[0][1]);
			case 1:
				glMultiTexCoord2fARB(GL_TEXTURE0_ARB + 1, Triangle->at(i).texCoords[0]->cor[0][0], Triangle->at(i).texCoords[0]->cor[0][1]);
			}
			glVertex3f(v1x, v1y, v1z);

			switch (Triangle->at(i).texCoords.size())
			{
			case 6:
				glMultiTexCoord2fARB(GL_TEXTURE0_ARB + 6, Triangle->at(i).texCoords[5]->cor[1][0], Triangle->at(i).texCoords[5]->cor[1][1]);
			case 5:
				glMultiTexCoord2fARB(GL_TEXTURE0_ARB + 5, Triangle->at(i).texCoords[4]->cor[1][0], Triangle->at(i).texCoords[4]->cor[1][1]);
			case 4:
				glMultiTexCoord2fARB(GL_TEXTURE0_ARB + 4, Triangle->at(i).texCoords[3]->cor[1][0], Triangle->at(i).texCoords[3]->cor[1][1]);
			case 3:
				glMultiTexCoord2fARB(GL_TEXTURE0_ARB + 3, Triangle->at(i).texCoords[2]->cor[1][0], Triangle->at(i).texCoords[2]->cor[1][1]);
			case 2:
				glMultiTexCoord2fARB(GL_TEXTURE0_ARB + 2, Triangle->at(i).texCoords[1]->cor[1][0], Triangle->at(i).texCoords[1]->cor[1][1]);
			case 1:
				glMultiTexCoord2fARB(GL_TEXTURE0_ARB + 1, Triangle->at(i).texCoords[0]->cor[1][0], Triangle->at(i).texCoords[0]->cor[1][1]);
			}
			glVertex3f(v2x, v2y, v2z);

			switch (Triangle->at(i).texCoords.size())
			{
			case 6:
				glMultiTexCoord2fARB(GL_TEXTURE0_ARB + 6, Triangle->at(i).texCoords[5]->cor[2][0], Triangle->at(i).texCoords[5]->cor[2][1]);
			case 5:
				glMultiTexCoord2fARB(GL_TEXTURE0_ARB + 5, Triangle->at(i).texCoords[4]->cor[2][0], Triangle->at(i).texCoords[4]->cor[2][1]);
			case 4:
				glMultiTexCoord2fARB(GL_TEXTURE0_ARB + 4, Triangle->at(i).texCoords[3]->cor[2][0], Triangle->at(i).texCoords[3]->cor[2][1]);
			case 3:
				glMultiTexCoord2fARB(GL_TEXTURE0_ARB + 3, Triangle->at(i).texCoords[2]->cor[2][0], Triangle->at(i).texCoords[2]->cor[2][1]);
			case 2:
				glMultiTexCoord2fARB(GL_TEXTURE0_ARB + 2, Triangle->at(i).texCoords[1]->cor[2][0], Triangle->at(i).texCoords[1]->cor[2][1]);
			case 1:
				glMultiTexCoord2fARB(GL_TEXTURE0_ARB + 1, Triangle->at(i).texCoords[0]->cor[2][0], Triangle->at(i).texCoords[0]->cor[2][1]);
			}
			glVertex3f(v3x, v3y, v3z);


		}
		glEnd();

		if (Triangle->at(i).texCoords.size() > 1)
		{
			if (m_pDoc->logTex)
			{
				ofstream f("texlog.txt", ios::app);
				f << "Triangle  " << i << endl;
				for (int k = 0; k < Triangle->at(i).texCoords.size(); k++)
				{
					f << Triangle->at(i).texCoords[k]->cor[0][0] << "," << Triangle->at(i).texCoords[k]->cor[0][1] << endl;
					f << Triangle->at(i).texCoords[k]->cor[1][0] << "," << Triangle->at(i).texCoords[k]->cor[1][1] << endl;
					f << Triangle->at(i).texCoords[k]->cor[2][0] << "," << Triangle->at(i).texCoords[k]->cor[2][1] << endl;
				}
				f << endl;
				f.close();
			}
		}

		switch (Triangle->at(i).texCoords.size())
		{
		case 6:
		{
				  glActiveTextureARB(GL_TEXTURE0_ARB + 6);
				  glDisable(GL_TEXTURE_2D);
		}
		case 5:
		{
				  glActiveTextureARB(GL_TEXTURE0_ARB + 5);
				  glDisable(GL_TEXTURE_2D);
		}
		case 4:
		{
				  glActiveTextureARB(GL_TEXTURE0_ARB + 4);
				  glDisable(GL_TEXTURE_2D);
		}
		case 3:
		{
				  glActiveTextureARB(GL_TEXTURE0_ARB + 3);
				  glDisable(GL_TEXTURE_2D);
		}
		case 2:
		{
				  glActiveTextureARB(GL_TEXTURE0_ARB + 2);
				  glDisable(GL_TEXTURE_2D);
		}
		case 1:
		{
				  glActiveTextureARB(GL_TEXTURE0_ARB + 1);
				  glDisable(GL_TEXTURE_2D);
		}
		}
		/*if (Triangle->at(i).texCoords.size() == 1)
		{
		glActiveTextureARB(GL_TEXTURE1_ARB);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, m_texture[1].ID);

		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glColor3f(1, 1, 1);
		glBegin(GL_TRIANGLES);//��ʾ
		glNormal3fv(Triangle->at(i).n);

		glMultiTexCoord2fARB(GL_TEXTURE1_ARB, Triangle->at(i).texCoords[0]->cor[0][0], Triangle->at(i).texCoords[0]->cor[0][1]);
		glVertex3f(v1x, v1y, v1z);

		glMultiTexCoord2fARB(GL_TEXTURE1_ARB, Triangle->at(i).texCoords[0]->cor[1][0], Triangle->at(i).texCoords[0]->cor[1][1]);
		glVertex3f(v2x, v2y, v2z);

		glMultiTexCoord2fARB(GL_TEXTURE1_ARB, Triangle->at(i).texCoords[0]->cor[2][0], Triangle->at(i).texCoords[0]->cor[2][1]);
		glVertex3f(v3x, v3y, v3z);
		glEnd();

		glActiveTextureARB(GL_TEXTURE1_ARB);
		glDisable(GL_TEXTURE_2D);

		}*/


	}

	glFlush();
	m_pDoc->logTex = false;

}

void CGLBaseView::drawPLY()
{

	vector<gl_face> * Triangle = &(m_pDoc->plyLoader.faceArry);
	vector<gl_point> * Vertex = &(m_pDoc->plyLoader.pointArry);
	vector<gl_point2d> * Vertex2d = &(m_pDoc->plyLoader.point2DArry);
	if (m_pDoc->plyLoader.pointArry[Triangle->at(0).ptnum[0]].x<-4e+8)
	{
		AfxMessageBox("model data is null");
		exit(-1);
	}
	int i;
	int a = 3;
	//glActiveTextureARB(GL_TEXTURE0_ARB);
	//glEnable(GL_TEXTURE_2D);
	//glBindTexture(GL_TEXTURE_2D, m_texture[0].ID);
	glBegin(GL_TRIANGLES);//��ʾģ�͵���
	for (int i = 0; i < m_pDoc->plyLoader.faceArry.size(); i++)
	{
		if (isFill)
		{
		
// 		if (m_pDoc->dr->m_triangleFaceArry[i].isMark&&false)
// 		{
// 			glColor3f(1, 0, 0);
// 		}
		/*else
		{
				float c = m_pDoc->dr->m_triangleFaceArry[i].disFromCenter/2.0;
				if (c>0.8)
				{
					glColor3f(0, 0, 0);
				}else if (c>0.5)
				{
					glColor3f(0, 0, c);
				}
				else if (c > 0.2)
				{
					glColor3f(0, c, 0);
				}
				else{
					glColor3f(c, 0, 0);
				}
		}*/
		}
		else
		{
			if (Triangle->at(i).testV)
			{
				glColor3f(1, 0, 0);
			}else
			{
				glColor3f(0.3, 0.3, 0.3);
			}
		}
		float v1x = m_pDoc->plyLoader.pointArry[Triangle->at(i).ptnum[0]].x;
		float v1y = m_pDoc->plyLoader.pointArry[Triangle->at(i).ptnum[0]].y;
		float v1z = m_pDoc->plyLoader.pointArry[Triangle->at(i).ptnum[0]].z;

		float v2x = m_pDoc->plyLoader.pointArry[Triangle->at(i).ptnum[1]].x;
		float v2y = m_pDoc->plyLoader.pointArry[Triangle->at(i).ptnum[1]].y;
		float v2z = m_pDoc->plyLoader.pointArry[Triangle->at(i).ptnum[1]].z;

		float v3x = m_pDoc->plyLoader.pointArry[Triangle->at(i).ptnum[2]].x;
		float v3y = m_pDoc->plyLoader.pointArry[Triangle->at(i).ptnum[2]].y;
		float v3z = m_pDoc->plyLoader.pointArry[Triangle->at(i).ptnum[2]].z;

		if (Triangle->at(i).texCoords.size() == 0)
		{
			if (v1x < -4e+8)
				break;
			glNormal3fv(Triangle->at(i).n);
			glVertex3f(v1x, v1y, v1z);
			//glTexCoord2f(0, 0);
			glVertex3f(v2x, v2y, v2z);
			//glTexCoord2f(0,1);
			glVertex3f(v3x, v3y, v3z);


			//	glTexCoord2f(1,1);
		}
	}
	glEnd();
	//glDisable(GL_TEXTURE_2D);
}

void CGLBaseView::OnDestroy()
{
	m_hRC = ::wglGetCurrentContext();
	::wglMakeCurrent(NULL, NULL);
	if (m_hRC)
	{
		::wglDeleteContext(m_hRC);
	}

	if (m_pDC)
	{
		delete m_pDC;
	}
	CView::OnDestroy();

}


BOOL CGLBaseView::OnEraseBkgnd(CDC* pDC)
{
	return CView::OnEraseBkgnd(pDC);
}


void CGLBaseView::OnSize(UINT nType, int cx, int cy)
{
	CView::OnSize(nType, cx, cy);
	VERIFY(wglMakeCurrent(m_pDC->m_hDC, m_hRC));//ȷ��RC�뵱ǰDC����
	if (cy == 0) cy = 1;

	w = cx;
	h = cy;

	VERIFY(wglMakeCurrent(NULL, NULL));//ȷ��DC�ͷ�RC
}


void CGLBaseView::OnLButtonDown(UINT nFlags, CPoint point)
{
	// TODO:  �ڴ������Ϣ�����������/�����Ĭ��ֵ

	mousepoint = point;
	SetCapture();
	CView::OnLButtonDown(nFlags, point);
}


void CGLBaseView::OnLButtonUp(UINT nFlags, CPoint point)
{
	// TODO:  �ڴ������Ϣ�����������/�����Ĭ��ֵ

	mousepoint = CPoint(0, 0);
	ReleaseCapture();
	CView::OnLButtonUp(nFlags, point);
}


void CGLBaseView::OnMouseMove(UINT nFlags, CPoint point)
{
	// TODO:  �ڴ������Ϣ�����������/�����Ĭ��ֵ

	if (GetCapture() == this)
	{
		//Increment the object rotation angles
		rotate_X += float((point.y - mousepoint.y) / 3.6);
		rotate_Y += float((point.x - mousepoint.x) / 3.6);
		//Redraw the view
		Invalidate(FALSE);
		//Set the mouse point
		mousepoint = point;
	};
	CView::OnMouseMove(nFlags, point);
}


BOOL CGLBaseView::OnMouseWheel(UINT nFlags, short zDelta, CPoint pt)
{
	// TODO:  �ڴ������Ϣ�����������/�����Ĭ��ֵ		texCoorGenTime	0	int

	if (zDelta > 0)//��ǰ�������Ŵ�
	{
		scale_X = float(((zDelta / 120) + 0.1)*scale_X);
		scale_Y = float(scale_X);
		scale_Z = float(scale_X);
	}
	else if (zDelta < 0)
	{
		zDelta = -zDelta;
		scale_X = float(((zDelta / 120) - 0.1)*scale_X);
		scale_Y = float(scale_X);
		scale_Z = float(scale_X);
	}

	Invalidate(FALSE);
	return CView::OnMouseWheel(nFlags, zDelta, pt);
}


void CGLBaseView::OnRButtonDown(UINT nFlags, CPoint point)
{
	// TODO:  �ڴ������Ϣ�����������/�����Ĭ��ֵ
	GLfloat  winX, winY;
	CRect re;
	GetWindowRect(&re);
	int screenHeight = re.Height(), screenWidth = re.Width();//��Ļ��͸�

	//�任Ҫ��ͼ�������˳��һ������������ת�����������

	winX = GLfloat(point.x);
	winY = GLfloat(screenHeight - point.y);

	gluUnProject((GLdouble)winX, (GLdouble)winY, 0.0, modelview, projection, viewport, &nearX, &nearY, &nearZ);
	gluUnProject((GLdouble)winX, (GLdouble)winY, 1.0, modelview, projection, viewport, &farX, &farY, &farZ);

	g_color = 0.0;

	selectedFace.clear();

	if (IntersectTriangle(m_pDoc->plyLoader.faceArry, m_pDoc->plyLoader.pointArry))
	{
		g_color = 1.0;
		m_pDoc->isChooseFace = false;
		Invalidate(FALSE);
	}
	CView::OnRButtonDown(nFlags, point);
}


BOOL CGLBaseView::PreTranslateMessage(MSG* pMsg)
{
	// TODO:  �ڴ����ר�ô����/����û���

	if (pMsg->message == WM_KEYDOWN)
	{
		keyBoard = pMsg->wParam;

		//������ʾ
		if (keyBoard > 51 && keyBoard < 58)
		{
			m_pDoc->showGroup = keyBoard - 51;
			m_pDoc->plyLoader.showPart = keyBoard - 51;
		}

		switch (keyBoard)
		{
			//��37��39��38��40 
		case 37:
		{
				   //��
				   trans_X -= 0.01;
		}
			break;
		case 38:
		{
				   //��
				   trans_Y += 0.01;
				   m_pDoc->show_ftep = true;
		}
			break;
		case 39:
		{
				   //��
				   trans_X += 0.01;
		}
			break;
		case 40:
		{
				   //��
				   trans_Y -= 0.01;
				   m_pDoc->show_ftep = false;
		}
			break;
		case 35:
		{
				   //Home
				   rotate_Z += 1;
		}
			break;
		case 36:
		{
				   //End
				   rotate_Z -= 1;
		}
			break;
		case 13:
		{
				   //�س���
				   ofstream f("running record.txt");
				   DWORD TimeStart, TimeEnd, TimeUsed;
				   DWORD m, s, ms;
				   TimeStart = GetTickCount();
				   for (int counti = 0; counti < 1; counti++)
				   {
					   m_pDoc->_ftep->buildByStep();
				   }
				   TimeEnd = GetTickCount();
				   TimeUsed = TimeEnd - TimeStart;
				   m = TimeUsed / 60000;
				   s = TimeUsed / 1000 - m * 60;
				   ms = TimeUsed % 1000;
				   f << "element nums:" << m_pDoc->_ftep->m_targetTexture->tes.size() << endl;
				   f << "cost time:" << m << " minutes " << s << " seconds" << ms << " milliseconds" << endl;
				   f.close();
		}
			break;
		case 9:
		{
				   //TAB
				   ofstream f("running record.txt");
				   DWORD TimeStart, TimeEnd, TimeUsed;
				   DWORD m, s, ms;
				   TimeStart = GetTickCount();
				   for (int counti = 0; counti < 500; counti++)
				   {
					   m_pDoc->_ftep->buildByStep();
				   }
				   TimeEnd = GetTickCount();
				   TimeUsed = TimeEnd - TimeStart;
				   m = TimeUsed / 60000;
				   s = TimeUsed / 1000 - m * 60;
				   ms = TimeUsed % 1000;
				   f << "element nums:" << m_pDoc->_ftep->m_targetTexture->tes.size() << endl;
				   f << "cost time:" << m << " minutes " << s << " seconds" << ms << " milliseconds" << endl;
				   f.close();
		}break;
		case 45:
		{
				   //Insert
				   m_pDoc->selectfaceLsit.clear();
				   if (isOpenDepthTest)
				   {
					   isOpenDepthTest = false;
				   }
				   else
				   {
					   isOpenDepthTest = true;
				   }
		}break;

		}


	}

	Invalidate(FALSE);

	return CView::PreTranslateMessage(pMsg);
}

//OpenGL�������� 2014.8.12
BOOL CGLBaseView::SetupPixelFormat()
{
	static PIXELFORMATDESCRIPTOR pfd =
	{
		sizeof(PIXELFORMATDESCRIPTOR),  // size of this pfd
		1,                              // version number
		PFD_DRAW_TO_WINDOW |            // support window
		PFD_SUPPORT_OPENGL |          // support OpenGL
		PFD_DOUBLEBUFFER,             // double buffered
		PFD_TYPE_RGBA,                  // RGBA type
		24,                             // 24-bit color depth
		0, 0, 0, 0, 0, 0,               // color bits ignored
		0,                              // no alpha buffer
		0,                              // shift bit ignored
		0,                              // no accumulation buffer
		0, 0, 0, 0,                     // accum bits ignored
		32,                             // 32-bit z-buffer
		0,                              // no stencil buffer
		0,                              // no auxiliary buffer
		PFD_MAIN_PLANE,                 // main layer
		0,                              // reserved
		0, 0, 0                         // layer masks ignored
	};
	int pixelformat;

	if ((pixelformat = ChoosePixelFormat(m_pDC->m_hDC, &pfd)) == 0)
	{
		MessageBox("ChoosePixelFormat failed");
		return FALSE;
	}

	if (SetPixelFormat(m_pDC->m_hDC, pixelformat, &pfd) == FALSE)
	{
		MessageBox("SetPixelFormat failed");
		return FALSE;
	}

	return TRUE;
}

void CGLBaseView::makeCheckImage(void)
{
	src = cvLoadImage("����ƻ��1.jpg");

	int i, j;
	for (i = 0; i<src->height; i++)
	{
		for (j = 0; j<src->width; j++)
		{
			if ((i != 0) && (j != 0) && (i != src->height + 1) && (j != src->width + 1))
			{
				checkImage[i][j][0] = CV_IMAGE_ELEM(src, uchar, src->height - i + 1, 3 * j + 2);//r
				checkImage[i][j][1] = CV_IMAGE_ELEM(src, uchar, src->height - i + 1, 3 * j + 1);//g
				checkImage[i][j][2] = CV_IMAGE_ELEM(src, uchar, src->height - i + 1, 3 * j);//b

				if (checkImage[i][j][0]>220 && checkImage[i][j][1]>220 && checkImage[i][j][2]>220)
					checkImage[i][j][3] = 0;//͸������Ϊ 0 
				else
					checkImage[i][j][3] = 255;//��͸������Ϊ 255, Ҳ�����Ժ��õ�1.0 
			}
			else
			{
				checkImage[i][j][0] = (GLubyte)0;
				checkImage[i][j][1] = (GLubyte)0;
				checkImage[i][j][2] = (GLubyte)0;
				checkImage[i][j][3] = (GLubyte)0;
			}

		}
	}
}
void CGLBaseView::LoadGLTextures()
{
	/// �ļ��� 
	string fileNamepro = "texels3/te (";
	string fileNameres = ").bmp";
	string fileName;
	int num = 51;
	for (int k = 1; k<num; k++)
	{
		std::stringstream ss;
		std::string numString;
		ss << k + 1;
		ss >> numString;
		fileName = fileNamepro + numString + fileNameres;
		if (!m_texture[k + 1].Load(fileName.c_str()))
		{
			return;
		}
	}
	/*/// �����ķ�λͼ
	for (int i = 0; i<7; i++)
	{
	if (!m_texture[i].Load(fileName[i]))
	{
	return;
	}

	}
	*/
	return;

}

void CGLBaseView::InitialOpenGL()
{
	PIXELFORMATDESCRIPTOR pfd;
	int  n;
	m_pDC = new CClientDC(this);
	ASSERT(m_pDC != NULL);
	if (!SetupPixelFormat())
		return;
	n = ::GetPixelFormat(m_pDC->GetSafeHdc());
	::DescribePixelFormat(m_pDC->GetSafeHdc(), n, sizeof(pfd), &pfd);
	m_hRC = wglCreateContext(m_pDC->GetSafeHdc());
	wglMakeCurrent(m_pDC->GetSafeHdc(), m_hRC);

	isFill = false;

}


void CGLBaseView::DrawScene()
{
	/** �û��Զ���Ļ��ƹ��� */
	bool isVectorShown = false;
	//����
	if (m_pDoc->plyLoader.TotalFaces > 0)
	{
		switch (keyBoard)
		{
		case 'p':
			glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);//�㻭ģʽ
			break;
		case 'P':
			glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);//�㻭ģʽ
			break;
		case 'g':
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);//����ģʽ
			break;
		case 'G':
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);//����ģʽ
			break;
		case 'f':
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);//���ģʽ
			break;
		case 'F':
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);//���ģʽ
			break;
		case 'v':
		{
					glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);//���ģʽ
					isVectorShown = true;
		}
			break;
		case 'V':
		{
					glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);//���ģʽ
					isVectorShown = true;
		}
			break;
		case '2':
		{
					glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);//����ģʽ
					modelShowIn3Dor2D = false;
		}
			break;
		case '3':
		{
					glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);//����ģʽ
					modelShowIn3Dor2D = true;
		}
			break;


		default:
		{
				   glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);//����ģʽ
				   isVectorShown = false;
		}
			break;
		}
		glColor3f(0.5, 0.5, 0.5);
		//m_pDoc->plyLoader.Draw();
		if (m_pDoc->isdrawbg)
		{
			// 			glEnable(GL_LIGHTING);
			// 			glEnable(GL_LIGHT0);
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
			glColor3f(0.8, 0.8, 0.8);
			isFill = true;
			drawPLY();
			drawPLYwithMultiTexture();
			// 			glDisable(GL_LIGHTING);
			// 			glDisable(GL_LIGHT0);
			if (m_pDoc->istestV)
			{
				glEnable(GL_POLYGON_OFFSET_LINE);
				glPolygonOffset(-1.0f, -1.0f);
				glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);;
				glColor3f(0.3, 0.3, 0.3);
				isFill = false;
				drawPLY();
				glDisable(GL_POLYGON_OFFSET_LINE);
			}

		}
		else
		{
			// 			glEnable(GL_LIGHTING);
			// 			glEnable(GL_LIGHT0);
			isFill = true;
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
			glColor3f(0.75, 0.75, 0.75);
			drawPLY();
			// 			glDisable(GL_LIGHTING);
			// 			glDisable(GL_LIGHT0);
			glEnable(GL_POLYGON_OFFSET_LINE);
			glPolygonOffset(-1.0f, -1.0f);
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);;
			glColor3f(0.3, 0.3, 0.3);
			isFill = false;
			drawPLY();
			glDisable(GL_POLYGON_OFFSET_LINE);



		}
		glDisable(GL_DEPTH_TEST);
		glDisable(GL_DEPTH_BUFFER_BIT);
		if (isOpenDepthTest)
		{
			glEnable(GL_DEPTH_TEST);
			glEnable(GL_DEPTH_BUFFER_BIT);
		}
		if (m_pDoc->_ftep&&m_pDoc->show_ftep&&!m_pDoc->isdrawbg)
		{//
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);//���ģʽ
			FindTextureElementPosition * ftep = m_pDoc->_ftep;
			ftep->draw();
		}
		if (isOpenDepthTest)
		{
			glDisable(GL_DEPTH_TEST);
			glDisable(GL_DEPTH_BUFFER_BIT);
		}

	}
	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset(-1.5f, -1.5f);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);;
	//����ѡ�����Ƭ
	vector<gl_face> * Triangle = &(m_pDoc->plyLoader.faceArry);
	vector<gl_point> * Vertex = &(m_pDoc->plyLoader.pointArry);
	vector<gl_point2d> * Vertex2d = &(m_pDoc->plyLoader.point2DArry);

	int counti = 0;
	for (vector<gl_face>::iterator it = m_pDoc->selectfaceLsit.begin(); it != m_pDoc->selectfaceLsit.end(); it++)
	{
		/*if(!Triangle->at(counti).textureclick)
		{*/
		glColor3f(0.25, 0.36, 0.85);

		glBegin(GL_TRIANGLES);
		glVertex3f(m_pDoc->plyLoader.pointArry[it->ptnum[0]].x,
			m_pDoc->plyLoader.pointArry[it->ptnum[0]].y,
			m_pDoc->plyLoader.pointArry[it->ptnum[0]].z);

		glVertex3f(m_pDoc->plyLoader.pointArry[it->ptnum[1]].x,
			m_pDoc->plyLoader.pointArry[it->ptnum[1]].y,
			m_pDoc->plyLoader.pointArry[it->ptnum[1]].z);

		glVertex3f(m_pDoc->plyLoader.pointArry[it->ptnum[2]].x,
			m_pDoc->plyLoader.pointArry[it->ptnum[2]].y,
			m_pDoc->plyLoader.pointArry[it->ptnum[2]].z);

		glEnd();
		/*}*/
		counti++;
	}

	glDisable(GL_POLYGON_OFFSET_FILL);
	if (isVectorShown)
	{
		glBegin(GL_LINES);//����������
		int i = 0;
		for (i = 0; i<Triangle->size(); i++)
		{
			glColor3f(1.0, 1.0, 1.0);
			glVertex3f(Triangle->at(i).corex, Triangle->at(i).corey, Triangle->at(i).corez);
			glVertex3f(Triangle->at(i).endx, Triangle->at(i).endy, Triangle->at(i).endz);
		}
		glEnd();
		glPopMatrix();
		glFinish();
	}
	else
	{

	}


	glFlush();
}

void CGLBaseView::draw2DModel()
{
	vector<gl_face> * Triangle = &(m_pDoc->plyLoader.faceArry);
	vector<gl_point2d> * Vertex2d = &(m_pDoc->plyLoader.point2DArry);
	glClear(GL_COLOR_BUFFER_BIT);
	glBegin(GL_TRIANGLES);
	rotate_X = 0;
	rotate_Y = 0;
	for (int i = 0; i<Triangle->size(); i++)
	{

		//if(i!=0&&i!=2677!=2386&&i!=2387)
		//{
		//	
		///*&&i!=2386&&i!=2387&&i!=2690&&i!=350	
		//&&i!=2386
		//	&&i!=2387&&i!=2690&&i!=350*/
		//	continue;
		//}
		//float v1x=Vertex2d->at(Triangle->at(i).ptnum_2d[0]).x;
		//float v1y=Vertex2d->at(Triangle->at(i).ptnum_2d[0]).y;
		//float v1z=0;

		//float v2x=Vertex2d->at(Triangle->at(i).ptnum_2d[1]).x;
		//float v2y=Vertex2d->at(Triangle->at(i).ptnum_2d[1]).y;
		//float v2z=0;

		//float v3x=Vertex2d->at(Triangle->at(i).ptnum_2d[2]).x;
		//float v3y=Vertex2d->at(Triangle->at(i).ptnum_2d[2]).y;
		//float v3z=0;

		float v1x = Triangle->at(i)._show2dx[0];
		float v1y = Triangle->at(i)._show2dy[0];
		float v1z = 0;

		float v2x = Triangle->at(i)._show2dx[1];
		float v2y = Triangle->at(i)._show2dy[1];
		float v2z = 0;

		float v3x = Triangle->at(i)._show2dx[2];
		float v3y = Triangle->at(i)._show2dy[2];
		float v3z = 0;


		if (i == 350)
		{
			cout << "";
		}
		if (Triangle->at(i).textureclick == true && ((Triangle->at(i).renderTime == m_pDoc->showGroup - 1) || (m_pDoc->showGroup == 1)))
		{
			/*	if(v1x<-4e+8)
			break;*/
			//glColor3f(0.5,0.5,0.5);//��ɫ
			glColor3f(Triangle->at(i).r, Triangle->at(i).g, Triangle->at(i).b);

			glVertex3f(v1x, v1y, v1z);
			glVertex3f(v2x, v2y, v2z);
			glVertex3f(v3x, v3y, v3z);
		}

	}
	glEnd();
	glFlush();
}

bool CGLBaseView::IntersectTriangle(vector<gl_face> facetarry, vector<gl_point> pointarry)
{
	GLfloat edge1[3];
	GLfloat edge2[3];
	for (vector<gl_face>::iterator face_it = facetarry.begin(); face_it != facetarry.end(); face_it++)
	{
		edge1[0] = pointarry[face_it->ptnum[1]].x - pointarry[face_it->ptnum[0]].x;
		edge1[1] = pointarry[face_it->ptnum[1]].y - pointarry[face_it->ptnum[0]].y;
		edge1[2] = pointarry[face_it->ptnum[1]].z - pointarry[face_it->ptnum[0]].z;

		edge2[0] = pointarry[face_it->ptnum[2]].x - pointarry[face_it->ptnum[0]].x;
		edge2[1] = pointarry[face_it->ptnum[2]].y - pointarry[face_it->ptnum[0]].y;
		edge2[2] = pointarry[face_it->ptnum[2]].z - pointarry[face_it->ptnum[0]].z;

		GLfloat dir[3];
		dir[0] = GLfloat(farX - nearX);
		dir[1] = GLfloat(farY - nearY);
		dir[2] = GLfloat(farZ - nearZ);

		GLfloat w = (GLfloat)sqrt((double)(pow((double)dir[0], 2.0) + (double)pow((double)dir[1], 2.0) + (double)pow((double)dir[2], 2.0)));
		dir[0] /= w;
		dir[1] /= w;
		dir[2] /= w;

		GLfloat pvec[3];
		pvec[0] = dir[1] * edge2[2] - dir[2] * edge2[1];
		pvec[1] = dir[2] * edge2[0] - dir[0] * edge2[2];
		pvec[2] = dir[0] * edge2[1] - dir[1] * edge2[0];

		GLfloat det;
		det = edge1[0] * pvec[0] + edge1[1] * pvec[1] + edge1[2] * pvec[2];

		GLfloat tvec[3];
		if (det > 0)
		{
			tvec[0] = GLfloat(nearX - pointarry[face_it->ptnum[0]].x);
			tvec[1] = GLfloat(nearY - pointarry[face_it->ptnum[0]].y);
			tvec[2] = GLfloat(nearZ - pointarry[face_it->ptnum[0]].z);
		}
		else
		{
			tvec[0] = GLfloat(pointarry[face_it->ptnum[0]].x - nearX);
			tvec[1] = GLfloat(pointarry[face_it->ptnum[0]].y - nearY);
			tvec[2] = GLfloat(pointarry[face_it->ptnum[0]].z - nearZ);

			det = -det;
		}

		if (det < 0.0001f)   continue;

		GLfloat u;
		u = tvec[0] * pvec[0] + tvec[1] * pvec[1] + tvec[2] * pvec[2];

		if (u < 0.0f || u > det)   continue;

		GLfloat qvec[3];
		qvec[0] = tvec[1] * edge1[2] - tvec[2] * edge1[1];
		qvec[1] = tvec[2] * edge1[0] - tvec[0] * edge1[2];
		qvec[2] = tvec[0] * edge1[1] - tvec[1] * edge1[0];

		GLfloat v;
		v = dir[0] * qvec[0] + dir[1] * qvec[1] + dir[2] * qvec[2];
		if (v < 0.0f || u + v > det)      continue;

		GLfloat t = edge2[0] * qvec[0] + edge2[1] * qvec[1] + edge2[2] * qvec[2];
		GLfloat fInvDet = 1.0f / det;
		t *= fInvDet;
		u *= fInvDet;
		v *= fInvDet;

		face_it->length = -t;
		selectedFace.push_back(*face_it);
		//m_pDoc->plyLoader.faceArry.at(face_it->facenum).isShowColorIn3D=true;
	}

	if (selectedFace.size() == 0)
	{
		return false;
	}
	else
	{
		float f_max_length = 0; int selectedNum = -1;
		for (size_t i = 0; i < selectedFace.size(); i++)
		{

			if (i == 0)
			{
				selectedNum = i;
				f_max_length = selectedFace[i].length;
			}
			else if (f_max_length < selectedFace[i].length)
			{
				f_max_length = selectedFace[i].length;
				selectedNum = i;
			}
		}

		if (selectedNum > -1)
		{
			m_pDoc->selectfaceLsit.push_back(selectedFace[selectedNum]);
		}

		m_pDoc->UpdateAllViews(NULL);
		m_pDoc->userSelectingTriangleIndex = selectedFace[selectedNum].facenum;
		return true;
	}
}
void CGLBaseView::DrawTest()
{
	glPushMatrix();
	glTranslatef(trans_X, trans_Y, trans_Z);

	glScalef(scale_X*1.0f, scale_Y*1.0f, scale_Z*1.0f);

	glRotatef(rotate_X, 1.0f, 0.0f, 0.0f);
	glRotatef(rotate_Y, 0.0f, 1.0f, 0.0f);



	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);//����ģʽ
	glEnable(GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);// ʹ�����
	/*glGetIntegerv(GL_VIEWPORT, viewport); // �õ��������һ�������ӿڵĲ���
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
	glGetDoublev(GL_PROJECTION_MATRIX, projection);*/

	/** �û��Զ���Ļ��ƹ��� */

	/** ��������0,�������� */
	glActiveTextureARB(GL_TEXTURE0_ARB);
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, m_texture[0].ID);

	/** ��������1,�������� */
	glActiveTextureARB(GL_TEXTURE1_ARB);
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, m_texture[1].ID);

	/** ����һ���ķ���ǽ�� */
	glPushMatrix();
	glTranslatef(-2.5, 0, 0);
	glScalef(2.0f, 2.0f, 2.0f);
	glBegin(GL_QUADS);
	glColor3f(0, 1, 0);
	/** ���ϵ� */
	//glMultiTexCoord2fARB(GL_TEXTURE0_ARB, 0.0f, 1.0f);
	//glMultiTexCoord2fARB(GL_TEXTURE1_ARB, 0.0f, 1.0f);
	glVertex3f(-1, 1, 0);

	/** ���µ� */
	//glMultiTexCoord2fARB(GL_TEXTURE0_ARB, 0.0f, 0.0f);
	//glMultiTexCoord2fARB(GL_TEXTURE1_ARB, 0.0f, 0.0f);
	glVertex3f(-1, -1, 0);

	/** ���µ� */
	//glMultiTexCoord2fARB(GL_TEXTURE0_ARB, 1.0f, 0.0f);
	//glMultiTexCoord2fARB(GL_TEXTURE1_ARB, 1.0f, 0.0f);
	glVertex3f(1, -1, 0);

	/** ���ϵ� */
	//glMultiTexCoord2fARB(GL_TEXTURE0_ARB, 1.0f, 1.0f);
	//glMultiTexCoord2fARB(GL_TEXTURE1_ARB, 1.0f, 1.0f);
	glVertex3f(1, 1, 0);

	//________________________

	/** ���ϵ� */
	glColor3f(1, 0, 0);
	//glMultiTexCoord2fARB(GL_TEXTURE0_ARB, 0.0f, 1.0f);
	//glMultiTexCoord2fARB(GL_TEXTURE1_ARB, 0.0f, 1.0f);
	glVertex3f(-1.2, 0.8, -1);

	/** ���ϵ� */
	//glMultiTexCoord2fARB(GL_TEXTURE0_ARB, 1.0f, 1.0f);
	//glMultiTexCoord2fARB(GL_TEXTURE1_ARB, 1.0f, 1.0f);
	glVertex3f(0.8, 0.8, -1);

	/** ���µ� */
	//glMultiTexCoord2fARB(GL_TEXTURE0_ARB, 1.0f, 0.0f);
	//glMultiTexCoord2fARB(GL_TEXTURE1_ARB, 1.0f, 0.0f);
	glVertex3f(0.8, -1.2, -1);

	/** ���µ� */
	//glMultiTexCoord2fARB(GL_TEXTURE0_ARB, 0.0f, 0.0f);
	//glMultiTexCoord2fARB(GL_TEXTURE1_ARB, 0.0f, 0.0f);
	glVertex3f(-1.2, -1.2, -1);




	//glEnd();    /**< ���ƽ��� */
	glColor3f(0, 0, 1);
	glVertex3f(-1.1, 0.9, -0.5);
	glVertex3f(-1.1, -1.1, -0.5);
	glVertex3f(0.9, -1.1, -0.5);
	glVertex3f(0.9, 0.9, -0.5);
	//glBegin(GL_QUADS);

	glEnd();
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_DEPTH_BUFFER_BIT);

	glPopMatrix();


	///** ��������0,�������� */
	//glActiveTextureARB(GL_TEXTURE0_ARB);
	//glEnable(GL_TEXTURE_2D);
	//glBindTexture(GL_TEXTURE_2D,  m_texture[2].ID);

	///** ��������1,�������� */
	//glActiveTextureARB(GL_TEXTURE1_ARB);
	//
	///** ���������������,�����ø����� */
	//if (multitexturing) 
	//	glEnable(GL_TEXTURE_2D); 
	//else 
	//	glDisable(GL_TEXTURE_2D);
	//glBindTexture(GL_TEXTURE_2D,  m_texture[3].ID);

	//static float wrap = 0;      /**< ����������� */     

	//glTranslatef(2.5, 0, 0);
	//glScalef(2.0f,2.0f,2.0f);
	//glBegin(GL_QUADS);

	//	/** ���ϵ� */
	//	glMultiTexCoord2fARB(GL_TEXTURE0_ARB, 0.0f, 1.0f);
	//	glMultiTexCoord2fARB(GL_TEXTURE1_ARB, 0.0f - wrap, 1.0f);
	//	glVertex3f(-1, 1, 0);

	//	/** ���µ� */
	//	glMultiTexCoord2fARB(GL_TEXTURE0_ARB, 0.0f, 0.0f);
	//	glMultiTexCoord2fARB(GL_TEXTURE1_ARB, 0.0f - wrap, 0.0f);
	//	glVertex3f(-1, -1, 0);

	//	/** ���µ� */
	//	glMultiTexCoord2fARB(GL_TEXTURE0_ARB, 1.0f, 0.0f);
	//	glMultiTexCoord2fARB(GL_TEXTURE1_ARB, 1.0f - wrap, 0.0f);
	//	glVertex3f(1, -1, 0);

	//	/** ���ϵ� */
	//	glMultiTexCoord2fARB(GL_TEXTURE0_ARB, 1.0f, 1.0f);
	//	glMultiTexCoord2fARB(GL_TEXTURE1_ARB, 1.0f - wrap, 1.0f);
	//	glVertex3f(1, 1, 0);
	//glEnd();											

	//wrap += 0.001f;                   /**< ���� */


	//glFlush();	                     /**< ǿ��ִ�����е�OpenGL���� */
}

// C_Left_Top_View.cpp : 实现文件
//

#include "stdafx.h"
#include "TextureProcessSystem.h"
#include "MainFrm.h"
#include "TextureProcessSystemDoc.h"
#include "GLBaseView.h"
#include "C_Left_Top_View.h"
#include <cv.h>
#include <highgui.h>
#include <math.h>


// C_Left_Top_View

IMPLEMENT_DYNCREATE(C_Left_Top_View, CGLBaseView)

C_Left_Top_View::C_Left_Top_View()
{

}

C_Left_Top_View::~C_Left_Top_View()
{
}

BEGIN_MESSAGE_MAP(C_Left_Top_View, CGLBaseView)
END_MESSAGE_MAP()

// C_Left_Top_View 诊断

#ifdef _DEBUG
void C_Left_Top_View::AssertValid() const
{
	CView::AssertValid();
}

#ifndef _WIN32_WCE
void C_Left_Top_View::Dump(CDumpContext& dc) const
{
	CView::Dump(dc);
}
#endif
#endif //_DEBUG


#define π 3.1415926

//void face(float *a1, float *a2, float *a3, float *res)//输入三个三维点，得平面Ax+By+Cz+D=0
//{
//	float vec1[3];
//	float vec2[3];
//	float vec3[3];
//	int i;
//	for (i = 0; i < 3; i++)
//	{
//		vec1[i] = a2[i] - a1[i];
//		vec2[i] = a3[i] - a2[i];
//	}
//
//	vec3[0] = vec1[1] * vec2[2] - vec1[2] * vec2[1];
//	vec3[1] = vec1[2] * vec2[0] - vec1[0] * vec2[2];
//	vec3[2] = vec1[0] * vec2[1] - vec1[1] * vec2[0];
//
//	res[0] = vec3[0];
//	res[1] = vec3[1];
//	res[2] = vec3[2];
//	res[3] = 0 - a1[0] * res[0] - a1[1] * res[1] - a1[2] * res[2];
//}
//
//void R(double a, double b, float *n, float *vec1, float *vec2, float *vec3, float *res1, float *res2, float *res3)//分别输入θ、α、法线、ABC点坐标 和 得新的旋转点数据A1B1C1
//{
//	CvMat* R1 = cvCreateMat(4, 4, CV_32FC1);
//	CvMat* R2 = cvCreateMat(4, 4, CV_32FC1);
//	CvMat* A = cvCreateMat(4, 1, CV_32FC1);
//
//	/***************矩阵R1*******************/
//	CV_MAT_ELEM(*R1, float, 0, 0) = cos(b)*cos(b)*cos(a) + sin(b)*sin(b);
//	CV_MAT_ELEM(*R1, float, 0, 1) = cos(b)*sin(a);
//	CV_MAT_ELEM(*R1, float, 0, 2) = cos(b)*cos(a)*sin(b) - sin(b)*cos(b);
//	CV_MAT_ELEM(*R1, float, 0, 3) = 0;
//
//	CV_MAT_ELEM(*R1, float, 1, 0) = -sin(a)*cos(b);
//	CV_MAT_ELEM(*R1, float, 1, 1) = cos(a);
//	CV_MAT_ELEM(*R1, float, 1, 2) = -sin(a)*sin(b);
//	CV_MAT_ELEM(*R1, float, 1, 3) = 0;
//
//	CV_MAT_ELEM(*R1, float, 2, 0) = sin(b)*cos(b)*(cos(a) - 1);
//	CV_MAT_ELEM(*R1, float, 2, 1) = sin(b)*sin(a);
//	CV_MAT_ELEM(*R1, float, 2, 2) = sin(b)*sin(b)*cos(a) + cos(b)*cos(b);
//	CV_MAT_ELEM(*R1, float, 2, 3) = 0;
//
//	CV_MAT_ELEM(*R1, float, 3, 0) = (vec1[0] * (1 - cos(b)*cos(b)*cos(a) - sin(b)*sin(b))) + (vec1[1] * sin(a)*cos(b)) + (vec1[2] * sin(b)*cos(b)*(1 - cos(a)));
//	CV_MAT_ELEM(*R1, float, 3, 1) = (-vec1[0] * cos(b)*sin(a)) + (vec1[1] * (1 - cos(a))) - (vec1[2] * sin(b)*sin(a));
//	CV_MAT_ELEM(*R1, float, 3, 2) = (vec1[0] * sin(b)*cos(b)*(1 - cos(a))) + (vec1[1] * sin(b)*sin(a)) + (vec1[2] * sin(b)*sin(b)*(1 - cos(a)));
//	CV_MAT_ELEM(*R1, float, 3, 3) = 1;
//
//	/***************求夹角β*****************/
//	CV_MAT_ELEM(*A, float, 0, 0) = n[0];
//	CV_MAT_ELEM(*A, float, 1, 0) = n[1];
//	CV_MAT_ELEM(*A, float, 2, 0) = n[2];
//	CV_MAT_ELEM(*A, float, 3, 0) = 1;
//
//	/*for (int i = 0; i < 4; i++)
//	for (int j = 0; j < 4; j++)
//	{
//	printf("%f ", CV_MAT_ELEM(*R1, float, i, j));
//	if (j == 3)
//	printf("\n");
//	}
//	printf("\n");
//	for (int i = 0; i < 4; i++)
//	for (int j = 0; j < 1; j++)
//	{
//	printf("%f ", CV_MAT_ELEM(*A, float, i, j));
//	if (j == 0)
//	printf("\n");
//	}*/
//
//
//	cvMatMul(R1, A, A);//旋转矩阵*法向量=新法向量，赋值给A矩阵
//
//	/*printf("\n");
//	for (int i = 0; i < 4; i++)
//	for (int j = 0; j<1; j++)
//	{
//	printf("%f ", CV_MAT_ELEM(*A, float, i, j));
//	if (j == 0)
//	printf("\n");
//	}*/
//
//	float m = (CV_MAT_ELEM(*A, float, 2, 0)) / (sqrt(CV_MAT_ELEM(*A, float, 0, 0)*CV_MAT_ELEM(*A, float, 0, 0) + CV_MAT_ELEM(*A, float, 1, 0)*CV_MAT_ELEM(*A, float, 1, 0) + CV_MAT_ELEM(*A, float, 2, 0)*CV_MAT_ELEM(*A, float, 2, 0)));
//	double β = acos(m);
//	if (β>π / 2)
//		β = π - β;
//	//β=-β;//取负
//	/***************矩阵R2*******************/
//	CV_MAT_ELEM(*R2, float, 0, 0) = cos(β);
//	CV_MAT_ELEM(*R2, float, 0, 1) = 0;
//	CV_MAT_ELEM(*R2, float, 0, 2) = sin(β);
//	CV_MAT_ELEM(*R2, float, 0, 3) = 0;
//
//	CV_MAT_ELEM(*R2, float, 1, 0) = 0;
//	CV_MAT_ELEM(*R2, float, 1, 1) = 1;
//	CV_MAT_ELEM(*R2, float, 1, 2) = 0;
//	CV_MAT_ELEM(*R2, float, 1, 3) = 0;
//
//	CV_MAT_ELEM(*R2, float, 2, 0) = -sin(β);
//	CV_MAT_ELEM(*R2, float, 2, 1) = 0;
//	CV_MAT_ELEM(*R2, float, 2, 2) = cos(β);
//	CV_MAT_ELEM(*R2, float, 2, 3) = 0;
//
//	CV_MAT_ELEM(*R2, float, 3, 0) = vec1[0] * (1 - cos(β)) + vec1[2] * sin(β);
//	CV_MAT_ELEM(*R2, float, 3, 1) = 0;
//	CV_MAT_ELEM(*R2, float, 3, 2) = -vec1[0] * sin(β) + vec1[2] * (1 - cos(β));
//	CV_MAT_ELEM(*R2, float, 3, 3) = 1;
//
//	//cvMatMul(R2,R1,R1);
//
//	/***************计算A点*******************/
//	CV_MAT_ELEM(*A, float, 0, 0) = vec1[0];
//	CV_MAT_ELEM(*A, float, 1, 0) = vec1[1];
//	CV_MAT_ELEM(*A, float, 2, 0) = vec1[2];
//	CV_MAT_ELEM(*A, float, 3, 0) = 1;
//
//	cvMatMul(R1, A, A);
//	cvMatMul(R2, A, A);
//
//	res1[0] = CV_MAT_ELEM(*A, float, 0, 0);
//	res1[1] = CV_MAT_ELEM(*A, float, 1, 0);
//	res1[2] = CV_MAT_ELEM(*A, float, 2, 0);
//
//	for (int i = 0; i < 3; i++)
//	{
//		if (abs(res1[i]) < 1e-005)
//			res1[i] = 0;
//	}
//
//	/***************计算B点*******************/
//	CV_MAT_ELEM(*A, float, 0, 0) = vec2[0];
//	CV_MAT_ELEM(*A, float, 1, 0) = vec2[1];
//	CV_MAT_ELEM(*A, float, 2, 0) = vec2[2];
//	CV_MAT_ELEM(*A, float, 3, 0) = 1;
//
//	cvMatMul(R1, A, A);
//	cvMatMul(R2, A, A);
//
//	res2[0] = CV_MAT_ELEM(*A, float, 0, 0);
//	res2[1] = CV_MAT_ELEM(*A, float, 1, 0);
//	res2[2] = CV_MAT_ELEM(*A, float, 2, 0);
//
//	for (int i = 0; i < 3; i++)
//	{
//		if (abs(res2[i]) < 1e-005)
//			res2[i] = 0;
//	}
//
//	/***************计算C点*******************/
//	CV_MAT_ELEM(*A, float, 0, 0) = vec3[0];
//	CV_MAT_ELEM(*A, float, 1, 0) = vec3[1];
//	CV_MAT_ELEM(*A, float, 2, 0) = vec3[2];
//	CV_MAT_ELEM(*A, float, 3, 0) = 1;
//
//	cvMatMul(R1, A, A);
//	cvMatMul(R2, A, A);
//
//	res3[0] = CV_MAT_ELEM(*A, float, 0, 0);
//	res3[1] = CV_MAT_ELEM(*A, float, 1, 0);
//	res3[2] = CV_MAT_ELEM(*A, float, 2, 0);
//
//	for (int i = 0; i < 3; i++)
//	{
//		if (abs(res3[i]) < 1e-005)
//			res3[i] = 0;
//	}
//
//
//}
//
//
//void trans3to2(float *Src_A, float *Src_B, float *Src_C, float *Des_A, float *Des_B, float *Des_C)
//{
//	/***************求法向量n****************/
//	float a[3] = { Src_A[0] - Src_B[0], Src_A[1] - Src_B[1], Src_A[2] - Src_B[2] };
//	float b[3] = { Src_A[0] - Src_C[0], Src_A[1] - Src_C[1], Src_A[2] - Src_C[2] };
//	float n[3];//向量
//	n[0] = a[1] * b[2] - a[2] * b[1];
//	n[1] = a[2] * b[0] - a[0] * b[2];
//	n[2] = a[0] * b[1] - a[1] * b[0];
//
//	/***************求夹角θ*****************/
//	float k[3] = { n[0], 0, n[2] };
//	double m = (n[0] * k[0] + n[1] * k[1] + n[2] * k[2]) / (sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2])*sqrt(k[0] * k[0] + k[1] * k[1] + k[2] * k[2]));
//	if (abs(m > 0.999))
//		m = 1;
//	double θ = acos(m);//结果θ，单位为弧度
//	if (θ > π / 2)
//		θ = π - θ;
//
//	/***************求夹角α*****************/
//	float O[3] = { 0, Src_A[1], 0 };
//	float O1[3] = { 0, Src_A[1], Src_A[2] + 8 };
//	float res[4];
//	float fac[4];
//	float D[3];
//	float A1[3], B1[3], C1[3];
//	face(Src_A, O, O1, res);
//	face(Src_A, Src_B, Src_C, fac);
//	D[0] = Src_A[0] + 5;
//	D[1] = (-res[3] / res[1]);
//	D[2] = (-fac[3] - D[0] * fac[0] - D[1] * fac[1]);
//
//	float vec[3] = { D[0] - Src_A[0], D[1] - Src_A[1], D[2] - Src_A[2] };
//	m = (vec[2]) / (sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]));
//	double α = acos(m);//结果α，单位为弧度
//	if (α > π / 2)
//		α = π - α;
//
//	//α=π-α;
//
//	/****************R计算矩阵******************************/
//	R(θ, α, n, Src_A, Src_B, Src_C, A1, B1, C1);
//
//	Des_A[0] = A1[0];
//	Des_A[1] = A1[0];
//	Des_A[2] = A1[0];
//	Des_B[0] = B1[0];
//	Des_B[1] = B1[0];
//	Des_B[2] = B1[0];
//	Des_C[0] = C1[0];
//	Des_C[1] = B1[0];
//	Des_C[2] = B1[0];
//}


void C_Left_Top_View::DrawScene()
{
	if (m_pDoc->selectfaceLsit.size() > 0)
	{
		gl_face selectface = m_pDoc->selectfaceLsit[m_pDoc->selectfaceLsit.size() - 1];

		Src_A[0] = m_pDoc->plyLoader.pointArry[selectface.ptnum[0]].x;
		Src_B[0] = m_pDoc->plyLoader.pointArry[selectface.ptnum[1]].x;
		Src_C[0] = m_pDoc->plyLoader.pointArry[selectface.ptnum[2]].x;

		Src_A[1] = m_pDoc->plyLoader.pointArry[selectface.ptnum[0]].y;
		Src_B[1] = m_pDoc->plyLoader.pointArry[selectface.ptnum[1]].y;
		Src_C[1] = m_pDoc->plyLoader.pointArry[selectface.ptnum[2]].y;

		Src_A[2] = m_pDoc->plyLoader.pointArry[selectface.ptnum[0]].z;
		Src_B[2] = m_pDoc->plyLoader.pointArry[selectface.ptnum[1]].z;
		Src_C[2] = m_pDoc->plyLoader.pointArry[selectface.ptnum[2]].z;

		/*trans3to2(Src_A, Src_B, Src_C, Des_A, Des_B, Des_C);*/


		glPushMatrix();



		glScalef(scale_X*1.0f, scale_Y*1.0f, scale_Z*1.0f);
		glRotatef(rotate_X, 1.0f, 0.0f, 0.0f);
		glRotatef(rotate_Y, 0.0f, 1.0f, 0.0f);

		glTranslatef(0.0, 0.0, 0.0);

		glColor3f(1.0, 0.36, 0.85);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);//填充模式
		glBegin(GL_TRIANGLES);

		glVertex3f(Src_A[0],Src_A[1],Src_A[2]);
		glVertex3f(Src_B[0], Src_B[1], Src_B[2]);
		glVertex3f(Src_C[0], Src_C[1], Src_C[2]);

		glVertex3f(Des_A[0], Des_A[1], Des_A[2]);
		glVertex3f(Des_B[0], Des_B[1], Des_B[2]);
		glVertex3f(Des_C[0], Des_C[1], Des_C[2]);

		glEnd();
		glPopMatrix();
		glFinish();
	}
}
// C_Left_Top_View 消息处理程序

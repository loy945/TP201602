
// TextureProcessSystemDoc.cpp : CTextureProcessSystemDoc 类的实现
//


#include "stdafx.h"
// SHARED_HANDLERS 可以在实现预览、缩略图和搜索筛选器句柄的
// ATL 项目中进行定义，并允许与该项目共享文档代码。
#ifndef SHARED_HANDLERS
#include "TextureProcessSystem.h"
#endif
#include "TextureProcessSystemDoc.h"
#include <propkey.h>
#include "plyloader.h"
#include "cv.h"
#include "highgui.h"
#include <opencv2/opencv.hpp> 
#include "PlaneRotate.h"
#include "LocalParameterization.h"
#include "Triangle.h"
#include "config.h"
#include <fstream>
using namespace std;
#ifdef PI
#else
#define PI 3.1415926
#endif
#define π 3.1415926
#define λ 2
#ifdef _DEBUG
#define new DEBUG_NEW
#endif
Templet *templet = (Templet *)malloc(sizeof (Templet)*1000);//size是你动态需要的大小.
// CTextureProcessSystemDoc

IMPLEMENT_DYNCREATE(CTextureProcessSystemDoc, CDocument)

BEGIN_MESSAGE_MAP(CTextureProcessSystemDoc, CDocument)
END_MESSAGE_MAP()


// CTextureProcessSystemDoc 构造/析构

CTextureProcessSystemDoc::CTextureProcessSystemDoc()
{
	// TODO:  在此添加一次性构造代码
	m_BackColor = RGB(93, 93, 185);
	centerIndex=-1;
	times=0;
	userSelectingTriangleIndex=-1;
	showGroup=1;
	_ftep = NULL;
	texGenTime = 0;
	logTex = false;
	show_ftep = true;
	offset[0] = 0;
	offset[1] = 0;
	offsetPT = new Point3D(0.5, 0.5, 0);
	isdrawbg = false;
	istestV = false;
}

CTextureProcessSystemDoc::~CTextureProcessSystemDoc()
{

}

BOOL CTextureProcessSystemDoc::OnNewDocument()
{
	if (!CDocument::OnNewDocument())
		return FALSE;

	// TODO:  在此添加重新初始化代码
	// (SDI 文档将重用该文档)

	return TRUE;
}

bool CTextureProcessSystemDoc::loadDataFromObj()
{
	for (int i = 0; i < objloader->F.size(); i++)
	{
		Mian * objFacei = &(objloader->F[i]);
		gl_face * plyFacei = &(plyLoader.faceArry[i]);
		for (int j = 0; j < 3; j++)
		{
			//obj 中面的第i个点的纹理坐标
			int objTexIndex = objFacei->T[j];
			double u = objloader->VT[objTexIndex].TU;
			double v = objloader->VT[objTexIndex].TV;
			//ply 中面的纹理坐标
			plyFacei->texCoord.cor[j][0] = u;
			plyFacei->texCoord.cor[j][1] = v;
		}
		plyFacei->texCoord.textureIndex = 1;
		plyFacei->texCoord.update = false;
	}
	
	return true;
}
BOOL CTextureProcessSystemDoc::OnOpenDocument(LPCTSTR lpszPathName)//打开模型文件 2014.8.11
{
	if (lpszPathName != NULL)
	{
		CString filename = lpszPathName;
		string _t_filename = filename.GetBuffer();

		if (!plyLoader.Load(_t_filename))
		{
			MessageBox(NULL, "打开模型文件失败", "错误", MB_OK);

			return FALSE;
		}
		else
		{
			//计算法线
				float A[3],B[3],C[3];
				float x1,x2,x3,y1,y2,y3,z1,z2,z3;
				vector<gl_face>  * Triangle=&(plyLoader.faceArry);
				for(int i=0;i<Triangle->size();i++)
				{
					A[0]=plyLoader.pointArry[Triangle->at(i).ptnum[0]].x;
					A[1]=plyLoader.pointArry[Triangle->at(i).ptnum[0]].y;
					A[2]=plyLoader.pointArry[Triangle->at(i).ptnum[0]].z;

					B[0]=plyLoader.pointArry[Triangle->at(i).ptnum[1]].x;
					B[1]=plyLoader.pointArry[Triangle->at(i).ptnum[1]].y;
					B[2]=plyLoader.pointArry[Triangle->at(i).ptnum[1]].z;

					C[0]=plyLoader.pointArry[Triangle->at(i).ptnum[2]].x;
					C[1]=plyLoader.pointArry[Triangle->at(i).ptnum[2]].y;
					C[2]=plyLoader.pointArry[Triangle->at(i).ptnum[2]].z;

					x1=A[0];y1=A[1];z1=A[2];
					x2=B[0];y2=B[1];z2=B[2];
					x3=C[0];y3=C[1];z3=C[2];
					float n[3];//向量
					n[0]=(y2-y1)*(z3-z1)-(y3-y1)*(z2-z1);
					n[1]=(z2-z1)*(x3-x1)-(z3-z1)*(x2-x1);
					n[2]=(x2-x1)*(y3-y1)-(x3-x1)*(y2-y1);


					/***************求法向量n****************/
					//float a[3]={A[0]-B[0],A[1]-B[1],A[2]-B[2]};
					//float b[3]={A[0]-C[0],A[1]-C[1],A[2]-C[2]};
					//float n[3];//向量
					//n[0]=a[1]*b[2]-a[2]*b[1];
					//n[1]=a[2]*b[0]-a[0]*b[2];
					//n[2]=a[0]*b[1]-a[1]*b[0];
					for(int j=0;j<3;j++)
					{
						Triangle->at(i).n[j] = n[j] / sqrtf(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]) ;
					}
				}
			//生成临接信息
 				dr = new DistanceRecord();
 				dr->init(&plyLoader);
			//读入OBJ文件
				objloader = new ObjLoader();
				objloader->load("bunny_floater_square_opennl_parameterized.obj");
			//将obj中的UV信息传递给plyLoader
				//loadDataFromObj();
				return TRUE;
		}
	}
	return FALSE;
}

// CTextureProcessSystemDoc 序列化

void CTextureProcessSystemDoc::Serialize(CArchive& ar)
{
	if (ar.IsStoring())
	{
		// TODO:  在此添加存储代码
	}
	else
	{
		// TODO:  在此添加加载代码
	}
}

#ifdef SHARED_HANDLERS

// 缩略图的支持
void CTextureProcessSystemDoc::Thumbnail(CDC& dc, LPRECT lprcBounds)
{
	// 修改此代码以绘制文档数据
	dc.FillSolidRect(lprcBounds, RGB(255, 255, 255));

	CString strText = _T("TODO: implement thumbnail drawing here");
	LOGFONT lf;

	CFont* pDefaultGUIFont = CFont::FromHandle((HFONT) GetStockObject(DEFAULT_GUI_FONT));
	pDefaultGUIFont->GetLogFont(&lf);
	lf.lfHeight = 36;

	CFont fontDraw;
	fontDraw.CreateFontIndirect(&lf);

	CFont* pOldFont = dc.SelectObject(&fontDraw);
	dc.DrawText(strText, lprcBounds, DT_CENTER | DT_WORDBREAK);
	dc.SelectObject(pOldFont);
}

// 搜索处理程序的支持
void CTextureProcessSystemDoc::InitializeSearchContent()
{
	CString strSearchContent;
	// 从文档数据设置搜索内容。
	// 内容部分应由“;”分隔

	// 例如:     strSearchContent = _T("point;rectangle;circle;ole object;")；
	SetSearchContent(strSearchContent);
}

void CTextureProcessSystemDoc::SetSearchContent(const CString& value)
{
	if (value.IsEmpty())
	{
		RemoveChunk(PKEY_Search_Contents.fmtid, PKEY_Search_Contents.pid);
	}
	else
	{
		CMFCFilterChunkValueImpl *pChunk = NULL;
		ATLTRY(pChunk = new CMFCFilterChunkValueImpl);
		if (pChunk != NULL)
		{
			pChunk->SetTextValue(PKEY_Search_Contents, value, CHUNK_TEXT);
			SetChunkValue(pChunk);
		}
	}
}

#endif // SHARED_HANDLERS

// CTextureProcessSystemDoc 诊断

#ifdef _DEBUG
void CTextureProcessSystemDoc::AssertValid() const
{
	CDocument::AssertValid();
}

void CTextureProcessSystemDoc::Dump(CDumpContext& dc) const
{
	CDocument::Dump(dc);
}
#endif //_DEBUG
void CTextureProcessSystemDoc::calculateValue(char rbffuntion)
{
		float c[3],coefficient[1],output[3];
		int i,size[1];


	vector<gl_face> * Triangle=&(plyLoader.faceArry);
	rbfcreate(rbffuntion,c,size,coefficient);//求rbf系数 
		//上述函数 输入值分别是用户选择功能rbffuntion 、c[3]为所求系数、size为向量个数n coefficient为插值系数
	int count=0;
	for(i=0;i<plyLoader.faceArry.size();i++)//循环对每个三角面片求插值
		{
			if(Triangle->at(i).isSetValue==false)
			{
				if (i == 3684)
				{
					int a = 0;
				}
				rotate(i,output);
				Triangle->at(i).value=rbf(output[0],output[1],size,c,coefficient,rbffuntion);//保存插值结果
				while(Triangle->at(i).value>π)
					{
						Triangle->at(i).value=(Triangle->at(i).value)-(2*π);
				    }
				while(Triangle->at(i).value< -(π))
					{
						Triangle->at(i).value=(Triangle->at(i).value)+(2*π);
				    }
				Triangle->at(i).isSetValue=true;
				destination(i,output);
				error(i);
			}
		}
}

void CTextureProcessSystemDoc::rbfcreate(char rbffunt,float *res,int *num,float *coe)//计算RBF系数
{
	int smooth=0;
    int i1=0,i2,i3,j3,i4,j4,i5,j5,i6,i7;
	char word=0,rbffuntion;
	double maxx=NULL,minx=NULL,maxy=NULL,miny=NULL;
	float σ,medianx,mediany,c0,c7,c8,r,de[3];	
	int ran_num,dim=3;
	int index;
	float value;
	vector<gl_face> * Triangle=&(plyLoader.faceArry);

	i1=1;
	for(int index=0;index<plyLoader.faceArry.size();index++)
	{
		if(Triangle->at(index).isSetValue)
		{
			templet[i1].z=Triangle->at(index).value;
			rotate(index,de);
			destination(index,de);
			error(index);
			
			templet[i1].x=de[0];
			templet[i1].y=de[1];
			
			i1++;
		}
	}
	i1--;

	//srand((unsigned)time(NULL));
 //   printf("随机选择的是：");
	//	 for(i1=1;i1<=10;i1++)
	//	 {
	//		 if(plyLoader.faceArry.size()>0)
	//		 {
	//			 ran_num = (plyLoader.faceArry.size())*rand() / (RAND_MAX)+1;
	//		 }
	//		 else
	//		 {
	//			 return;
	//		 }
	//		if(Triangle->at(ran_num).isSetValue==false)
	//			{
	//				printf("第%d个 ",ran_num);
	//				Triangle->at(ran_num).isSetValue=true;
	//			}
	//		else
	//			{
	//				ran_num=ran_num+rand()%100;
	//				if(ran_num>=plyLoader.faceArry.size())
	//					ran_num=ran_num-rand()%100;
	//				printf("第%d个 ",ran_num);
	//				Triangle->at(ran_num).isSetValue=true;
	//			}
	//			

	//		Triangle->at(ran_num).value=templet[i1].z=(((180)*rand() / (RAND_MAX)+1)*π)/360;//随机赋值value

	//		rotate(ran_num,de);
	//		destination(ran_num,de);
	//		error(ran_num);

	//		templet[i1].x=de[0];
	//		templet[i1].y=de[1];
	//	 }
	//	 printf("三角面片。");
	//	 i1--;

    CvMat* φ = cvCreateMat( i1, i1, CV_32FC1 );
	CvMat* P = cvCreateMat( i1, dim, CV_32FC1 );
	CvMat* P1 = cvCreateMat( dim, i1, CV_32FC1 );//P转置矩阵
	CvMat* A = cvCreateMat( i1+dim, i1+dim, CV_32FC1 );
	CvMat* A1 = cvCreateMat( i1+dim, i1+dim, CV_32FC1 );//A逆矩阵
	CvMat* B = cvCreateMat( i1+dim, 1, CV_32FC1 );
	CvMat* C = cvCreateMat( i1+dim, 1, CV_32FC1 );

/******************求相关系数σ***************************/

	for(i2=1;i2<=i1;i2++)
		{
			medianx=templet[i2].x;
			mediany=templet[i2].y;
			if(i2==1)
			{
				minx=medianx;
				miny=mediany;
			}
			if(medianx>maxx)
				maxx=medianx;
	        if(medianx<minx)
				minx=medianx;
			if(mediany>maxy)
				maxy=mediany;
	        if(mediany<miny)
				miny=mediany;	
		}
	if(dim==3)
		σ=pow((((maxx-minx)*(maxy-miny))/i1),1/2.0);
	else
		σ=((maxx-minx)/i1);
/*********************求系数Ci*******************************/
	rbffuntion=rbffunt;
		switch(rbffuntion)
	{
	case'l':{
				for(i3=1;i3<=i1;i3++)
				  for(j3=1;j3<=i1;j3++)
				  {
				   r=sqrt(pow((templet[i3].x- templet[j3].x),2)+pow((templet[i3].y- templet[j3].y),2));
				   r=rbfliner(r,σ);
				   CV_MAT_ELEM( *φ,float,i3-1,j3-1)=r;
				  }
				  break;
			};//调用线性函数
	case'c':{
				for(i3=1;i3<=i1;i3++)
				  for(j3=1;j3<=i1;j3++)
				  {
				   r=sqrt(pow((templet[i3].x- templet[j3].x),2)+pow((templet[i3].y- templet[j3].y),2));
				   r=rbfclbic(r,σ);
				   CV_MAT_ELEM( *φ,float,i3-1,j3-1)=r;
				  }
				  break;
			};//调用三次函数
	case'g':{
				for(i3=1;i3<=i1;i3++)
				  for(j3=1;j3<=i1;j3++)
				  {
				   r=sqrt(pow((templet[i3].x- templet[j3].x),2)+pow((templet[i3].y- templet[j3].y),2));
				   r=rbfgaussian(r,σ);
				   CV_MAT_ELEM( *φ,float,i3-1,j3-1)=r;
				  }
				  break;
			};//调用高斯函数
	case'm':{
				for(i3=1;i3<=i1;i3++)
				  for(j3=1;j3<=i1;j3++)
				  {
				   r=sqrt(pow((templet[i3].x- templet[j3].x),2)+pow((templet[i3].y- templet[j3].y),2));
				   r=rbfmultiquadrics(r,σ);
				   CV_MAT_ELEM( *φ,float,i3-1,j3-1)=r;
				  }
				  break;
			};//调用mul函数
	case't':{
				for(i3=1;i3<=i1;i3++)
				  for(j3=1;j3<=i1;j3++)
				  {
				   r=sqrt(pow((templet[i3].x- templet[j3].x),2)+pow((templet[i3].y- templet[j3].y),2));
				   r=rbfthinplate(r,σ);
				   CV_MAT_ELEM( *φ,float,i3-1,j3-1)=r;
				  }
				  break;
			};//调用薄版样条函数
	default:printf("RBFfuntion输入有误\n");
		}
/***************************矩阵扩充*****************************************************/

	for(i4=1;i4<=i1;i4++)
		for(j4=1;j4<=dim;j4++)
		{
			if(j4==1)
				CV_MAT_ELEM( *P,float,i4-1,j4-1)=1;
			else if(j4==2)
				CV_MAT_ELEM( *P,float,i4-1,j4-1)=templet[i4].x;
			else if(j4==3)
				CV_MAT_ELEM( *P,float,i4-1,j4-1)=templet[i4].y;
		}
	cvTranspose(P, P1);
/*********************计算A矩阵 以及A矩阵求逆得到A1***********************************/
	for(i5=1;i5<=i1+dim;i5++)
		for(j5=1;j5<=i1+dim;j5++)
		{
			if((i5<=i1)&&(j5<=i1))
			CV_MAT_ELEM( *A,float,i5-1,j5-1)=CV_MAT_ELEM( *φ,float,i5-1,j5-1);
			else if((i5<=i1)&&(j5>i1))
				CV_MAT_ELEM( *A,float,i5-1,j5-1)=CV_MAT_ELEM( *P,float,i5-1,j5-i1-1);
			else if((i5>i1)&&(j5<=i1))
				CV_MAT_ELEM( *A,float,i5-1,j5-1)=CV_MAT_ELEM( *P1,float,i5-i1-1,j5-1);
			else
				CV_MAT_ELEM( *A,float,i5-1,j5-1)=0;
		}
		cvInvert( A, A1, CV_LU );//这里有问题，用不同模式 输出A1的值会不一样（暂时选取CV_LU，是最常见模式）
/*****************************求矩阵B****************************************/
	for(i6=1;i6<=i1+dim;i6++)
	{
		if(i6<=i1)
			CV_MAT_ELEM( *B,float,i6-1,0)=templet[i6].z;
		else
			CV_MAT_ELEM( *B,float,i6-1,0)=0;
	}
/*********************计算输入向量对应的C，并保存到结构体中************************/
		cvMatMulAdd( A1, B, NULL, C );
	for (i7=1;i7<=i1;i7++)
		templet[i7].c=CV_MAT_ELEM( *C,float,i7-1,0);
	c0=CV_MAT_ELEM( *C,float,i1,0);
	c7=CV_MAT_ELEM( *C,float,i1+1,0);
	if(dim==3)
		c8=CV_MAT_ELEM( *C,float,i1+2,0);
	else
		c8=0;

	res[0]=c0;
	res[1]=c7;
	res[2]=c8;

	num[0]=i1;

	coe[0]=σ;

}

double CTextureProcessSystemDoc::rbfliner(float r,float constant)//线性函数
{
	double u;
	u=r;
	return u;
}

double CTextureProcessSystemDoc::rbfclbic(float r,float constant)//三次方函数
{
	double u;
	u=r*r*r;
	return u;
}

double CTextureProcessSystemDoc::rbfgaussian(float r,float constant)//高斯函数
{
	double u;
	u=exp(-0.5*r*r/(constant*constant));
	return u;
}

double CTextureProcessSystemDoc::rbfmultiquadrics(float r,float constant)//mul函数
{
	double u;
	u=sqrt(1+r*r/(constant*constant));
	return u;
}

double CTextureProcessSystemDoc::rbfthinplate(float r,float constant)//薄版样条函数
{
	double u;
    u=r*r*log(r+1);
	return u;
}
void CTextureProcessSystemDoc::rotate(int i,float *de)//输入要计算的第i个三角面片编号，结果保存在矩阵de中
{
	float A[3],B[3],C[3];
	vector<gl_face>  * Triangle=&(plyLoader.faceArry);
	A[0]=plyLoader.pointArry[Triangle->at(i).ptnum[0]].x;
	A[1]=plyLoader.pointArry[Triangle->at(i).ptnum[0]].y;
	A[2]=plyLoader.pointArry[Triangle->at(i).ptnum[0]].z;

	B[0]=plyLoader.pointArry[Triangle->at(i).ptnum[1]].x;
	B[1]=plyLoader.pointArry[Triangle->at(i).ptnum[1]].y;
	B[2]=plyLoader.pointArry[Triangle->at(i).ptnum[1]].z;

	C[0]=plyLoader.pointArry[Triangle->at(i).ptnum[2]].x;
	C[1]=plyLoader.pointArry[Triangle->at(i).ptnum[2]].y;
	C[2]=plyLoader.pointArry[Triangle->at(i).ptnum[2]].z;
	/***************求法向量n****************/
	float x1,x2,x3,y1,y2,y3,z1,z2,z3;
		x1=A[0];y1=A[1];z1=A[2];
		x2=B[0];y2=B[1];z2=B[2];
		x3=C[0];y3=C[1];z3=C[2];
		float n[3];//向量
		n[0]=(y2-y1)*(z3-z1)-(y3-y1)*(z2-z1);
		n[1]=(z2-z1)*(x3-x1)-(z3-z1)*(x2-x1);
		n[2]=(x2-x1)*(y3-y1)-(x3-x1)*(y2-y1);
	/*float a[3]={A[0]-B[0],A[1]-B[1],A[2]-B[2]};
	float b[3]={A[0]-C[0],A[1]-C[1],A[2]-C[2]};
	float n[3];//向量
	n[0]=a[1]*b[2]-a[2]*b[1];
	n[1]=a[2]*b[0]-a[0]*b[2];
	n[2]=a[0]*b[1]-a[1]*b[0];
	/*for(int j=0;j<3;j++)
	{
		Triangle->at(i).n[j]=n[j];
	}*/
	

	/***************求夹角value*****************/
	float k[3]={n[0],0,n[2]};
	double m=(n[0]*k[0]+n[1]*k[1]+n[2]*k[2])/(sqrt(n[0]*n[0]+n[1]*n[1]+n[2]*n[2])*sqrt(k[0]*k[0]+k[1]*k[1]+k[2]*k[2]));
	if(abs(m>0.999))
		m=1;
	double value=acos(m);//结果value，单位为弧度
	if(value>π/2)
		value=π-value;

	/***************求夹角α*****************/
	float O[3]={0,A[1],0};
	float O1[3]={0,A[1],A[2]+8};
	float res[4];
	float fac[4];
	float D[3];

	face(A,O,O1,res);
	face(A,B,C,fac);

	D[0]=A[0]+5;
	D[1]=(-res[3]/res[1]);
	D[2]=(-fac[3]-D[0]*fac[0]-D[1]*fac[1]);

	float vec[3]={D[0]-A[0],D[1]-A[1],D[2]-A[2]};
	m=(vec[2])/(sqrt(vec[0]*vec[0]+vec[1]*vec[1]+vec[2]*vec[2]));
	double α=acos(m);//结果α，单位为弧度
	if(α>π/2)
		α=π-α;
	//*****************************//

	R(value,α,n,A,de,i);//最终插值结果保存在de中
}
void CTextureProcessSystemDoc::face(float *a1,float *a2,float *a3,float *res)//平面
{
	float vec1[3];                    
	float vec2[3];
	float vec3[3];                    
	int i;
	for(i=0;i<3;i++)
	{
		vec1[i]=a2[i]-a1[i];
		vec2[i]=a3[i]-a2[i];
	}

	vec3[0]=vec1[1]*vec2[2]-vec1[2]*vec2[1];
	vec3[1]=vec1[2]*vec2[0]-vec1[0]*vec2[2];
	vec3[2]=vec1[0]*vec2[1]-vec1[1]*vec2[0];

	res[0]=vec3[0];
	res[1]=vec3[1];
	res[2]=vec3[2];
	res[3]=0-a1[0]*res[0]-a1[1]*res[1]-a1[2]*res[2];
}

/*****************求旋转矩阵R函数*********************/
void CTextureProcessSystemDoc::R(double a,double b,float *n,float *vec,float *res,int i)//分别输入value、α、法线、ABC点坐标 和 得新的旋转点数据A1B1C1
{
	CvMat* R1  = cvCreateMat( 4, 4, CV_32FC1 );
	CvMat* R2  = cvCreateMat( 4, 4, CV_32FC1 );
	CvMat* A   = cvCreateMat( 4, 1, CV_32FC1 );
	vector<gl_face> * Triangle=&(plyLoader.faceArry);
	vector<gl_point> * Vertex=&(plyLoader.pointArry);
	vector<gl_point2d> * Vertex2d=&(plyLoader.point2DArry);
	/***************矩阵R1*******************/
	Triangle->at(i).R1[0][0]=CV_MAT_ELEM( *R1,float,0,0)=cos(b)*cos(b)*cos(a)+sin(b)*sin(b);
	Triangle->at(i).R1[0][1]=CV_MAT_ELEM( *R1,float,0,1)=cos(b)*sin(a);
	Triangle->at(i).R1[0][2]=CV_MAT_ELEM( *R1,float,0,2)=cos(b)*cos(a)*sin(b)-sin(b)*cos(b);
	Triangle->at(i).R1[0][3]=CV_MAT_ELEM( *R1,float,0,3)=0;

	Triangle->at(i).R1[1][0]=CV_MAT_ELEM( *R1,float,1,0)=-sin(a)*cos(b);
	Triangle->at(i).R1[1][1]=CV_MAT_ELEM( *R1,float,1,1)=cos(a);
	Triangle->at(i).R1[1][2]=CV_MAT_ELEM( *R1,float,1,2)=-sin(a)*sin(b);
	Triangle->at(i).R1[1][3]=CV_MAT_ELEM( *R1,float,1,3)=0;

	Triangle->at(i).R1[2][0]=CV_MAT_ELEM( *R1,float,2,0)=sin(b)*cos(b)*(cos(a)-1);
	Triangle->at(i).R1[2][1]=CV_MAT_ELEM( *R1,float,2,1)=sin(b)*sin(a);
	Triangle->at(i).R1[2][2]=CV_MAT_ELEM( *R1,float,2,2)=sin(b)*sin(b)*cos(a)+cos(b)*cos(b);
	Triangle->at(i).R1[2][3]=CV_MAT_ELEM( *R1,float,2,3)=0;

	Triangle->at(i).R1[3][0]=CV_MAT_ELEM( *R1,float,3,0)=(vec[0]*(1-cos(b)*cos(b)*cos(a)-sin(b)*sin(b))) + (vec[1]*sin(a)*cos(b)) + (vec[2]*sin(b)*cos(b)*(1-cos(a)));
	Triangle->at(i).R1[3][1]=CV_MAT_ELEM( *R1,float,3,1)=(-vec[0]*cos(b)*sin(a)) + (vec[1]*(1-cos(a))) - (vec[2]*sin(b)*sin(a));
	Triangle->at(i).R1[3][2]=CV_MAT_ELEM( *R1,float,3,2)=(vec[0]*sin(b)*cos(b)*(1-cos(a))) + (vec[1]*sin(b)*sin(a)) + (vec[2]*sin(b)*sin(b)*(1-cos(a)));
	Triangle->at(i).R1[3][3]=CV_MAT_ELEM( *R1,float,3,3)=1;

	/***************求夹角β*****************/
	CV_MAT_ELEM( *A,float,0,0)=n[0];
	CV_MAT_ELEM( *A,float,1,0)=n[1];
	CV_MAT_ELEM( *A,float,2,0)=n[2];
	CV_MAT_ELEM( *A,float,3,0)=1;

	/*for(int i=0;i<4;i++)
		for(int j=0;j<4;j++)
			{
				printf("%f ",CV_MAT_ELEM( *R1,float,i,j));
	            if(j==3)
					printf("\n");
		    }
			printf("\n");
	for(int i=0;i<4;i++)
		for(int j=0;j<1;j++)
			{
				printf("%f ",CV_MAT_ELEM( *A,float,i,j));
	            if(j==0)
					printf("\n");
		    }*/


	cvMatMul(R1,A,A);//旋转矩阵*法向量=新法向量，赋值给A矩阵

	/*printf("\n");
	for(int i=0;i<4;i++)
		for(int j=0;j<1;j++)
			{
				printf("%f ",CV_MAT_ELEM( *A,float,i,j));
	            if(j==0)
					printf("\n");
		    }*/

	float m=(CV_MAT_ELEM( *A,float,2,0))/(sqrt(CV_MAT_ELEM( *A,float,0,0)*CV_MAT_ELEM( *A,float,0,0)+CV_MAT_ELEM( *A,float,1,0)*CV_MAT_ELEM( *A,float,1,0)+CV_MAT_ELEM( *A,float,2,0)*CV_MAT_ELEM( *A,float,2,0)));
	double β=acos(m);
	if(β>π/2)
		β=π-β;
	//β=-β;//取负
	/***************矩阵R2*******************/
	Triangle->at(i).R2[0][0]=CV_MAT_ELEM( *R2,float,0,0)=cos(β);
	Triangle->at(i).R2[0][1]=CV_MAT_ELEM( *R2,float,0,1)=0;
	Triangle->at(i).R2[0][2]=CV_MAT_ELEM( *R2,float,0,2)=sin(β);
	Triangle->at(i).R2[0][3]=CV_MAT_ELEM( *R2,float,0,3)=0;

	Triangle->at(i).R2[1][0]=CV_MAT_ELEM( *R2,float,1,0)=0;
	Triangle->at(i).R2[1][1]=CV_MAT_ELEM( *R2,float,1,1)=1;
	Triangle->at(i).R2[1][2]=CV_MAT_ELEM( *R2,float,1,2)=0;
	Triangle->at(i).R2[1][3]=CV_MAT_ELEM( *R2,float,1,3)=0;

	Triangle->at(i).R2[2][0]=CV_MAT_ELEM( *R2,float,2,0)=-sin(β);
	Triangle->at(i).R2[2][1]=CV_MAT_ELEM( *R2,float,2,1)=0;
	Triangle->at(i).R2[2][2]=CV_MAT_ELEM( *R2,float,2,2)=cos(β);
	Triangle->at(i).R2[2][3]=CV_MAT_ELEM( *R2,float,2,3)=0;

	Triangle->at(i).R2[3][0]=CV_MAT_ELEM( *R2,float,3,0)=vec[0]*(1-cos(β))+vec[2]*sin(β);
	Triangle->at(i).R2[3][1]=CV_MAT_ELEM( *R2,float,3,1)=0;
	Triangle->at(i).R2[3][2]=CV_MAT_ELEM( *R2,float,3,2)=-vec[0]*sin(β)+vec[2]*(1-cos(β));
	Triangle->at(i).R2[3][3]=CV_MAT_ELEM( *R2,float,3,3)=1;

	//***************计算质心*******************/
	int pt1num=Triangle->at(i).ptnum[0];
	int pt2num=Triangle->at(i).ptnum[1];
	int pt3num=Triangle->at(i).ptnum[2];
	/*
	Triangle->at(i).corex=(Vertex->at(pt1num).x+Vertex->at(pt2num).x+Vertex->at(pt3num).x)/3;
	Triangle->at(i).corey=(Vertex->at(pt1num).y+Vertex->at(pt2num).y+Vertex->at(pt3num).y)/3;
	Triangle->at(i).corez=(Vertex->at(pt1num).z+Vertex->at(pt2num).z+Vertex->at(pt3num).z)/3;
*/
	//
	CV_MAT_ELEM( *A,float,0,0)=Triangle->at(i).corex;
	CV_MAT_ELEM( *A,float,1,0)=Triangle->at(i).corey;
	CV_MAT_ELEM( *A,float,2,0)=Triangle->at(i).corez;
	CV_MAT_ELEM( *A,float,3,0)=1;

	cvMatMul(R1,A,A);
	cvMatMul(R2,A,A);

	Triangle->at(i).pcorex=res[0]=CV_MAT_ELEM( *A,float,0,0);
	Triangle->at(i).pcorey=res[1]=CV_MAT_ELEM( *A,float,1,0);
	res[2]=CV_MAT_ELEM( *A,float,2,0);
	//**********为了以后方便，计算第i个三角面片三个顶点对应的二维坐标***************//


	CV_MAT_ELEM( *A,float,0,0)=Vertex->at(pt1num).x; 
	CV_MAT_ELEM( *A,float,1,0)=Vertex->at(pt1num).y; 
	CV_MAT_ELEM( *A,float,2,0)=Vertex->at(pt1num).z; 
	CV_MAT_ELEM( *A,float,3,0)=1;

	cvMatMul(R1,A,A);
	cvMatMul(R2,A,A);

	gl_point2d  pt2d_1;
	pt2d_1.x = CV_MAT_ELEM(*A, float, 0, 0);
	pt2d_1.y = CV_MAT_ELEM(*A, float, 1, 0);
	pt2d_1.pointNum=Vertex2d->size();
	Triangle->at(i).ptnum_2d[0]=pt2d_1.pointNum;
	Vertex2d->push_back(pt2d_1);
	Triangle->at(i)._2dx[0]=CV_MAT_ELEM( *A,float,0,0);
	Triangle->at(i)._2dy[0]=CV_MAT_ELEM( *A,float,1,0);
	//if (Triangle->at(i)._2dx[0] > 9999 || Triangle->at(i)._2dx[0]<-9999 || Triangle->at(i)._2dy[0]>9999 || Triangle->at(i)._2dy[0] < -9999)
	//{
	//	return; 
	//}
	//A点对应的xy坐标
	CV_MAT_ELEM( *A,float,0,0)=Vertex->at(pt2num).x; 
	CV_MAT_ELEM( *A,float,1,0)=Vertex->at(pt2num).y; 
	CV_MAT_ELEM( *A,float,2,0)=Vertex->at(pt2num).z; 
	CV_MAT_ELEM( *A,float,3,0)=1;

	cvMatMul(R1,A,A);
	cvMatMul(R2,A,A);

	gl_point2d  pt2d_2;
	pt2d_2.x=CV_MAT_ELEM( *A,float,0,0);
	pt2d_2.y=CV_MAT_ELEM( *A,float,1,0);
	pt2d_2.pointNum=Vertex2d->size();
	Triangle->at(i).ptnum_2d[1]=pt2d_2.pointNum;
	Vertex2d->push_back(pt2d_2);
	Triangle->at(i)._2dx[1]=CV_MAT_ELEM( *A,float,0,0);
	Triangle->at(i)._2dy[1]=CV_MAT_ELEM( *A,float,1,0);
	if (Triangle->at(i)._2dx[1] > 9999 || Triangle->at(i)._2dx[1]<-9999 || Triangle->at(i)._2dy[1]>9999 || Triangle->at(i)._2dy[1] < -9999)
	{
		return;
	}
	//B点对应的xy坐标
	
	CV_MAT_ELEM( *A,float,0,0)=Vertex->at(pt3num).x; 
	CV_MAT_ELEM( *A,float,1,0)=Vertex->at(pt3num).y; 
	CV_MAT_ELEM( *A,float,2,0)=Vertex->at(pt3num).z; 
	CV_MAT_ELEM( *A,float,3,0)=1;

	cvMatMul(R1,A,A);
	cvMatMul(R2,A,A);

	gl_point2d pt2d_3;
	pt2d_3.x=CV_MAT_ELEM( *A,float,0,0);
	pt2d_3.y=CV_MAT_ELEM( *A,float,1,0);
	pt2d_3.pointNum=Vertex2d->size();
	Triangle->at(i).ptnum_2d[2]=pt2d_3.pointNum;
	Vertex2d->push_back(pt2d_3);
	Triangle->at(i)._2dx[2]=CV_MAT_ELEM( *A,float,0,0);
	Triangle->at(i)._2dy[2]=CV_MAT_ELEM( *A,float,1,0);
	
	if (Triangle->at(i)._2dx[2] > 9999 || Triangle->at(i)._2dx[2]<-9999 || Triangle->at(i)._2dy[2]>9999 || Triangle->at(i)._2dy[2] < -9999)
	{
		return;
	}
	//C点对应的xy坐标

	//显示当前2d坐标
	for(int k=0;k<3;k++)
	{
		Triangle->at(i)._show2dx[k]=Triangle->at(i)._2dx[k];
		Triangle->at(i)._show2dy[k]=Triangle->at(i)._2dy[k];
	}
}
void CTextureProcessSystemDoc::destination(int i,float *de)//求R1 R2的逆矩阵 返回原始坐标
{
	float l;
	float v1x,v1y,v1z,v2x,v2y,v2z,v3x,v3y,v3z;
	vector<gl_face> * Triangle=&(plyLoader.faceArry);

	v1x=plyLoader.pointArry[Triangle->at(i).ptnum[0]].x;
	v1y=plyLoader.pointArry[Triangle->at(i).ptnum[0]].y;
	v1z=plyLoader.pointArry[Triangle->at(i).ptnum[0]].z;

	v2x=plyLoader.pointArry[Triangle->at(i).ptnum[1]].x;
	v2y=plyLoader.pointArry[Triangle->at(i).ptnum[1]].y;
	v2z=plyLoader.pointArry[Triangle->at(i).ptnum[1]].z;

	v3x=plyLoader.pointArry[Triangle->at(i).ptnum[2]].x;
	v3y=plyLoader.pointArry[Triangle->at(i).ptnum[2]].y;
	v3z=plyLoader.pointArry[Triangle->at(i).ptnum[2]].z;


	l=(0.3)*sqrt((v1x-v2x)*(v1x-v2x)+(v1y-v2y)*(v1y-v2y)+(v1z-v2z)*(v1z-v2z));
	CvMat* R1  = cvCreateMat( 4, 4, CV_32FC1 );
	CvMat* R2  = cvCreateMat( 4, 4, CV_32FC1 );
	CvMat* A   = cvCreateMat( 4, 1, CV_32FC1 );

	CV_MAT_ELEM( *A,float,0,0)=de[0]+l*cos(Triangle->at(i).value);
	CV_MAT_ELEM( *A,float,1,0)=de[1]+l*sin(Triangle->at(i).value);
	CV_MAT_ELEM( *A,float,2,0)=de[2];
	CV_MAT_ELEM( *A,float,3,0)=1;

	for(int i1=0;i1<4;i1++)
		for(int j1=0;j1<4;j1++)
		{
			CV_MAT_ELEM( *R1,float,i1,j1)=Triangle->at(i).R1[i1][j1];
			CV_MAT_ELEM( *R2,float,i1,j1)=Triangle->at(i).R2[i1][j1];
		}

    cvInvert(R1,R1,CV_LU);//求R1和R2矩阵的逆，并且直接保存在R1 R2中
	cvInvert(R2,R2,CV_LU);

	cvMatMul(R2,A,A);
	cvMatMul(R1,A,A);

	Triangle->at(i).endx=CV_MAT_ELEM( *A,float,0,0);
	Triangle->at(i).endy=CV_MAT_ELEM( *A,float,1,0);
	Triangle->at(i).endz=CV_MAT_ELEM( *A,float,2,0);

}
void CTextureProcessSystemDoc::error(int i)//因为发现误差过大，所以本函数为修正误差函数，输入需要修正误差的三角面片编号i
{
	float a[3],b[3],c[3],fac1[4],fac2[4],fac3[4];
	float point[3];
	//float t;
	float v1x,v1y,v1z,v2x,v2y,v2z,v3x,v3y,v3z;
	vector<gl_face> *  Triangle=&(plyLoader.faceArry);

	v1x=plyLoader.pointArry[Triangle->at(i).ptnum[0]].x;
	v1y=plyLoader.pointArry[Triangle->at(i).ptnum[0]].y;
	v1z=plyLoader.pointArry[Triangle->at(i).ptnum[0]].z;

	v2x=plyLoader.pointArry[Triangle->at(i).ptnum[1]].x;
	v2y=plyLoader.pointArry[Triangle->at(i).ptnum[1]].y;
	v2z=plyLoader.pointArry[Triangle->at(i).ptnum[1]].z;

	v3x=plyLoader.pointArry[Triangle->at(i).ptnum[2]].x;
	v3y=plyLoader.pointArry[Triangle->at(i).ptnum[2]].y;
	v3z=plyLoader.pointArry[Triangle->at(i).ptnum[2]].z;

	a[0] = v1x;
	a[1] = v1y;
	a[2] = v1z;

	b[0] = v2x;
	b[1] = v2y;
	b[2] = v2z;

	c[0] = v3x;
	c[1] = v3y;
	c[2] = v3z;

	face(a,b,c,fac1);

	fac2[0]=fac1[1];
	fac2[1]=-fac1[0];
	fac2[2]=0;
	fac2[3]=(fac1[0]*Triangle->at(i).endy)-(fac1[1]*Triangle->at(i).endx);

	fac3[0]=fac1[2];
	fac3[1]=0;
	fac3[2]=-fac1[0];
	fac3[3]=(fac1[0]*Triangle->at(i).endz)-(fac1[2]*Triangle->at(i).endx);

	node(fac1,fac2,fac3,point);

	//t = -(fac[0]*Triangle->at(i).endx+fac[1]*Triangle->at(i).endy+fac[2]*Triangle->at(i).endz+fac[3])/(fac[0]*fac[0]+fac[1]*fac[1]+fac[2]*fac[2]);

	/*Triangle->at(i).endx = fac[0]*t+Triangle->at(i).endx;
	Triangle->at(i).endy = fac[1]*t+Triangle->at(i).endy;
	Triangle->at(i).endz = fac[2]*t+Triangle->at(i).endz;*/

	Triangle->at(i).endx = point[0];
	Triangle->at(i).endy = point[1];
	Triangle->at(i).endz = point[2];
}
void CTextureProcessSystemDoc::node(float * fac1,float *fac2,float *fac3,float *point)//输入三个平面，得到三个平面的交点
{
	float a;
	CvMat* D  = cvCreateMat( 3, 3, CV_32FC1 );
	CvMat* D1 = cvCreateMat( 3, 3, CV_32FC1 );
	CvMat* D2 = cvCreateMat( 3, 3, CV_32FC1 );
	CvMat* D3 = cvCreateMat( 3, 3, CV_32FC1 );

	CV_MAT_ELEM( *D3,float,0,0)=CV_MAT_ELEM( *D2,float,0,0)=CV_MAT_ELEM( *D,float,0,0)=fac1[0];//y
	CV_MAT_ELEM( *D3,float,1,0)=CV_MAT_ELEM( *D2,float,1,0)=CV_MAT_ELEM( *D,float,1,0)=fac2[0];
	CV_MAT_ELEM( *D3,float,2,0)=CV_MAT_ELEM( *D2,float,2,0)=CV_MAT_ELEM( *D,float,2,0)=fac3[0];

	CV_MAT_ELEM( *D3,float,0,1)=CV_MAT_ELEM( *D1,float,0,1)=CV_MAT_ELEM( *D,float,0,1)=fac1[1];//z
	CV_MAT_ELEM( *D3,float,1,1)=CV_MAT_ELEM( *D1,float,1,1)=CV_MAT_ELEM( *D,float,1,1)=fac2[1];
	CV_MAT_ELEM( *D3,float,2,1)=CV_MAT_ELEM( *D1,float,2,1)=CV_MAT_ELEM( *D,float,2,1)=fac3[1];

	CV_MAT_ELEM( *D2,float,0,2)=CV_MAT_ELEM( *D1,float,0,2)=CV_MAT_ELEM( *D,float,0,2)=fac1[2];
	CV_MAT_ELEM( *D2,float,1,2)=CV_MAT_ELEM( *D1,float,1,2)=CV_MAT_ELEM( *D,float,1,2)=fac2[2];
	CV_MAT_ELEM( *D2,float,2,2)=CV_MAT_ELEM( *D1,float,2,2)=CV_MAT_ELEM( *D,float,2,2)=fac3[2];

    CV_MAT_ELEM( *D3,float,0,2)=CV_MAT_ELEM( *D2,float,0,1)=CV_MAT_ELEM( *D1,float,0,0)=(-fac1[3]);//x
	CV_MAT_ELEM( *D3,float,1,2)=CV_MAT_ELEM( *D2,float,1,1)=CV_MAT_ELEM( *D1,float,1,0)=(-fac2[3]);
	CV_MAT_ELEM( *D3,float,2,2)=CV_MAT_ELEM( *D2,float,2,1)=CV_MAT_ELEM( *D1,float,2,0)=(-fac3[3]);

	a=cvDet(D);
	point[0]=cvDet(D1)/a;
	point[1]=cvDet(D2)/a;
	point[2]=cvDet(D3)/a;
	//printf("point为%3f %3f %3f\n",point[0],point[1],point[2]);
}

float CTextureProcessSystemDoc::rbf(float d,float e,int *size,float *c,float *coefficient,char rbffuntion)//RBF插值函数
{
	int i10,j10,k10,i9;
	float value,f;

	CvMat* R = cvCreateMat( 2, size[0], CV_32FC1 );
	CvMat* X = cvCreateMat( 1, size[0], CV_32FC1 );
	CvMat* nodes = cvCreateMat( 2, size[0], CV_32FC1 );

	/********************新增的R矩阵计算***********************************/
			
			for(i10=1;i10<=size[0];i10++)
					{
						CV_MAT_ELEM( *nodes,float,0,i10-1)=templet[i10].x;
						CV_MAT_ELEM( *nodes,float,1,i10-1)=templet[i10].y;
						CV_MAT_ELEM( *R,float,0,i10-1)=d;
						CV_MAT_ELEM( *R,float,1,i10-1)=e;
					}

			cvSub(R,nodes,R,NULL);//矩阵减法保存在R中
			for(j10=1;j10<=size[0];j10++)
					{ 
						CV_MAT_ELEM( *R,float,0,j10-1)= CV_MAT_ELEM( *R,float,0,j10-1)* CV_MAT_ELEM( *R,float,0,j10-1);
						CV_MAT_ELEM( *R,float,1,j10-1)= CV_MAT_ELEM( *R,float,1,j10-1)* CV_MAT_ELEM( *R,float,1,j10-1);
					}
			for(k10=1;k10<=size[0];k10++)
				CV_MAT_ELEM( *X,float,0,k10-1)=sqrt(CV_MAT_ELEM( *R,float,0,k10-1)+CV_MAT_ELEM( *R,float,1,k10-1));

		/**************************插值计算***********************************/
			value=c[0]+c[1]*d+c[2]*e;
				switch(rbffuntion)
		{
		case'l':{
				for(i9=1;i9<=size[0];i9++)
					{
					 f=rbfliner(CV_MAT_ELEM( *X,float,0,i9-1),coefficient[0]);
					 f=templet[i9].c*f;
					 value+=f;
					}
					  break;
				};//调用线性函数
		case'c':{
				for(i9=1;i9<=size[0];i9++)
					{
					 f=rbfclbic(CV_MAT_ELEM( *X,float,0,i9-1),coefficient[0]);
					 f=templet[i9].c*f;
					 value+=f;
					}
					  break;
				};//调用三次函数
		case'g':{
				for(i9=1;i9<=size[0];i9++)
					{
					 f=rbfgaussian(CV_MAT_ELEM( *X,float,0,i9-1),coefficient[0]);
					 f=templet[i9].c*f;
					 value+=f;
					}
					  break;
				};//调用高斯函数
		case'm':{
				for(i9=1;i9<=size[0];i9++)
					{
					 f=rbfmultiquadrics(CV_MAT_ELEM( *X,float,0,i9-1),coefficient[0]);
					 f=templet[i9].c*f;
					 value+=f;
					}
					  break;
				};//调用mul函数
		case't':{
				for(i9=1;i9<=size[0];i9++)
					{
					 f=rbfthinplate(CV_MAT_ELEM( *X,float,0,i9-1),coefficient[0]);
					 f=templet[i9].c*f;
					 value+=f;
					}
					  break;
				};//调用薄版样条函数
		default:printf("RBFfuntion输入有误\n");
		}
		return value;
}

int CTextureProcessSystemDoc::findAllPoint()
{
	int k=0;
	int j=0;
	vector<gl_face> * Triangle=&(plyLoader.faceArry);
	vector<int>  res;
	for( int i=0;i<Triangle->size();i++)
	{
		/*findFaceByTwoPoint(Triangle->at(i).pt1num,Triangle->at(i).pt2num,Triangle->at(i).facenum);
		findFaceByTwoPoint(Triangle->at(i).pt2num,Triangle->at(i).pt3num,Triangle->at(i).facenum);
		findFaceByTwoPoint(Triangle->at(i).pt3num,Triangle->at(i).pt1num,Triangle->at(i).facenum);*/
		res=findFaceByTwoPointNewWay(Triangle->at(i).ptnum[0],Triangle->at(i).ptnum[1],Triangle->at(i).facenum);
		while(res.size()>0)
		{
		CString s1;
		s1.Format("三角形%d 的第1个和第2个点，也被三角形%d 包含",i,res.back());
		res.pop_back();
		//AfxMessageBox(s1);
		}
		k++;
	}
	return k;

}
vector<int>  CTextureProcessSystemDoc::findFaceByTwoPointNewWay(int point1,int point2,int oldFaceIndex)
{
	vector<int > triList1,triList2,resVector;
	triList1=plyLoader.pointArry[point1].beLongToFaceIndex;//包含第一个点的三角形
	triList2=plyLoader.pointArry[point2].beLongToFaceIndex;
	//这个比较可以用sort排序后优化一下，暂时用两重循环
	for(int i=0;i<triList1.size();i++)
	{
		for(int j=0;j<triList2.size();j++)
		{
			if(triList1.at(i)==triList2.at(j))
			{
				if(triList1.at(i)!=oldFaceIndex)
				{
				resVector.push_back(triList1.at(i));
				}
			}
		}
	}

	return resVector;

}
/*void CTextureProcessSystemDoc::crosspoint1(float * a,float *b,float *c)
{
	c[0]=(b[1]-a[1])/(a[0]-b[0]);
	c[1]=a[0]*c[0]+a[1];
}

void CTextureProcessSystemDoc::crosspoint2(float a,float *b,float *c)
{
	c[0]=a;
	c[1]=b[0]*a+b[1];
}
void CTextureProcessSystemDoc::count_TexcoordByHuang(int a, int b, int c, int d, int e)
{
	//保证输入有效性
	if (a < 0 || b < 0 || c < 0 || d < 0 || e < 0)
	{
		return;
	}
	if (d == 3684)
	{
		int qwet = 0;
	}
	if (a == 3684)
	{
		int qwet = 0;
	}
	//a 是未知三角形，d是已知三角形  的索引号
	//编号第a三角形第c个顶点和第d个三角形第e个顶点重合 
	//要求纹理坐标的点在三角形a中是第b个点
	gl_point2d a_pt3d[3];
	gl_point2d a_pt2d[3];

	gl_point2d d_pt3d[3];
	gl_point2d d_pt2d[3];

	vector<gl_face> * Triangle=&(plyLoader.faceArry);
	vector<gl_point> * Vertex=&(plyLoader.pointArry);
	vector<gl_point2d> * Vertex2d=&(plyLoader.point2DArry);




	for(int i=0;i<3;i++)
	{
		a_pt3d[i].x=Vertex->at(Triangle->at(a).ptnum[i]).x;
		a_pt3d[i].y=Vertex->at(Triangle->at(a).ptnum[i]).y;
		a_pt2d[i].x=Vertex2d->at(Triangle->at(a).ptnum_2d[i]).x;
		a_pt2d[i].y=Vertex2d->at(Triangle->at(a).ptnum_2d[i]).y;

		d_pt3d[i].x=Vertex->at(Triangle->at(d).ptnum[i]).x;
		d_pt3d[i].y=Vertex->at(Triangle->at(d).ptnum[i]).y;
		d_pt2d[i].x=Vertex2d->at(Triangle->at(d).ptnum_2d[i]).x;
		d_pt2d[i].y=Vertex2d->at(Triangle->at(d).ptnum_2d[i]).y;
	}

	float a_pcorex = Triangle->at(a).pcorex + offset[0];
	float a_pcorey = Triangle->at(a).pcorey + offset[1];
	float a_h=Triangle->at(a).h;
	float a_xita=Triangle->at(a).value;
	//此处固定d=0;
	d=centerIndex;
	float d_pcorex = Triangle->at(d).pcorex + offset[0];
	float d_pcorey = Triangle->at(d).pcorey + offset[1];
	float d_h=Triangle->at(d).h;
	float d_xita=Triangle->at(d).value;

	//d_xita=1.1877480;
	//d_h=0.011479380;

	/*float s_i_n, c_o_s;
	s_i_n = sin(d_xita);
	c_o_s = cos(d_xita);

	if (sin(d_xita) == 0 || cos(d_xita) == 0)
	{
		s_i_n = 1;
		c_o_s = 1;
	}

	float o2x = (-d_h*λ / 2)*c_o_s - (-d_h*λ / 2)*s_i_n + d_pcorex;
	
	float o2y = -(d_h*λ / 2)*s_i_n + (-d_h*λ / 2)*c_o_s + d_pcorey;


	float A = ((a_pt2d[b].x - o2x) / (d_h*λ))*c_o_s + ((a_pt2d[b].y - o2y) / (d_h*λ))*s_i_n;
	float B = (-(a_pt2d[b].x - o2x) / (d_h*λ))*s_i_n + ((a_pt2d[b].y - o2y) / (d_h*λ))*c_o_s;*/
	/*if (A > 9999 || A <-9999 || B > 9999 || B <-9999)
	{
		return;
	}

	
	float o2x = (-d_h*λ / 2)*cos(d_xita) - (-d_h*λ / 2)*sin(d_xita) + d_pcorex;

	float o2y = -(d_h*λ / 2)*sin(d_xita) + (-d_h*λ / 2)*cos(d_xita) + d_pcorey;


	float A = ((a_pt2d[b].x - o2x) / (d_h*λ))*cos(d_xita) + ((a_pt2d[b].y - o2y) / (d_h*λ))*sin(d_xita);
	float B = (-(a_pt2d[b].x - o2x) / (d_h*λ))*sin(d_xita) + ((a_pt2d[b].y - o2y) / (d_h*λ))*cos(d_xita);

	Triangle->at(a).texCoord.cor[b][0] = A;
	Triangle->at(a).texCoord.cor[b][1] = B;
	//Triangle->at(a)._2du[b]=A;
	//Triangle->at(a)._2dv[b]=B;

	//至此纹理已经算出来了
	/*if(!Triangle->at(a).textureclick)
	{
		//Triangle->at(a).textureclick=true;
		count++;
		Triangle->at(a).renderTime=times;
	}
	

}
void CTextureProcessSystemDoc::alterFace2dCoord(int a,int d,int b,int c1,int c2,int e1,int e2)
{
	//int i;
	//if(a==2387)
	//{
	//	i=i+1;
	//}
	PlaneRotate * pr1=new PlaneRotate();
	gl_point a_pt3d[3];
	gl_point2d a_pt2d[3];

	gl_point d_pt3d[3];
	gl_point2d d_pt2d[3];

	vector<gl_face> * Triangle=&(plyLoader.faceArry);
	vector<gl_point> * Vertex=&(plyLoader.pointArry);
	vector<gl_point2d> * Vertex2d=&(plyLoader.point2DArry);

	for(int i=0;i<3;i++)
	{
		a_pt2d[i].x=Vertex2d->at(Triangle->at(a).ptnum_2d[i]).x;
		a_pt2d[i].y=Vertex2d->at(Triangle->at(a).ptnum_2d[i]).y;
		a_pt3d[i].x=Vertex->at(Triangle->at(a).ptnum[i]).x;
		a_pt3d[i].y=Vertex->at(Triangle->at(a).ptnum[i]).y;
		a_pt3d[i].z=Vertex->at(Triangle->at(a).ptnum[i]).z;


		d_pt2d[i].x=Vertex2d->at(Triangle->at(d).ptnum_2d[i]).x;
		d_pt2d[i].y=Vertex2d->at(Triangle->at(d).ptnum_2d[i]).y;
		d_pt3d[i].x=Vertex->at(Triangle->at(d).ptnum[i]).x;
		d_pt3d[i].y=Vertex->at(Triangle->at(d).ptnum[i]).y;
		d_pt3d[i].z=Vertex->at(Triangle->at(d).ptnum[i]).z;
	}

	float A[2],B[2],C[2];//第0号三角形面片数据


	
	A[0]=d_pt2d[e1].x;
	A[1]=d_pt2d[e1].y;//0号顶点 e1

	B[0]=d_pt2d[e2].x;
	B[1]=d_pt2d[e2].y;//1号顶点 e2

	C[0]=d_pt2d[3-e1-e2].x;
	C[1]=d_pt2d[3-e1-e2].y;//2号顶点

	float D[2],E[2],F[2];//第2677号三角形面片数据
	D[0]=a_pt2d[c1].x;
	D[1]=a_pt2d[c1].y;//1号顶点 c2

	E[0]=a_pt2d[c2].x;
	E[1]=a_pt2d[c2].y;//0号顶点 c1

	F[0]=a_pt2d[b].x;
	F[1]=a_pt2d[b].y;//386号顶点 b

	double G[2];//所求第2677号三角形第2号顶点新数据放在此数组中
	float dd[3],ee[3],ff[3];
	dd[0]=a_pt3d[c1].x;
	dd[1]=a_pt3d[c1].y;
	dd[2]=a_pt3d[c1].z;
	ee[0]=a_pt3d[c2].x;
	ee[1]=a_pt3d[c2].y;
	ee[2]=a_pt3d[c2].z;
	ff[0]=a_pt3d[b].x;
	ff[1]=a_pt3d[b].y;
	ff[2]=a_pt3d[b].z;



	/**********以上是面片顶点数据载入***********

	double kAB;
	double kDE,kDF,kEF;//斜率k

	double k1,k2;
	double iAB,iDE,iDF,iEF;
	double x,y;

	double l2 = pow(double(pow(D[0]-E[0],2)+pow(D[1]-E[1],2)),0.5); 
	x=A[0]-B[0];
	y=A[1]-B[1];
	kAB=x/(pow(pow(x,2)+pow(y,2),0.5));
	iAB = acos(kAB);



    if(y<0)
	{
		iAB=2*PI-iAB;
		//B[1] = A[1]+l2*sin(iAB);
	}
	//else
		//B[1] = A[1]+l2*sin(iAB);

	if(iAB>PI)
		iAB=iAB-PI;
	//B[0] = A[0]+l2*cos(iAB);

    //kAB=(A[1]-B[1])/(A[0]-B[0]);//

	x=D[0]-E[0];
	y=D[1]-E[1];
	kDE=x/(pow(pow(x,2)+pow(y,2),0.5));
	iDE = acos(kDE);
	if(y<0)
		iDE=2*PI-iDE;
	if(iDE>PI)
		iDE=iDE-PI;
	//kDE=(D[1]-E[1])/(D[0]-E[0]);//
    //iDE=atan(kDE);

	x=F[0]-D[0];
	y=F[1]-D[1];
	kDF=x/(pow(pow(x,2)+pow(y,2),0.5));
	iDF = acos(kDF);
	if(y<0)
		iDF=2*PI-iDF;
	if(iDF>PI)
		iDF=iDF-PI;
	//kDF=(D[1]-F[1])/(D[0]-F[0]);//
	//iDF = atan(kDF);

	x=F[0]-E[0];
	y=F[1]-E[1];
	kEF=x/(pow(pow(x,2)+pow(y,2),0.5));
	iEF = acos(kEF);
	if(y<0)
		iEF=2*PI-iEF;
	if(iEF>PI)
		iEF=iEF-PI;
	//kEF=(F[1]-E[1])/(F[0]-E[0]);//F点为需要求的坐标点
	//iEF = atan(kEF);


	k1=iEF+(iAB-iDE);//E、B重叠，新角度
	k2=iDF+(iAB-iDE);//D、A重叠，新角度
	
	G[0]=((tan(k1)*B[0]-B[1])-(tan(k2)*A[0]-A[1]))/(tan(k1)-tan(k2));//新坐标x
	//G[1]=k2*G[0]-(k2*A[0]-A[1]);//新坐标y
	G[1]=(tan(k2)*(tan(k1)*B[0]-B[1])-tan(k1)*(tan(k2)*A[0]-A[1]))/(tan(k1)-tan(k2));


	//pr1->init(A[0],A[1],B[0],B[1],C[0],C[1],D[0],D[1],E[0],E[1],F[0],F[1]);
	//pr1->init(A[0],A[1],B[0],B[1],C[0],C[1],dd[0],dd[1],dd[2],ee[0],ee[1],ee[2],ff[0],ff[1],ff[2]);
	//pr1->getRes();
	//G[0]=pr1->resx;
	//G[1]=pr1->resy;
	

	Vertex2d->at(Triangle->at(a).ptnum_2d[b]).x=G[0];
	Vertex2d->at(Triangle->at(a).ptnum_2d[b]).y=G[1];

	Vertex2d->at(Triangle->at(a).ptnum_2d[c1]).x=d_pt2d[e1].x;
	Vertex2d->at(Triangle->at(a).ptnum_2d[c1]).y=d_pt2d[e1].y;

	Vertex2d->at(Triangle->at(a).ptnum_2d[c2]).x=d_pt2d[e2].x;
	Vertex2d->at(Triangle->at(a).ptnum_2d[c2]).y=d_pt2d[e2].y;

	Triangle->at(a)._show2dx[b]=G[0];
	Triangle->at(a)._show2dy[b]=G[1];
	Triangle->at(a)._show2dx[c1]=d_pt2d[e1].x;
	Triangle->at(a)._show2dy[c1]=d_pt2d[e1].y;
	Triangle->at(a)._show2dx[c2]=d_pt2d[e2].x;
	Triangle->at(a)._show2dy[c2]=d_pt2d[e2].y;

	/*a_pt2d[b].x=G[0];
	a_pt2d[b].y=G[1];

	a_pt2d[c1].x=d_pt2d[e1].x;
	a_pt2d[c1].y=d_pt2d[e1].y;

	a_pt2d[c2].x=d_pt2d[e2].x;
	a_pt2d[c2].y=d_pt2d[e2].y;
}

void CTextureProcessSystemDoc::findDiffrentPoint(int a,int d,int &b,int &c1,int &c2,int e1,int e2)
{
	if (a == 3684)
	{
		int poi = 0;
	}
	//保证输入有效性
	if(a<0||d<0)
	{
		return ;
	}
	vector<gl_face> * Triangle=&(plyLoader.faceArry);
	//a和d是输入的三角形,a未知d已知
	//未知三角形a中，未知的点是b
	//a中点c1和d中的点e1重合
	//a中点c2和d中的点e2重合
	//pt1Index,int pt2Index 是传入两点的索引号
	/*if(Triangle->at(a).is2DCordFixed)
	{
		return;
	}

	
	for(int i=0;i<3;i++)
	{
		int k=Triangle->at(a).ptnum[i];
		k;
		int m=Triangle->at(d).ptnum[e1];
		m;
		if(Triangle->at(a).ptnum[i]==Triangle->at(d).ptnum[e1])
		c1=i;
		
	}
	for(int j=0;j<3;j++)
	{
		int w=Triangle->at(a).ptnum[j];
		w;
		int q=Triangle->at(d).ptnum[e2];
		q;

		if(Triangle->at(a).ptnum[j]==Triangle->at(d).ptnum[e2])
		c2=j;
	}
	//3个顶点号b c1 c2分别为0 1 2，c1 c2已求得，易得
	b=3-c1-c2;

	//把重合顶点纹理坐标赋值
	//Triangle->at(a)._2du[c1]=Triangle->at(d)._2du[e1];
	//Triangle->at(a)._2dv[c1]=Triangle->at(d)._2dv[e1];
	//Triangle->at(a)._2du[c2]=Triangle->at(d)._2du[e2];
	//Triangle->at(a)._2dv[c2]=Triangle->at(d)._2dv[e2];
	Triangle->at(a).texCoord.cor[c1][0] = Triangle->at(d).texCoord.cor[e1][0];
	Triangle->at(a).texCoord.cor[c1][1] = Triangle->at(d).texCoord.cor[e1][1];
	Triangle->at(a).texCoord.cor[c2][0] = Triangle->at(d).texCoord.cor[e2][0];
	Triangle->at(a).texCoord.cor[c2][1] = Triangle->at(d).texCoord.cor[e2][1];
	//把映射在2D上的三角形进行选择 修改未知点的2D坐标
	alterFace2dCoord(a,d,b,c1,c2,e1,e2);
	Triangle->at(a).is2DCordFixed=true;

}
int  CTextureProcessSystemDoc::count_Texcoord(int d,int e1,int e2)
{
	int count=0;//计算纹理成功的三角形个数
	//保证输入有效性
	if(d<0||e1<0||e2<0)
	{
		return count;
	}
	//已知d是三角形编号，e1,e2是传入2点在三角形d中的编号e1,e1∈（0,1,2）,
	//pt1Index pt2Index 是传入两点的索引号
	vector<gl_face> * Triangle=&(plyLoader.faceArry);
	vector<int>  res;
	int a=-1;
	int c1=-1;
	int c2=-1;
	int b=-1;
	//传入2个点的索引号和当前三角形索引号d，得到目标三角形面a
	res=findFaceByTwoPointNewWay(Triangle->at(d).ptnum[e1],Triangle->at(d).ptnum[e2],d);
	while(res.size()>0)
	{
		a=res.back();
	/*	if(d==2677)
		{
			cout<<"2677->"<<a<<endl;
		}*/
		/*if(!Triangle->at(a).textureclick)
		{
			count++;
			findDiffrentPoint(a,d,b,c1,c2,e1,e2);//输入a d e1 e2得到 b  c1 c2 
			count_TexcoordByHuang(a,b,c1,d,e1);//计算纹理
			Triangle->at(a).texCoord.texCoorGenTime = texGenTime;
			Triangle->at(a).texCoord.update = false;
			Triangle->at(a).updateTexCoord();

		//}
		res.pop_back();
	}
	return count;
}

int CTextureProcessSystemDoc::count_3FaceTexcoord(int d)
{
	int k=0;//对k个面新加了纹理
	//做计算的三角形面片必须已经纹理
	vector<gl_face> * Triangle=&(plyLoader.faceArry);
	/*if(!Triangle->at(d).textureclick)
		return 0;
	//传进来三角形面片的信息，计算出3个邻接三角形的纹理坐标
	bool res1=false,res2=false,res3=false;
	k=count_Texcoord(d,0,1)+count_Texcoord(d,1,2)+count_Texcoord(d,2,0);
	
	return  k;
}


void CTextureProcessSystemDoc::count_h(int i)
{
	float a[2],b[2],c[2];
	float m,l1,l2,l3;
	float h,aa,bb;
	bool button=true;
	vector<gl_face> * Triangle=&(plyLoader.faceArry);
	vector<gl_point> * Vertex=&(plyLoader.pointArry);
	vector<gl_point2d> * Vertex2d=&(plyLoader.point2DArry);

	/*if(Triangle->at(i).isProcessedTexCor)
		return;
	if(Triangle->at(i).textureclick)
		return;


	if((Triangle->at(i).value==0)&&(Vertex2d->at(Triangle->at(i).ptnum_2d[0]).x==Vertex2d->at(Triangle->at(i).ptnum_2d[1]).x))
	{
		printf("无交点");
		button=false;
	}
	else if((Vertex2d->at(Triangle->at(i).ptnum_2d[0]).y-Vertex2d->at(Triangle->at(i).ptnum_2d[1]).y)/(Vertex2d->at(Triangle->at(i).ptnum_2d[0]).x-Vertex2d->at(Triangle->at(i).ptnum_2d[1]).x)==(Triangle->at(i).value+π/2))
	{
		printf("无交点");
		button=false;
	}
	else if(Triangle->at(i).value==0)
	{
		m=Triangle->at(i).pcorex;

		b[0]=(Vertex2d->at(Triangle->at(i).ptnum_2d[0]).y-Vertex2d->at(Triangle->at(i).ptnum_2d[1]).y)/(Vertex2d->at(Triangle->at(i).ptnum_2d[0]).x-Vertex2d->at(Triangle->at(i).ptnum_2d[1]).x);
		b[1]=Vertex2d->at(Triangle->at(i).ptnum_2d[0]).y-b[0]*Vertex2d->at(Triangle->at(i).ptnum_2d[0]).x;

		crosspoint2(m,b,c);
	}
	else if(Vertex2d->at(Triangle->at(i).ptnum_2d[0]).x-Vertex2d->at(Triangle->at(i).ptnum_2d[1]).x==0)
	{
		
		m=Vertex2d->at(Triangle->at(i).ptnum_2d[0]).x;

		a[0]=Triangle->at(i).value+π/2;
		a[1]=Triangle->at(i).pcorey-a[0]*Triangle->at(i).pcorex;

		crosspoint2(m,a,c);
	}
	else
	{
		a[0]=Triangle->at(i).value+π/2;
		a[1]=Triangle->at(i).pcorey-a[0]*Triangle->at(i).pcorex;

		b[0]=(Vertex2d->at(Triangle->at(i).ptnum_2d[0]).y-Vertex2d->at(Triangle->at(i).ptnum_2d[1]).y)/(Vertex2d->at(Triangle->at(i).ptnum_2d[0]).x-Vertex2d->at(Triangle->at(i).ptnum_2d[1]).x);
		b[1]=Vertex2d->at(Triangle->at(i).ptnum_2d[0]).y-b[0]*Vertex2d->at(Triangle->at(i).ptnum_2d[0]).x;

		crosspoint1(a,b,c);
	}
	if(button==true)
	{
		aa=(Triangle->at(i).pcorex-c[0]);
		bb=(Triangle->at(i).pcorey-c[1]);
		l1=sqrt(pow(aa,2)+pow(bb,2));
	}
	else
	{
		l1=0;
	    button=true;
	}

	if((Triangle->at(i).value==0)&&(Vertex2d->at(Triangle->at(i).ptnum_2d[0]).x==Vertex2d->at(Triangle->at(i).ptnum_2d[2]).x))
	{
		printf("无交点");
		button=false;
	}
	else if((Vertex2d->at(Triangle->at(i).ptnum_2d[0]).y-Vertex2d->at(Triangle->at(i).ptnum_2d[2]).y)/(Vertex2d->at(Triangle->at(i).ptnum_2d[0]).x-Vertex2d->at(Triangle->at(i).ptnum_2d[2]).x)==(Triangle->at(i).value+π/2))
	{
		printf("无交点");
		button=false;
	}
	else if(Triangle->at(i).value==0)
	{
		m=Triangle->at(i).pcorex;

		b[0]=(Vertex2d->at(Triangle->at(i).ptnum_2d[0]).y-Vertex2d->at(Triangle->at(i).ptnum_2d[2]).y)/(Vertex2d->at(Triangle->at(i).ptnum_2d[0]).x-Vertex2d->at(Triangle->at(i).ptnum_2d[2]).x);
		b[1]=Vertex2d->at(Triangle->at(i).ptnum_2d[0]).y-b[0]*Vertex2d->at(Triangle->at(i).ptnum_2d[0]).x;

		crosspoint2(m,b,c);
	}
	else if(Vertex2d->at(Triangle->at(i).ptnum_2d[0]).x-Vertex2d->at(Triangle->at(i).ptnum_2d[2]).x==0)
	{
		
		m=Vertex2d->at(Triangle->at(i).ptnum_2d[0]).x;

		a[0]=Triangle->at(i).value+π/2;
		a[1]=Triangle->at(i).pcorey-a[0]*Triangle->at(i).pcorex;

		crosspoint2(m,a,c);
	}
	else
	{
		a[0]=Triangle->at(i).value+π/2;
		a[1]=Triangle->at(i).pcorey-a[0]*Triangle->at(i).pcorex;

		b[0]=(Vertex2d->at(Triangle->at(i).ptnum_2d[0]).y-Vertex2d->at(Triangle->at(i).ptnum_2d[2]).y)/(Vertex2d->at(Triangle->at(i).ptnum_2d[0]).x-Vertex2d->at(Triangle->at(i).ptnum_2d[2]).x);
		b[1]=Vertex2d->at(Triangle->at(i).ptnum_2d[0]).y-b[0]*Vertex2d->at(Triangle->at(i).ptnum_2d[0]).x;

		crosspoint1(a,b,c);
	}
	if(button==true)
	{
		aa=(Triangle->at(i).pcorex-c[0]);
		bb=(Triangle->at(i).pcorey-c[1]);
		l2=sqrt(pow(aa,2)+pow(bb,2));
	}
	else
	{
		l2=0;
	    button=true;
	}

	if((Triangle->at(i).value==0)&&(Vertex2d->at(Triangle->at(i).ptnum_2d[2]).x==Vertex2d->at(Triangle->at(i).ptnum_2d[1]).x))
	{
		printf("无交点");
		button=false;
	}
	else if((Vertex2d->at(Triangle->at(i).ptnum_2d[2]).y-Vertex2d->at(Triangle->at(i).ptnum_2d[0]).y)/(Vertex2d->at(Triangle->at(i).ptnum_2d[2]).x-Vertex2d->at(Triangle->at(i).ptnum_2d[1]).x)==(Triangle->at(i).value+π/2))
	{
		printf("无交点");
		button=false;
	}
	else if(Triangle->at(i).value==0)
	{
		m=Triangle->at(i).pcorex;

		b[0]=(Vertex2d->at(Triangle->at(i).ptnum_2d[2]).y-Vertex2d->at(Triangle->at(i).ptnum_2d[1]).y)/(Vertex2d->at(Triangle->at(i).ptnum_2d[2]).x-Vertex2d->at(Triangle->at(i).ptnum_2d[1]).x);
		b[1]=Vertex2d->at(Triangle->at(i).ptnum_2d[2]).y-b[0]*Vertex2d->at(Triangle->at(i).ptnum_2d[2]).x;

		crosspoint2(m,b,c);
	}
	else if(Vertex2d->at(Triangle->at(i).ptnum_2d[2]).x-Vertex2d->at(Triangle->at(i).ptnum_2d[1]).x==0)
	{
		
		m=Vertex2d->at(Triangle->at(i).ptnum_2d[2]).x;

		a[0]=Triangle->at(i).value+π/2;
		a[1]=Triangle->at(i).pcorey-a[0]*Triangle->at(i).pcorex;

		crosspoint2(m,a,c);
	}
	else
	{
		a[0]=Triangle->at(i).value+π/2;
		a[1]=Triangle->at(i).pcorey-a[0]*Triangle->at(i).pcorex;

		b[0]=(Vertex2d->at(Triangle->at(i).ptnum_2d[2]).y-Vertex2d->at(Triangle->at(i).ptnum_2d[1]).y)/(Vertex2d->at(Triangle->at(i).ptnum_2d[2]).x-Vertex2d->at(Triangle->at(i).ptnum_2d[1]).x);
		b[1]=Vertex2d->at(Triangle->at(i).ptnum_2d[2]).y-b[0]*Vertex2d->at(Triangle->at(i).ptnum_2d[2]).x;

		crosspoint1(a,b,c);
	}
	if(button==true)
	{
		aa=(Triangle->at(i).pcorex-c[0]);
		bb=(Triangle->at(i).pcorey-c[1]);
		l3=sqrt(pow(aa,2)+pow(bb,2));
	}
	else
	{
		l3=0;
	    button=true;
	}

	if((l1==0)||(l2==0)||(l3==0))
		h=l1+l2+l3;
	else if((l1==l2)||(l2==l3))
		h=l1+l3;
	else if(l1==l3)
		h=l2+l3;
	else
	{
		float max;
		max=l1;
		if(l2>max)
			max=l2;
		if(l3>max)
			max=l3;
		h=l1+l2+l3-max;
	}


	/*if (!Triangle->at(i).textureclick||true)
	{
		//Triangle->at(i).h = h;
		Triangle->at(i).h = 0.01;
		//Triangle->at(i).textureclick = true;
		Triangle->at(i).is2DCordFixed = true;
		count++;
		//计算纹理

		centerIndex = i;
		count_TexcoordByHuang(i, 0, 0, i, 0);
		count_TexcoordByHuang(i, 1, 0, i, 0);
		count_TexcoordByHuang(i, 2, 0, i, 0);
		Triangle->at(i).renderTime = times;
		addIndextoProcesseTriangleIndex(i, i);
		//添加center面片的纹理坐标
		Triangle->at(i).texCoord.update = false;
		Triangle->at(i).texCoord.texCoorGenTime = texGenTime;
		Triangle->at(i).updateTexCoord();
	//}
	//return h;
 }

int CTextureProcessSystemDoc::findFaceByTwoPoint(int point1,int point2,int oldFaceIndex)
{
	vector<gl_face> * Triangle=&(plyLoader.faceArry);
	for( int i=0;i<Triangle->size();i++)
	{
		//第一个点匹配了Point1
		if(Triangle->at(i).ptnum[0]==point1)
		{
			if(Triangle->at(i).ptnum[1]==point2)
			{
				//找到
				if(Triangle->at(i).facenum!=oldFaceIndex) return Triangle->at(i).facenum;

			}else if(Triangle->at(i).ptnum[2]==point2)
			{
				//找到
				if(Triangle->at(i).facenum!=oldFaceIndex) return Triangle->at(i).facenum;
			}
			
		}
		//第一个点匹配了Point2
		if(Triangle->at(i).ptnum[0]==point2)
		{
			if(Triangle->at(i).ptnum[1]==point1)
			{
				//找到
				if(Triangle->at(i).facenum!=oldFaceIndex) return Triangle->at(i).facenum;

			}else if(Triangle->at(i).ptnum[2]==point1)
			{
				//找到
				if(Triangle->at(i).facenum!=oldFaceIndex) return Triangle->at(i).facenum;
			}
			
		}
		//第一个点没匹配，第二个点匹配了Point1
		if(Triangle->at(i).ptnum[1]==point1)
		{
			if(Triangle->at(i).ptnum[2]==point2)
			{
				//找到
				if(Triangle->at(i).facenum!=oldFaceIndex) return Triangle->at(i).facenum;
			}
		}
		//第一个点没匹配，第二个点匹配了Point2
		if(Triangle->at(i).ptnum[1]==point2)
		{
			if(Triangle->at(i).ptnum[2]==point1)
			{
				//找到
				if(Triangle->at(i).facenum!=oldFaceIndex) return Triangle->at(i).facenum;
			}
		}
		
	}
	return -1;
}

void CTextureProcessSystemDoc::reset2dCoord()
{
	vector<gl_face> * Triangle = &(plyLoader.faceArry);
	vector<gl_point> * Vertex = &(plyLoader.pointArry);
	vector<gl_point2d> * Vertex2d = &(plyLoader.point2DArry);
	int count = 0;

	for (int i = 0; i < Triangle->size(); i++)
	{
		//重置纹理坐标
		Triangle->at(i).texCoord.resetValue();
		for (int j = 0; j < 3; j++)
		{
			Vertex2d->at(Triangle->at(i).ptnum_2d[j]).x = Triangle->at(i)._2dx[j];
			Vertex2d->at(Triangle->at(i).ptnum_2d[j]).y = Triangle->at(i)._2dy[j];

			Triangle->at(i).texCoord.cor[j][0] = Triangle->at(i)._2dx[j];
			Triangle->at(i).texCoord.cor[j][1] = Triangle->at(i)._2dy[j];
		}


	
		/*	Triangle->at(i).isProcessedTexCor=false;*/
		/*if(Triangle->at(i).isProcessedTexCor||Triangle->at(i).textureclick)
		{
		count = 0;
		for (int j = 0; j<3; j++)
		{
			auto vx = Triangle->at(i).texCoord.cor[j][0];
			auto vy = Triangle->at(i).texCoord.cor[j][1];
			if (vx>1 || vx<0 || vy>1 || vy<0)
			{
				count++;
			}
		}

		if (count>2)
		{
			//超过判断条件 不贴图 下次重新贴这些三角形
			Triangle->at(i).textureclick = false;
			Triangle->at(i).is2DCordFixed = false;
			for (int j = 0; j < 3; j++)
			{
				Triangle->at(i).texCoord.cor[j][0] = -1;
				Triangle->at(i).texCoord.cor[j][1] = -1;
			}
		}
	}
}
void CTextureProcessSystemDoc::resetOuterTriangleTex()
{
	return;
	vector<gl_face> * Triangle=&(plyLoader.faceArry);
	vector<gl_point> * Vertex=&(plyLoader.pointArry);
	vector<gl_point2d> * Vertex2d=&(plyLoader.point2DArry);
	int count=0;
	for(int i=0;i<Triangle->size();i++)
	{
		for(int j=0;j<3;j++)
		{					
			Vertex2d->at(Triangle->at(i).ptnum_2d[j]).x=Triangle->at(i)._2dx[j];
			Vertex2d->at(Triangle->at(i).ptnum_2d[j]).y=Triangle->at(i)._2dy[j];
		}
	/*	Triangle->at(i).isProcessedTexCor=false;
		/*if(Triangle->at(i).isProcessedTexCor||Triangle->at(i).textureclick)
		{
			count=0;
			for(int j=0;j<3;j++)
			{
				auto vx=Triangle->at(i).texCoord.cor[j][0];
				auto vy=Triangle->at(i).texCoord.cor[j][1];
				if(vx>1||vx<0||vy>1||vy<0)
				{
					count++;
				}
			}
			
			if(count>2)
			{
				//超过判断条件 不贴图 下次重新贴这些三角形
				Triangle->at(i).textureclick=false;
				Triangle->at(i).is2DCordFixed=false;
				for(int j=0;j<3;j++)
				{
					Triangle->at(i).texCoord.cor[j][0] = -1;
					Triangle->at(i).texCoord.cor[j][1] = -1;
				}
			}

		//}
		/*else
		{
			Triangle->at(i).textureclick=false;
		}
		Triangle->at(i).isProcessedTexCor=false;
		
	}
}
// CTextureProcessSystemDoc 命令
void CTextureProcessSystemDoc::calTexCor(){
	vector<gl_face> * Triangle=&(plyLoader.faceArry);
	int a,b,c1,c2,d,e1,e2;
	if(toProcesseTriangleIndex.size()<=0)
	{
		return;
	}else
	{
		int index=toProcesseTriangleIndex.at(0);
		d=index;
		Triangle->at(index).r=(rand()%100)/100.0;
		Triangle->at(index).g=(rand()%100)/100.0;
		Triangle->at(index).b=(rand()%100)/100.0;
		int index1=findFaceIndex(index,0,1);
		int index2=findFaceIndex(index,1,2);
		int index3=findFaceIndex(index,2,0);
		if(addIndextoProcesseTriangleIndex(index,index1))
		{
			//a和d是输入的三角形,a未知d已知
			//未知三角形a中，未知的点是b
			//a中点c1和d中的点e1重合
			//a中点c2和d中的点e2重合
			a=index1;
			e1=0;
			e2=1;
			findDiffrentPoint(a,d,b,c1,c2,e1,e2);//输入a d e1 e2得到 b  c1 c2 
			count_TexcoordByHuang(a,b,c1,d,e1);
			Triangle->at(a).texCoord.texCoorGenTime = texGenTime;
			Triangle->at(a).texCoord.update = false;
			Triangle->at(a).updateTexCoord();
		}

		if(addIndextoProcesseTriangleIndex(index,index2))
		{
			a=index2;
			e1=1;
			e2=2;
			findDiffrentPoint(a,d,b,c1,c2,e1,e2);//输入a d e1 e2得到 b  c1 c2 
			count_TexcoordByHuang(a,b,c1,d,e1);
			Triangle->at(a).texCoord.texCoorGenTime = texGenTime;
			Triangle->at(a).texCoord.update = false;
			Triangle->at(a).updateTexCoord();
		}

		if(addIndextoProcesseTriangleIndex(index,index3))
		{
			a=index3;
			e1=2;
			e2=0;
			findDiffrentPoint(a,d,b,c1,c2,e1,e2);//输入a d e1 e2得到 b  c1 c2 
			count_TexcoordByHuang(a,b,c1,d,e1);
			Triangle->at(a).texCoord.texCoorGenTime = texGenTime;
			Triangle->at(a).texCoord.update = false;
			Triangle->at(a).updateTexCoord();
		}
			
		deleteIndexfromProcesseTriangleIndex(index);
		pocessedTriangleIndex.push_back(index);
	}
}*/
int CTextureProcessSystemDoc::findFaceIndex(int d,int e1,int e2)
{
	vector<gl_face> * Triangle=&(plyLoader.faceArry);
	vector<int>  res;
	res=findFaceByTwoPointNewWay(Triangle->at(d).ptnum[e1],Triangle->at(d).ptnum[e2],d);
	if(res.size()>0)
		return res.at(0);
}
/*bool CTextureProcessSystemDoc::addIndextoProcesseTriangleIndex(int pareIndex,int index)
{

	//如果已经处理过了 则不再处理
	for (int i = 0; i < pocessedTriangleIndex.size(); i++)
	{
		if (pocessedTriangleIndex[i] == index)
		{
			return false;
		}
	}
	
	for(int i=0;i<toProcesseTriangleIndex.size();i++)
	{
		if(index==toProcesseTriangleIndex.at(i))
		{
			return false;
		}
	}
		//此处加入法线判断2015.4.15 
	vector<gl_face> * Triangle=&(plyLoader.faceArry);
	gl_face  face1=Triangle->at(index);
	gl_face  face2=Triangle->at(pareIndex);
	gl_face  centerFace=Triangle->at(centerIndex);
	//求两个平面的夹角
	double cos=getNorError(face1.n,face2.n);
	double centerCos=getNorError(face1.n,centerFace.n);
	if((cos<_limitCos)||(centerCos<_centerLimitCos))
		return false;
	toProcesseTriangleIndex.push_back(index);
	return true;
}*/
/*void CTextureProcessSystemDoc::deleteIndexfromProcesseTriangleIndex(int index)
{
	for(int i=0;i<toProcesseTriangleIndex.size();i++)
	{
		if(index==toProcesseTriangleIndex.at(i))
		{
			toProcesseTriangleIndex.erase(i+toProcesseTriangleIndex.begin());
		}
	}
	
}*/
/*void CTextureProcessSystemDoc::calTexCorByIndex(int processedindex,int steps)
{
	times++;
	toProcesseTriangleIndex.clear();
	pocessedTriangleIndex.clear();
	reset2dCoord();
	count_h(processedindex);
	r=(rand()%256)/256.0;
	g=(rand()%256)/256.0;
	b=(rand()%256)/256.0;
	for(int W=0;W<steps;W++)
	{
		calTexCor();
	}
		//把超过纹理范围的三角形的纹理开关关掉不显示
	//resetOuterTriangleTex();
}*/
vector<gl_face *> CTextureProcessSystemDoc::markConnectedFace(double centerLimitCos,double limitCos,int steps)
{
	_centerLimitCos=centerLimitCos;
	_limitCos=limitCos;
	_steps=steps;
	vector<gl_face> * Triangle=&(plyLoader.faceArry);
	vector<gl_face *> resfaces;
	vector<int> connectedSelectFaceIndex;
	connectedSelectFaceIndex.push_back(userSelectingTriangleIndex);//加入第一个点
	Triangle->at(userSelectingTriangleIndex).r=1.0;
	Triangle->at(userSelectingTriangleIndex).g=0;
	Triangle->at(userSelectingTriangleIndex).b=0;
	for(int i=0;i<steps;i++)
	{
		//找到队列中的一个元素
		if(connectedSelectFaceIndex.size()<=0)
		{
			return resfaces;
		}else
		{
		
		}
		int index=connectedSelectFaceIndex.at(0);

		int index1=findFaceIndex(index,0,1);
		int index2=findFaceIndex(index,1,2);
		int index3=findFaceIndex(index,2,0);

		gl_face * centerFace=&Triangle->at(userSelectingTriangleIndex);
		gl_face * face=&Triangle->at(index);
		bool alreadyAddIn = false;
		for (int w = 0; w < resfaces.size(); w++)
		{
			if (resfaces[w]->facenum == index)
			{
				alreadyAddIn = true;
			}
		}
		if (!alreadyAddIn)
		{
			resfaces.push_back(&Triangle->at(index));
		}			
		gl_face * face1=&Triangle->at(index1);
		gl_face * face2=&Triangle->at(index2);
		gl_face * face3=&Triangle->at(index3);
		double cos1=getNorError(face->n,face1->n);
		double cos2=getNorError(face->n,face2->n);
		double cos3=getNorError(face->n,face3->n);
		double centerCos1=getNorError(centerFace->n,face1->n);
		double centerCos2=getNorError(centerFace->n,face2->n);
		double centerCos3=getNorError(centerFace->n,face3->n);
		if(cos1>=limitCos&&centerCos1>=centerLimitCos)
		{
			connectedSelectFaceIndex.push_back(index1);
		}
		if(cos2>=limitCos&&centerCos2>=centerLimitCos)
		{
			connectedSelectFaceIndex.push_back(index2);
		}
		if(cos3>=limitCos&&centerCos3>=centerLimitCos)
		{
			connectedSelectFaceIndex.push_back(index3);
		}
		//删除这个元素
		for(int i=0;i<connectedSelectFaceIndex.size();i++)
		{
			if(index==connectedSelectFaceIndex.at(i))
			{
				connectedSelectFaceIndex.erase(i+connectedSelectFaceIndex.begin());
			}
		}
	}
	return resfaces;
}
double CTextureProcessSystemDoc::getNorError(float * n1,float * n2)
{
	double n1_n2=n1[0]*n2[0]+n1[1]*n2[1]+n1[2]*n2[2];
	double n1_len=n1[0]*n1[0]+n1[1]*n1[1]+n1[2]*n1[2];
	double n2_len=n2[0]*n2[0]+n2[1]*n2[1]+n2[2]*n2[2];
	double cos=n1_n2/(sqrtf(n1_len)*sqrtf(n2_len));
	return cos;
}
/*bool CTextureProcessSystemDoc::ispocessed(int index)
{
	/*for (int i = 0; i < pocessedTriangleIndex.size(); i++)
	{
		if (pocessedTriangleIndex[i] == index)
		{
			return true;
		}
	}
	return false;
}*/
/*void CTextureProcessSystemDoc::calVertex2D(float pos[3], int index)
{
	//计算第index个targetTextureELement的偏移量
	//目标中心基元的位置
	float resPos[3];
	float centerPos[3];
	CvMat* R1 = cvCreateMat(4, 4, CV_32FC1);
	CvMat* R2 = cvCreateMat(4, 4, CV_32FC1);
	CvMat* A = cvCreateMat(4, 1, CV_32FC1);
	centerPos[0] = plyLoader.faceArry[index].pcorex;
	centerPos[1] = plyLoader.faceArry[index].pcorey;
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			CV_MAT_ELEM(*R1, float, i, j) = plyLoader.faceArry[index].R1[i][j];
			CV_MAT_ELEM(*R2, float, i, j) = plyLoader.faceArry[index].R2[i][j];
		}
	}
	
	CV_MAT_ELEM(*A, float, 0, 0) = pos[0];
	CV_MAT_ELEM(*A, float, 1, 0) = pos[1];
	CV_MAT_ELEM(*A, float, 2, 0) = pos[2];
	CV_MAT_ELEM(*A, float, 3, 0) = 1;

	cvMatMul(R1, A, A);
	cvMatMul(R2, A, A);

	resPos[0] = CV_MAT_ELEM(*A, float, 0, 0);
	resPos[1] = CV_MAT_ELEM(*A, float, 1, 0);
	resPos[2] = CV_MAT_ELEM(*A, float, 2, 0);

	for (int i = 0; i < 3; i++)
	{
		if (abs(resPos[i]) < 1e-005)
			resPos[i] = 0;
	}
	for (int  i = 0; i < 2; i++)
	{
		offset[i] = resPos[i] - centerPos[i];
	}
	

}*/
void CTextureProcessSystemDoc::buildTexCoordByIndex(int index, int maxDeep, int maxNum, float radius, int textureIndex)
{
	vector<int> v; 
	float kn = 1;
	int deep = 1;
	Point3D centerPt;
	centerPt.setValue(Point3D(0, 0, 0));
	LocalParameterization lp;
	double strech = 999;
	double area = 0;
	float scale = 1;
// 	while (strech>0.5)
// 	{
		v.clear();
		buildTexCoord(index, v, deep, maxDeep, maxNum, radius);
		area=getSelectArea(v);
		scale = area*40000;
		strech = lp.localPara(&plyLoader, v, index, offsetPT, mapScale);
		maxNum++;
	//}
	lp.updateTextureCoord(textureIndex);
	

/*
	//确定缩放比例
	float minCoord[3] = { 999, 999, 999 }; 
	float maxCoord[3] = { -999, -999, -999 };
	float coord[3];
	//找出包围盒的对角点
	for (int i = 0; i < v.size(); i++)
	{
		for (int j = 0; j < 3; j++)
		{
			coord[0] = plyLoader.pointArry[plyLoader.faceArry[v[i]].ptnum[j]].x;
			coord[1] = plyLoader.pointArry[plyLoader.faceArry[v[i]].ptnum[j]].y;
			coord[2] = plyLoader.pointArry[plyLoader.faceArry[v[i]].ptnum[j]].z;
			for (int k = 0; k<3; k++)
			{
				if (coord[k]>maxCoord[k])
				{
					maxCoord[k] = coord[k];
				}
				if (coord[k]<minCoord[k])
				{
					minCoord[k] = coord[k];
				}
			}
		}
	}
	//包围盒对角线长度
	float sunLen = 0;
	for (int k = 0; k < 3; k++)
	{
		sunLen += pow((maxCoord[k] - minCoord[k]), 2);
	}
	float lens3d = sqrtf(sunLen);
	kn = sqrtf(2) / lens3d ;

	//确定偏移量
	//纹理中心在纹理坐标中的具体位置
	//不妨设，中心在中心平面的重心，则
	Point3D pt[3];
	for (int k = 0; k < 3; k++)
	{
		pt[k].x = plyLoader.pointArry[plyLoader.faceArry[v[0]].ptnum[k]].u;
		pt[k].y = plyLoader.pointArry[plyLoader.faceArry[v[0]].ptnum[k]].v;
	}
	centerPt.x = (pt[0].x + pt[1].x + pt[2].x) / 3;
	centerPt.y = (pt[0].y + pt[1].y + pt[2].y) / 3;
	*/
	kn = 1;
	//直接添加纹理坐标
	for (int i = 0; i < v.size(); i++)
	{
		
		/*for (int j = 0; j < 3; j++)
		{
			plyLoader.faceArry[v[i]].texCoord.cor[j][0] = (plyLoader.pointArry[plyLoader.faceArry[v[i]].ptnum[j]].u - centerPt.x)*kn + centerPt.x;
			plyLoader.faceArry[v[i]].texCoord.cor[j][1] = (plyLoader.pointArry[plyLoader.faceArry[v[i]].ptnum[j]].v - centerPt.y)*kn + centerPt.y;
		}		*/
	
		bool addIn = true;
		for (int j = 0; j < 3; j++)
		{
			if (!lp.effectFaceVector[lp.find2by1(v[i], lp.vfp)])
			{
				addIn = false;
				continue;
			}
			else
			{
				addIn = true;
			}
			plyLoader.faceArry[v[i]].texCoord.cor[j][0] = plyLoader.pointArry[plyLoader.faceArry[v[i]].ptnum[j]].u;
			plyLoader.faceArry[v[i]].texCoord.cor[j][1] = plyLoader.pointArry[plyLoader.faceArry[v[i]].ptnum[j]].v;
			plyLoader.faceArry[v[i]].texCoord.textureIndex = textureIndex;
		}
		if (addIn)
		{
			plyLoader.faceArry[v[i]].texCoord.update = false;
			plyLoader.faceArry[v[i]].updateTexCoord();
			//testV
			plyLoader.faceArry[v[i]].testV = true;
		}

		ofstream f("vlog.txt", ios::app);
		f << "vlog Triangle  " << v[i] << "-"<<addIn<<endl;
			f << plyLoader.faceArry[v[i]].texCoord.cor[0][0] << "," << plyLoader.faceArry[v[i]].texCoord.cor[0][1] << endl;
			f << plyLoader.faceArry[v[i]].texCoord.cor[1][0] << "," << plyLoader.faceArry[v[i]].texCoord.cor[1][1] << endl;
			f << plyLoader.faceArry[v[i]].texCoord.cor[2][0] << "," << plyLoader.faceArry[v[i]].texCoord.cor[2][1] << endl;		
			f << endl;
		f.close();

	}
}
void CTextureProcessSystemDoc::buildTexCoord(int index, vector<int>&v, int &deep, int maxDeep, int maxNum, float radius)
{
	vector<int> vlist;
	int nearestIndex = index;
	float mindis = 999;
	//广度优先遍历
	vlist.push_back(index);
	int count = 0;
	Point3D centerPT;
	centerPT.x = plyLoader.faceArry[index].corex;
	centerPT.y = plyLoader.faceArry[index].corey;
	centerPT.z = plyLoader.faceArry[index].corez;
	while (vlist.size() > 0 && count < maxNum)
	{
		//找出与中心最近的
		bool vAddIn = false;
		while (!vAddIn)
		{
			mindis = 999;
			for (int i = 0; i < vlist.size(); i++)
			{
				Point3D pt;
				pt.x = plyLoader.faceArry[vlist[i]].corex;
				pt.y = plyLoader.faceArry[vlist[i]].corey;
				pt.z = plyLoader.faceArry[vlist[i]].corez;
				float dis = (centerPT - pt).getDistance();
				if (dis < mindis)
				{
					nearestIndex = i;
					mindis = dis;
				}
			}
			vAddIn = true;
			for (int i = 0; i < v.size(); i++)
			{
				if (vlist[nearestIndex] == v[i])
				{
					vAddIn = false;
				}
			}
			if (vAddIn)
			{
				v.push_back(vlist[nearestIndex]);
				count++;
			}
		}
		//
		int m_index = vlist[nearestIndex];
		//m_index 出队
		vlist.erase(vlist.begin() + nearestIndex);

		int new_indexdex[3];
		new_indexdex[0] = findFaceIndex(m_index, 0, 1);
		new_indexdex[1] = findFaceIndex(m_index, 1, 2);
		new_indexdex[2] = findFaceIndex(m_index, 2, 0);
	  
		for (int j = 0; j < 3; j++)
		{
			bool addIn = true;
			/*addIn = [&](int a, int c, float r)mutable throw()->bool
			{
				Point3D ptA(this->plyLoader.faceArry[a].corex, 
					this->plyLoader.faceArry[a].corey,
					this->plyLoader.faceArry[a].corez);
				Point3D ptC(this->plyLoader.faceArry[c].corex,
					this->plyLoader.faceArry[c].corey,
					this->plyLoader.faceArry[c].corez);
				if ((ptA - ptC).getDistance() > r)
					return false;
				else
					return true;
			}(new_indexdex[j],m_index,radius);*///半径限制
			//在已经遍历或者已经在队列中的，不再添加
			for (int i = 0; i < vlist.size(); i++)
			{
				if (new_indexdex[j] == vlist[i])
				{
					addIn = false;
					break;
				}
			}
			for (int i = 0; i < v.size(); i++)
			{
				if (new_indexdex[j] == v[i])
				{
					addIn = false;
					break;
				}
			}
			if (addIn)
			{
				vlist.push_back(new_indexdex[j]);
			}
		}
	}
	return;
}
double CTextureProcessSystemDoc::getSelectArea(vector<int>&v)
{
	double res = 0;
	int ptnum[3];
	Point3D pt[3];
	for (int i = 0; i < v.size(); i++)
	{
		for (int j = 0; j < 3; j++)
		{
			ptnum[j] = plyLoader.faceArry[i].ptnum[j];
			pt[j].x = plyLoader.pointArry[ptnum[j]].x;
			pt[j].y = plyLoader.pointArry[ptnum[j]].y;
			pt[j].z = plyLoader.pointArry[ptnum[j]].z;
		}
		Triangle * tri = new Triangle();
		tri->setValue(pt);
		res += tri->getArea();
		delete tri;
	}
	return res;

}
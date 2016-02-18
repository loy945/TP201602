#include "StdAfx.h"
#include "TextureElement.h"
#define pi 3.1415926
#include <Eigen/Dense>  
#include <Eigen/LU>
using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;

TextureElement::TextureElement(int index)
{
	textureIndex=1;
	this->index=index;
	for(int i=0;i<3;i++)
	{
		pos[i]=0;
		n[i]=0;
	}
	face=NULL;
	isfixed = false;
	isShow = true;
	linkSize = 0;
	link.clear();
}
/*
TextureElement::TextureElement(int index, gl_face * face, int textureIndex, float position[3])
{
	textureIndex = textureIndex;
	this->index = index;
	this->face = face;
	for (int i = 0; i<3; i++)
	{
		n[i] = face->n[i];
		pos[i] = position[i];
	}
	

}*/
TextureElement::TextureElement(int index, gl_face * face, int textureIndex, Point3D &pt)
{
	textureIndex = textureIndex;
	this->index = index;
	this->face = face;
	for (int i = 0; i<3; i++)
	{
		n[i] = 0;
	}
	pos[0] = pt.x;
	pos[1] = pt.y;
	pos[2] = pt.z;
	isfixed = false;
	isShow = true;
	linkSize = 0;
	link.clear();
}
TextureElement::TextureElement(int index,gl_face * face,int textureIndex)
{	
	textureIndex=textureIndex;
	this->index=index;
	this->face=face;


	pos[0]=face->corex;
	pos[1]=face->corey;
	pos[2]=face->corez;
	for(int i=0;i<3;i++)
	{
		n[i]=0;
		
	}
	isfixed = false;
	isShow = true;
	linkSize = 0;
	link.clear();
}


TextureElement::~TextureElement(void)
{
}
bool TextureElement::init(gl_face * face)
{
	if(!face)
	{
		return false;
	}
	this->face=face;
	double sun=0;
	for(int i=0;i<3;i++)
	{
		this->n[i]=face->n[i];
		sun+=pow(this->n[i],2);
	}
	for(int i=0;i<3;i++)
	{
		this->n[i]=this->n[i]/(sqrt(sun));
	}
	isfixed = false;
	isShow = true;
}
void TextureElement::addLinkNode(LinkData * node)
{
	for(int i=0;i<link.size();i++)
	{
		if (link[i]->index == node->index)
			return;
	}
	link.push_back(node);
	linkSize++;
	
}
void TextureElement::deleteNdoe(LinkData * node)
{
	for (int i = 0; i<link.size(); i++)
	{
		if (link[i]->index == node->index)
		{
			link.erase(link.begin() + i);
			linkSize--;
		}
	}
}

double TextureElement::getDisFrom(float * position)
{
	float res[3];
	projection(n,position,pos,res);
	

	//res是空间一点position 投影到 以n为法向量 并过pos点的平面 上 的坐标
	double sun=0;
	for(int i=0;i<3;i++)
	{
		sun += pow(pos[i] - position[i], 2);
	}
	float dis = sqrt(sun);
	return dis;
}
void TextureElement::VectorProjection(float *n, float *v, float res[3])
{
	//n是平面法向量，v是要投影的向量
	int i, j;
	double NN[3][3], norm2, t;
	norm2 = n[0] * n[0] + n[1] * n[1] + n[2] * n[2];
	for (i = 0; i<3; i++)
	{
		for (j = 0; j<3; j++)
		{
			NN[i][j] = -n[i] * n[j];
			if (i == j)
			{
				NN[i][j] += norm2;
			}
		}
	}
	for (i = 0; i<3; i++)
	{
		t = 0;
		for (j = 0; j<3; j++)
		{
			t += v[j] * NN[i][j];
		}
		res[i] = t / norm2;
	}
}


double  TextureElement::getAngleFrom(float * position)//position平面外一点 要投影到平面上
{
	bool eror4 = false;
	if (n[0] == 0 && n[1] == 0 && n[2] == 0)
	{
		eror4 = true;
	}

	//空间向量e(1,0,0) 投影成vB
	float e[3] = { 1, 0, 0 };
	float vB[3];
	VectorProjection(n, e, vB);
	//向量PP投影成PPpro
	float PP[3], PPpro[3];
	Vector3f vOB, vP;
	for (int i = 0; i<3; i++)
	{
		PP[i] = position[i] - pos[i];
	}
	VectorProjection(n, PP, PPpro);
	for (int i = 0; i<3; i++)
	{
		vP[i] = PPpro[i];
		vOB[i] = vB[i];
	}
	///求向量夹角
	float value = 0;

	
	value = (vOB.dot(vP) / (vOB.norm()*vP.norm()));
	float xita = 0;
	xita = acosf(value);
	//vP与vOB叉乘得到方向向量kn
	Vector3f kn = vOB.cross(vP);
	//平面发行量pn
	Vector3f pn;
	for (int i = 0; i < 3; i++)
	{
		pn[i] = n[i];
	}
	//判断kn与n是否同方向
	if (kn.dot(pn)< 0)
	{
		return 360-xita / pi * 180;
	}
	
	return xita / pi * 180;
}

double getError(LinkData &link1,LinkData &link2)
{
	double len1=link1.distance;
	double len2=link2.distance;
	double angle1=link1.angle;
	double angle2=link2.angle;
	//默认相同类型
	int cid1=1;
	int cid2=1;
//参数
	int cid=0;
	double xita=0;
	double l;
	double w1=50;
	double w2=1;
	double w3=1;
	double w4=50;
	double w5=1000;

	//类别
	if(cid1==cid1)
	{
		cid=0;
	}else
	{
		cid=1;
	}
	//L
	l=abs(len1-len2);
	//xita
	xita=abs(angle1-angle2);
	if (xita > 180)
	{
		xita = 360 - xita;
	}
	//return w1*q+w2*xita+w3*l+w4*cid+w5*subid;
	//returnError=w2*xita+w3*l+w4*cid+w5*subid;
	double returnError=w2*xita+w3*l+w4*cid;
	return returnError;
}
bool comp(LinkData * a,LinkData * b)
{
    return a->angle<b->angle;
}

void TextureElement::textureElementSort()
{
	if (this->linkSize<1)
	{
		return;
	}
	if(link.size()<=1)
	{
		//数量大于1才需要排序
		return;
	}
	//把连同的所有纹理元素按象限逆时针排序
	sort(link.begin(),link.end(),comp);
}
void TextureElement::projection(float *n ,float * position,float * planeCenterPos,float res[3])
{

	Vector3f nor,a;//nor是平面法向量，a是要投影到此平面上的向量
	for (int i = 0; i < 3; i++)
	{
		nor[i] = n[i];
		a[i] = position[i] - planeCenterPos[i];
	}
	//法向量单位化
	nor = nor.normalized();
	//将a投影到法向量上获得向量b
	Vector3f b;
	b = (a.dot(nor) / nor.norm())*a;
	Vector3f c = a - b;
	for (int i = 0; i < 3; i++)
	{
		res[i] = c[i] + planeCenterPos[i];
	}
	
	
}
void TextureElement::setPos(Point3D * pt)
{
	pos[0] = pt->x;
	pos[1] = pt->y;
	pos[2] = pt->z;
}
Point3D * TextureElement::getPos()
{
	Point3D * pt = new Point3D(pos);
	return pt;
}
LinkData * TextureElement::findLinkDataById(int index)
{
	for (int i = 0; i < link.size(); i++)
	{
		if (index == link[i]->index)
		{
			return link[i];
		}
	}
}
bool TextureElement::isLinkTo(int index)
{
	for (int i = 0; i < this->link.size(); i++)
	{
		if (link[i]->index == index)
		{
			return true;
		}
	}
	return false;
}

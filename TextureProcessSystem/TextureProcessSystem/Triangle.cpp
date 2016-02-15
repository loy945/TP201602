#pragma once
#include "stdafx.h"
#include "Triangle.h"
#include "Line.h"

Triangle::Triangle(void)
{
}


Triangle::~Triangle(void)
{
}

void Triangle::setValue(Point3D point[3])
{
	for(int i=0;i<3;i++)
	{
		pt[i].setValue(point[i]);
	}
}

bool Triangle::isAPointInTriangle(Point3D point)
{
	//如果point与三角形顶点重合，不算做在三角形内
	for(int i=0;i<3;i++)
	{
		if(this->pt[i]==point)
		{
			return false;	
		}
	}
	


	//用面积方法存在误差
	Triangle tr1,tr2,tr3;
	tr1.setValue(pt);
	tr2.setValue(pt);
	tr3.setValue(pt);
	tr1.pt[0].setValue(point);
	tr2.pt[1].setValue(point);
	tr3.pt[2].setValue(point);
	double s=getArea();
	double s1=tr1.getArea();
	double s2=tr2.getArea();
	double s3=tr3.getArea();
	double res=s-s1-s2-s3;
	if(res>=0)
	{
		return true;
	}
		return false;
	
	//判断三角形ABC与点D，AB<->CD AD<->CB BC<->AD是否有交点2015.3.25
	//Point3D pt1=this->pt[0];
	//Point3D pt2=this->pt[1];
	//Point3D pt3=this->pt[2];
	//Point3D pt4=point;
	//Line l1;
	//if(l1.isCross(pt1,pt2,pt3,pt4)||l1.isCross(pt1,pt3,pt2,pt4)||l1.isCross(pt3,pt2,pt1,pt4))
	//{
	//	return false;
	//}
	//return true;
}
double Triangle::getArea()
{
	double a=sqrt((pow((pt[0].x-pt[1].x),2)+pow((pt[0].y-pt[1].y),2)));
	double b=sqrt((pow((pt[2].x-pt[1].x),2)+pow((pt[2].y-pt[1].y),2)));
	double c=sqrt((pow((pt[2].x-pt[0].x),2)+pow((pt[2].y-pt[0].y),2)));
	double p=(a+b+c)/2 ;
	double s=(p*(p-a)*(p-b)*(p-c));

	return s;
}

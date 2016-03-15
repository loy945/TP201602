#pragma once
#include <math.h>
#include "config.h"
class Point3D
{
public:
	Point3D(void);
	Point3D(const Point3D& t)
	{
		this->beLongFaceID = t.beLongFaceID;
		this->x = t.x;
		this->y = t.y;
		this->z = t.z;
	}
	~Point3D(void);
	Point3D(float pos[3]);
	Point3D(float x, float y, float z){ this->x = x; this->y = y; this->z = z; beLongFaceID = 0; }
	void setValue(float x, float y, float z){ this->x = x; this->y = y; this->z = z; beLongFaceID = 0; }
	void setValue(Point3D &point){ this->x = point.x; this->y = point.y; this->z = point.z; this->beLongFaceID = point.beLongFaceID; }
	double getDistance(){
		return sqrt(this->x*this->x+this->y*this->y+this->z*this->z);
	
	}
	friend Point3D  operator - (Point3D &pt1,Point3D &pt2)
	{
	
		Point3D pt;
		pt.x=pt1.x-pt2.x;
		pt.y=pt1.y-pt2.y;
		pt.z=pt1.z-pt2.z;
		return pt;
	}
	
		friend Point3D  operator + (Point3D &pt1,Point3D &pt2)
	{
	
		Point3D pt;
		pt.x=pt1.x+pt2.x;
		pt.y=pt1.y+pt2.y;
		pt.z=pt1.z+pt2.z;
		return pt;
	}
	friend Point3D  operator * (Point3D &pt1, float k)
	{
		Point3D pt;
		pt.x = pt1.x * k;
		pt.y = pt1.y * k;
		pt.z = pt1.z * k;
		return pt;
	}
	friend Point3D  operator / (Point3D &pt1, float k)
	{
		Point3D pt;
		pt.x = pt1.x / k;
		pt.y = pt1.y / k;
		pt.z = pt1.z / k;
		return pt;
	}

	bool operator == (const Point3D & pt ) const
	{
		if ((abs(this->x - pt.x)<minFloat) && (abs(this->y - pt.y)<minFloat) && (abs(this->z - pt.z)<minFloat))
		{
			return true;
		}
		return false;
		
	}
	float x;
	float y;
	float z;
	int beLongFaceID;

};

class Point3D_3
{
public:
	Point3D pt[3];
	bool update;
public:
	Point3D_3();
	bool is_valid(Point3D pt[3])
	{
		//bool texCor[3] = { true, true, true };//判断某一点纹理坐标是否有效
		int texCorError=0; //3个点中无效的个数
		for (int i = 0; i < 3; i++)
		{
			if ((pt[i].x>1 || pt[i].x<0) || (pt[i].y>1 || pt[i].y<0))
			{
				/*texCor[i] = false;*/
				texCorError++;
			}
		}
		if (texCorError >= 3)
		{
			//面片纹理坐标无效，不添加
			return false;
		}
		else
		{
			return true;
		}
	}
};
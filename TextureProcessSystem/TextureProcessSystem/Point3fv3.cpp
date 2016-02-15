#include "StdAfx.h"
#include "Point3fv3.h"


Point3fv3::Point3fv3()
{
	update = false;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			cor[i][j] = -1;
		}
	}
	texCoorGenTime = 0;
}


Point3fv3::~Point3fv3()
{
}
bool Point3fv3::is_valid(float cor[3][3])
{
	int texCorError = 0; //3个点中无效的个数
	for (int i = 0; i < 3; i++)
	{
		if ((cor[i][0]>1 || cor[i][0]<0) || (cor[i][0]>1 || cor[i][0]<0))
		{
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

void Point3fv3::setValue(Point3fv3 & pt3fv3)
{
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			cor[i][j] = pt3fv3.cor[i][j];
		}
	}
	this->update = pt3fv3.update;
	this->texCoorGenTime = pt3fv3.texCoorGenTime;
	this->textureIndex = pt3fv3.textureIndex;
	 
}

void Point3fv3::resetValue()
{
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			cor[i][j] = -1;
		}
	}
	this->update = true;
	this->texCoorGenTime = 0;
}

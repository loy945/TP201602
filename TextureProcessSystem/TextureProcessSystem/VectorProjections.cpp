#include "StdAfx.h"
#include "VectorProjections.h"


VectorProjections::VectorProjections()
{
}


VectorProjections::~VectorProjections()
{
}

VectorProjections::VectorProjections(float *n, float *v, float res[3])
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
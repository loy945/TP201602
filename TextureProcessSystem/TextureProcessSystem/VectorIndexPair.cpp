#include "StdAfx.h"
#include "VectorIndexPair.h"


VectorIndexPair::VectorIndexPair()
{
}


VectorIndexPair::~VectorIndexPair()
{
}
VectorIndexPair::VectorIndexPair(vector<vector<int> >  &linkPairs, int index, int paratype, int &res)
{
	VectorIndexPair();
	if (paratype == 1)
	{
		res = find1(linkPairs, index);
	}
	if (paratype == 2)
	{
		res = find2(linkPairs, index);
	}
}
int VectorIndexPair::find1(vector<vector<int> >  &linkPairs, int index2)
{
	for (int i = 0; i < linkPairs.size(); i++)
	{
		if (linkPairs[i][2] == index2)
		{
			return linkPairs[i][0];
		}
	}
	return -1;
}
int VectorIndexPair::find2(vector<vector<int> >  &linkPairs, int index1)
{
	for (int i = 0; i < linkPairs.size(); i++)
	{
		if (linkPairs[i][0] == index1)
		{
			return linkPairs[i][2];
		}
	}
	return -1;
}
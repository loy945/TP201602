#pragma once
#include "stdafx.h"
#include <vector>
using namespace std;
class VectorIndexPair
{
public:
	VectorIndexPair();
	~VectorIndexPair();
	VectorIndexPair(vector<vector<int> > &linkPairs, int index, int paratype, int &res);
	int find1(vector<vector<int> >  &linkPairs, int index1);
	int find2(vector<vector<int> >  &linkPairs, int index2);
};


#include "StdAfx.h"
#include "TriangleFace.h"


TriangleFace::TriangleFace()
{
	faceNearByNums = 0;
	tempLinkPara = NULL;
	cost = 99999;
	isMark = false;
}


TriangleFace::~TriangleFace()
{
}

#pragma once
#include "plyloader.h"
class TriangleFace :public gl_face
{
public:
	TriangleFace();
	~TriangleFace();
public:
	int faceNearByIndex[3];
	float faceNearByDistance[3];
	int faceNearByNums;
	TriangleFace * tempLinkPara;//指向来源，寻找路径的时候作参考
	float cost;//从来源处到这一节点总长度
	bool isMark;
	pair<float, int> nearestTE;//到最近元素的测地线距离及elementIndex
};


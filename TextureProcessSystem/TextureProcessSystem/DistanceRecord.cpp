#include "StdAfx.h"
#include "DistanceRecord.h"
#include <math.h>
#include <fstream>
using namespace std;
DistanceRecord::DistanceRecord()
{
}


DistanceRecord::~DistanceRecord()
{
}
void DistanceRecord::buildNeighbourTriangleIndex(int faceIndex)
{
	int resNum = 0;
	//1.找到指定的三角形面片
	TriangleFace * face1 = NULL;
	for (int i = 0; i < m_triangleFaceArry.size(); i++)
	{
		if (m_triangleFaceArry[i].facenum == faceIndex)
		{
			face1 = &m_triangleFaceArry[i];
			break;
		}
	}
	if (face1)
	{
		for (int i = 0; i < m_triangleFaceArry.size(); i++)
		{
			TriangleFace * face2 = &m_triangleFaceArry[i];
			if (face1->facenum == face2->facenum)
			{
				continue;
			}
			int j = 0, k = 0;
			int counter = 0;//三角形face1，和三角形face2,公共点数量
			while (true)
			{
				if (face1->ptnum[j] == face2->ptnum[k])
				{
					counter++;
				}
				j++;
				if (j > 2)
				{
					j = 0;
					k++;
				}
				if (k > 2)
				{
					break;
				}
			}
			if (counter >= 2)
			{
				//有两个以上的公共点
				//添加相邻关系,如果已经添加则跳过
				bool isAdded = false;
				for (int w = 0; w < face1->faceNearByNums; w++)
				{
					if (face1->faceNearByIndex[w] == face2->facenum)
					{
						isAdded = true;
					}
				}
				if (!isAdded)
				{
					//没有添加过的情况，则添加
					face1->faceNearByNums++;
					face1->faceNearByIndex[face1->faceNearByNums - 1] = face2->facenum;
				}
			}
		}
	}

}

void DistanceRecord::init(Model_PLY * plyModel)
{
	this->m_plyModel = plyModel;

	for (int i = 0; i < m_plyModel->faceArry.size(); i++)
	{
		TriangleFace * tempTF = new TriangleFace();
		//建立对应的一个节点，保存面片序号，以及包含顶点的序号，面片重心坐标
		tempTF->facenum = m_plyModel->faceArry[i].facenum; tempTF->isMark = false;
		for (int j = 0; j < 3; j++)
		{
			tempTF->ptnum[j] = m_plyModel->faceArry[i].ptnum[j];
		}
		tempTF->corex = m_plyModel->faceArry[i].corex;
		tempTF->corey = m_plyModel->faceArry[i].corey;
		tempTF->corez = m_plyModel->faceArry[i].corez;
		this->m_triangleFaceArry.push_back(*tempTF);
	}
	bool read=false;
	read = readFromFile("DistanceRecord.log");
	if (!read)
	{
		for (int i = 0; i < m_triangleFaceArry.size(); i++)
		{
			buildNeighbourTriangleIndex(m_triangleFaceArry[i].facenum);
		}
		buildDistanceData();
		outputToFile("DistanceRecord.log");
	}
}
bool DistanceRecord::outputToFile(const char * fileName)
{
	fstream f(fileName, ios::out|ios::ate);
	for (int i = 0; i < m_plyModel->faceArry.size(); i++)
	{
		TriangleFace * face1 = &m_triangleFaceArry[i];
		f << face1->faceNearByNums << " ";
		for (int k = 0; k < face1->faceNearByNums; k++)
		{
			f << face1->faceNearByIndex[k] <<" "<< face1->faceNearByDistance[k] << endl;
		}
	}
	f.close();
	return true;
}
bool DistanceRecord::readFromFile(const char * fileName)
{
	fstream f(fileName);
	for (int i = 0; i < m_plyModel->faceArry.size(); i++)
	{
		TriangleFace * face1 = &m_triangleFaceArry[i];
		int num = 0;
		f >> num;
		if (f.eof())
		{
			return false;
		}
		face1->faceNearByNums =num;		
		for (int k = 0; k < face1->faceNearByNums; k++)
		{
			f >>face1->faceNearByIndex[k] >> face1->faceNearByDistance[k];
		}
	}
	f.close();
	return true;
}
void DistanceRecord::buildDistanceData()
{
	for (int i = 0; i < m_plyModel->faceArry.size(); i++)
	{
		for (int k = 0; k < 3; k++)
		{
			for (int j = 0; j < m_plyModel->faceArry.size(); j++)
			{
				TriangleFace * face1 = &m_triangleFaceArry[i];
				TriangleFace * face2 = &m_triangleFaceArry[j];
				if (face2->facenum == face1->faceNearByIndex[k])
				{
					float dx = face1->corex - face2->corex;
					float dy = face1->corey - face2->corey;
					float dz = face1->corez - face2->corez;
					face1->faceNearByDistance[k] = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2));
				}
			}
		}
	}
}
float DistanceRecord::getDistanceBetweenTwoTriangles(int index1, int index2)
{
	//1.找到指定的三角形面片
	TriangleFace * face1 = NULL;
	for (int i = 0; i < m_triangleFaceArry.size(); i++)
	{
		if (m_triangleFaceArry[i].facenum == index1)
		{
			face1 = &m_triangleFaceArry[i];
			break;
		}
	}
	TriangleFace * face2 = NULL;
	for (int i = 0; i < m_triangleFaceArry.size(); i++)
	{
		if (m_triangleFaceArry[i].facenum == index2)
		{
			face2 = &m_triangleFaceArry[i];
			break;
		}
	}

	if (face1&&face2)
	{
		for (int i = 0; i < face1->faceNearByNums; i++)
		{
			if (face1->faceNearByIndex[i] == face2->facenum)
			{
				return face1->faceNearByDistance[i];
			}
		}
		//并不相邻
		return -1;
	}

}
float DistanceRecord::getGeodesicDistanceBetweenTwoTriangles(int index1, int index2)
{
	//1.找到指定的三角形面片
	TriangleFace * face1 = NULL;
	for (int i = 0; i < m_triangleFaceArry.size(); i++)
	{
		if (m_triangleFaceArry[i].facenum == index1)
		{
			face1 = &m_triangleFaceArry[i];
			break;
		}
	}
	TriangleFace * face2 = NULL;
	for (int i = 0; i < m_triangleFaceArry.size(); i++)
	{
		if (m_triangleFaceArry[i].facenum == index2)
		{
			face2 = &m_triangleFaceArry[i];
			break;
		}
	}

	if (face1&&face2)
	{
		//从face1开始进行深度遍历
		face1->cost = 0;
		vector<TriangleFace *> checkingList;
		vector<TriangleFace *> checkedList;
		checkingList.push_back(face1);
		while (checkingList.size() > 0)
		{
			TriangleFace * face3 = checkingList[0];
			if (face3->facenum == face2->facenum)
			{
				//成功找到
				return face2->cost;


			}
			else
			{
				checkingList.erase(checkingList.begin());
				for (int w = 0; w < face3->faceNearByNums; w++)
				{
					TriangleFace * faceN = NULL;
					for (int i = 0; i < m_triangleFaceArry.size(); i++)
					{
						if (m_triangleFaceArry[i].facenum == face3->faceNearByIndex[w])
						{
							faceN = &m_triangleFaceArry[i];
							float distance = getDistanceBetweenTwoTriangles(face3->facenum, faceN->facenum);
							if (distance> 0)
							{
								if (faceN->cost >= face3->cost + distance)
								{
									faceN->cost = face3->cost + distance;
									faceN->tempLinkPara = face3;
								}

							}

						}
					}
					bool isChecked = false;
					for (int i = 0; i < checkedList.size(); i++)
					{
						if (faceN->facenum == checkedList[i]->facenum)
						{
							isChecked = true;
						}
					}
					if (!isChecked)
					{
						checkingList.push_back(faceN);
					}

				}
				checkedList.push_back(face3);

			}
		}

	}
}

float DistanceRecord::getGeodesicDistanceBetweenTwoTrianglesWithBEPoint(int index1, int index2, Point3D pt1, Point3D pt2)
{	//1.找到指定的三角形面片
	TriangleFace * face1 = NULL;
	for (int i = 0; i < m_triangleFaceArry.size(); i++)
	{
		if (m_triangleFaceArry[i].facenum == index1)
		{
			face1 = &m_triangleFaceArry[i];
			break;
		}
	}
	TriangleFace * face2 = NULL;
	for (int i = 0; i < m_triangleFaceArry.size(); i++)
	{
		if (m_triangleFaceArry[i].facenum == index2)
		{
			face2 = &m_triangleFaceArry[i];
			break;
		}
	}

	if (face1&&face2)
	{
		//从face1开始进行深度遍历
		face1->cost = 0;
		vector<TriangleFace *> checkingList;
		vector<TriangleFace *> checkedList;
		checkingList.push_back(face1);
		while (checkingList.size() > 0)
		{
			TriangleFace * face3 = checkingList[0];
			if (face3->facenum == face2->facenum)
			{
				//成功找到
				return face2->cost;
			}
			else
			{
				checkingList.erase(checkingList.begin());
				for (int w = 0; w < face3->faceNearByNums; w++)
				{
					TriangleFace * faceN = NULL;
					for (int i = 0; i < m_triangleFaceArry.size(); i++)
					{
						if (m_triangleFaceArry[i].facenum == face3->faceNearByIndex[w])
						{
							faceN = &m_triangleFaceArry[i];
							//在首尾分别重新计算指定点到下一面片中心点距离
							if (face3->facenum == face1->facenum)
							{
								//计算开始距离
								faceN->cost = sqrt(pow(pt1.x - faceN->corex, 2) + pow(pt1.y - faceN->corey, 2) + pow(pt1.z - faceN->corez, 2));
							}
							else if (faceN->facenum == face2->facenum)
							{
								//计算结束距离
								faceN->cost = face3->cost+sqrt(pow(pt2.x - face3->corex, 2) + pow(pt2.y - face3->corey, 2) + pow(pt2.z - face3->corez, 2));
							}
							else
							{
								float distance = getDistanceBetweenTwoTriangles(face3->facenum, faceN->facenum);
								if (distance > 0)
								{
									if (faceN->cost >= face3->cost + distance)
									{
										faceN->cost = face3->cost + distance;
										faceN->tempLinkPara = face3;
									}
								}
							}

							
						}
					}
					bool isChecked = false;
					for (int i = 0; i < checkedList.size(); i++)
					{
						if (faceN->facenum == checkedList[i]->facenum)
						{
							isChecked = true;
						}
					}
					if (!isChecked)
					{
						checkingList.push_back(faceN);
					}
				}
				checkedList.push_back(face3);
			}
		}
	}
	return -404;
}
void DistanceRecord::initDistanceFromCenter(int index)
{
	TriangleFace * face1 = NULL;
	vector<TriangleFace *> checkingList;
	vector<TriangleFace *> checkedList;
	vector<TriangleFace *> all;
	for (int i = 0; i < m_triangleFaceArry.size(); i++)
	{
		if (m_triangleFaceArry[i].facenum == index)
		{
			face1 = &m_triangleFaceArry[i];
		}
		all.push_back(&m_triangleFaceArry[i]);
	}

	face1->disFromCenter = 0;	
	checkingList.push_back(face1);
	while (checkingList.size() > 0)
	{
		TriangleFace * face3 = checkingList[0];
		checkingList.erase(checkingList.begin());
		checkedList.push_back(face3);
		for (int w = 0; w < face3->faceNearByNums; w++)
		{
			TriangleFace * faceN = NULL;
			faceN = &m_triangleFaceArry[face3->faceNearByIndex[w]];
			//在首尾分别重新计算指定点到下一面片中心点距离					
			if (faceN->disFromCenter >= face3->disFromCenter + faceN->faceNearByDistance[w])
			{
				faceN->disFromCenter = face3->disFromCenter + faceN->faceNearByDistance[w];
			}
			bool canAddedIn = false;
			for (int j = 0; j < all.size(); j++)
			{
				if (faceN->facenum == all[j]->facenum)
				{
					canAddedIn = true;
					all.erase(all.begin() + j);
					break;
				}
			}
			if (canAddedIn)
			{
				checkingList.push_back(faceN);
			}

		}
	}
	int d = 0;
	d = 1;
}
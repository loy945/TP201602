#include "StdAfx.h"
#include "Texture.h"
#include <fstream>
using namespace std;
Texture::Texture(void)
{
}


Texture::~Texture(void)
{
}
bool Texture::addTE(TextureElement * te)
{
	/*for (int i = 0; i < tes.size(); i++)
	{
		if (tes[i]->index == te->index)
			return false;
	}*/
	tes.push_back(te);
	return true;	
}
bool Texture::deleteTE(TextureElement * te)
{
	for (int i = 0; i < tes.size(); i++)
	{
		if (tes[i]->index == te->index)
		{
			tes.erase(tes.begin()+i);
			return true;
		}		
	}
	return false;
}
bool Texture::isConnected(int index1,int index2)
{
	TextureElement * te=tes.at(index1);
	for(int i=0;i<te->link.size();i++)
	{
		LinkData * node=te->link[i];
		if(node->index==index2)
			return true;
	}
	return false;

}

bool Texture::addLink(TextureElement*te1,TextureElement *te2)
{
	return true;
}
int Texture::countSides(int index)
{
	int count=0;
	//检测邻域有多少条环形变，即关联节点之间存在多少关系
	TextureElement * te1=tes.at(index);

	int i = 0, j = 0;
	for(i;i<te1->link.size();i++)
	{
		j = i + 1;
		if (j >= te1->link.size())
		{
			j = 0;
		}
		int index1 = te1->link[i]->index;
		int index2 = te1->link[j]->index;
		bool islink = isConnected(index1, index2);
		if (islink)
		{
			count++;
		}
	}
	

	return count;
}
bool Texture::isRingStruct(int index)
{
	/*cout<<"lines:"<<this->countSides(index)<<endl;
	cout<<"node:"<<this->tes.at(index)->link.size()<<endl;*/
	bool res=false;
	res=(this->countSides(index)==tes.at(index)->link.size());
	 //cout<<"node"<<index+1<<(res?"  true":" false")<<endl;
	 return res;
}
double Texture::getError(LinkData *  link1,LinkData * link2)
{
	double len1=link1->distance;
	double len2=link2->distance;
	double angle1=link1->angle;
	double angle2=link2->angle;
	int cid1 = link1->cid;
	int cid2 = link2->cid;
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
	if(cid1==cid2)
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

TextureElement * Texture::findOptimalMatchTextureElement(TextureElement *te, vector<int> &addList, vector<vector<int> >  &linkPairs)
{

	double minEorror = 9999;
	double eorror = 0;
	TextureElement  * res = NULL;
	vector<int>  f;
	vector<vector<int>>  l;
	int m_start = 0;
	int m_steps = 0;
	for (int i = 0; i < tes.size(); i++)
	{
		TextureElement * tei = tes.at(i);
		if (!isRingStruct(tei->index)) continue;
		eorror = differentBetweenTwoTextureElement(te, tei, f, l);
		if (eorror < minEorror)
		{
			minEorror = eorror;
			res = tei;
			addList.assign(f.begin(), f.end());
			linkPairs.assign(l.begin(), l.end());
		}
		f.clear();
		l.clear();
	}
	//确定Link基元的序号
	ofstream f1("textureElementLog.txt", ios::out);
	f1 << "error " << minEorror << endl;
	f1.close();

	return res;
}
double Texture::differentBetweenTwoTextureElement(TextureElement *te1, TextureElement *te2, vector<int> &addList, vector<vector<int> >  &linkPairs)//te2的连通信息完整，而te1的不完整，用te1的连通信息（4点，2个三角形）与te2的每个顶点局部连通关系（4点，2个三角形）作比较
{

	double error = 0;
	double minEorror = 9999;
	int size1 = te1->link.size();
	int size2 = te2->link.size();
	int i = 0;
	int j = 0;
	int tempError = 0;
	int tempMinError = 9999;
	int minJ;
	int matchSize, a0, ae, b0, be, _b0, _be, angA0, angAE, angB0, angBE, angelDis1, angelDis2, angelDis3, angelDis4;
	double angleError = 0;
	vector<IndexPair> matchIndexs;
	vector<IndexPair>   bestMatchIndexs;
	matchIndexs.clear();
	bool isAdded = false;
	bool isAddedToAddList = false;
	if (size1>size2)
	{
		return 9999;
	}
	//计算te1部分环的起始点
	int te1S = 0, te1E = 0;//部分环的起始点和终止点
	if (!this->getSLSE(te1, te1S, te1E))
	{
		return 9999;//如果没有部分环，那么停止匹配
	}
	for (int dj = 0; dj < size2; dj++)//te2是ring，有size个部分环
	{
		tempError = 0;
		for (int di = 0; di<size1; di++)//每个部分环有size1个元素
		{
			j = (di + dj) % size2;
			i = (di + te1S) % size1;
			tempError += getError(te1->link.at(i), te2->link.at(j));
			IndexPair ip;
			ip.sampleIndex = j;
			ip.targetIndex = i;
			matchIndexs.push_back(ip);

		}
		if (tempError < tempMinError)
		{
			angleError = 0;
			//判断是否可以作为扩张纹理元素
			/*if (size2 - size1 <= 1)
			{
			//待补全纹理元素只差一个相邻边，不用考虑
			}else
			{
			//如果要添加的边数大于2,则要保证te1的各个link角度位于te2的匹配min-1和max+1之间
			matchSize = matchIndexs.size();
			a0 = matchIndexs[0].targetIndex;
			ae = matchIndexs[matchSize-1].targetIndex;
			b0 = matchIndexs[0].sampleIndex;
			be = matchIndexs[matchSize - 1].sampleIndex;
			_b0 = b0 - 1;
			_be = be + 1;
			if (_b0 < 0)
			{
			_b0 = te2->link.size() - 1;
			}
			if (_be >= te2->link.size())
			{
			_be = 0;
			}
			angA0 = te1->link[a0]->angle;
			angAE = te1->link[ae]->angle;
			angB0 = te2->link[_b0]->angle;
			angBE = te2->link[_be]->angle;
			if (angAE>angA0)
			{
			if (angB0 > angA0 && angB0 < angAE)
			{
			angleError = 999;
			}
			if (angBE > angA0 && angBE < angAE)
			{
			angleError = 999;
			}
			}else
			{
			angleError = 999;
			if (angB0 > angA0 && angB0 < angAE)
			{
			angleError = 0;
			}
			if (angBE > angA0 && angBE < angAE)
			{
			angleError = 0;
			}
			}
			}*/
			//
			if (angleError == 0)
			{
				linkPairs.clear();
				bestMatchIndexs.clear();
				tempMinError = tempError;
				for (int it = 0; it < matchIndexs.size(); it++)
				{
					bestMatchIndexs.push_back(matchIndexs[it]);
					vector<int> linkPair;
					linkPair.push_back(matchIndexs[it].sampleIndex);
					linkPair.push_back(matchIndexs[it].targetIndex);
					linkPair.push_back(te1->link[matchIndexs[it].targetIndex]->index);//(顺序号,对应顶点号)
					linkPairs.push_back(linkPair);
				}
				matchIndexs.clear();
			}
			else
			{
				angleError += 1;
				matchIndexs.clear();
			}
		}
		else
		{
			matchIndexs.clear();
		}
	}
	addList.clear();
	for (int k = 0; k<size2; k++)
	{
		isAdded = false;
		for (int n = 0; n<bestMatchIndexs.size(); n++)
		{
			if (bestMatchIndexs.at(n).sampleIndex == k)
			{
				isAdded = true;
			}
		}
		if (isAdded)
		{

		}
		else
		{
			addList.push_back(k);
		}
	}
	return tempMinError;
}
void Texture::textureSort()
{
	for(int i=0;i<tes.size();i++)
	{
		tes.at(i)->textureElementSort();
	}
}
bool Texture::getSLSE(TextureElement * te, int &start, int &end)
{
	if (te->link.size() == 0)
	{
		return false;
	}
	else
	{
		if (te->link.size() == 1)
		{
			start = 0;
			end = 0;
			return false;
		}
		else
		{
			start = 0;
			end = 0;
			int i = 0;
			int j = 0;
			int linkSize = te->link.size();
			while (true)
			{
				j = (i + 1) % linkSize;
				if (!te->link[i]->linkElement->isLinkTo(te->link[j]->index))
				{
					end = i;
					for (int k = 0; k < linkSize - 1; k++)//走linkSize-1个间距，从j开始,必须全部连通
					{
						int jk1 = (j + k) % linkSize;
						int jk2 = (j + k + 1) % linkSize;
						if (!te->link[jk1]->linkElement->isLinkTo(te->link[jk2]->index))
						{
							return false;
						}
					}
					start = j;
					return true;
				}
				i = (i + 1) % linkSize;
			}
		}
	}
}
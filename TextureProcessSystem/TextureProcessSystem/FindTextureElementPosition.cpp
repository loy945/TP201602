#pragma once
#include "StdAfx.h"
#include "FindTextureElementPosition.h"
#include "cv.h"
#include "highgui.h"
#include <opencv2/opencv.hpp> 
#include "config.h"
#include "VectorIndexPair.h"
#include "V_calculate.h"
#include <fstream>
#include "Line.h"
using namespace std;
#define pi 3.1415926
using namespace cv;

FindTextureElementPosition::FindTextureElementPosition(void)
{
}


FindTextureElementPosition::~FindTextureElementPosition(void)
{
}

void FindTextureElementPosition::init(CTextureProcessSystemDoc * pDoc)
{
	this->pDoc=pDoc;
	gl_face * theFace;
	m_sampleTexture=new SampleTexture();
	m_sampleTexture->initSampleTextureFromFile("x.txt","y.txt","TRI.txt","cal.txt");
	m_sampleTexture->textureSort();
	m_targetTexture=new TargetTexture();

	//添加第一个target基元
	/*pDoc->plyLoader.faceArry.at(pDoc->userSelectingTriangleIndex).isShowColorIn3D=true;
	pDoc->plyLoader.faceArry.at(pDoc->userSelectingTriangleIndex).textureclick=true;*/
	pDoc->plyLoader.faceArry.at(pDoc->userSelectingTriangleIndex).r=1;
	pDoc->plyLoader.faceArry.at(pDoc->userSelectingTriangleIndex).g=0;
	pDoc->plyLoader.faceArry.at(pDoc->userSelectingTriangleIndex).b=0;
	
	//计算第一个基元位置
	Point3D p1, p2, p3,pCore;//三角形三个顶点
	p1.x = pDoc->plyLoader.pointArry[pDoc->plyLoader.faceArry.at(pDoc->userSelectingTriangleIndex).ptnum[0]].x;
	p1.y = pDoc->plyLoader.pointArry[pDoc->plyLoader.faceArry.at(pDoc->userSelectingTriangleIndex).ptnum[0]].y;
	p1.z = pDoc->plyLoader.pointArry[pDoc->plyLoader.faceArry.at(pDoc->userSelectingTriangleIndex).ptnum[0]].z;
	p2.x = pDoc->plyLoader.pointArry[pDoc->plyLoader.faceArry.at(pDoc->userSelectingTriangleIndex).ptnum[1]].x;
	p2.y = pDoc->plyLoader.pointArry[pDoc->plyLoader.faceArry.at(pDoc->userSelectingTriangleIndex).ptnum[1]].y;
	p2.z = pDoc->plyLoader.pointArry[pDoc->plyLoader.faceArry.at(pDoc->userSelectingTriangleIndex).ptnum[1]].z;
	p3.x = pDoc->plyLoader.pointArry[pDoc->plyLoader.faceArry.at(pDoc->userSelectingTriangleIndex).ptnum[2]].x;
	p3.y = pDoc->plyLoader.pointArry[pDoc->plyLoader.faceArry.at(pDoc->userSelectingTriangleIndex).ptnum[2]].y;
	p3.z = pDoc->plyLoader.pointArry[pDoc->plyLoader.faceArry.at(pDoc->userSelectingTriangleIndex).ptnum[2]].z;
	pCore = (p1 + p2 + p3) / 3.0;

	float corx = pDoc->plyLoader.faceArry.at(pDoc->userSelectingTriangleIndex).corex;
	float cory = pDoc->plyLoader.faceArry.at(pDoc->userSelectingTriangleIndex).corey;
	float corz = pDoc->plyLoader.faceArry.at(pDoc->userSelectingTriangleIndex).corez;
	firstPoint.x = corx;
	firstPoint.y = cory;
	firstPoint.z = corz;
	//

 	TextureElement * te = new TextureElement(0, &(pDoc->plyLoader.faceArry.at(pDoc->userSelectingTriangleIndex)), 1);
	initElements = te;


	m_targetTexture->addTE(te);
	targetCenter=te;
	getCoverRange(targetCenter, 0.08);
	setN(targetCenter);
	smapleCenter=m_sampleTexture->findFirstTextrueElement();
	te->textureIndex = smapleCenter->textureIndex;
	te->cid = smapleCenter->cid;

	//***************************
	P = addPointToSelectList(pDoc->markConnectedFace(0, 0, selectRadius));
	for(int i=0;i<smapleCenter->link.size();i++)
	{
		LinkData *  ld=smapleCenter->link.at(i);
		//theFace=findFace(ld->distance,ld->angle,te,pDoc->markConnectedFace(0,0,1000));
		Point3D pt(0,0,0);
		getPoint(ld->distance, ld->angle, te, P, pt);
		theFace = &pDoc->plyLoader.faceArry[pt.beLongFaceID];
		TextureElement * tex = new TextureElement(m_targetTexture->tes.size(), theFace, 1, pt);
		getCoverRange(tex, 0.08);
		setN(tex);
		tex->cid = ld->cid;
		tex->textureIndex = ld->textureIndex;
		m_targetTexture->addTE(tex);
		m_targetTexture->addLink(targetCenter,tex);
	}
	//补全连通信息
	vector<int> matchF;
	vector<vector<int> >  linkPairs;
	for (int i = 0; i<targetCenter->link.size(); i++)
	{
		vector<int> a;
		a.push_back(i);
		a.push_back(i);
		a.push_back(i+1);
		matchF.push_back(i);
		linkPairs.push_back(a);
	}
	addlinks(targetCenter, matchF, linkPairs);
	markCoveredFace(targetCenter);
	//统计个数
	int w=0;
	if(m_targetTexture->tes.size()>0)
	{
		w++;
	}
	m_targetTexture->textureSort();
	
	return;


}


void FindTextureElementPosition::getDR(DistanceRecord * pDR)
{
	this->m_pDR = pDR;
}
void FindTextureElementPosition::getPoint(double dis, double angel, TextureElement * centerTE, vector<Point3D> P,Point3D &res)
{
	double error = 999;
	double minerror = 999;
	

	for (int i = 0; i < P.size();i++)
	{	
		error = getPointError(dis, angel, centerTE, P[i]);
		if (minerror>error)
		{
			minerror = error;
			res.setValue(P[i]);
		}
	}

	return ;
}
double FindTextureElementPosition::getPointError(double dis, double angel, TextureElement * centerTE, Point3D p)
{
	double k1 = 50;
	double k2 = 1;
	double distanceError;
	float pos[3];
	pos[0] = p.x;
	pos[1] = p.y;
	pos[2] = p.z;

	float disfromCenter = centerTE->getDisFrom(pos);
	distanceError = abs(dis - (disfromCenter * lenFactor));

	double angleError;
	//投影到中心平面求角度
	float angleFromCenter = centerTE->getAngleFrom(pos);
	//投影到折中平面求角度
	//float angleFromCenter = centerTE->getAngleFrom(pos,centerTE->n);

	angleError = abs(angel - angleFromCenter);
	if (angleError > 180)
	{
		angleError = 360 - angleError;
	}
	double error = k1*distanceError + k2*angleError;
	return error;
}
vector<Point3D> FindTextureElementPosition::addPointToSelectList(vector<gl_face *> F)
{
	Point3D pt[10];
	gl_face * f;
	vector<Point3D> res;
	for (int i = 0; i < F.size(); i++)
	{
		 f = F[i];
		//三角形面片的三个点
		for (int k = 0; k < 3; k++)
		{
			pt[k].x = pDoc->plyLoader.pointArry.at(f->ptnum[k]).x;
			pt[k].y = pDoc->plyLoader.pointArry.at(f->ptnum[k]).y;
			pt[k].z = pDoc->plyLoader.pointArry.at(f->ptnum[k]).z;
		}
		//面片的重心
		pt[3] = (pt[0] + pt[1] + pt[2]) / 3.0;
		//边中点
		pt[4] = (pt[0] + pt[1]) / 2.0;
		pt[5] = (pt[1] + pt[2]) / 2.0;
		pt[6] = (pt[2] + pt[0]) / 2.0;
		//顶点与重心的连线中心点
		pt[7] = (pt[3] + pt[0]) / 2.0;
		pt[8] = (pt[3] + pt[1]) / 2.0;
		pt[9] = (pt[3] + pt[2]) / 2.0;

		for (int k = 0; k < 10; k++)
		{
			pt[k].beLongFaceID = f->facenum;
			res.push_back(pt[k]);
		}



	}
	return res;
}
void FindTextureElementPosition::draw()
{
	float w = 0.009, h = 0.009, d = 0.009;
	float x, y, z = 0;
	int i = 0, j = 0;

	glColor3f(1, 0, 1);
	glBegin(GL_TRIANGLES);
	Point3D p1, p2, p3,pCore;//三角形三个顶点
	p1.x = pDoc->plyLoader.pointArry[pDoc->plyLoader.faceArry.at(pDoc->userSelectingTriangleIndex).ptnum[0]].x;
	p1.y = pDoc->plyLoader.pointArry[pDoc->plyLoader.faceArry.at(pDoc->userSelectingTriangleIndex).ptnum[0]].y;
	p1.z = pDoc->plyLoader.pointArry[pDoc->plyLoader.faceArry.at(pDoc->userSelectingTriangleIndex).ptnum[0]].z;
	p2.x = pDoc->plyLoader.pointArry[pDoc->plyLoader.faceArry.at(pDoc->userSelectingTriangleIndex).ptnum[1]].x;
	p2.y = pDoc->plyLoader.pointArry[pDoc->plyLoader.faceArry.at(pDoc->userSelectingTriangleIndex).ptnum[1]].y;
	p2.z = pDoc->plyLoader.pointArry[pDoc->plyLoader.faceArry.at(pDoc->userSelectingTriangleIndex).ptnum[1]].z;
	p3.x = pDoc->plyLoader.pointArry[pDoc->plyLoader.faceArry.at(pDoc->userSelectingTriangleIndex).ptnum[2]].x;
	p3.y = pDoc->plyLoader.pointArry[pDoc->plyLoader.faceArry.at(pDoc->userSelectingTriangleIndex).ptnum[2]].y;
	p3.z = pDoc->plyLoader.pointArry[pDoc->plyLoader.faceArry.at(pDoc->userSelectingTriangleIndex).ptnum[2]].z;
	glVertex3f(p1.x, p1.y, p1.z);
	glVertex3f(p2.x, p2.y, p2.z);
	glVertex3f(p3.x, p3.y, p3.z);
	glEnd();

// 	for (i = 0; i < m_targetTexture->tes.size(); i++)
// 	{
// 		if (!m_targetTexture->tes[i]->isShow)
// 			continue;
// 		//画基元法线
// 		glColor3f(1, 1, 0);
// 		glLineWidth(1.0);
// 		glBegin(GL_LINES);
// 		glVertex3fv(m_targetTexture->tes[i]->pos);
// 		double x = m_targetTexture->tes[i]->pos[0] + m_targetTexture->tes[i]->n[0] ;
// 		double y = m_targetTexture->tes[i]->pos[1] + m_targetTexture->tes[i]->n[1];
// 		double z = m_targetTexture->tes[i]->pos[2] + m_targetTexture->tes[i]->n[2];
// 		glVertex3f(x,y,z);
// 		glEnd();
// 	}
	
	glColor3f(0, 0, 1);
	glBegin(GL_QUADS);

	for (i = 0; i < m_targetTexture->tes.size(); i++)
	{
		if (!m_targetTexture->tes[i]->isShow)
			continue;

		if (m_targetTexture->tes[i]->index == targetCenter->index)
		{
			glColor3f(1, 1, 1);
		}
		else if (m_targetTexture->tes[i]->isfixed)
		{
			glColor3f(0, 1, 0);
		}
		else 
		{
			bool latestaddin = false;
			for (j = 0; j < latestAddIn.size(); j++)
			{
				if (latestAddIn[j] == m_targetTexture->tes[i]->index)
				{
					latestaddin = true;
				}
			}
			if (latestaddin)
			{
				glColor3f(1, 0, 0);
			}
			else
			{
				glColor3f(0, 0, 1);
			}
			
		}
		x = m_targetTexture->tes[i]->pos[0];
		y = m_targetTexture->tes[i]->pos[1];
		z = m_targetTexture->tes[i]->pos[2];

	
		//front
		glVertex3f(x - w, y + h, z + d);
		glVertex3f(x - w, y - h, z + d);
		glVertex3f(x + w, y - h, z + d);
		glVertex3f(x + w, y + h, z + d);

		//bottom
		glVertex3f(x - w, y - h, z + d);
		glVertex3f(x + w, y - h, z + d);
		glVertex3f(x + w, y - h, z - d);
		glVertex3f(x - w, y - h, z - d);

		//back
		glVertex3f(x + w, y + h, z - d);
		glVertex3f(x + w, y - h, z - d);
		glVertex3f(x - w, y - h, z - d);
		glVertex3f(x - w, y + h, z - d);

		//top
		glVertex3f(x - w, y + h, z + d);
		glVertex3f(x + w, y + h, z + d);
		glVertex3f(x + w, y + h, z - d);
		glVertex3f(x - w, y + h, z - d);

		//left
		glVertex3f(x - w, y + h, z + d);
		glVertex3f(x - w, y - h, z + d);
		glVertex3f(x - w, y - h, z - d);
		glVertex3f(x - w, y + h, z - d);

		//right
		glVertex3f(x + w, y + h, z + d);
		glVertex3f(x + w, y - h, z + d);
		glVertex3f(x + w, y - h, z - d);
		glVertex3f(x + w, y + h, z - d);

	}
	glColor3f(1, 0, 1);
	w = 0.001, h = 0.001, d = 0.001;
	for (i = 0; i < P.size(); i++)
	{
		x = P[i].x;
		y = P[i].y;
		z = P[i].z;


		//front
		glVertex3f(x - w, y + h, z + d);
		glVertex3f(x - w, y - h, z + d);
		glVertex3f(x + w, y - h, z + d);
		glVertex3f(x + w, y + h, z + d);

		//bottom
		glVertex3f(x - w, y - h, z + d);
		glVertex3f(x + w, y - h, z + d);
		glVertex3f(x + w, y - h, z - d);
		glVertex3f(x - w, y - h, z - d);

		//back
		glVertex3f(x + w, y + h, z - d);
		glVertex3f(x + w, y - h, z - d);
		glVertex3f(x - w, y - h, z - d);
		glVertex3f(x - w, y + h, z - d);

		//top
		glVertex3f(x - w, y + h, z + d);
		glVertex3f(x + w, y + h, z + d);
		glVertex3f(x + w, y + h, z - d);
		glVertex3f(x - w, y + h, z - d);

		//left
		glVertex3f(x - w, y + h, z + d);
		glVertex3f(x - w, y - h, z + d);
		glVertex3f(x - w, y - h, z - d);
		glVertex3f(x - w, y + h, z - d);

		//right
		glVertex3f(x + w, y + h, z + d);
		glVertex3f(x + w, y - h, z + d);
		glVertex3f(x + w, y - h, z - d);
		glVertex3f(x + w, y + h, z - d);

	}

	glEnd(); // 四边形绘制结束

	//画线
// 	glColor3f(1, 1, 1);
// 	glBegin(GL_LINES);
// 	for (i = 0; i < m_targetTexture->lines.size(); i++)
// 	{
// 		if (m_targetTexture->lines[i].startElement->isShow&&m_targetTexture->lines[i].endElement->isShow)
// 		{
// 		glVertex3f(m_targetTexture->lines[i].start.x, m_targetTexture->lines[i].start.y, m_targetTexture->lines[i].start.z);
// 		glVertex3f(m_targetTexture->lines[i].end.x, m_targetTexture->lines[i].end.y, m_targetTexture->lines[i].end.z);
// 		}
// 	}
// 	glEnd();
	glFlush();//立即渲染
}
bool FindTextureElementPosition::amendTargetTE(TextureElement * te)
{
	if (!te)
	{
		return false;
	}
	for (int i = 0; i < te->link.size(); i++)
	{
		te->link[i]->angle = te->getAngleFrom(te->link[i]->linkElement->pos);
		te->link[i]->distance = te->getDisFrom(te->link[i]->linkElement->pos);
	}
	return true;
}
bool FindTextureElementPosition::getCoverRange(TextureElement * te, double radius)
{
	if (!te)
	{
		return false;
	}
	int faceNum = te->face->facenum;

	vector<int> faces;
	vector<float> distancesFromFace1;
	vector<int> facesDetected;

	faces.push_back(faceNum);
	float zeroDistance = 0;
	distancesFromFace1.push_back(zeroDistance);
	while (faces.size() > 0)
	{
		int face = faces[0];
		float dis = distancesFromFace1[0];
		for (int j = 0; j < m_pDR->m_triangleFaceArry[face].faceNearByNums; j++)
		{
			int num = m_pDR->m_triangleFaceArry[face].faceNearByIndex[j];
			float distanceFromFace1 = dis + m_pDR->m_triangleFaceArry[face].faceNearByDistance[j];
			bool isAddedIn = false;
			for (int k = 0; k < facesDetected.size(); k++)
			{
				if (num == facesDetected[k])
				{
					isAddedIn = true;
				}
			}
			if (!isAddedIn)
			{
				int facei = num;
				if (distanceFromFace1 < radius)
				{
					faces.push_back(facei);
					distancesFromFace1.push_back(distanceFromFace1);
				}
			}
		}
		if (faces.size()>0)
		{
			faces.erase(faces.begin());
		}
		if (distancesFromFace1.size() > 0)
		{
			distancesFromFace1.erase(distancesFromFace1.begin());
		}
		facesDetected.push_back(face);
		te->coverFace.push_back(&pDoc->plyLoader.faceArry[face]);
	}


	return true;

}
bool FindTextureElementPosition::detectRegion(TextureElement * te)
{
	if (!te)
	{
		return false;
	}
	int faceNum = te->face->facenum;

	vector<int> faces;
	vector<float> distancesFromFace1;
	vector<int> facesDetected;	

	faces.push_back(faceNum);
	float zeroDistance = 0;
	distancesFromFace1.push_back(zeroDistance);
	while (faces.size() > 0)
	{
		int face = faces[0];
		float dis = distancesFromFace1[0];

		//检测这个面是否有基元
		for (int i = 0; i < m_targetTexture->tes.size()-1; i++)
		{
			if (m_targetTexture->tes[i]->face->facenum == face)
			{
				return false;
			}
		}
		for (int j = 0; j < m_pDR->m_triangleFaceArry[face].faceNearByNums; j++)
		{
			int num = m_pDR->m_triangleFaceArry[face].faceNearByIndex[j];
			float distanceFromFace1 = dis + m_pDR->m_triangleFaceArry[face].faceNearByDistance[j];
			bool isAddedIn = false;
			for (int k = 0; k < facesDetected.size(); k++)
			{
				if (num == facesDetected[k])
				{
					isAddedIn = true;
				}
			}
			if (!isAddedIn)
			{
				int facei = num;			
				if (distanceFromFace1 < minDis)
				{
					faces.push_back(facei);
					distancesFromFace1.push_back(distanceFromFace1);
				}
			}
		}
		if (faces.size()>0)
		{
			faces.erase(faces.begin());
		}		
		if (distancesFromFace1.size()>0)
		{
			distancesFromFace1.erase(distancesFromFace1.begin());
		}		
		facesDetected.push_back(face);
	}

	return true;


}
void FindTextureElementPosition::setN(TextureElement * targetCenter)
{
	int coverSize = targetCenter->coverFace.size();
	for (int k = 0; k < 3; k++)
	{
		targetCenter->n[k] = 0;
	}
	for (int it = 0; it < coverSize; it++)
	{
		for (int k = 0; k < 3; k++)
		{
			targetCenter->n[k] += targetCenter->coverFace[it]->n[k];
		}
	}
	for (int k = 0; k < 3; k++)
	{
		targetCenter->n[k] = targetCenter->n[k] / coverSize;
	}
}
bool FindTextureElementPosition::buildTargetTextureElement(TextureElement * centerTE, vector<int> &matchF, vector<vector<int> >  &linkPairs)
{
	targetCenter = centerTE;
	if (!centerTE)
	{
		return false;
	}	
	if (targetCenter->linkSize>1)
	{
		centerTE->textureElementSort();
	}
	
	//修正当前基元的link数据
	amendTargetTE(centerTE);
	latestAddIn.clear();
	gl_face * theFace;
	TextureElement * p_OMTE;
	//分布最匹配的样本基元
	p_OMTE = m_sampleTexture->findOptimalMatchTextureElement(centerTE, matchF, linkPairs);
	if (!p_OMTE)
	{
		centerTE->isfixed = true;
		return false;
	}
	//print 日志
	ofstream f("textureElementLog.txt", ios::app);
	for (int k = 0; k < m_targetTexture->tes.size(); k++)
	{
		f << "targetTextureID	" << m_targetTexture->tes[k]->index<<endl;
		f << "	linkData:" << endl;
		for (int i = 0; i < m_targetTexture->tes[k]->link.size();i++)
		{
			f << "	"<<m_targetTexture->tes[k]->link[i]->index << "," << m_targetTexture->tes[k]->link[i]->angle<<endl;
		}
	}
	f << endl;
	f << "centerTE	" << centerTE->index << endl;
	f << "	linkData:" << endl;
	for (int i = 0; i <centerTE->link.size(); i++)
	{
		f << "	" << centerTE->link[i]->index << " " << centerTE->link[i]->angle << endl;
	}
	f << endl;	
	f << "p_OMTE	" << p_OMTE->index << endl;
	f << "	linkData:" << endl;
	for (int i = 0; i <p_OMTE->link.size(); i++)
	{
		f << "	" << p_OMTE->link[i]->index << " " << p_OMTE->link[i]->angle << endl;
	}
	f << endl;
	//matchF
	f << "matchF" << endl;
	for (int i = 0; i <matchF.size(); i++)
	{
		f << "	" << matchF[i] << endl;
	}
	f << endl;

	


	//顶点候选集
	P.clear();
	pDoc->userSelectingTriangleIndex = centerTE->face->facenum;
	P = addPointToSelectList(pDoc->markConnectedFace(0, 0, selectRadius));
	//开始添加
	for (int i = 0; i < matchF.size(); i++)
	{
		LinkData *ld = p_OMTE->link[matchF[i]];
		Point3D pt(0, 0, 0);
		getPoint(ld->distance, ld->angle, centerTE, P, pt);
		theFace = &pDoc->plyLoader.faceArry[pt.beLongFaceID];
		int newElementIndex = m_targetTexture->tes.size();
		TextureElement * tex = NULL;	
		tex = new TextureElement(newElementIndex, theFace, 1, pt);	
		getCoverRange(tex, 0.08);
		setN(tex);
		tex->cid = ld->cid;
		tex->textureIndex = ld->textureIndex;
		m_targetTexture->addTE(tex);
		m_targetTexture->addLink(targetCenter, tex);
		latestAddIn.push_back(tex->index);
		vector<int> linkPair;
		linkPair.push_back(matchF[i]);
		linkPair.push_back(matchF[i]);//这里应该要输入sample里面匹配link序号,暂时不处理
		linkPair.push_back(tex->index);
		linkPairs.push_back(linkPair);
		if (!detectRegion(tex))
		{
			tex->isfixed = true;
			tex->isShow = true;
		}
		if (m_pDR->m_triangleFaceArry[pt.beLongFaceID].isMark)
		{
			tex->isfixed = true;
			tex->isShow = true;
		}
	}
	//linkPairs
	f << "linkPairs" << endl;
	for (int i = 0; i <linkPairs.size(); i++)
	{
		f << "	" << linkPairs[i][0] << " " << linkPairs[i][1] << " " << linkPairs[i][2] << endl;
	}
	f << endl;
	f.close();
	return true;
}
void FindTextureElementPosition::addlinks(TextureElement * centerTE, vector<int> &matchF,vector<vector<int> >  &linkPairs)
{
	int i = 0, j = 0,k = 0;
	/*for (i = 0; i < centerTE->link.size(); i++)
	{
		j = i + 1;
		if (j >= centerTE->link.size())
		{LINK
			j = 0;
		}
		if (centerTE->link[i]->linkElement->isShow&&centerTE->link[j]->linkElement->isShow)
		{
			m_targetTexture->addLink(centerTE->link[i]->linkElement, centerTE->link[j]->linkElement);
		}
		
	}*/
	int index = 0;
	int pindex = 0;
	int aindex = 0;
	for (i = 0; i < matchF.size(); i++)
	{
		j = matchF[i] - 1;
		if (j < 0)
		{
			j = linkPairs.size() - 1;
		}
		k = matchF[i] + 1;
		if (k>linkPairs.size() - 1)
		{
			k = 0;
		}
		VectorIndexPair(linkPairs, matchF[i], 2, index);
		VectorIndexPair(linkPairs, j, 2, pindex);
		VectorIndexPair(linkPairs, k, 2, aindex);

 		m_targetTexture->addLink(m_targetTexture->tes[index], m_targetTexture->tes[pindex]);
		m_targetTexture->addLink(m_targetTexture->tes[index], m_targetTexture->tes[aindex]);

	}
 	centerTE->isfixed = true;
}
TextureElement * FindTextureElementPosition::theNearsetTE()
{
	float distance = 9999, minDistance = 9999;
	TextureElement * res=nullptr;
	for (int i = 0; i < m_targetTexture->tes.size(); i++)
	{
		if (!m_targetTexture->tes[i]->isfixed)
		{
			Point3D pt;
			pt .x= m_targetTexture->tes[i]->pos[0];
			pt. y= m_targetTexture->tes[i]->pos[1];
			pt. z= m_targetTexture->tes[i]->pos[2];
			distance = (pt - firstPoint).getDistance();
			int faceNum = m_targetTexture->tes[i]->face->facenum;
			if (distance < minDistance)
			{
				minDistance = distance;
				res = m_targetTexture->tes[i];
			}
		}
	}
	return res;
}

bool FindTextureElementPosition::buildByStep()
{
	TextureElement * nexte = nullptr;
	nexte = theNearsetTE();
	if (!nexte)
	{
		return false;
	}
	
	vector<int> matchF;
	vector<vector<int> >  linkPairs;
	//生成新基元
	if (buildTargetTextureElement(nexte, matchF, linkPairs))
	{
		//nexte->textureElementSort();
		//amend();
		//添加连通关系以及连通边
		//m_targetTexture->textureSort();
		addlinks(targetCenter, matchF, linkPairs);
		//冲突检测
		//detectCross1(targetCenter);
		//重叠检测
		//detectLapped();
		//排序
		//标记覆盖的面
		nexte->textureElementSort();
		markCoveredFace(targetCenter);
		
	}
}
void FindTextureElementPosition::amend()
{
	Point3D pt1;
	Point3D pt2;
	for (int i = 0; i < latestAddIn.size(); i++)
	{
		int index = latestAddIn[i];
		TextureElement * te = m_targetTexture->tes[index];
		pt1.x = te->pos[0];
		pt1.y = te->pos[1];
		pt1.z = te->pos[2];
		for (int j = 0; j < m_targetTexture->tes.size(); j++)
		{
			if (m_targetTexture->tes[j]->index == te->index)
				continue;
			pt2.x = m_targetTexture->tes[j]->pos[0];
			pt2.y = m_targetTexture->tes[j]->pos[1];
			pt2.z = m_targetTexture->tes[j]->pos[2];
			Point3D pt3 = pt1 - pt2;
			float dis = pt3.getDistance();
			if (dis < minTEDistance)
			{
				te->isfixed = true;
				te->isShow = false;
			}
		}
	}

}
void FindTextureElementPosition::detectLapped()
{
	for (int i = 0; i < m_targetTexture->tes.size(); i++)
	{
		if (m_targetTexture->tes[i]->isfixed==false)
		{
			for (int j = 0; j < m_targetTexture->tes.size(); j++)
			{
				if (m_targetTexture->tes[j]->isfixed == true)
				{
					TextureElement * te1 = m_targetTexture->tes[i];
					TextureElement * te2 = m_targetTexture->tes[j];
					if (te1->face->facenum == te2->face->facenum)
					{
						continue;
					}
					Point3D pt1;
					Point3D pt2;
					pt1.x = te1->pos[0];
					pt1.y = te1->pos[1];
					pt1.z = te1->pos[2];
					pt2.x = te2->pos[0];
					pt2.y = te2->pos[1];
					pt2.z = te2->pos[2];
					//i没固定 j已固定
					//检测i j 之间的距离是否过近
 					float dis = m_pDR->getGeodesicDistanceBetweenTwoTrianglesWithBEPoint(te1->face->facenum, te2->face->facenum, pt1, pt2);
					if (dis<minTEDistance)
					{
						te1->isfixed = true;
						break;
					}
					
				}
			}
		}		
	}

}
void FindTextureElementPosition::detectCross1(TextureElement * te)
{
	int i, j;

	i = 0;
	for (j = 0; j<targetCenter->link.size(); j++)
	{
		//中心te为起点，第j个邻域基元tte为终点，
		LinkData * cn = targetCenter->link.at(j);
		TextureElement * tte = cn->linkElement;
		Line l1;
		l1.startElement = targetCenter;
		l1.endElement = tte;
		l1.start.x = targetCenter->pos[0];
		l1.start.y = targetCenter->pos[1];
		l1.start.z = targetCenter->pos[2];
		l1.end.x = tte->pos[0];
		l1.end.y = tte->pos[1];
		l1.end.z = tte->pos[2];

		//与所有line求交点
		for (i = 0; i<m_targetTexture->lines.size(); i++)
		{
			if (l1.isCrossIn3DWithN(m_targetTexture->lines.at(i),targetCenter->n))
			{
				tte->isfixed = true;
			}
		}

		/*m_targetTexture->addLines(l1);*/
	}
	return;
}
void FindTextureElementPosition::markCoveredFace(TextureElement * te)
{
	if (!te->isfixed)
	{
		return;
	}
	int i = 0;
	int linkSize=te->link.size();
	if (linkSize <= 0)
	{
		return;
	}
	//中心法线
	float centerN[3];
	centerN[0] = te->n[0];
	centerN[1] = te->n[1];
	centerN[2] = te->n[2];
	float centerN4[4];
	centerN4[0] = te->n[0];
	centerN4[1] = te->n[1];
	centerN4[2] = te->n[2];
	centerN4[3] = 1;
	//中心位置
	float centerPos[3];
	centerPos[0] = te->pos[0];
	centerPos[1] = te->pos[1];
	centerPos[2] = te->pos[2];

	float cosX, sinX, cosY, sinY;
	CV_calculate::getRotateAngle(centerN4, cosX, sinX, cosY, sinY);
	//将相邻元素投影、旋转到XOY平面上
	Point3D * linkElementPos = new Point3D[te->link.size()];
	//获取每一个相邻元素位置,并存储
	float vert[20][3];//每一个vert[i][0],vert[i][1],保存这个顶点投转后的x,y值
	int vertNum = 0;
	float range = 0;
	for (int i = 0; i < te->link.size(); i++)
	{	
		TextureElement * tei = te->link[i]->linkElement;
		linkElementPos[i] = tei->pos;
		float facePos[4];
		facePos[0] = linkElementPos[i].x;
		facePos[1] = linkElementPos[i].y;
		facePos[2] = linkElementPos[i].z;
		facePos[3] = 1;
		float res1[4] = { 0, 0, 0, 0 };
		float res2[4] = { 0, 0, 0, 0 };
		//te->projection(te->n, facePos, te->pos, res1); // 此处element的法向量n是平滑setN()过的		
		CV_calculate::rotateWith(facePos, res1, cosX, sinX, cosY, sinY, centerPos[0], centerPos[1], centerPos[2]);
		CV_calculate::projectToXoY(res1, res2);
		vert[i][0] = res2[0];
		vert[i][1] = res2[1];
		vert[i][2] = 0;
		vertNum++;
		float dis=sqrt(pow(vert[i][0], 2) + pow(vert[i][1], 2));
		if (dis>range)
		{
			range = dis;
		}
	}
	
	//从te->face开始，广度优先遍历
	vector<int> processingIndex;//每次从中选取一个最近的面
	vector<float> processingdistance;//对应processingIndex
	vector<int> processed;//遍历过的存在这，所有面只遍历一次
	vector<float> processedistance;//对应processedIndex的距离
	//初始化
	processingIndex.push_back(te->face->facenum);
	float distance = 0;
	processingdistance.push_back(distance);
	while (processingIndex.size()>0)
	{
		//找到distance最小的
		float minDistane = 999;
		float minIndex = 0;//对应最小距离面片的序号
		for (int i = 0; i < processingdistance.size(); i++)
		{
			if (processingdistance[i]<minDistane)
			{
				minDistane = processingdistance[i];
				minIndex = i;
			}
		}
		//处理完之后出队
		int sIndex = processingIndex[minIndex];
		processingIndex.erase(processingIndex.begin() + minIndex);
		processingdistance.erase(processingdistance.begin() + minIndex);
		//判断是否在范围中
		if (minDistane<=range)
		{
			//将这个面的序号和距离加入已处理队列
			processed.push_back(sIndex);
			processedistance.push_back(minDistane);
			//把他相邻的所有面加入待处理队列
			TriangleFace * f = &m_pDR->m_triangleFaceArry[sIndex];
			f->isMark = true;
			for (int j = 0; j < f->faceNearByNums;j++)
			{
				int index = f->faceNearByIndex[j];
				TriangleFace * tf = &m_pDR->m_triangleFaceArry[index];
				//判断这个面是否已经处理或正在处理
				bool isAddIn = false;
				for (int i = 0; i < processingIndex.size(); i++)
				{
					if (processingIndex[i]==tf->facenum)
					{
						isAddIn = true;
					}
				}
				for (int i = 0; i < processed.size(); i++)
				{
					if (processed[i] == tf->facenum)
					{
						isAddIn = true;
					}
				}
				//
				if (!isAddIn)
				{
					float facePos[4], res1[4], res2[4];
					facePos[0] = tf->corex;
					facePos[1] = tf->corey;
					facePos[2] = tf->corez;
					facePos[3] = 1;
					CV_calculate::rotateWith(facePos, res1, cosX, sinX, cosY, sinY, centerPos[0], centerPos[1], centerPos[2]);
					CV_calculate::projectToXoY(res1, res2);
					float tfdis = sqrt(pow(res2[0], 2) + pow(res2[1], 2));
					res2[2] = 0;
					//判断这个面是否在连线范围内
					bool inArea = true;
					for (int i = 0; i < vertNum; i++)
					{
						int p = i + 1;
						if (p >= vertNum)
						{
							p = 0;
						}
						float posZero[3] = { 0, 0, 0 };
						if (CV_calculate::isCross(vert[i], vert[p], res2, posZero))
						{
							inArea = false;
						}
					}
					if (inArea)
					{
						//把这个三角形的序号和投影旋转后里中心的距离加入待处理队列
						processingIndex.push_back(index);
						processingdistance.push_back(tfdis);
					}					
				}				
			}
		}
		else
		{
			int d = 0;
			d = 1;
		}
	}
}

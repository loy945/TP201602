#pragma once
#include "TextureProcessSystemDoc.h"
#include "TargetTexture.h"
#include "SampleTexture.h"
#include "Point3D.h"
#include "plyloader.h"
#include "DistanceRecord.h"
class FindTextureElementPosition
{
public:
	FindTextureElementPosition(void);
	~FindTextureElementPosition(void);
	void init(CTextureProcessSystemDoc * pDoc);
	void getDR(DistanceRecord * pDR);
	void addTargetTE(gl_face * face,int textureIndex);
	void getPoint(double dis, double angel, TextureElement * centerTE, vector<Point3D> P, Point3D &res);
	double getPointError(double dis, double angel, TextureElement * centerTE, Point3D P);
	vector<Point3D> addPointToSelectList(vector<gl_face *> F);
	bool buildTargetTextureElement(TextureElement * centerTE, vector<int> &matchF,vector<vector<int> >  &linkPairs);
	void addlinks(TextureElement * centerTE, vector<int> &matchF,vector<vector<int> >  &linkPairs);//对于所有种子位置都已固定的基元，补齐所有二形边
	TextureElement * theNearsetTE();
	bool buildByStep();
	void amend();
	bool amendTargetTE(TextureElement * te);//修正当前基元的link数据
	void detectCross1(TextureElement * te);//检测1型边冲突,以te为中心的一圈基元，若一型边有交叉则对应基元停止扩张
	void detectLapped();
	bool detectRegion(TextureElement * te);
	bool getCoverRange(TextureElement * te, double radius);
	void setN(TextureElement * te);
	void markCoveredFace(TextureElement * te);
public:
	SampleTexture * m_sampleTexture;
	TargetTexture * m_targetTexture;
	TextureElement * smapleCenter;
	TextureElement * targetCenter;
public:
	CTextureProcessSystemDoc * pDoc;
	DistanceRecord * m_pDR;
	vector<Point3D> P;
	Point3D firstPoint;
	TextureElement * initElements;
	vector<int> latestAddIn;
public:
	void draw();
};


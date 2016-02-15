#pragma once
#include "Texture.h"

class SampleTexture :public Texture
{
public:
	SampleTexture(void);
	~SampleTexture(void);
	bool initSampleTextureFromFile(const char * fileNameX, const char * fileNameY, const char * fileNameTRI, const char * fileNameCAL);
	TextureElement * findFirstTextrueElement();
	virtual bool addLink(TextureElement *te1,TextureElement *te2);
	void addDelaunayTRI(TextureElement *te1, TextureElement *te2, TextureElement *te3);
};


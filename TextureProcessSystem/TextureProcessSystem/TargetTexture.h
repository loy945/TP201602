#pragma once
#include "texture.h"
class TargetTexture :
	public Texture
{
public:
	TargetTexture(void);
	~TargetTexture(void);
	bool addLink(TextureElement *te1,TextureElement *te2);
	bool deleteLink(TextureElement *te1, TextureElement *te2);
};


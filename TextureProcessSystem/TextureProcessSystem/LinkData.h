#pragma once
class TextureElement;
class LinkData
{
public:
	LinkData(void);
	LinkData(const LinkData & t)
	{
		this->index = t.index;
		this->distance = t.distance;
		this->angle = t.angle;
		this->linkElement = t.linkElement;
		this->cid = t.cid;
		this->textureIndex = t.textureIndex;
	}
	~LinkData(void);
public:
	int index;
	double distance;
	double angle;
	TextureElement * linkElement;
	int cid;
	int textureIndex;
public:
	bool isLinkTo(TextureElement * linkElement)
	{
		if (this->linkElement == linkElement)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
public:
	bool operator == (const int index)
	{
		if(this->index==index)
		{
			return true;
		}else
		{
			return false;
		}
	}
};


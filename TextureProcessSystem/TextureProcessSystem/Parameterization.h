#include "plyloader.h"
class Parameterization
{
public:
	Parameterization(){ return; }
	~Parameterization(){}
	void setparameterization(char * inPLY2, char * outPLY2);
	void face_Parameterization(Model_PLY * ply, vector<int> faceIndexs);
	
};
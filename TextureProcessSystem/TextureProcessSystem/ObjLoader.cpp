#include "StdAfx.h"
#include "ObjLoader.h"
#include <fstream>
#include<sstream> 
using namespace std;
ObjLoader::ObjLoader()
{
}


ObjLoader::~ObjLoader()
{
}
bool ObjLoader::load(const char * fileName)
{
	ifstream ifs(fileName);//cube bunny Eight
	string s;
	Mian *f;
	POINT3 *v;
	FaXiangLiang *vn;
	WenLi    *vt;
	while (getline(ifs, s))
	{
		if (s.length() < 2)continue;
		if (s[0] == 'v'){
			if (s[1] == 't'){//vt 0.581151 0.979929 纹理
				istringstream in(s);
				vt = new WenLi();
				string head;
				in >> head >> vt->TU >> vt->TV;
				this->VT.push_back(*vt);
			}
			else if (s[1] == 'n'){//vn 0.637005 -0.0421857 0.769705 法向量
				istringstream in(s);
				vn = new FaXiangLiang();
				string head;
				in >> head >> vn->NX >> vn->NY >> vn->NZ;
				this->VN.push_back(*vn);
			}
			else{//v -53.0413 158.84 -135.806 点
				istringstream in(s);
				v = new POINT3();
				string head;
				in >> head >> v->X >> v->Y >> v->Z;
				this->V.push_back(*v);
			}
		}
		else if (s[0] == 'f'){//f 2443//2656 2442//2656 2444//2656 面
			for (int k = s.size() - 1; k >= 0; k--){
				if (s[k] == '/')s[k] = ' ';
			}
			istringstream in(s);
			f = new Mian();
			string head;
			in >> head;
			int i = 0;
			while (i < 3)
			{
				if (this->V.size() != 0)
				{
					in >> f->V[i];
					f->V[i] -= 1;
				}
				if (this->VT.size() != 0)
				{
					in >> f->T[i];
					f->T[i] -= 1;
				}
				if (this->VN.size() != 0)
				{
					in >> f->N[i];
					f->N[i] -= 1;
				}
				i++;
			}
			this->F.push_back(*f);
		}
	}
	return true;
}
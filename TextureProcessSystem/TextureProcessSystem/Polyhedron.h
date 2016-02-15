/*
A fast and simple stretch-minimizing mesh parameterization C++ code
Copyright:(c) Shin Yoshizawa, 2008
E-mail: shin@riken.jp

 All right is reserved by Shin Yoshizawa.
This C++ sources are allowed for only primary user of 
research and educational purposes. Don't use secondary: copy, distribution, 
diversion, business purpose, and etc.. 
 */
#include "PointTool.h"
#include "PolarList.h"
#include "IDSet.h"


#define PI 3.1415926535897932385
#define next(p) p->next
#define back(p) p->back
// parameterization class
class Polyhedron{
public:
	int numberV;
	int numberF;
	int boundarytype;
	/*
	   0: unit square
	   1: unit circle
	   2: natural boundary of the intrinsic parameterization
	   */
	int paramtype;
	/*
	   1: U1 parameterization: single optimization step
	   2: Uopt parameterization: optimal parameterization
	   */
	int weighttype;
	/*
	  0: Shape Preserving: Floater CAGD 1997.
	  1: w = 1: Tutte 1963.
	  2: Harmonic Map: Eck et al. SIGGRAPH 1995.
	  3: Intrinsic Map: Desbrun et al. EUROGRAPHICS 2002.
	  ((1-intrinsiclambda)*Chi + intrinsiclambda*Authalic)
	  If intrinsiclambda=1 then it is equivalent to Eck's map
	  4: Mean value: Floater CAGD 2003.

	  */
	double intrinsiclambda;

	int smooth;
	/*
	   For the optimal low stretch parameterization,
	   1: fast and aggressive
	   2: smooth: if you will get a problem (unstability) using 1 then it is better to use 2.
	   */
	Point3D **point;
	int pickID;/* first boundary point ID: it will be conner of the unit square.
		  if pickID=-1 then pickID will be the first boundary vertex ID.
		  */

	int boundarysigma;/* When we test current stretch distortion,
				 0 without boundary triangles,
				 1 with boundary triangles
				 */
	int level;

	/* Parameter Mesh Coordinates */
	double *pU;
	double *pV;
	/*******************************/

	int **Face;
	double *areaMap3D;
	double sumarea3D;
	double constsumarea3D;

	double *sigma;
	double *E;
	double *G;

	IDSet *IDtool;
	PointTool *PT;
	/*************************/

	// 1-ring neighbor structure
	PolarList **PHead;
	PolarList **PTail;
	IDList **IHead;
	IDList **ITail;
	IDList **VHead;
	IDList **VTail;
	IDList **FHead;
	IDList **FTail;
	int *boundary;
	int *neighborI;
	int *neighborF;

	int numboundary;
	Point3D **bc;
	Point3D **dbc;

	double totalarea3D;
	double totalarea2D;
	double gammaP;/* transfer parameter: eta in the paper */

	int iteNum; /* Maximum iteration number in PCBCG */
	double PCBCGerror;/* Error threshold in PCBCG */



	Polyhedron();
	virtual ~Polyhedron();
	// parameterization
	void param();
	void param(int indexCenterInPara, float scale, Point3D * offset);

	void Polyhedron::readmesh(char *filename);
	void Polyhedron::writemesh(char *filename);
	Polyhedron(const Polyhedron& rhs);
	const Polyhedron &operator=(const Polyhedron& rhs);
	void Polyhedron::BoundaryMap();
	void Polyhedron::memorydelete();
	void Polyhedron::memoryallocate(int dV, int dF);
	void Polyhedron::setPoint(int i, float dx, float dy, float dz);
	void Polyhedron::SetBoundaryLines();
	void Polyhedron::setFace(int i, int di, int dj, int dk);
	void Polyhedron::setPolarMapNaturalB();
	void Polyhedron::setPolarMap();
	void Polyhedron::setAreaMap3D();
	// U1 low stretch parameterization
	void Polyhedron::ParametrizationOptimal(int itenum, double error);
	void Polyhedron::ParametrizationSingle(int itenum, double error);
	void Polyhedron::ParametrizationSmoothOptimal(int itenum, double error);

	// compute natural boundary 
	void Polyhedron::setNaturalB(int itenum, double error);
	void Polyhedron::setEckHC();
	double Polyhedron::getHarmonicTerm(int i, int backID, int nowID, int nextID);
	void Polyhedron::setDesbrunC();
	double Polyhedron::getAuthalicTerm(int i, int backID, int nowID, int nextID);
	double Polyhedron::getChiTerm(int i, int backID, int nowID, int nextID);
	void Polyhedron::setMVCC();
	void Polyhedron::setFloaterC();
	double Polyhedron::getMVCTerm(int i, int backID, int nowID, int nextID);
	void Polyhedron::SortIndexP();
	void Polyhedron::SortIndexPNaturalB();
	void Polyhedron::setSigmaZero();
	void Polyhedron::setLaplaceC();
	double Polyhedron::getCurrentE();
public:
	void mark();//标记贴图区域
	void getRect(Point3D *rect[2]);//获得贴图区域
	Point3D * getPos(Point3D **tir, Point3D * offset);
	float getFaceArea();//计算贴图区域三角形面积
	
public:
	bool *faceEffect;//影响区域
	int m_indexCenterInPara;
	float m_scale;
	Point3D * m_2DOffset;
	Point3D * m_centerPos;
	double centerFaceArea;
	double len;//最终缩放程度
};
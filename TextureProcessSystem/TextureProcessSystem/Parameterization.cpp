/*
A fast and simple stretch-minimizing mesh parameterization C++ code
Copyright:(c) Shin Yoshizawa, 2004
E-mail: shin.yoshizawa@mpi-sb.mpg.de
URL: http://www.mpi-sb.mpg.de/~shin
Affiliation: Max-Planck-Institut fuer Informatik: Computer Graphics Group 
 Stuhlsatzenhausweg 85, 66123 Saarbruecken, Germany
 Phone +49 681 9325-408 Fax +49 681 9325-499 

 All right is reserved by Shin Yoshizawa.
This C++ sources are allowed for only primary user of 
research and educational purposes. Don't use secondary: copy, distribution, 
diversion, business purpose, and etc.. 
 */
#include "stdafx.h"
#include"Point3D.h"
#include"Point2d.h"
#include"IDList.h"
#include"PointTool.h"
#include"PolarList.h"
#include"IDSet.h"
#include"PCBCGSolver.h"
#include"Polyhedron.h"
#include"Parameterization.h"
// Main 
/*int main(int argc,char *argv[]){
  
  if(argv[1]==NULL)return 0;
  Polyhedron *mymesh = new Polyhedron();
  mymesh->readmesh(argv[1]);
  mymesh->param();
  if(argv[2]==NULL){
    delete mymesh;
    return 0;
  }
  mymesh->writemesh(argv[2]);
  delete mymesh;
  return 0;
}*/
void Parameterization::setparameterization(char * inPLY2, char * outPLY2)
{
	Polyhedron *mymesh = new Polyhedron();
	

	mymesh->param();
	mymesh->writemesh(outPLY2);
}
void Parameterization::face_Parameterization(Model_PLY * ply, vector<int> faceIndexs)
{
	Polyhedron *mymesh = new Polyhedron();
	FILE *in = NULL;
	in = fopen("manneq1.ply2", "r");
	int dV = 0;
	int dF = 0;
	int i, j;
	int di = 0;
	int dj = 0;
	int dk = 0;
	double dx = 0.0;
	double dy = 0.0;
	double dz = 0.0;
	/*size_t ssize = fscanf(in, "%d", &dV);
	ssize = fscanf(in, "%d", &dF);*/
	dV = ply->pointArry.size();
	dF = ply->faceArry.size();
	mymesh->memoryallocate(dV, dF);
	for (i = 0; i<mymesh->numberV; i++){
		//ssize = fscanf(in, "%lf %lf %lf", &dx, &dy, &dz);
		dx = ply->pointArry[i].x;
		dy = ply->pointArry[i].y;
		dz = ply->pointArry[i].z;
		mymesh->setPoint(i, dx, dy, dz);
	}
	int val = 3;
	for (i = 0; i<mymesh->numberF; i++){
		//ssize = fscanf(in, "%d %d %d %d", &val, &di, &dj, &dk);
		di = ply->faceArry[i].ptnum[0];
		dj = ply->faceArry[i].ptnum[1];
		dk = ply->faceArry[i].ptnum[2];
		mymesh->setFace(i, di, dj, dk);
		mymesh->IDtool->AppendVFSort(i, mymesh->FHead[mymesh->Face[i][0]], mymesh->FTail[mymesh->Face[i][0]]);
		mymesh->IDtool->AppendVFSort(i, mymesh->FHead[mymesh->Face[i][1]], mymesh->FTail[mymesh->Face[i][1]]);
		mymesh->IDtool->AppendVFSort(i, mymesh->FHead[mymesh->Face[i][2]], mymesh->FTail[mymesh->Face[i][2]]);
	}
	fclose(in);
	/* feature analysis */
	mymesh->SetBoundaryLines();
	mymesh->setAreaMap3D();

	mymesh->param();
	mymesh->writemesh("newnew-cow_convert.ply2");
}
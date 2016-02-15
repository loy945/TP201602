/*
A fast and simple stretch-minimizing mesh parameterization C++ code
Copyright:(c) Shin Yoshizawa, 
8
E-mail: shin@riken.jp

 All right is reserved by Shin Yoshizawa.
This C++ sources are allowed for only primary user of 
research and educational purposes. Don't use secondary: copy, distribution, 
diversion, business purpose, and etc.. 
 */
#pragma once
#include "Point3D.h"
#include "Point2d.h"
class PointTool{
 public:
  Point3D *ddv1;
  Point3D *ddv2;
  Point3D *ddv3;
  Point2d *uij;
  Point2d *vij;

  PointTool(){
    ddv1 = new Point3D(0.0,0.0,0.0);
    ddv2 = new Point3D(0.0,0.0,0.0);
    ddv3 = new Point3D(0.0,0.0,0.0);
    uij = new Point2d(0.0,0.0);
    vij = new Point2d(0.0,0.0);
    
  }
  virtual ~PointTool(){
    delete ddv1;
    delete ddv2;
    delete ddv3;
    delete uij;
    delete vij;
  }
 private:
  
  PointTool(const PointTool& rhs);
  const PointTool &operator=(const PointTool& rhs);
 public:
  
  double getArea(Point3D* dv1,Point3D* dv2,Point3D* dv3);
    
  void Normalize3D(Point3D* inout);
  void Normalize2D(Point2d* inout);
  
  double Point2DSize(Point2d *dv);
  double Point2DSize(double d1x,double d1y,double d2x,double d2y);
  double Point2DSizeSq(double d1x,double d1y,double d2x,double d2y);
  double Distance(Point3D *in1,Point3D *in2);
  double getAlphaT(Point2d *dv,double ax,double ay,double bx,double by,double cx,double cy);
  void makeVector(Point3D *out,Point3D *in1,Point3D *in2);
  void CrossVector(Point3D *out,Point3D *in1,Point3D *in2);
  double InnerProduct(Point3D *in1,Point3D *in2);
  double InnerProduct(Point2d *in1,Point2d *in2);
  
  double Point3DSize(Point3D *in);
  
  int getBraycentricC(Point3D *evaluation,Point3D *baryparam,Point3D *v1,Point3D *v2,Point3D *v3, Point3D **bc);
   
  void setCenter(Point3D *out,Point3D *in1,Point3D *in2,Point3D *in3);
 
  void setParametricDs(Point3D *out,Point3D *q1,Point3D *q2,Point3D *q3,double t1,double t2,double t3,double A);
  void setParametricDt(Point3D *out,Point3D *q1,Point3D *q2,Point3D *q3,double s1,double s2,double s3,double A);

 

  double getParametricA(double t1,double t2,double t3,double s1,double s2,double s3);
  void ScalarVector(Point3D *out,double dv,Point3D *in);
 
  double getS(Point2d *v0,Point2d *v1,Point2d *v2);
  
  
  void setC(double *out,Point2d *v1,Point2d *v2,Point2d *v3,Point2d *v4);
 


};

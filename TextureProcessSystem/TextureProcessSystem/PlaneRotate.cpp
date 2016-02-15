#include "StdAfx.h"
#include "PlaneRotate.h"
#include <math.h>

PlaneRotate::PlaneRotate(void)
{
}


PlaneRotate::~PlaneRotate(void)
{
}
int PlaneRotate::double_equals(double const a, double const b)
{
	static const double ZERO = 1e-9;
	return fabs(a - b) < ZERO;
}
void PlaneRotate::init(float ax,float ay,float bx,float by,float cx,float cy,
		float dx,float dy,float dz,float ex,float ey,float ez,float fx,float fy,float fz)
{
	init(ax,ay,bx,by,cx,cy,dx,dy,ex,ey,fx,fy);
	this->dz=dz;
	this->ez=ez;
	this->fz=fz;
	circles[0].r=sqrtf(pow((fx-dx),2)+pow((fy-dy),2)+pow((fz-dz),2));
	circles[1].r=sqrtf(pow((fx-ex),2)+pow((fy-ey),2)+pow((fz-ez),2));
}
void PlaneRotate::init(float ax,float ay,float bx,float by,float cx,float cy,
		float dx,float dy,float ex,float ey,float fx,float fy)
{

	this->ax=ax;
	this->ay=ay;
	this->bx=bx;
	this->by=by;
	this->cx=cx;
	this->cy=cy;
	this->dx=dx;
	this->dy=dy;
	this->ex=ex;
	this->ey=ey;
	this->fx=fx;
	this->fy=fy;

	circles[0].center.x=ax;//e1---c1
	circles[0].center.y=ay;//e1
	circles[1].center.x=bx;//e2---c2
	circles[1].center.y=by;//e2


	//fx,fy b; dx,dy c1, ex,ey c2;
	circles[0].r=sqrtf(pow((fx-dx),2)+pow((fy-dy),2));
	circles[1].r=sqrtf(pow((fx-ex),2)+pow((fy-ey),2));
	
}
void PlaneRotate::getRes()
{


	switch (insect())
	{
	case -1:
		printf("THE CIRCLES ARE THE SAME/n");
		break;
	case 0:
		printf("NO INTERSECTION/n");
		break;
	case 1:
		printf("(%.3lf %.3lf)\n", points[0].x, points[0].y);
		break;
	case 2:
		{
			printf("(%.3lf %.3lf) (%.3lf %.3lf)\n",
				points[0].x, points[0].y,
				points[1].x, points[1].y);

		/*	if((func(ax,ay,bx,by,cx,cy)*func(ax,ay,bx,by,points[0].x,points[0].y))>0)
			{
				resx=points[0].x;
				resy=points[0].y;
			}else
			{
				resx=points[1].x;
 				resy=points[1].y;
			}*/
		}
		break;
	}

	point_t _C;
	_C.x=cx;
	_C.y=cy;
	if(distance(&points[0],&_C)>distance(&points[1],&_C))
	{
		resx=points[0].x;
		resy=points[0].y;
	}
	else
	{
		resx=points[1].x;
 		resy=points[1].y;
	}

}
int PlaneRotate::insect()
{
	double d, a, b, c, p, q, r;
	double cos_value[2], sin_value[2];
	if (double_equals(circles[0].center.x, circles[1].center.x)
		&& double_equals(circles[0].center.y, circles[1].center.y)
		&& double_equals(circles[0].r, circles[1].r)) {
		return -1;
	}

	d = distance(&circles[0].center, &circles[1].center);
	if (d > circles[0].r + circles[1].r
		|| d < fabs(circles[0].r - circles[1].r)) {
		return 0;
	}

	a = 2.0 * circles[0].r * (circles[0].center.x - circles[1].center.x);
	b = 2.0 * circles[0].r * (circles[0].center.y - circles[1].center.y);
	c = circles[1].r * circles[1].r - circles[0].r * circles[0].r
		- distance_sqr(&circles[0].center, &circles[1].center);
	p = a * a + b * b;
	q = -2.0 * a * c;
	if (double_equals(d, circles[0].r + circles[1].r)
		|| double_equals(d, fabs(circles[0].r - circles[1].r))) {
		cos_value[0] = -q / p / 2.0;
		sin_value[0] = sqrt(1 - cos_value[0] * cos_value[0]);

		points[0].x = circles[0].r * cos_value[0] + circles[0].center.x;
		points[0].y = circles[0].r * sin_value[0] + circles[0].center.y;

		if (!double_equals(distance_sqr(&points[0], &circles[1].center),
			circles[1].r * circles[1].r)) {
			points[0].y = circles[0].center.y - circles[0].r * sin_value[0];
		}
		return 1;
	}

	r = c * c - b * b;
	cos_value[0] = (sqrt(q * q - 4.0 * p * r) - q) / p / 2.0;
	cos_value[1] = (-sqrt(q * q - 4.0 * p * r) - q) / p / 2.0;
	sin_value[0] = sqrt(1 - cos_value[0] * cos_value[0]);
	sin_value[1] = sqrt(1 - cos_value[1] * cos_value[1]);

	points[0].x = circles[0].r * cos_value[0] + circles[0].center.x;
	points[1].x = circles[0].r * cos_value[1] + circles[0].center.x;
	points[0].y = circles[0].r * sin_value[0] + circles[0].center.y;
	points[1].y = circles[0].r * sin_value[1] + circles[0].center.y;

	if (!double_equals(distance_sqr(&points[0], &circles[1].center),
		circles[1].r * circles[1].r)) {
		points[0].y = circles[0].center.y - circles[0].r * sin_value[0];
	}
	if (!double_equals(distance_sqr(&points[1], &circles[1].center),
		circles[1].r * circles[1].r)) {
		points[1].y = circles[0].center.y - circles[0].r * sin_value[1];
	}
	if (double_equals(points[0].y, points[1].y)
		&& double_equals(points[0].x, points[1].x)) {
		if (points[0].y > 0) {
			points[1].y = -points[1].y;
		}
		else {
			points[0].y = -points[0].y;
		}
	}
	return 2;
}
double PlaneRotate::distance_sqr(struct point_t const* a, struct point_t const* b)
{
	return (a->x - b->x) * (a->x - b->x) + (a->y - b->y) * (a->y - b->y);
}
double PlaneRotate::distance(struct point_t const* a, struct point_t const* b)
{
	return sqrt(distance_sqr(a, b));
}
//double PlaneRotate::func(float x1,float y1,float x2,float y2,float x,float y)
//{
//	return ((y-y1)*(x2-x1)-(x-x1)*(y2-y1));
//}

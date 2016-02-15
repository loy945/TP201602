#pragma once
struct point_t 
{
	double x, y;
};

struct circle_t 
{
	struct point_t center;
	double r;
};

class PlaneRotate
{
public:
	PlaneRotate(void);
	~PlaneRotate(void);
	void init(float ax,float ay,float bx,float by,float cx,float cy,
		float dx,float dy,float ex,float ey,float fx,float fy);

	void init(float ax,float ay,float bx,float by,float cx,float cy,
		float dx,float dy,float dz,float ex,float ey,float ez,float fx,float fy,float fz);

	int insect();
	double distance_sqr(struct point_t const* a, struct point_t const* b);
	double distance(struct point_t const* a, struct point_t const* b);
	int double_equals(double const a, double const b);
	void getRes();
	//double func(float x1,float y1,float x2,float y2,float x,float y);
	struct circle_t circles[2];
	struct point_t points[2];
	float ax;
	float ay;
	float bx;
	float by;
	float cx;
	float cy;
	float dx;
	float dy;
	float ex;
	float ey;
	float fx;
	float fy;
	float resx;
	float resy;

	float dz;
	float ez;
	float fz;

};


#include "cv.h"
#include "highgui.h"
#include <opencv2/opencv.hpp> 
#define checkImageWidth 48
#define checkImageHeight 48
static GLubyte checkImage[checkImageWidth][checkImageHeight][4];
static GLuint texName;
static IplImage *src;

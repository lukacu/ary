#ifndef _ARY_UTILITIES
#define _ARY_UTILITIES

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ary/camera.h>

using namespace cv;

namespace ary {

Matx44f rotationMatrix(float x, float y, float z);

Matx44f translationMatrix(float x, float y, float z);

void drawSystem(Mat& image, const CameraPosition& camera, const Matx33f& intrinsics, const Mat& distortion, int size = 30, int width = 3);

void warpImage(Mat& target, Mat& image, const Mat& transformation, int flags = INTER_LINEAR);

string pathJoin(const string& p1, const string& p2);

string pathParent(const string& p);

void fillPolygon(Mat& image, const vector<Point2f>& points, Scalar value);

}

#endif

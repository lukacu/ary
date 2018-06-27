
#include <iostream>

#include <opencv2/calib3d/calib3d.hpp>

#include "ary/utilities.h"

using namespace cv;
using namespace std;

namespace ary {

float draw_scale = 1;

void setDrawScale(float scale) {

	draw_scale = MAX(0.000000001f, scale);

}

Matx44f rotationMatrix(float x, float y, float z) {
	Matx44f RX = Matx44f(
	                 1, 0,      0,       0,
	                 0, cos(x), -sin(x), 0,
	                 0, sin(x),  cos(x), 0,
	                 0, 0,       0,     1);

	Matx44f RY = Matx44f(
	                 cos(y), 0, -sin(y), 0,
	                 0, 1,          0, 0,
	                 sin(y), 0,  cos(y), 0,
	                 0, 0,          0, 1);

	Matx44f RZ = Matx44f(
	                 cos(z), -sin(z), 0, 0,
	                 sin(z),  cos(z), 0, 0,
	                 0,          0,           1, 0,
	                 0,          0,           0, 1);

	return RX * RY * RZ;

}

Matx44f translationMatrix(float x, float y, float z) {
	return Matx44f(
	           1, 0, 0, x,
	           0, 1, 0, y,
	           0, 0, 1, z,
	           0, 0, 0, 1);

}

void drawSystem(Mat& image, const CameraPosition& camera, const Matx33f& intrinsics, const Mat& distortion, float size, int width) {

		size *= draw_scale;

		std::vector<cv::Point2f> model2ImagePts;
		Mat modelPts = (Mat_<float>(4, 3) <<
		                0, 0, 0,
		                size, 0, 0,
		                0, size, 0,
		                0, 0, size);

		projectPoints(modelPts, camera.rotation, camera.translation, intrinsics, distortion, model2ImagePts);
		Mat rotation;
		Rodrigues(camera.rotation, rotation);

		bool up = cv::sum(rotation.col(2))[0] > 0;

		if (!up)
			cv::line(image, model2ImagePts.at(0), model2ImagePts.at(3), cvScalar(255, 100, 100), width);

		cv::line(image, model2ImagePts.at(0), model2ImagePts.at(1), cvScalar(0, 0, 255), width);
		cv::line(image, model2ImagePts.at(0), model2ImagePts.at(2), cvScalar(0, 255, 0), width);
		if (up)
			cv::line(image, model2ImagePts.at(0), model2ImagePts.at(3), cvScalar(255, 0, 0), width);

}

void warpImage(Mat& target, Mat& image, const Mat& transformation, int flags) {

    Mat src_image, src_mask, mask;
    mask = Mat::ones(image.size(), CV_8UC1);
    cv::warpPerspective(image, src_image, transformation, target.size(), flags);
    cv::warpPerspective(mask, src_mask, transformation, target.size(), flags);

    multiply(src_mask, src_image, src_image);
    multiply(Scalar::all(1.0)-src_mask, target, target);
    add(src_image, target, target);

}

void fillPolygon(Mat& image, const vector<Point2f>& points, Scalar value) {

	vector<Point> mask_points(points.size());
	int mask_npoints = points.size();
	for (int i = 0; i < points.size(); i++) {
		mask_points[i] = Point(points[i].x, points[i].y);
	}
	const Point* mask_points_ptr = &(mask_points[0]);
	fillPoly(image, &mask_points_ptr, &mask_npoints, 1, value);

}


string pathJoin(const string& p1, const string& p2) {

#ifdef _WIN32
   char sep = '\\';
#else
   char sep = '/';
#endif

  string tmp = p1;

  if (!p1.empty() && p1[p1.length()] != sep) { // Need to add a
     tmp += sep;                // path separator
     return(tmp + p2);
  }
  else
     return(p1 + p2);
}

string pathParent(const string& p) {

#ifdef _WIN32
   string sep = "\\";
#else
   string sep = "/";
#endif

  size_t position = (p[p.length()] != sep[0]) ? p.find_last_of(sep) : p.find_last_of(sep, p.length() - 1);

  if (position == string::npos) return "";

  return p.substr(0, position);
}


}




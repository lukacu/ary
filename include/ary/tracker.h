#ifndef _ARY_TRACKER_H
#define _ARY_TRACKER_H

#include <opencv2/core/core.hpp>

#include <ary/camera.h>
#include <ary/localizer.h>

using namespace std;
using namespace cv;

namespace ary {

class PlaneTracker : public CameraUser {
public:
	PlaneTracker(const SharedCameraModel& camera);
	~PlaneTracker();

	void initialize(const Mat& image, SharedPlanarLocalization localization, int id = 0);

	SharedLocalization update(const Mat& image);

    int getAge();

private:

	vector<Point3f> keypoints_start;
	vector<Point2f> keypoints_current;
	Mat previous, mask;
	vector<uchar> status;
	Mat errors;

	CameraPosition camera;
	vector<Point2f> plane_corners;
	Size plane_size;

    int age;
    int identifier;

};

}

#endif

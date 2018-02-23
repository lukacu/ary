#ifndef _ARY_KEYPOINT_LOCALIZER
#define _ARY_KEYPOINT_LOCALIZER

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "ary/camera.h"
#include "ary/localizer.h"

using namespace std;
using namespace cv;

namespace ary {

class KeypointPlaneLocalizer;

class KeypointPlaneModel {
friend KeypointPlaneLocalizer;
public:

    KeypointPlaneModel();
    ~KeypointPlaneModel();

protected:

	vector<KeyPoint> keypoints;
	Mat descriptors;
	Size2f size;

};

enum KeypointType { ORB, AKAZE };

class KeypointPlaneLocalizer: public Localizer {
public:
	KeypointPlaneLocalizer(const SharedCameraModel& camera, KeypointType type = ORB);

	virtual ~KeypointPlaneLocalizer();

	bool add(const Mat& image, float size);

	virtual vector<SharedLocalization> localize(const Mat& image);

	void setMask(const Mat& mask = Mat());

private:

	Mat mask;

    KeypointType type;

    vector<KeypointPlaneModel> models;

	Ptr<Feature2D> detector;
	Ptr<DescriptorMatcher> matcher;
};

}

#endif

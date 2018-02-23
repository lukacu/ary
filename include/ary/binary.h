#ifndef _ARY_BINARY_LOCALIZER
#define _ARY_BINARY_LOCALIZER

#include <opencv2/core/core.hpp>

#include "ary/camera.h"
#include "ary/localizer.h"

using namespace std;
using namespace cv;

namespace ary {

class BinaryPatternLocalizer;

class BinaryPattern {
friend BinaryPatternLocalizer;
public:
	BinaryPattern(const Mat& image, float size);
	~BinaryPattern();
	float getSize();

protected:

	double match(const Mat& src, int& orientation);

private:

	float size;
	std::vector<cv::Mat> markers;

};


class BinaryPatternLocalizer: public Localizer {
public:

	BinaryPatternLocalizer(const SharedCameraModel& camera, double threshold = 5, int block_size = 45, double confidence_threshold = 0.70);
	virtual ~BinaryPatternLocalizer();

    void add(const Mat& model, float size);

    int size();
	virtual vector<SharedLocalization> localize(const Mat& image);

private:

    CameraPosition calculateExtrinsics(const float size, const vector<Point2f>& imagePoints);
    void rectifyPattern(const Mat& src, const vector<Point2f>& roi, Rect& clamp, Mat& dst);
    int identifyPattern(const Mat& src, double& confidence, int& orientation);

	float block_size;
	std::vector<BinaryPattern> patterns;
	double confidence_threshold, threshold;

	Mat binImage, grayImage;

};

}

#endif

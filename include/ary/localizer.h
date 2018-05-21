#ifndef _ARY_LOCALIZER
#define _ARY_LOCALIZER

#include <opencv2/core/core.hpp>

#include "ary/camera.h"

using namespace std;
using namespace cv;

namespace ary {

class Localization {
public:
	Localization(int id, const CameraPosition& camera, float quality = 1);
	virtual ~Localization();
	virtual CameraPosition getCameraPosition() const;
    int getIdentifier() const;

    float getQuality() const;

    virtual void draw(Mat& image, SharedCameraModel& model) const;

protected:

    int id;
	CameraPosition camera;
    float quality;

};

class PlanarLocalization : public Localization {
public:
	PlanarLocalization(int id, const CameraPosition& camera, Size2f size, vector<Point2f> corners, float quality = 1);
	~PlanarLocalization();

	Size2f getPlaneSize() const;
	const vector<Point2f> getPlaneCorners() const;

    virtual void draw(Mat& image, SharedCameraModel& model) const;

private:
	vector<Point2f> corners;
	Size2f size;
};

typedef Ptr<Localization> SharedLocalization;
typedef Ptr<PlanarLocalization> SharedPlanarLocalization;

class Localizer : public CameraUser {
public:
	Localizer(const SharedCameraModel& camera);
	virtual ~Localizer();
	virtual vector<SharedLocalization> localize(const Mat& image) = 0;

};

void localizationToMask(const SharedPlanarLocalization& localization, Size mask_size, Mat& mask);

Matx44f positionToMatrix(const CameraPosition& position);

CameraPosition matrixToPosition(const Matx44f& matrix);

}

#endif

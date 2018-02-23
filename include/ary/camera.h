#ifndef _ARY_CAMERA
#define _ARY_CAMERA

#include <opencv2/core.hpp>

using namespace std;
using namespace cv;

namespace ary {

typedef struct CameraPosition {
	Matx31f rotation;
	Matx31f translation;
} CameraPosition;

class CameraModel {
public:
	CameraModel(Size sensor);
	CameraModel(const Matx33f& intrinsics, const Mat& distortion);
    CameraModel(const string& calibration);

	virtual ~CameraModel();

	Matx33f getIntrinsics();
	Mat getDistortion();

	void write(FileStorage& fs) const;
    void read(const FileNode& node);

private:
	Matx33f intrinsics;
	Mat distortion;
};

static void write(FileStorage& fs, const std::string&, const CameraPosition& x);
static void read(const FileNode& node, CameraPosition& x, const CameraPosition& default_value = CameraPosition());

typedef Ptr<CameraModel> SharedCameraModel;

static void write(FileStorage& fs, const std::string&, const SharedCameraModel& x);
static void read(const FileNode& node, SharedCameraModel& x, const SharedCameraModel& default_value = SharedCameraModel());

class CameraUser {
public:
	CameraUser(const SharedCameraModel& camera);
	virtual ~CameraUser();
	SharedCameraModel getCameraModel();

private:

	SharedCameraModel camera;
};

}

#endif


#include <stdexcept>
#include "ary/camera.h"

namespace ary {

CameraModel::CameraModel(Size sensor) {

    intrinsics(0, 0) = 700;
    intrinsics(1, 1) = 700;
    intrinsics(0, 2) = (float)(sensor.width) / 2;
    intrinsics(1, 2) = (float)(sensor.height) / 2;
    intrinsics(2, 2) = 1;

}

CameraModel::CameraModel(const string& calibration) {

    FileStorage fs;

    if (!fs.open(calibration, FileStorage::READ))
        throw new runtime_error("Calibration file does not extist");

    Mat ti, td;

    fs["intrinsics"] >> ti;
    fs["distortion"] >> td;

    intrinsics = ti;
    distortion = td;
}


CameraModel::CameraModel(const Matx33f& I, const Mat& D) {
	intrinsics = I;
	D.copyTo(distortion);
}

CameraModel::~CameraModel() {


}

Matx33f CameraModel::getIntrinsics() {

	return intrinsics;
}

Mat CameraModel::getDistortion() {
	return distortion;

}

CameraUser::CameraUser(const SharedCameraModel& camera) : camera(camera) {}

CameraUser::~CameraUser() {}

SharedCameraModel CameraUser::getCameraModel() {
	return camera;
}

static void write(FileStorage& fs, const std::string&, const CameraPosition& x) {
	fs << "{" << "T" << (Mat)x.translation << "R" << (Mat)x.rotation << "}";
}

static void read(const FileNode& node, CameraPosition& x, const CameraPosition& default_value) {
	if (node.empty()) {
		x = default_value;
	} else {
		Mat T, R;
		node["T"] >> T;
		node["R"] >> R;
		x.translation = T;
		x.rotation = T;
	}
}

static void write(FileStorage& fs, const std::string&, const SharedCameraModel& x) {
	fs << "{" << "intrinsics" << (Mat)x->getIntrinsics() << "distrotion" << x->getDistortion() << "}";

}

static void read(const FileNode& node, SharedCameraModel& x, const SharedCameraModel& default_value) {
	if (node.empty()) {
		x = default_value;
	} else {
		Mat I, D;
		node["intrinsics"] >> I;
		node["distortion"] >> D;
		x = SharedCameraModel(new CameraModel(I, D));
	}
}

}

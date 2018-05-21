#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "ary/localizer.h"
#include "ary/utilities.h"

using namespace std;
using namespace cv;

namespace ary {

Matx44f positionToMatrix(const CameraPosition& position) {

    Matx44f matrix;

    cv::Mat basis, origin;

    origin = Mat(position.translation);
    cv::Rodrigues(position.rotation, basis);

    matrix = 0;

    matrix(0, 0) = basis.at<float>(0, 0);
    matrix(0, 1) = basis.at<float>(0, 1);
    matrix(0, 2) = basis.at<float>(0, 2);
    matrix(0, 3) = origin.at<float>(0);

    matrix(1, 0) = basis.at<float>(1, 0);
    matrix(1, 1) = basis.at<float>(1, 1);
    matrix(1, 2) = basis.at<float>(1, 2);
    matrix(1, 3) = origin.at<float>(1);

    matrix(2, 0) = basis.at<float>(2, 0);
    matrix(2, 1) = basis.at<float>(2, 1);
    matrix(2, 2) = basis.at<float>(2, 2);
    matrix(2, 3) = origin.at<float>(2);

    matrix(3, 3) = 1;

    return matrix;
}

CameraPosition matrixToPosition(const Matx44f& matrix) {

    CameraPosition position;

    Mat rotation(3, 3, CV_32FC1);

    rotation.at<float>(0, 0) = matrix(0, 0);
    rotation.at<float>(0, 1) = matrix(0, 1);
    rotation.at<float>(0, 2) = matrix(0, 2);

    rotation.at<float>(1, 0) = matrix(1, 0);
    rotation.at<float>(1, 1) = matrix(1, 1);
    rotation.at<float>(1, 2) = matrix(1, 2);

    rotation.at<float>(2, 0) = matrix(2, 0);
    rotation.at<float>(2, 1) = matrix(2, 1);
    rotation.at<float>(2, 2) = matrix(2, 2);

    position.translation(0) = matrix(0, 3);
    position.translation(1) = matrix(1, 3);
    position.translation(2) = matrix(2, 3);

    cv::Rodrigues(rotation, position.rotation);

    return position;
}

Localization::Localization(int id, const CameraPosition& camera, float quality): id(id), camera(camera), quality(quality) {

}

Localization::~Localization() {

}

int Localization::getIdentifier() const {
    return id;
}

float Localization::getQuality() const {
    return quality;
}


void Localization::draw(cv::Mat& image, SharedCameraModel& model) const {

    drawSystem(image, camera, model->getIntrinsics(), model->getDistortion());

}

CameraPosition Localization::getCameraPosition() const {
	return camera;
}

PlanarLocalization::PlanarLocalization(int id, const CameraPosition& camera, Size2f size, vector<Point2f> corners, float quality) : Localization(id, camera, quality), size(size), corners(corners) {

}

PlanarLocalization::~PlanarLocalization() {

}

Size2f PlanarLocalization::getPlaneSize() const {
	return size;
}

const vector<Point2f> PlanarLocalization::getPlaneCorners() const {
	return corners;
}

void PlanarLocalization::draw(cv::Mat& image, SharedCameraModel& model) const {


    for (int i = 1; i < corners.size(); i++) {
	    line(image, corners[i-1], corners[i], Scalar(0, 255, 0), 4);
    }
    line(image, corners[corners.size()-1], corners[0], Scalar(0, 255, 0), 4);

    Localization::draw(image, model);

}


Localizer::Localizer(const SharedCameraModel& camera) : CameraUser(camera) {}

Localizer::~Localizer() {}

void localizationToMask(const SharedPlanarLocalization& localization, Size mask_size, Mat& mask) {

	vector<Point2f> plane_corners = localization->getPlaneCorners();
	Size plane_size = localization->getPlaneSize();

	mask.create(mask_size, CV_8UC1); mask.setTo(0);

	fillPolygon(mask, plane_corners, Scalar(255));
}


}

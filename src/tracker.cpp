#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d/calib3d.hpp>

//#ifdef _ARY_DESKTOP
#include <opencv2/highgui/highgui.hpp>
//  #endif

#include "ary/tracker.h"

using namespace std;
using namespace cv;

namespace ary {

PlaneTracker::PlaneTracker(const SharedCameraModel& camera) : CameraUser(camera) {

}

PlaneTracker::~PlaneTracker() {

}

Point3f unprojectPoint(Point2f p, const Matx33f& rotation, const Matx31f& translation, const Matx33f& intrinsics, const Mat& distortion) {

	std::vector<Point2f> ptsIn, ptsOut;
    ptsIn.push_back(p);
    undistortPoints(ptsIn, ptsOut, intrinsics, distortion, noArray(), intrinsics);

    float x = ptsOut[0].x;
    float y = ptsOut[0].y;

    Mat LOS(3,1,CV_32F);

    LOS.at<float>(0) = ptsOut[0].x;
    LOS.at<float>(1) = ptsOut[0].y;
    LOS.at<float>(2) = 1;

	Mat intrInv = Mat(intrinsics.inv());
    LOS = intrInv * LOS;

    Mat rotationInv = Mat(rotation.t());
    Mat translationInv = (-rotationInv * Mat(translation));
	LOS = rotationInv * LOS;

	LOS *= -(translationInv.at<float>(2) / LOS.at<float>(2));
	Point3f r;

	r.x = LOS.at<float>(0) + translationInv.at<float>(0);
	r.y = LOS.at<float>(1) + translationInv.at<float>(1);
	r.z = LOS.at<float>(2) + translationInv.at<float>(2);

	return r;
}


void PlaneTracker::initialize(const Mat& image, SharedPlanarLocalization localization, int id) {
	previous = image.clone();

	plane_corners = localization->getPlaneCorners();
	plane_size = localization->getPlaneSize();

	mask.create(previous.size(), CV_8UC1); mask.setTo(0);
	vector<Point> mask_points;
	int mask_npoints = plane_corners.size();
	for (int i = 0; i < plane_corners.size(); i++) {
		mask_points.push_back(Point(plane_corners[i].x, plane_corners[i].y));
	}
	const Point* mask_points_ptr = &(mask_points[0]);

	fillPoly(mask, &mask_points_ptr, &mask_npoints, 1, Scalar(255));
	goodFeaturesToTrack(image, keypoints_current, 1000, 0.1, 2, mask);

	keypoints_start.clear();

	Matx33f rotation;
	Matx31f translation;
	Rodrigues(localization->getCameraPosition().rotation, rotation);
	translation = localization->getCameraPosition().translation;

	Matx31f p;
	for (int i = 0; i < keypoints_current.size(); i++) {
		keypoints_start.push_back(unprojectPoint(keypoints_current[i], rotation, translation, getCameraModel()->getIntrinsics(), getCameraModel()->getDistortion()));
	}
/*
	vector<Point2f> image_points2;
	projectPoints(keypoints_start, camera.rotation, camera.translation, getCamera()->getIntrinsics(), getCamera()->getDistortion(), image_points2);

	Mat test;
	image.copyTo(test);

	for (int i = 0; i < keypoints_start.size(); i++) {
		line(test, image_points2[i], keypoints_current[i], Scalar(255), 2);
	cout <<  keypoints_current[i] << image_points2[i] << keypoints_start[i] << endl;

	}

	imshow("Test", test);
	waitKey(0);*/

    age = 0;
    identifier = id;

}

int PlaneTracker::getAge() {
    return age;

}

SharedLocalization PlaneTracker::update(const Mat& image) {

    age++;

	if (keypoints_current.size() < 10)
		return Ptr<Localization>();

	vector<Point2f> updates;
	calcOpticalFlowPyrLK(previous, image, keypoints_current, updates, status, errors);

	vector<Point3f> object_points;
	vector<Point2f> image_points;

	for (int i = 0; i < keypoints_current.size(); i++) {
		if (status[i]) {
			object_points.push_back(keypoints_start[i]);
			image_points.push_back(updates[i]);
		}
	}

	Mat rotVec;
	Mat transVec;

	vector<int> inliers(image_points.size());

	solvePnPRansac(object_points, image_points, getCameraModel()->getIntrinsics(), getCameraModel()->getDistortion(), rotVec, transVec, false, 150, 1, 0.99f, inliers);


	if (inliers.size() < 10)
		return Ptr<Localization>();

	keypoints_current.clear();
	keypoints_start.clear();
	for (int i = 0; i < inliers.size(); i++) {
		keypoints_current.push_back(image_points[inliers[i]]);
		keypoints_start.push_back(object_points[inliers[i]]);
	}

	image.copyTo(previous);

	CameraPosition updated_camera = camera;

	updated_camera.translation = Matx31f(transVec);
	updated_camera.rotation = Matx31f(rotVec);

/*
	vector<Point2f> image_points2;
	projectPoints(keypoints_start, updated_camera.rotation, updated_camera.translation, getCameraModel()->getIntrinsics(), getCameraModel()->getDistortion(), image_points2);

	Mat test;
	image.copyTo(test);

	for (int i = 0; i < keypoints_start.size(); i++) {
		line(test, image_points2[i], keypoints_current[i], Scalar(255), 2);
	//cout <<  keypoints_current[i] << image_points2[i] << keypoints_start[i] << endl;

	}

	imshow("Test", test);
	//waitKey(0);
*/

	return Ptr<PlanarLocalization>(new PlanarLocalization(identifier, updated_camera, plane_size, plane_corners));
}

}


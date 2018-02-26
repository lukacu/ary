#include <iostream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "ary/keypoints.h"

using namespace cv;

namespace ary {

KeypointPlaneModel::KeypointPlaneModel() {



}

KeypointPlaneModel::~KeypointPlaneModel() {


}

KeypointPlaneLocalizer::KeypointPlaneLocalizer(const SharedCameraModel& camera, KeypointType type) : type(type), Localizer(camera) {

    switch (type) {
    case ORB:
    	detector = ORB::create().dynamicCast<Feature2D>();
	    matcher = Ptr<DescriptorMatcher>(new BFMatcher(NORM_HAMMING, true));
        break;
    case AKAZE:
    	detector = AKAZE::create().dynamicCast<Feature2D>();
	    matcher = Ptr<DescriptorMatcher>(new BFMatcher(NORM_HAMMING, true));
        break;
    }

}

KeypointPlaneLocalizer::~KeypointPlaneLocalizer() { }

bool KeypointPlaneLocalizer::add(const Mat& image, float size) {

	KeypointPlaneModel model;

	Mat tmp_image1, tmp_image2;
	tmp_image1 = image;

	model.size = Size2f(tmp_image1.size().width * size, tmp_image1.size().height * size);

	while (true) {
		Mat descriptors;
		vector<KeyPoint> keypoints;

		detector->detect(tmp_image1, keypoints);
		detector->compute(tmp_image1, keypoints, descriptors);
		//descriptors.convertTo(descriptors, CV_32F);

		Size size = tmp_image1.size();
		if (model.descriptors.empty()) {
			descriptors.copyTo(model.descriptors);
		} else {
            vconcat(model.descriptors, descriptors, model.descriptors);
        }

		float xscale = (float) model.size.width / (float) size.width;
		float yscale = (float) model.size.height / (float) size.height;

		for (vector<KeyPoint>::iterator it = keypoints.begin(); it != keypoints.end(); it++) {
			(*it).pt.x *= xscale;
			(*it).pt.y *= yscale;
			model.keypoints.push_back(*it);
		}

        if (type == AKAZE) break; // No need for additional features.

		if (size.width * size.height < 30000) break;

		pyrDown(tmp_image1, tmp_image2, Size());
		tmp_image1 = tmp_image2;
	}

    models.push_back(model);

	return true;
}

bool dmatch_comparator (DMatch& i, DMatch& j) {
    return (i.distance<j.distance);
}


vector<SharedLocalization> KeypointPlaneLocalizer::localize(const Mat& image) {

	Mat H = Mat::zeros(3, 3, CV_32F);
	vector<KeyPoint> keypoints;
	Mat descriptors;

    Mat gray = image;

	detector->detect(gray, keypoints, mask);
	detector->compute(gray, keypoints, descriptors);
	//descriptors.convertTo(descriptors, CV_32F);

    vector<SharedLocalization> localizations;

    for (int m = 0; m < models.size(); m++) {

	    vector<vector<DMatch> > matches;
	    // matcher freezes if it tries to match an empty matrix
	    if (models[m].descriptors.empty() || descriptors.empty()) continue;

        matcher->knnMatch(models[m].descriptors, descriptors, matches, 1);

        float threshold_absolute = 1000;
        float threshold_percentage = 1;

        switch (type) {
        case ORB:
        	threshold_absolute = 40;
            threshold_percentage = 0.5f;
            break;
        case AKAZE:
        	threshold_absolute = 100;
            threshold_percentage = 0.3f;
            break;
        }

	    vector<DMatch> best_matches;
	    for (int i = 0; i < matches.size(); i++) {
            if (!matches[i].empty())
			    best_matches.push_back(matches[i][0]);
	    }

        if (threshold_percentage < 1) {
            std::sort (best_matches.begin(), best_matches.end(), dmatch_comparator);
            best_matches = std::vector<DMatch>(best_matches.begin(), best_matches.begin() + (int)((float)best_matches.size() * threshold_percentage));
        }

	    vector<Point3f> object_points;
	    vector<Point2f> image_points;

	    // prepare keypoints from good matches for transformation
	    for(uint i = 0; i < best_matches.size(); i++) {
            if (best_matches[i].distance > threshold_absolute) continue;
		    Point2f p = models[m].keypoints[ best_matches[i].queryIdx ].pt;
		    object_points.push_back( Point3f(p.x, p.y, 0) );
		    image_points.push_back( keypoints[ best_matches[i].trainIdx ].pt );
	    }

	    // was able to match base image to (a subset) of the current image?
	    if (image_points.size() < 10) continue;

	    Mat rotVec, transVec;
        vector<int> inliers;
	    CameraPosition camera;
	    if (!solvePnPRansac(object_points, image_points, getCameraModel()->getIntrinsics(), getCameraModel()->getDistortion(), rotVec, transVec, false, 200, 4, 0.99f, inliers))
            continue;
/*
	    vector<Point3f> object_inliers;
	    vector<Point2f> image_inliers;

	    // prepare keypoints from good matches for transformation
	    for(uint i = 0; i < inliers.size(); i++) {
		    object_inliers.push_back(object_points[inliers[i]]);
            image_inliers.push_back(image_points[inliers[i]]);
	    }


        if (!solvePnP(object_inliers, image_inliers, getCameraModel()->getIntrinsics(), getCameraModel()->getDistortion(), rotVec, transVec, true, SOLVEPNP_ITERATIVE))
            continue;
*/
    //useExtrinsicGuess=false, int iterationsCount=100, float reprojectionError=8.0, int minInliersCount=100

	    transVec.convertTo(camera.translation, CV_32F);
	    rotVec.convertTo(camera.rotation, CV_32F);

	    vector<Point3f> models_points;
	    models_points.push_back(Point3f(0, 0, 0));
	    models_points.push_back(Point3f(models[m].size.width, 0, 0));
	    models_points.push_back(Point3f(models[m].size.width, models[m].size.height, 0));
	    models_points.push_back(Point3f(0, models[m].size.height, 0));

        float quality = (float) inliers.size() / (float) models[m].keypoints.size();

	    image_points.clear();
	    projectPoints(models_points, rotVec, transVec, getCameraModel()->getIntrinsics(), getCameraModel()->getDistortion(), image_points);

        localizations.push_back(new PlanarLocalization(m, camera, models[m].size, image_points, quality));

    }

	return localizations;

}

void KeypointPlaneLocalizer::setMask(const Mat& m) {
	mask = m;
}

}




#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "ary/complex.h"
#include "ary/binary.h"
#include "ary/utilities.h"

#include <opencv2/highgui/highgui.hpp>

namespace ary {

#define VERIFIER_SIZE 128

PlaneVerifier::PlaneVerifier(const SharedCameraModel& camera) : CameraUser(camera) {}

PlaneVerifier::~PlaneVerifier() {};

TemplatePlaneVerifier::TemplatePlaneVerifier(const SharedCameraModel& camera, const Mat& image) : PlaneVerifier(camera) {

	model_size = image.size();
	resize(image, model_template, Size(VERIFIER_SIZE, (int)(((float)VERIFIER_SIZE * (float) image.rows) / (float) image.cols)));

	model_mask.create(model_template.size(), CV_8UC1); model_mask.setTo(1);

}

TemplatePlaneVerifier::~TemplatePlaneVerifier() {

}

bool TemplatePlaneVerifier::verify(const Mat& image, const SharedPlanarLocalization& localization) {

	vector<Point3f> model_points;
	model_points.push_back(Point3f(0, 0, 0));
	model_points.push_back(Point3f(model_size.width, 0, 0));
	model_points.push_back(Point3f(model_size.width, model_size.height, 0));
	model_points.push_back(Point3f(0, model_size.height, 0));

	vector<Point2f> image_points;
	projectPoints(model_points, localization->getCameraPosition().rotation, localization->getCameraPosition().translation, getCameraModel()->getIntrinsics(), getCameraModel()->getDistortion(), image_points);

	vector<Point2f> template_points;
	template_points.push_back(Point2f(0, 0));
	template_points.push_back(Point2f(model_template.cols, 0));
	template_points.push_back(Point2f(model_template.cols, model_template.rows));
	template_points.push_back(Point2f(0, model_template.rows));

	Mat homography = getPerspectiveTransform(image_points, template_points);

	Mat verification;
	cv::warpPerspective(image, verification, homography, model_template.size());

	vector<Point2f> bounds_points;
	bounds_points.push_back(Point2f(0, 0));
	bounds_points.push_back(Point2f(image.cols, 0));
	bounds_points.push_back(Point2f(image.cols, image.rows));
	bounds_points.push_back(Point2f(0, image.rows));
	vector<Point2f> mask_points;

	perspectiveTransform(bounds_points, mask_points, homography);

	model_mask.setTo(0);

	fillPolygon(model_mask, mask_points, Scalar(255));
/*
imshow("Verif", verification);
imshow("model", model_template);

imwrite("model.png", model_template);
imwrite("query.png", verification);

imshow("mask", model_mask);*/
	double match_score = match_template(model_template, verification, model_mask);

//cout << match_score << endl;

	return (match_score > 0.4);

}

double TemplatePlaneVerifier::match_template(const Mat& tmpl, const Mat& query, const Mat& mask) {

	double confidence;
	double N = (double)countNonZero(mask);
	double nom, den;

	Scalar mean_query, std_query;
	Scalar mean_tmpl, std_tmpl;


	meanStdDev(query, mean_query, std_query, mask);
	meanStdDev(tmpl, mean_tmpl, std_tmpl, mask);

	double sum_query = norm(query, NORM_L1, mask);
	double sum_tmpl = norm(tmpl, NORM_L1, mask);

  // cout << N << " " << sum_query << " " << sum_tmpl << endl;

	nom = query.dot(tmpl) - mean_query[0] * sum_tmpl - mean_tmpl[0] * sum_query + N * mean_tmpl[0] * mean_query[0] ;
	den = N * std_query[0] * std_tmpl[0];
	confidence = nom / den;

  //  cout << nom << " " << den << endl;

    if (confidence > 1) confidence = 1;
	if (confidence < 0 || confidence != confidence) confidence = 0;

	return confidence;
}

KeypointTrackingLocalizer::KeypointTrackingLocalizer(const SharedCameraModel& camera) : Localizer(camera), tracking_timeout(0), tracking_select(-1) {

	localizer = Ptr<KeypointPlaneLocalizer>(new KeypointPlaneLocalizer(camera, ary::AKAZE));

    tracker = Ptr<PlaneTracker>(new PlaneTracker(camera));
}

KeypointTrackingLocalizer::~KeypointTrackingLocalizer() {

}

void KeypointTrackingLocalizer::add(const Mat& model, float size) {

    localizer->add(model, size);

  	verifier.push_back(Ptr<TemplatePlaneVerifier>(new TemplatePlaneVerifier(getCameraModel(), model)));

}

vector<SharedLocalization> KeypointTrackingLocalizer::localize(const Mat& image) {

    vector<SharedLocalization> localizations;

    if (tracking_select < 0) {

        tracking_timeout--;

		if (tracking_timeout > 0) return localizations;

        vector<SharedLocalization> candidates = localizer->localize(image);

        if (candidates.size() < 1 || !verifier[candidates[0]->getIdentifier()]->verify(image, candidates[0].dynamicCast<PlanarLocalization>())) {

            tracking_timeout = 10;
            return localizations;
        }

        tracker->initialize(image, candidates[0].dynamicCast<PlanarLocalization>());

        tracking_timeout = 50;
        tracking_select = candidates[0]->getIdentifier();

        localizations.push_back(candidates[0]);

    } else {

        SharedLocalization tracking = tracker->update(image);

        if (!tracking || !verifier[tracking_select]->verify(image, tracking.dynamicCast<PlanarLocalization>())) {
            tracking_select = -1;
            return localizations;
        }

        tracking_timeout--;

		if (tracking_timeout < 1) {

			localizationToMask(tracking.dynamicCast<PlanarLocalization>(), image.size(), mask);

			localizer->setMask(mask);
			vector<SharedLocalization> candidates = localizer->localize(image);

            for (size_t j = 0; j < candidates.size(); j++) {
			    if (candidates[j]->getIdentifier() == tracking_select && verifier[tracking_select]->verify(image, candidates[j].dynamicCast<PlanarLocalization>())) {
				    tracker->initialize(image, candidates[j].dynamicCast<PlanarLocalization>());
				    tracking_timeout = 50;
                    break;
			    }
            }
		    if (tracking_timeout < 1) {
		        tracking_select = -1;
		        localizer->setMask();
            }

        } else {

            localizations.push_back(tracking);
        }
    }


	return localizations;

}

SceneAnchor::SceneAnchor(const string name, const Matx44f& transform, int group) : name(name), transform(transform), group(group) {

}

SceneAnchor::~SceneAnchor() {

}

Matx44f SceneAnchor::getTransform() {
    return transform;
}


string SceneAnchor::getName() {
    return name;
}

int SceneAnchor::getGroup() {
    return group;
}

Scene::Scene(const SharedCameraModel& camera, const string description) : Localizer(camera) {

    localizers.push_back(Ptr<Localizer>(new BinaryPatternLocalizer(camera)));
    localizers.push_back(Ptr<Localizer>(new KeypointTrackingLocalizer(camera)));

    load(description);

}

bool Scene::load(const string description) {

	FileStorage fs(description, FileStorage::READ);

	if (!fs.isOpened()) {
		return false;
	}

    FileNode anchor_descriptions = fs["anchors"];

    string root = pathParent(description);

    anchors.resize(localizers.size());

	for (FileNodeIterator it = anchor_descriptions.begin(); it != anchor_descriptions.end(); ++it) {
		float anchor_size;
		float rx, ry, rz, ox, oy, oz;
        string anchor_type, anchor_group, anchor_template, anchor_name;

        read((*it)["name"], anchor_name, "");
        read((*it)["type"], anchor_type, "binary");
        read((*it)["group"], anchor_group, "");

		read((*it)["size"], anchor_size, 50);
		read((*it)["rotation"]["x"], rx, 0);
		read((*it)["rotation"]["y"], ry, 0);
		read((*it)["rotation"]["z"], rz, 0);
		read((*it)["origin"]["x"], ox, 0);
		read((*it)["origin"]["y"], oy, 0);
		read((*it)["origin"]["z"], oz, 0);

		rx = M_PI * (rx) / 180;
		ry = M_PI * (ry) / 180;
		rz = M_PI * (rz) / 180;

		Matx44f offset = translationMatrix(ox, oy, oz) * rotationMatrix(rx, ry, rz);

        int localizer = -1;

        if (anchor_type == "binary") {

            read((*it)["template"], anchor_template, "");
            if (anchor_template.empty()) continue;
		    string template_file = pathJoin(root, anchor_template);

            Mat tmp = imread(template_file, IMREAD_GRAYSCALE);
            if (tmp.empty()) {
                cerr << "Cannot find " << template_file << endl;
                continue;
            }

            localizers[0].dynamicCast<BinaryPatternLocalizer>()->add(tmp, anchor_size);
            localizer = 0;

        } else if (anchor_type == "keypoints") {

            read((*it)["template"], anchor_template, "");
            if (anchor_template.empty()) continue;
		    string template_file = pathJoin(root, anchor_template);

            Mat tmp = imread(template_file, IMREAD_GRAYSCALE);
            if (tmp.empty()) {
                cerr << "Cannot find " << template_file << endl;
                continue;
            }

            localizers[1].dynamicCast<KeypointTrackingLocalizer>()->add(tmp, anchor_size);
            localizer = 1;

        }

        if (localizer == -1) continue;

        int group = 0;
        if (!anchor_group.empty()) {
            std::vector<string>::iterator it;

            it = find (groups.begin(), groups.end(), anchor_group);
            if (it != groups.end()) {
                group = (it - groups.begin()) + 1;
            } else {
                groups.push_back(anchor_group);
                group = (int) groups.size();
            }

        }

        anchors[localizer].push_back(Ptr<SceneAnchor>(new SceneAnchor(anchor_name, offset, group)));

	}

}

Scene::~Scene() {}

int Scene::size() {
    return groups.size() + 1;
}

vector<SharedLocalization> Scene::localize(const Mat& image) {

	vector<SharedLocalization> group_localization(groups.size() + 1, Ptr<Localization>());

    for (size_t i = 0; i < localizers.size(); i++) {
        vector<SharedLocalization> localizations = localizers[i]->localize(image);

        for (size_t j = 0; j < localizations.size(); j++) {
            int group = anchors[i][localizations[j]->getIdentifier()]->getGroup();

            Matx44f offset = anchors[i][localizations[j]->getIdentifier()]->getTransform();
            Matx44f origin = positionToMatrix(localizations[j]->getCameraPosition());

            CameraPosition l = matrixToPosition(positionToMatrix(localizations[j]->getCameraPosition()) * offset);

            group_localization[group] = Ptr<Localization>(new Localization(group, l, 1));

        }

    }

    vector<SharedLocalization> localizations;

    for (size_t i = 0; i < group_localization.size(); i++) {
        if (group_localization[i]) localizations.push_back(group_localization[i]);

    }

    return localizations;
}

}

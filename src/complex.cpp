
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "ary/complex.h"
#include "ary/binary.h"
#include "ary/utilities.h"

#include <opencv2/highgui/highgui.hpp>

#if CV_MAJOR_VERSION == 2
#define DYNAMIC_CAST(h, C) ((h).ptr<C>())
#elif CV_MAJOR_VERSION == 3
#define DYNAMIC_CAST(h, C) ((h).dynamicCast<C>())
#endif

namespace ary {

#define VERIFIER_SIZE 128

inline Point3f extract_homogeneous(Matx41f hv) {

    Point3f f = Point3f(hv(0, 0) / hv(3, 0), hv(1, 0) / hv(3, 0), hv(2, 0) / hv(3, 0));
    return f;

}

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

	double match_score = match_template(model_template, verification, model_mask);

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

	nom = query.dot(tmpl) - mean_query[0] * sum_tmpl - mean_tmpl[0] * sum_query + N * mean_tmpl[0] * mean_query[0] ;
	den = N * std_query[0] * std_tmpl[0];
	confidence = nom / den;

    if (confidence > 1) confidence = 1;
	if (confidence < 0 || confidence != confidence) confidence = 0;

	return confidence;
}


/*
double TemplatePlaneVerifier::match_template(const Mat& tmpl, const Mat& query, const Mat& mask) {

	double confidence;
	double N = (double)countNonZero(mask);
	double nom, den;

	Scalar mean_query, std_query;
	Scalar mean_tmpl, std_tmpl;

	meanStdDev(query, mean_query, std_query, mask);
	meanStdDev(tmpl, mean_tmpl, std_tmpl, mask);

    Mat diff = Mat(tmpl.size(), CV_32FC1);

    uchar* tmpl_ptr = reinterpret_cast<uchar*>(tmpl.data);
    uchar* query_ptr = reinterpret_cast<uchar*>(query.data);
    float* diff_ptr = reinterpret_cast<float*>(diff.data);

    for(int i = 0; i < tmpl.rows * tmpl.cols; i++, tmpl_ptr++, query_ptr++, diff_ptr++) {
        *diff_ptr = (float) (abs(((double) (*tmpl_ptr) - mean_tmpl[0]) / std_tmpl[0] - ((double) (*query_ptr) - mean_query[0]) / std_query[0]));
    }


imshow("diff", diff);


	double sum_query = norm(diff, NORM_L1, mask);


	return 1 - ( sum_query / N);
}
*/
KeypointTrackingLocalizer::KeypointTrackingLocalizer(const SharedCameraModel& camera) : Localizer(camera), tracking_timeout(0), tracking_select(-1) {

#if CV_MAJOR_VERSION == 2
    localizer = Ptr<KeypointPlaneLocalizer>(new KeypointPlaneLocalizer(camera, ary::ORB));
#elif CV_MAJOR_VERSION == 3
    localizer = Ptr<KeypointPlaneLocalizer>(new KeypointPlaneLocalizer(camera, ary::AKAZE));
#endif

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

        if (candidates.size() < 1) { // || !verifier[candidates[0]->getIdentifier()]->verify(image, candidates[0].dynamicCast<PlanarLocalization>())) {

            tracking_timeout = 10;
            return localizations;
        }

        tracker->initialize(image, candidates[0].dynamicCast<PlanarLocalization>());

        tracking_timeout = 50;
        tracking_select = candidates[0]->getIdentifier();

        localizations.push_back(candidates[0]);

    } else {

        SharedLocalization tracking = tracker->update(image);

        if (!tracking) { // || !verifier[tracking_select]->verify(image, tracking.dynamicCast<PlanarLocalization>())) {
            tracking_select = -1;
            return localizations;
        }

        tracking_timeout--;

		if (tracking_timeout < 1) {

			localizationToMask(tracking.dynamicCast<PlanarLocalization>(), image.size(), mask);

			localizer->setMask(mask);
			vector<SharedLocalization> candidates = localizer->localize(image);

            for (size_t j = 0; j < candidates.size(); j++) {
			    if (candidates[j]->getIdentifier() == tracking_select) { // && verifier[tracking_select]->verify(image, candidates[j].dynamicCast<PlanarLocalization>())) {
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

Matx44f SceneAnchor::getTransform() const {
    return transform;
}


string SceneAnchor::getName() const {
    return name;
}

int SceneAnchor::getGroup() const {
    return group;
}

AnchoredLocalization::AnchoredLocalization(int id, const CameraPosition& camera, vector<SharedLocalization> localizations, vector<Ptr<SceneAnchor> > anchors, float quality) : 
    Localization(id, camera, quality), localizations(localizations), anchors(anchors) {

}

AnchoredLocalization::~AnchoredLocalization() {

}

void AnchoredLocalization::draw(Mat& image, SharedCameraModel& model) const {

    for (size_t i = 0; i < localizations.size(); i++) {
        localizations[i]->draw(image, model);

        std::vector<cv::Point2f> model2ImagePts;
        
        Matx44f offset = anchors[i]->getTransform();

        Point3f origin = extract_homogeneous(offset * Scalar(0, 0, 0, 1));

        Mat modelPts = (Mat_<float>(3, 3) <<
                        0.01, 0.01, 0.01,
                        0, 0, 0,
                        origin.x, origin.y, origin.z);

        CameraPosition camera = localizations[i]->getCameraPosition();

        projectPoints(modelPts, camera.rotation, camera.translation, model->getIntrinsics(), model->getDistortion(), model2ImagePts);

        line(image, model2ImagePts[1], model2ImagePts[2], Scalar(255, 255, 0), 2);

        putText(image, anchors[i]->getName(), model2ImagePts[0], FONT_HERSHEY_SIMPLEX, 0.5f, Scalar(255, 100, 255), 2);

    }

    drawSystem(image, getCameraPosition(), model->getIntrinsics(), model->getDistortion());

}

Scene::Scene(const SharedCameraModel& camera, const string& description) : Localizer(camera) {

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
        String anchor_type, anchor_group, anchor_template, anchor_name;

        read((*it)["identifier"], anchor_name, String(""));
        read((*it)["type"], anchor_type, String("binary"));
        read((*it)["group"], anchor_group, String(""));

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

            read((*it)["template"], anchor_template, String(""));
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

            read((*it)["template"], anchor_template, string(""));
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

	return true;

}

Scene::~Scene() {}

int Scene::size() {
    return groups.size() + 1;
}

#define MODEL_SIZE 0.05

vector<SharedLocalization> Scene::localize(const Mat& image) {

    vector<vector<Point3f> > group_points_plane(groups.size() + 1);
    vector<vector<Point2f> > group_points_image(groups.size() + 1);

    vector<vector<SharedLocalization> > group_localizations(groups.size() + 1);

    vector<vector<Ptr<SceneAnchor> > > group_anchors(groups.size() + 1);

    for (size_t i = 0; i < localizers.size(); i++) {
        vector<SharedLocalization> localizations = localizers[i]->localize(image);

        for (size_t j = 0; j < localizations.size(); j++) {
            int group = anchors[i][localizations[j]->getIdentifier()]->getGroup();

            Matx44f offset = (anchors[i][localizations[j]->getIdentifier()]->getTransform()).inv();

            vector<Point2f> image_points(4);
            vector<Point3f> reference_points;

            reference_points.push_back(Point3f(0, 0, 0));
            reference_points.push_back(Point3f(MODEL_SIZE, 0, 0));
            reference_points.push_back(Point3f(0, MODEL_SIZE, 0));
            reference_points.push_back(Point3f(0, 0, MODEL_SIZE));

            projectPoints(reference_points, localizations[j]->getCameraPosition().rotation, localizations[j]->getCameraPosition().translation, getCameraModel()->getIntrinsics(), getCameraModel()->getDistortion(), image_points);

            group_points_plane[group].push_back(extract_homogeneous(offset * Scalar(0, 0, 0, 1)));
            group_points_plane[group].push_back(extract_homogeneous(offset * Scalar(MODEL_SIZE, 0, 0, 1)));
            group_points_plane[group].push_back(extract_homogeneous(offset * Scalar(0, MODEL_SIZE, 0, 1)));
            group_points_plane[group].push_back(extract_homogeneous(offset * Scalar(0, 0, MODEL_SIZE, 1)));

            for (int k = 0; k < 4; k++) {
                group_points_image[group].push_back(image_points[k]);
            }

            group_localizations[group].push_back(localizations[j]);
            group_anchors[group].push_back(anchors[i][localizations[j]->getIdentifier()]);

        }

    }

    vector<SharedLocalization> localizations;

    for (size_t i = 0; i < group_points_plane.size(); i++) {

        if (group_points_plane[i].size() < 4) continue;

        if (group_localizations[i].size() == 1) {

            Matx44f offset = (group_anchors[i][0]->getTransform());

            CameraPosition position = matrixToPosition((positionToMatrix(group_localizations[i][0]->getCameraPosition()) * offset));

            localizations.push_back(Ptr<Localization>(new AnchoredLocalization(i, position, group_localizations[i], group_anchors[i])));

            continue;

        }

        Mat rotVec, transVec;

        vector<Point3f> points_plane;
        vector<Point2f> points_image;

        if (group_points_plane[i].size() > 104) {
            vector<int> inliers;    
            solvePnPRansac(group_points_plane[i], group_points_image[i], 
                    getCameraModel()->getIntrinsics(), getCameraModel()->getDistortion(),
                    rotVec, transVec, false, 100, 50.0, 4, inliers);

            for (size_t j = 0; j < inliers.size(); j++) {
                if (inliers[j] != 0) {
                    points_plane.push_back(group_points_plane[i][j]);
                    points_image.push_back(group_points_image[i][j]);
                }
            }

        } else {
            points_plane = group_points_plane[i];
            points_image = group_points_image[i];
        }

        solvePnP(points_plane, points_image, 
            getCameraModel()->getIntrinsics(), getCameraModel()->getDistortion(), rotVec, transVec);
        
        CameraPosition position;

        rotVec.convertTo(position.rotation, CV_32F);
        transVec.convertTo(position.translation, CV_32F);

        localizations.push_back(Ptr<Localization>(new AnchoredLocalization(i, position, group_localizations[i], group_anchors[i])));

    }

    return localizations;
}

}

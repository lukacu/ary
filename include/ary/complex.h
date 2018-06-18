#ifndef _ARY_COMPLEX
#define _ARY_COMPLEX

#include <ary/localizer.h>
#include <ary/keypoints.h>
#include <ary/tracker.h>

using namespace std;
using namespace cv;

namespace ary {

class PlaneVerifier : public CameraUser {
public:
	PlaneVerifier(const SharedCameraModel& camera);
	virtual ~PlaneVerifier();
	virtual bool verify(const Mat& image, const SharedPlanarLocalization& localization) = 0;

};

class TemplatePlaneVerifier : public PlaneVerifier {
public:
	TemplatePlaneVerifier(const SharedCameraModel& camera, const Mat& image);
	virtual ~TemplatePlaneVerifier();
	virtual bool verify(const Mat& image, const SharedPlanarLocalization& localization);

private:
	double match_template(const Mat& tmpl, const Mat& query, const Mat& mask);

	Mat model_template;
	Size model_size;
	Mat model_mask;

};

class KeypointTrackingLocalizer: public Localizer {
public:
	KeypointTrackingLocalizer(const SharedCameraModel& camera);

	virtual ~KeypointTrackingLocalizer();

	virtual vector<SharedLocalization> localize(const Mat& image);

    void add(const Mat& model, float size);

private:

	Mat mask;

	SharedLocalization localization;

	Ptr<KeypointPlaneLocalizer> localizer;
	vector<Ptr<TemplatePlaneVerifier> > verifier;
	Ptr<PlaneTracker> tracker;

    int tracking_timeout;
    int tracking_select;

};

class SceneAnchor {
public:
    SceneAnchor(const string name, const Matx44f& transform, int group);
    virtual ~SceneAnchor();

    Matx44f getTransform();
    string getName();
    int getGroup();

private:

    string name;
    Matx44f transform;
    int group;

};


class Scene: public Localizer {
public:
	Scene(const SharedCameraModel& camera, const string& description);

	virtual ~Scene();

	virtual vector<SharedLocalization> localize(const Mat& image);

    int size();

private:

    bool load(const string description);

	vector<Ptr<Localizer> > localizers;

    vector<string> groups;

    vector<vector<Ptr<SceneAnchor> > > anchors;


};


}

#endif

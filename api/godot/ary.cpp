
#include <iostream>
#include <stdexcept>
#include <vector>

#include <opencv2/calib3d/calib3d.hpp>

#include <ary/complex.h>

#include "ary.h"
#include "camera.h"

using namespace ary;
using namespace cv;

class Wrapper {
public:

     Wrapper() {

        camera = camera_open();

        if (!camera) throw std::runtime_error("Unable to open camera");

        try {
            model = Ptr<CameraModel>(new CameraModel("calibration.xml"));
        } catch (const std::runtime_error& e) {
            model = Ptr<CameraModel>(new CameraModel(Size(camera->getWidth(), camera->getHeight())));
        }

	    scene = Ptr<Scene>(new Scene(model, "./ary.yml"));

        anchor_states.resize(scene->size());
        anchor_names.resize(scene->size());

        for (int i = 0; i < scene->size(); i++) {
            anchor_names[i] = format("ARy anchor %d", i);
        }

    }
    virtual ~Wrapper() {}

    SharedCamera camera;

    SharedCameraModel model;
	Ptr<Scene> scene;

    Mat frame;
    Mat gray;

    unsigned long counter;

    vector<SharedLocalization> anchor_states;

    vector<string> anchor_names;

};

void* ary_create() {

    try {

        Wrapper* w = new Wrapper();

        return w;

    } catch (const std::runtime_error& e) {
        return NULL;
    }

}

void ary_delete(void *obj) {

    if (!obj) return;

    delete (Wrapper*) obj;

}

int ary_process(void *obj) {

    Wrapper* user_data = (Wrapper*) obj;

    if (!user_data->camera) return 0;

    user_data->frame = user_data->camera->getFrameIfNewer(user_data->counter);

	if (user_data->frame.empty()) return 0;

    cvtColor(user_data->frame, user_data->gray, COLOR_RGB2GRAY);

	vector<SharedLocalization> localizations = user_data->scene->localize(user_data->gray);

    for (int i = 0; i < user_data->anchor_states.size(); i++) {

        user_data->anchor_states[i].release();

    }

	for (int i = 0; i < localizations.size(); i++) {

        user_data->anchor_states[localizations[i]->getIdentifier()] = localizations[i];

	}

}

int ary_anchor_count(void *obj) {

    Wrapper* user_data = (Wrapper*) obj;

    return user_data->scene->size();

}

int ary_anchor_active(void *obj, int id) {

     Wrapper* user_data = (Wrapper*) obj;

    if (id < 0 || id >= user_data->scene->size()) return 0;

    return  user_data->anchor_states[id] ? 1 : 0;

}

int ary_anchor_type(void *obj, int id) {

    return 0;

}



ary_transform ary_anchor_transform(void *obj, int id) {

    ary_transform t;

    t.data[0] = 1;
    t.data[1] = 0;
    t.data[2] = 0;
    t.data[3] = 0;
    t.data[4] = 1;
    t.data[5] = 0;
    t.data[6] = 0;
    t.data[7] = 0;
    t.data[8] = 1;
    t.data[9] = 0;
    t.data[10] = 0;
    t.data[11] = 0;

    Wrapper* user_data = (Wrapper*) obj;

    if (id < 0 || id >= user_data->scene->size()) return t;

    CameraPosition pos = user_data->anchor_states[id]->getCameraPosition();

    pos.rotation(1) = - pos.rotation(1);
    pos.rotation(2) = - pos.rotation(2);

    pos.translation(1) = - pos.translation(1);
    pos.translation(2) = - pos.translation(2);

    cv::Mat basis, origin;

    origin = Mat(pos.translation);
    cv::Rodrigues(pos.rotation, basis);

    basis = basis.t();
    origin = (basis * origin) * -1;

    t.data[0] = basis.at<float>(0, 0);
    t.data[1] = basis.at<float>(1, 0);
    t.data[2] = basis.at<float>(2, 0);
    t.data[3] = basis.at<float>(0, 1);
    t.data[4] = basis.at<float>(1, 1);
    t.data[5] = basis.at<float>(2, 1);
    t.data[6] = basis.at<float>(0, 2);
    t.data[7] = basis.at<float>(1, 2);
    t.data[8] = basis.at<float>(2, 2);

    t.data[9] = origin.at<float>(0, 0);
    t.data[10] = origin.at<float>(1, 0);
    t.data[11] = origin.at<float>(2, 0);

    return t;

}

ary_transform ary_anchor_itransform(void *obj, int id) {

    ary_transform t;

    t.data[0] = 1;
    t.data[1] = 0;
    t.data[2] = 0;
    t.data[3] = 0;
    t.data[4] = 1;
    t.data[5] = 0;
    t.data[6] = 0;
    t.data[7] = 0;
    t.data[8] = 1;
    t.data[9] = 0;
    t.data[10] = 0;
    t.data[11] = 0;

    Wrapper* user_data = (Wrapper*) obj;

    if (id < 0 || id >= user_data->scene->size()) return t;

    CameraPosition pos = user_data->anchor_states[id]->getCameraPosition();

    pos.rotation(1) = - pos.rotation(1);
    pos.rotation(2) = - pos.rotation(2);

    pos.translation(1) = - pos.translation(1);
    pos.translation(2) = - pos.translation(2);

    cv::Mat basis, origin;

    origin = Mat(pos.translation);
    cv::Rodrigues(pos.rotation, basis);

    t.data[0] = basis.at<float>(0, 0);
    t.data[1] = basis.at<float>(1, 0);
    t.data[2] = basis.at<float>(2, 0);
    t.data[3] = basis.at<float>(0, 1);
    t.data[4] = basis.at<float>(1, 1);
    t.data[5] = basis.at<float>(2, 1);
    t.data[6] = basis.at<float>(0, 2);
    t.data[7] = basis.at<float>(1, 2);
    t.data[8] = basis.at<float>(2, 2);

    t.data[9] = origin.at<float>(0, 0);
    t.data[10] = origin.at<float>(1, 0);
    t.data[11] = origin.at<float>(2, 0);

    return t;

}

const char* ary_anchor_name(void *obj, int id) {

    Wrapper* user_data = (Wrapper*) obj;

    if (id < 0 || id >= user_data->scene->size()) return NULL;

    return user_data->anchor_names[id].c_str();

}

ary_size ary_camera_size(void *obj) {

    ary_size size;

    Wrapper* user_data = (Wrapper*) obj;

    if (!user_data->camera) return size;

    size.width = user_data->camera->getWidth();

    size.height = user_data->camera->getHeight();

    return size;

}



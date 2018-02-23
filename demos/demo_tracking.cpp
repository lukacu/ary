#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <opencv2/videoio.hpp>

#include <ary/camera.h>
#include <ary/complex.h>

using namespace std;
using namespace cv;
using namespace ary;

#define WINDOW_NAME "ARy Test - Binary"


int main(int argc, char** argv) {

	Mat model, frame, gray;

	namedWindow(WINDOW_NAME, WINDOW_AUTOSIZE);

	VideoCapture capture(1);

	capture >> frame;
	if (frame.empty()) {
		cout <<  "Could not open camera" << endl;
		return -1;
	}

	SharedCameraModel camera(new CameraModel(frame.size()));
	KeypointTrackingLocalizer localizer(camera);

    for (int i = 1; i < argc; i++) {

	    // get ground truth image
	    model = imread(argv[i], IMREAD_GRAYSCALE);
	    if (model.empty()) {
		    cout <<  "Could not open the model image" << endl;
		    continue;
	    }

        localizer.add(model, 1);

    }

	while (true) {

		capture >> frame;
		if (frame.empty()) break;

        cvtColor(frame, gray, COLOR_RGB2GRAY);

		vector<SharedLocalization> localizations = localizer.localize(gray);

		for (int i = 0; i < localizations.size(); i++) {

            localizations[i]->draw(frame, camera);

		}

		imshow(WINDOW_NAME, frame);
		waitKey(30);

	}

	return 0;
}

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

    if (argc < 2) return 0;

	SharedCameraModel camera(new CameraModel(frame.size()));
	Scene scene(camera, argv[1]);

	while (true) {

		capture >> frame;
		if (frame.empty()) break;

        cvtColor(frame, gray, COLOR_RGB2GRAY);

		vector<SharedLocalization> localizations = scene.localize(gray);

		for (int i = 0; i < localizations.size(); i++) {

            localizations[i]->draw(frame, camera);

		}

		imshow(WINDOW_NAME, frame);
		waitKey(30);

	}

	return 0;
}

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#if CV_MAJOR_VERSION == 3
#include <opencv2/videoio.hpp>
#endif

#include <ary/camera.h>
#include <ary/complex.h>

using namespace std;
using namespace cv;
using namespace ary;

#define WINDOW_NAME "ARy Test - Scene"


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
	Ptr<Scene> scene(new Scene(camera, argv[1]));

	while (true) {

		capture >> frame;
		if (frame.empty()) break;

        cvtColor(frame, gray, COLOR_RGB2GRAY);

		vector<SharedLocalization> localizations = scene->localize(gray);

		for (int i = 0; i < localizations.size(); i++) {

            localizations[i]->draw(frame, camera);

		}

		imshow(WINDOW_NAME, frame);

		int k = waitKey(30);
		if ((char)k == 'r') {
			cout << "Reloading markers" << endl;
			scene = Ptr<Scene>(new Scene(camera, argv[1]));
		} else if ((char)k == 'q') {
			break;
		}
	}

	return 0;
}

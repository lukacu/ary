#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#if CV_MAJOR_VERSION == 3
#include <opencv2/videoio.hpp>
#endif

#include <ary/camera.h>
#include <ary/binary.h>
#include <ary/utilities.h>

using namespace std;
using namespace cv;
using namespace ary;

Mat generate_homography(float perspective_variation, float size_variation, Size bounds, vector<Point2f> points) {

    Mat_<float> H;

    while (true) {

        float size_random = max(0.01f,  1 + (0.5f - (static_cast <float> (rand()) / static_cast <float> (RAND_MAX))) * size_variation * 2);
        float p1_random = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX)) * perspective_variation;
        float p2_random = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX)) * perspective_variation;

        float rotation_random = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX)) * M_PI * 2;
        float x_random = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX)) * bounds.width;
        float y_random = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX)) * bounds.height;

        Mat_<float> R = (Mat_<float>(3, 3) << cos(rotation_random), -sin(rotation_random), 0, sin(rotation_random), cos(rotation_random), 0, 0, 0, 1);
        Mat_<float> T = (Mat_<float>(3, 3) << 1, 0, x_random, 0, 1, y_random, 0, 0, 1);
        Mat_<float> S = (Mat_<float>(3, 3) << size_random, 0, 0, 0, size_random, 0, 0, 0, 1);
        Mat_<float> P = (Mat_<float>(3, 3) << 1, 0, 0, 0, 1, 0, p1_random, p2_random, 1);

        H = S * T * R * P ;

        H /= H.at<float>(2, 2);

        if (points.size() == 0) break;

        bool valid = true;

        for (int i = 0; i < points.size(); i++) {

            Mat_<float> p = (Mat_<float>(3, 1) << points[i].x, points[i].y, 1);

            p = H * p;

            p /= p(2);

            valid &= (p(0) > 0 && p(0) < bounds.width && p(1) > 0 && p(1) < bounds.height);


        }

        if (valid) break;

    }


    return H;

}



int main(int argc, char** argv) {

	Mat model, frame, gray;

    Size frame_size(800, 600);

    CommandLineParser parser(argc, argv,
      "{help h       |          | Print this help}"
      "{iterations i | 100      | Number of iterations}"
      "{size         | 0.1      | Size variance}"
      "{perspective  | 0.005    | Perspective variance}"
      "{noise        | 1.0      | Amount of pixel noise}"
      "{@M0 | | }"
      "{@M1 | | }"
      "{@M2 | | }"
      "{@M3 | | }"
      "{@M4 | | }"
      "{@M5 | | }"
    );

    if (!parser.check()) {
        parser.printErrors();
        return 1;
    }

    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }

  //  srand(clock());

    int iterations = parser.get<int>("iterations");

	SharedCameraModel camera(new CameraModel(frame_size));
	BinaryPatternLocalizer localizer(camera);

    vector<Mat> markers;

    for (int i = 0; i < 6; i++) {

        string name = parser.get<string>(format("@M%d", i));

        if (name.empty()) break;

	    // get ground truth image
	    model = imread(name, IMREAD_GRAYSCALE);
	    if (model.empty()) {
		    cout <<  "Could not open the model image" << endl;
		    continue;
	    }

        localizer.add(model, 50);
        markers.push_back(model);

    }

    float size_variation = parser.get<float>("size");
    float perspective_variation = parser.get<float>("perspective");

    Mat_<int> confusion(localizer.size(), localizer.size());
    Mat_<float> errors(localizer.size(), 1);

    confusion.setTo(0);
    errors.setTo(0);

    frame = Mat(frame_size, CV_8UC1);

    for (int m = 0; m < localizer.size(); m++) {

        cout << "Processing marker " << m << endl;

        vector<Point2f> points;
        points.push_back(Point2f(0, 0));
        points.push_back(Point2f((float)markers[m].cols, 0));
        points.push_back(Point2f((float)markers[m].cols, (float)markers[m].rows));
        points.push_back(Point2f(0, (float)markers[m].rows));

	    for (int a = 0; a < iterations; a++) {

            randu(frame, Scalar(200), Scalar(250));

            //frame.setTo(100);
            Mat H = generate_homography(perspective_variation, size_variation, frame_size, points);

            warpImage(frame, markers[m], H);

		    vector<SharedLocalization> localizations = localizer.localize(frame);

		    for (int j = 0; j < localizations.size(); j++) {

                confusion(localizations[j]->getIdentifier(), m)++;

                if (localizations[j]->getIdentifier() != m) continue;

                SharedPlanarLocalization pl = localizations[j].dynamicCast<PlanarLocalization>();

                vector<Point2f> corners = pl->getPlaneCorners();

                float error = 0;

                for (int k = 0; k < points.size(); k++) {

                    Mat_<float> p = (Mat_<float>(3, 1) << points[k].x, points[k].y, 1);

                    p = H * p; p /= p(2);

                    error += (float)(pow((p(0) - corners[k].x), 2) + pow((p(1) - corners[k].y), 2));

                }

                errors(m) += error / 4;

		    }

	    }

        errors(m) /= confusion(m, m);

    }

    cout << "Confusion matrix" << endl << confusion << endl;

    cout << "Corner errors" << endl << errors << endl;



	return 0;
}

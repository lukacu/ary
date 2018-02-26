#include <iostream>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "ary/binary.h"

using namespace cv;
using namespace std;

namespace ary {

#define PATTERN_SIZE 100

BinaryPattern::BinaryPattern(const Mat& img, float size) : size(size) {

	if (img.cols != img.rows) {
		throw runtime_error("Not a square pattern");
	}

	int msize = PATTERN_SIZE;

	Mat src(msize, msize, CV_8UC1);
	Point2f center((msize - 1) / 2.0f, (msize - 1) / 2.0f);
	Mat rot_mat(2, 3, CV_32F);

	resize(img, src, Size(msize, msize));
	Mat subImg = src(Range(msize / 4, 3 * msize / 4), Range(msize / 4, 3 * msize / 4));
	markers.push_back(subImg);

	rot_mat = getRotationMatrix2D(center, 90, 1.0);

	for (int i = 1; i < 4; i++) {
		Mat dst = Mat(msize, msize, CV_8UC1);
		rot_mat = getRotationMatrix2D(center, -i * 90, 1.0);
		warpAffine(src, dst, rot_mat, Size(msize, msize));
		Mat subImg = dst(Range(msize / 4, 3 * msize / 4), Range(msize / 4, 3 * msize / 4));
		markers.push_back(subImg);
	}

}

BinaryPattern::~BinaryPattern() {};

float BinaryPattern::getSize()  {
	return size;
}


double BinaryPattern::match(const Mat& src, int& orientation) {

	int i;
	double tempsim;
	double N = (double)(PATTERN_SIZE * PATTERN_SIZE / 4);
	double nom, den;

	Scalar mean_ext, std_ext, mean_int, std_int;

	Mat interior = src(cv::Range(PATTERN_SIZE / 4, 3 * PATTERN_SIZE / 4), cv::Range(PATTERN_SIZE / 4, 3 * PATTERN_SIZE / 4));

	meanStdDev(src, mean_ext, std_ext);
	meanStdDev(interior, mean_int, std_int);

	//printf("ext: %f int: %f \n", mean_ext.val[0], mean_int.val[0]);

	if ((mean_ext.val[0] > mean_int.val[0]))
		return -1;

	double normSrcSq = pow(norm(interior), 2);

	//zero_mean_mode;
	int zero_mean_mode = 1;

	//use correlation coefficient as a robust similarity measure
	double confidence = -1.0;
	for (i = 0; i < markers.size(); i++) {

		double const nnn = pow(norm(markers.at(i)), 2);

		if (zero_mean_mode == 1) {

			double const mmm = mean(markers.at(i)).val[0];

			nom = interior.dot(markers.at(i)) - (N * mean_int.val[0] * mmm);
			den = sqrt( (normSrcSq - (N * mean_int.val[0] * mean_int.val[0]) ) * (nnn - (N * mmm * mmm)));
			tempsim = nom / den;
		}
		else
		{
			tempsim = interior.dot(markers.at(i)) / (sqrt(normSrcSq * nnn));
		}

		if (tempsim > confidence) {
			confidence = tempsim;
			orientation = i;
		}
	}

	return confidence;
}


BinaryPatternLocalizer::BinaryPatternLocalizer(const SharedCameraModel& camera, double threshold, float block_size, double confidence_threshold) : Localizer(camera),
    threshold(threshold), block_size(block_size), confidence_threshold(confidence_threshold) {

}

BinaryPatternLocalizer::~BinaryPatternLocalizer() {

}

void BinaryPatternLocalizer::add(const Mat& model, float size) {

    patterns.push_back(BinaryPattern(model, size));

}


int BinaryPatternLocalizer::size() {

    return (int)patterns.size();

}


vector<SharedLocalization> BinaryPatternLocalizer::localize(const Mat& image) {

	vector<Point2f> roi(4);
	vector<Point2f> refined(4);
    vector<Point2f> rotated(4);

	vector<vector<Point> > contours;
	vector<Point> polycont;

    //binarize image
    if (image.channels() == 3) {
	    cvtColor(image, grayImage, CV_BGR2GRAY);
    }
    else {
	    image.copyTo(grayImage);
    }

    adaptiveThreshold(grayImage, binImage, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY_INV, block_size, (int) threshold);
    dilate(binImage, binImage, Mat());

	int avsize = (binImage.rows + binImage.cols) / 2;

	//find contours in binary image
	cv::findContours(binImage, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

	unsigned int i;
	Point p;
	int pMinX, pMinY, pMaxY, pMaxX;
    Mat rectifiedPatch(PATTERN_SIZE, PATTERN_SIZE, CV_8UC1);;

    vector<SharedLocalization> localizations;

	for (i = 0; i < contours.size(); i++) {
		Mat contourMat = Mat (contours[i]);
		const double per = arcLength( contourMat, true);
		//check the perimeter

		if (per > (avsize / 6) && per < (4 * avsize)) {

			polycont.clear();
			approxPolyDP( contourMat, polycont, per * 0.02, true);
			//check rectangularity and convexity
			if (polycont.size() == 4 && isContourConvex(Mat (polycont))) {

				//locate the 2D box of contour,
				p = polycont.at(0);
				pMinX = pMaxX = p.x;
				pMinY = pMaxY = p.y;
				int j;
				for (j = 1; j < 4; j++) {
					p = polycont.at(j);
					if (p.x < pMinX) {
						pMinX = p.x;
					}
					if (p.x > pMaxX) {
						pMaxX = p.x;
					}
					if (p.y < pMinY) {
						pMinY = p.y;
					}
					if (p.y > pMaxY) {
						pMaxY = p.y;
					}
				}
				Rect box(pMinX, pMinY, pMaxX - pMinX + 1, pMaxY - pMinY + 1);

				//find the upper left vertex
				double d;
				double dmin = (4 * avsize * avsize);
				int v1 = -1;
				for (j = 0; j < 4; j++) {
					d = norm(polycont.at(j));
					if (d < dmin) {
						dmin = d;
						v1 = j;
					}
				}

				//store vertices in refinedVertices and enable sub-pixel refinement if you want
				for (j = 0; j < 4; j++) {
					refined[j] = polycont.at(j);
				}

				//refine corners
				cornerSubPix(grayImage, refined, Size(3, 3), Size(-1, -1), TermCriteria(1, 3, 1));

				//rotate vertices based on upper left vertex; this gives you the most trivial homogrpahy
				for (j = 0; j < 4; j++) {
					roi[j] = Point2f(refined.at((4 + v1 - j) % 4).x - pMinX, refined.at((4 + v1 - j) % 4).y - pMinY);
				}

	            //rectify the candidate pattern (find homography and warp the ROI)
	            rectifyPattern(grayImage, roi, box, rectifiedPatch);

	            double confidence = 0;
	            int orientation;

	            int id = identifyPattern(rectifiedPatch, confidence, orientation);
	            //push-back pattern in the stack of foundPatterns and find its extrinsics

	            if (id >= 0) {

		            for (j = 0; j < 4; j++) {
			            rotated[j] = refined.at((8 - orientation + v1 - j) % 4);
		            }

		            //find the transformation (from camera CS to pattern CS)
		            CameraPosition camera = calculateExtrinsics(patterns[id].getSize(), rotated);
		            localizations.push_back(Ptr<PlanarLocalization>(new PlanarLocalization(id, camera, Size2f(patterns[id].getSize(), patterns[id].getSize()), rotated, (float) confidence)));

	            }


			}
		}
	}

    return localizations;
}

//solves the exterior orientation problem between patten and camera
CameraPosition BinaryPatternLocalizer::calculateExtrinsics(const float size, const vector<Point2f>& imagePoints) {

    CameraPosition camera;

	//3D points in pattern coordinate system
	vector<Point3f> objectPoints(4);
	objectPoints[0] = Point3f(-size / 2, -size / 2, 0);
	objectPoints[1] = Point3f(size / 2, -size / 2, 0);
	objectPoints[2] = Point3f(size / 2, size / 2, 0);
	objectPoints[3] = Point3f(-size / 2, size / 2, 0);

    Mat rotation, translation;

	solvePnP(objectPoints, imagePoints, getCameraModel()->getIntrinsics(), getCameraModel()->getDistortion(), rotation, translation);

	rotation.convertTo(camera.rotation, CV_32F);
	translation.convertTo(camera.translation, CV_32F);

    return camera;
}


void BinaryPatternLocalizer::rectifyPattern(const Mat& src, const vector<Point2f>& roi, Rect& clamp, Mat& dst) {


	vector<Point2f> normalized(4);

	//corner of normalized area
	normalized[0] = Point2f(0, 0);
	normalized[1] = Point2f(PATTERN_SIZE - 1, 0);
	normalized[2] = Point2f(PATTERN_SIZE - 1, PATTERN_SIZE - 1);
	normalized[3] = Point2f(0, PATTERN_SIZE - 1);

	//compute the homography
	Mat homography = getPerspectiveTransform(roi, normalized);

	cv::Mat subImg = src(cv::Range(clamp.y, clamp.y + clamp.height), cv::Range(clamp.x, clamp.x + clamp.width));

	//warp the input based on the homography model to get the normalized ROI
	cv::warpPerspective( subImg, dst, homography, Size(PATTERN_SIZE, PATTERN_SIZE));

}

int BinaryPatternLocalizer::identifyPattern(const Mat& src, double& confidence, int& orientation) {
	if (patterns.size() < 1) {
		return -1;
	}

	unsigned int j;
	orientation = 0;
	int match = -1;

	//use correlation coefficient as a robust similarity measure
	confidence = confidence_threshold;
	for (j = 0; j < patterns.size(); j++) {
        int o;

		double m = patterns[j].match(src, o);

		if (m > confidence) {
			confidence = m;
			match = j;
            orientation = o;
		}

	}

	return match;

}


}

#include "aruco/aruco.h"
#include "aruco/MarkerDetector.h"
#include "iostream"
#include <string>
#include "aruco/MarkerDetector_impl.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;
using namespace aruco;

int main() {
	VideoCapture cap;
	cap.open(0);
	if (!cap.isOpened()) { return -1; }

	//string Name;
	//cin >> Name;
	Mat myImage;
	//= imread(Name);

	namedWindow("window", WINDOW_AUTOSIZE);
	char key = -1;
	while (key != 27) {
		cap >> myImage;
		cap.read(myImage);

		// creation d’un detecteur de marqueurs
		MarkerDetector myDetector;

		// liste de marqueurs : sera remplie par ArUco
		// detect markers and for each one, draw infoand i t s boundaries in the image
		for (auto m : myDetector.detect(myImage)) {
			cout << m << endl;
			m.draw(myImage);
		}
		imshow("window", myImage);
		key = waitKey(42);
	}
	imshow("window", myImage);
	waitKey(0);
	return 1;
}
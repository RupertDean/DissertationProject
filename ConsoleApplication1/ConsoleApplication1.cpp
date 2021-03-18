#define WINVER 0x0500
#define TRAINING false
#define LOAD_PROFILE false
#define CVUI_IMPLEMENTATION
#define WINDOW_NAME "CVUI Hello World!"

#include "opencv2/opencv.hpp"
#include "opencv2/face.hpp"
#include "cvui.h"

#include <windows.h>
#include <iostream>
#include <time.h>
#include <fstream>
#include <math.h>

using namespace cv::face;
using namespace std;


// Function to detect faces in given image
void faceDetector(const cv::Mat& image, vector<cv::Rect>& faces, cv::CascadeClassifier& face_cascade) {
	// Histogram equalization generally aids in face detection
	equalizeHist(image, image);
	faces.clear();

	// Run the cascade classifier
	face_cascade.detectMultiScale(
		image,
		faces,
		1.5, // pyramid scale factor
		3,   // lower thershold for neighbors count
			 // here we hint the classifier to only look for one face
		cv::CASCADE_SCALE_IMAGE + cv::CASCADE_FIND_BIGGEST_OBJECT);
}

// Returns distance between 2 2D points
float dist(cv::Point2f a, cv::Point2f b) {
	return sqrtf(pow(b.x - a.x, 2) + pow(b.y - a.y, 2));
}

// Returns a vector with distances between points on the face
// 0 and 1 for face angle, 2 left cheek, 3 right cheek, 4 eye to brow and mouth to chin, 5 mouth size
// 6 left eye size, 7 right eye size
vector<float> calculateVals(vector<cv::Point2f> current, cv::Rect face) {
	vector<float> vals = {
	0, 0,

	dist(current[0], current[36]) + dist(current[2], current[33]) + dist(current[4], current[48]),

	dist(current[16], current[45]) + dist(current[14], current[33]) + dist(current[12], current[54]),

	dist(current[17], current[36]) + dist(current[19], current[37]) + dist(current[21], current[39]) +
		dist(current[22], current[42]) + dist(current[24], current[44]) + dist(current[26], current[45]) +
		dist(current[5], current[48]) + dist(current[8], current[57]) + dist(current[11], current[54]),

	(dist(current[50], current[58]) + dist(current[51], current[57]) + dist(current[52], current[56])) /
		dist(current[48], current[54]),

	(dist(current[37], current[41]) + dist(current[38], current[40])) / dist(current[36], current[39]),

	(dist(current[43], current[47]) + dist(current[44], current[46])) / dist(current[43], current[45])

	};

	for (int i = 0; i < vals.size(); i++) {
		vals[i] /= face.area();
	}

	vals[0] = ((face.y + face.height) - face.y) / ((face.x + face.width) - face.x);
	vals[1] = (current[0].y - current[16].y) / (current[0].x - current[16].x);

	return vals;
}

// Estimates movements based on differences in distances between base and current images
cv::String calculateMovements(vector<float> base, vector<float> current, INPUT kb, INPUT ms) {
	float angleCurrent = atan((current[1] - current[0]) / (1.0f + current[0] * current[1])) * 180.0f / 3.4159265f;
	float angleBase = atan((base[1] - base[0]) / (1.0f + base[0] * base[1])) * 180.0f / 3.4159265f;

	cv::String result = "";


	if (current[5] / base[5] > 1.3f) {
		kb.ki.wVk = VK_SPACE; // virtual-key code for the space button
		kb.ki.dwFlags = 0; // 0 for key press

		SendInput(1, &kb, sizeof(INPUT));

		result += "MO ";
	}

	if (angleCurrent / angleBase > 1.2f) {
		ms.mi.dwFlags = MOUSEEVENTF_RIGHTDOWN;
		SendInput(1, &ms, sizeof(INPUT));

		result += "RR ";
	}
	else if (angleCurrent / angleBase < 0.8f) {
		ms.mi.dwFlags = MOUSEEVENTF_LEFTDOWN;
		SendInput(1, &ms, sizeof(INPUT));

		result += "RL ";
	}
	else {
		ms.mi.dwFlags = MOUSEEVENTF_RIGHTUP;
		SendInput(1, &ms, sizeof(INPUT));

		ms.mi.dwFlags = MOUSEEVENTF_LEFTUP;
		SendInput(1, &ms, sizeof(INPUT));
	}


	if (current[4] / base[4] < 0.9f) {

		result += "PD ";
	}
	else if (current[4] / base[4] > 1.1f) {

		result += "PU ";
	}



	if (current[6] / base[6] < 0.8f && current[7] / base[7] < 0.8f) {
		kb.ki.wVk = 0x44; // virtual-key code for the A key
		kb.ki.dwFlags = 0; // 0 for key press

		//SendInput(1, &kb, sizeof(INPUT));

		result += "EC ";
	}
	else if (current[6] / base[6] < 0.9f) {
		kb.ki.wVk = 0x44; // virtual-key code for the A key
		kb.ki.dwFlags = 0; // 0 for key press

		//SendInput(1, &kb, sizeof(INPUT));

		result += "LE ";
	}
	else if (current[7] / base[7] < 0.9f) {
		kb.ki.wVk = 0x45; // virtual-key code for the E key
		kb.ki.dwFlags = 0; // 0 for key press

		SendInput(1, &kb, sizeof(INPUT));

		result += "RE ";
	}


	if (current[2] / base[2] < 0.9f && current[3] / base[3] > 1.05f) {
		kb.ki.wVk = 0x44; // virtual-key code for the A key
		kb.ki.dwFlags = 0; // 0 for key press

		SendInput(1, &kb, sizeof(INPUT));

		result += "YR ";
	}
	else if (current[3] / base[3] < 0.9f && current[2] / base[2] > 1.05f) {
		kb.ki.wVk = 0x41; // virtual-key code for the D key
		kb.ki.dwFlags = 0; // 0 for key press

		SendInput(1, &kb, sizeof(INPUT));

		result += "YL ";
	}

	if (result == "") {
		return "None";
	}

	return result;

}

// Same as previous function but uses raw input
cv::String calculateMovementsRaw(vector<float> base, vector<float> current, INPUT kb, INPUT ms) {
	POINT p;
	float angleCurrent = atan((current[1] - current[0]) / (1.0f + current[0] * current[1])) * 180.0f / 3.4159265f;
	float angleBase = atan((base[1] - base[0]) / (1.0f + base[0] * base[1])) * 180.0f / 3.4159265f;

	cv::String result = "";


	if (current[5] / base[5] > 1.3f) {
		kb.ki.wVk = VK_SPACE; // virtual-key code for the space button
		kb.ki.dwFlags = 0; // 0 for key press

		SendInput(1, &kb, sizeof(INPUT));

		result += "MO ";
	}

	if (angleCurrent / angleBase > 1.2f) {
		ms.mi.dwFlags = MOUSEEVENTF_RIGHTDOWN;
		SendInput(1, &ms, sizeof(INPUT));

		result += "RR ";
	}
	else if (angleCurrent / angleBase < 0.8f) {
		ms.mi.dwFlags = MOUSEEVENTF_LEFTDOWN;
		SendInput(1, &ms, sizeof(INPUT));

		result += "RL ";
	}
	else {
		ms.mi.dwFlags = MOUSEEVENTF_RIGHTUP;
		SendInput(1, &ms, sizeof(INPUT));

		ms.mi.dwFlags = MOUSEEVENTF_LEFTUP;
		SendInput(1, &ms, sizeof(INPUT));
	}


	if (current[4] / base[4] < 0.9f) {


		result += "PD ";
	}
	else if (current[4] / base[4] > 1.1f) {


		result += "PU ";
	}



	if (current[6] / base[6] < 0.8f && current[7] / base[7] < 0.8f) {
		

		result += "EC ";
	}
	else if (current[6] / base[6] < 0.9f) {
		

		result += "LE ";
	}
	else if (current[7] / base[7] < 0.9f) {
		

		result += "RE ";
	}


	if (current[2] / base[2] < 0.9f && current[3] / base[3] > 1.05f) {
		

		result += "YR ";
	}
	else if (current[3] / base[3] < 0.9f && current[2] / base[2] > 1.05f) {
		

		result += "YL ";
	}

	if (result == "") {
		return "None";
	}

	return result;
}

// Saves landmarks in current frame
vector<float> saveBaseLandmarks(vector<cv::Point2f> shapes, cv::Rect face) {
	vector<cv::Point2f> base = shapes;
	vector<float> vals = calculateVals(base, face);
	return vals;
}

int main(int, char**) {
	cv::Mat window = cv::Mat(1000, 1000, CV_8UC3);
	int count = 0;

	// Init a OpenCV window and tell cvui to use it.
	cv::namedWindow(WINDOW_NAME);
	cvui::init(WINDOW_NAME);

	// Keyboard input device
	INPUT kb = {};
	kb.type = INPUT_KEYBOARD;
	kb.ki.wScan = 0;
	kb.ki.time = 0;
	kb.ki.dwExtraInfo = 0;

	// Mouse input device
	INPUT ms = {};
	ms.type = INPUT_MOUSE;

	// Hardware input device
	INPUT hd = {};
	hd.type = INPUT_HARDWARE;


	// open the first webcam plugged in the computer
	cv::VideoCapture camera(0);
	if (!camera.isOpened()) {
		cerr << "ERROR: Could not open camera" << endl;
		return 1;
	}

	// this will contain the image from the webcam
	cv::Mat frame;
	cv::Mat gray;
	cv::Mat flipped;
	cv::Mat reduced;

	vector<cv::Rect> faces;

	// load cascade for face detection
	const cv::String cascade_name = "C:/lib/opencv/data/haarcascades/haarcascade_frontalface_default.xml";
	cv::CascadeClassifier face_cascade;

	if (not face_cascade.load(cascade_name)) {
		cerr << "Cannot load cascade classifier from file: " << cascade_name << endl;
		return -1;
	}

#if LOAD_PROFILE
	// load cascade for profile face detection
	const String profile_cascade_name = "C:/lib/opencv/data/haarcascades/haarcascade_profileface.xml";
	CascadeClassifier profile_face_cascade;

	if (not profile_face_cascade.load(profile_cascade_name)) {
		cerr << "Cannot load cascade classifier from file: " << profile_cascade_name  << endl;
		return -1;
	}
#endif

	const string facemark_filename = "C:/lib/opencv/data/landmark_models/lbfmodel.yaml";
	cv::Ptr<cv::face::Facemark> facemark = createFacemarkLBF();
	facemark->loadModel(facemark_filename);
	cout << "Loaded facemark LBF model" << endl;

	vector<vector<cv::Point2f>> shapes;
	vector<float> base = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

	cv::Rect lastFace;
	bool cropped = false;
	float offsetX, offsetY = 0, width = 0, height = 0;

	constexpr int buffer = 30;

	int mode = 1;
	cv::String message;

	while (true) {
		// Fill the frame with a nice color
		window = cv::Scalar(49, 52, 49);

		// Update cvui internal stuff
		cvui::update();

#if TRAINING
		int i = 1;

		ofstream faces_csv;
		faces_csv.open("Python_Part/faces.csv");

		faces_csv << "num" + ',' + to_string(140) + ',' + "Pitch Up" + ',' + "Pitch Down" + ',' + "Roll Left" + ',' + "Roll Right" + ',' + "Yaw Left" + ',' + "Yaw Right" + ',' + "Mouth Open" + ',' + "Eyes Closed" + ",\n";

		while (1) {
			camera >> frame;

			cvtColor(frame, gray, COLOR_BGR2GRAY);

			// ... obtain an image in img
			faceDetector(gray, faces, face_cascade);
			// Check if faces detected or not
			if (faces.size() != 0) {
				// We assume a single face so we look at the first only
				cv::rectangle(frame, faces[0], Scalar(255, 0, 0), 2);

				if (facemark->fit(gray, faces, shapes)) {
					// Draw the detected landmarks
					drawFacemarks(frame, shapes[0], cv::Scalar(0, 0, 255));
				}

			}

			imshow("Webcam", frame);

			if (cv::waitKey(10) == 32) {
				saveBaseLandmarks(shapes);

				for (int i = 0; i < 68; i++) {
					faces_csv << to_string(shapes[0][i].x) + ',' + to_string(shapes[0][i].y) + ',';
				}

				faces_csv << to_string(faces[0].x) + ',' + (to_string(faces[0].x) + to_string(faces[0].width));
				faces_csv << to_string(faces[0].y) + ',' + (to_string(faces[0].y) + to_string(faces[0].height));

				faces_csv << "\n";

				i++;
			}

			if (cv::waitKey(1) == 27) break;
		}

		faces_csv.close();
#endif

#if not TRAINING
		// capture the next frame from the webcam
		camera >> frame;

		if (cropped) {
			reduced = frame(cv::Rect(offsetX, offsetY, width, height));
		}
		else {
			reduced = frame;
		}

		// The cascade classifier works best on grayscale images
		cvtColor(reduced, gray, cv::COLOR_BGR2GRAY);

		// ... obtain an image in img
		faceDetector(gray, faces, face_cascade);
		// Check if faces detected or not
		if (faces.size() > 0) {
			// We assume a single face so we look at the first only
			cv::rectangle(reduced, faces[0], cv::Scalar(255, 0, 0), 2);

			if (cropped) {
				offsetX += faces[0].x - buffer;
				offsetY += faces[0].y - buffer;
				width = faces[0].width + (2 * buffer);
				height = faces[0].height + (2 * buffer);
			}
			else {
				offsetX = faces[0].x - buffer;
				offsetY = faces[0].y - buffer;
				width = faces[0].width + (2 * buffer);
				height = faces[0].height + (2 * buffer);
				cropped = true;
			}


			if (offsetX < 0) {
				offsetX = 0;
			}
			else if (offsetX + width > frame.cols) {
				width -= ((offsetX + width) - frame.cols);
			}
			if (offsetY < 0) {
				offsetY = 0;
			}
			else if (offsetY + height > frame.rows) {
				offsetY -= ((offsetY + height) - frame.rows);
			}


			if (facemark->fit(gray, faces, shapes)) {
				// Draw the detected landmarks
				drawFacemarks(reduced, shapes[0], cv::Scalar(0, 0, 255));
			}

			// printf("%f, %f \n", shapes[0][67].x, shapes[0][67].y);

			if (cv::waitKey(1) == 32) base = saveBaseLandmarks(shapes[0], faces[0]);

			if (mode == 1) {
				message = calculateMovements(base, calculateVals(shapes[0], faces[0]), kb, ms);
			}
			else if (mode == -1) {
				message = calculateMovementsRaw(base, calculateVals(shapes[0], faces[0]), kb, ms);
			}

			putText(reduced, message, cv::Point2f(faces[0].x, faces[0].y + faces[0].height), 0, 1.0, cv::Scalar(255, 255, 255), 2);
		}
		else {
			cout << "no face detected" << endl;
			cropped = false;
			offsetX = 0;
			offsetY = 0;
			width = 0;
			height = 0;
		}



#if LOAD_PROFILE
		else {
			faceDetector(gray, faces, profile_face_cascade);
			if (faces.size() != 0) {
				// We assume a single face so we look at the first only
				cv::rectangle(frame, faces[0], Scalar(0, 255, 0), 2);

				if (facemark->fit(gray, faces, shapes)) {
					// Draw the detected landmarks
					drawFacemarks(frame, shapes[0], cv::Scalar(0, 0, 255));
				}

				if (cv::waitKey(1) == 32) base = saveBaseLandmarks(shapes, faces[0].area());
				calculateMovements(base, calculateVals(shapes, faces[0].area()), ip);
			}
			else {
				flip(gray, flipped, 1);
				faceDetector(flipped, faces, profile_face_cascade);

				if (faces.size() != 0) {
					// We assume a single face so we look at the first only
					cv::rectangle(frame, faces[0], Scalar(0, 255, 0), 2);

					if (facemark->fit(gray, faces, shapes)) {
						// Draw the detected landmarks
						drawFacemarks(frame, shapes[0], cv::Scalar(0, 0, 255));
					}

					if (cv::waitKey(1) == 32) base = saveBaseLandmarks(shapes, faces[0].area());
					calculateMovements(base, calculateVals(shapes, faces[0].area()), ip);
				}
			}
		}
#endif

		// Show a button at position (110, 80)
		if (cvui::button(window, 110, 80, "Switch modes")) {
			// The button was clicked, so let's increment our counter.
			mode *= -1;
		}

		// Show how many times the button has been clicked.
		// Text at position (250, 90), sized 0.4, in red.
		cvui::printf(window, 250, 90, 0.4, 0xff0000, "Button click count: %d", count);


		cv::resize(reduced, frame, cv::Size(200, 200), 200, 200, 1);
		cvui::image(frame, 10, 10, window);
		cv::waitKey(1);

		// Show everything on the screen
		cv::imshow(WINDOW_NAME, frame);

#endif
	}


	return 0;
}

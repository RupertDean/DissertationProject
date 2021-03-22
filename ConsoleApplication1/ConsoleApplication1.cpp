// Disable a warning about converting between types thrown due to openCV types
#pragma warning( disable : 4244 )

// Define program constants
#define WINVER 0x0500
#define CVUI_IMPLEMENTATION
#define MY_ESTIMATION_METHOD 0
#define WINDOW_NAME "Facial Game Control"

// Include modules and header files
#include "opencv2/opencv.hpp"
#include "opencv2/face.hpp"
#include "cvui.h"

#include <windows.h>
#include <iostream>
#include <time.h>
#include <fstream>
#include <math.h>
#include <conio.h>
#include <algorithm>

// Namespaces
using namespace cv::face;
using namespace std;

// Returns the index of a pressed key, given the key is listed in the arrays in main
int charFinder(vector<int> keycodes, cv::Mat window) {
	// Bool to loop while a key hasn't been recognised
	bool pressed = false;
	int i = 0;

	while (!pressed) {
		// Loop through available keys
		for (i = 0; i < keycodes.size(); i++) {
			// If key is pressed down
			if (GetAsyncKeyState(keycodes[i]) & 0x8000) {
				// Break the loop
				pressed = true;
				break;
			}
		}

		cv::waitKey(1);
	}

	// Return index of key
	return i;
}

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
float dist2(cv::Point2f a, cv::Point2f b) {
	return sqrtf(pow(b.x - a.x, 2) + pow(b.y - a.y, 2));
}

// Distance between 2 3D points
float dist3(cv::Point3f a, cv::Point3f b) {
	return sqrtf(pow(b.x - a.x, 2) + pow(b.y - a.y, 2) + pow(b.z - a.z, 2));
}

// Midpoint of 2 2D points, ratio can be passed in to get a point weighted further to one side
cv::Point2f mid2(cv::Point2f a, cv::Point2f b, float ratio = 0.5f) {
	return cv::Point2f{ a.x + (ratio * (b.x - a.x)), a.y + (ratio * (b.y - a.y)) };
}

// Midpoint of 2 3D points
cv::Point3f mid3(cv::Point3f a, cv::Point3f b) {
	return cv::Point3f { a.x + (0.5f * (b.x - a.x)), a.y + (0.5f * (b.y - a.y)), a.z + (0.5f * (b.z - a.z)) };
}

#if MY_ESTIMATION_METHOD
// Returns a vector with distances between points on the face
// 0 and 1 for face angle, 2 left cheek, 3 right cheek, 4 eye to brow 5 mouth to chin
// 6 mouth size, 7 left eye size, 8 right eye size
vector<float> calculateVals(vector<cv::Point2f> current, cv::Rect face) {
	vector<float> vals = {
	0, 0,

	dist2(current[0], current[28]) + dist2(current[2], current[33]) + dist2(current[4], current[48]),

	dist2(current[16], current[28]) + dist2(current[14], current[33]) + dist2(current[13], current[54]),

	dist2(current[17], current[36]) + dist2(current[19], current[37]) + dist2(current[21], current[39]) +
		dist2(current[22], current[42]) + dist2(current[24], current[44]) + dist2(current[26], current[45]),
		
	dist2(current[5], current[48]) + dist2(current[8], current[57]) + dist2(current[11], current[54]),

	(dist2(current[50], current[58]) + dist2(current[51], current[57]) + dist2(current[52], current[56])),

	(dist2(current[37], current[41]) + dist2(current[38], current[40])),

	(dist2(current[43], current[47]) + dist2(current[44], current[46]))

	};

	// Calculate area of bounding rectangle and normalise values
	for (int i = 0; i < vals.size(); i++) {
		vals[i] /= face.area();
	}

	// Values for calculating angle of head tilt
	vals[0] = ((face.y + face.height) - face.y) / ((face.x + face.width) - face.x);
	vals[1] = (current[0].y - current[16].y) / (current[0].x - current[16].x);

	return vals;
}

// Estimates movements based on differences in distances between base and current images
cv::String calculateMovements(vector<float> base, vector<float> current, INPUT kb, INPUT ms, int(&saved)[10], vector<int>(&keycodes), int threshold) {
	// Calculate angles of head tilt
	float angleCurrent = atan((current[1] - current[0]) / (1.0f + current[0] * current[1])) * 180.0f / 3.4159265f;
	float angleBase = atan((base[1] - base[0]) / (1.0f + base[0] * base[1])) * 180.0f / 3.4159265f;

	// String to store movements
	cv::String result = "";

	if (current[2] / base[2] < 0.9f && current[3] / base[3] > 1.1f) {
		kb.ki.wVk = keycodes[saved[1]];
		kb.ki.dwFlags = 0;
		SendInput(1, &kb, sizeof(INPUT));

		result += "YR ";
	}
	else if (current[3] / base[3] < 0.9f && current[2] / base[2] > 1.1f) {
		kb.ki.wVk = keycodes[saved[0]];
		kb.ki.dwFlags = 0;
		SendInput(1, &kb, sizeof(INPUT));

		result += "YL ";
	}
	else {
		if (GetKeyState(keycodes[saved[0]]) & 0x8000) {
			kb.ki.wVk = keycodes[saved[0]];
			kb.ki.dwFlags = KEYEVENTF_KEYUP;
			SendInput(1, &kb, sizeof(INPUT));
		}
		if (GetKeyState(keycodes[saved[1]]) & 0x8000) {
			kb.ki.wVk = keycodes[saved[1]];
			kb.ki.dwFlags = KEYEVENTF_KEYUP;
			SendInput(1, &kb, sizeof(INPUT));
		}
	}

	if (current[4] / base[4] < 0.9f) {
		kb.ki.wVk = keycodes[saved[3]];
		kb.ki.dwFlags = 0;
		SendInput(1, &kb, sizeof(INPUT));

		result += "PD ";
	}
	else if (current[4] / base[4] > 1.1f) {
		kb.ki.wVk = keycodes[saved[2]];
		kb.ki.dwFlags = 0;
		SendInput(1, &kb, sizeof(INPUT));

		result += "PU ";
	}
	else {
		if (GetKeyState(keycodes[saved[2]]) & 0x8000) {
			kb.ki.wVk = keycodes[saved[2]];
			kb.ki.dwFlags = KEYEVENTF_KEYUP;
			SendInput(1, &kb, sizeof(INPUT));
		}
		if (GetKeyState(keycodes[saved[3]]) & 0x8000) {
			kb.ki.wVk = keycodes[saved[3]];
			kb.ki.dwFlags = KEYEVENTF_KEYUP;
			SendInput(1, &kb, sizeof(INPUT));
		}
	}

	if (angleCurrent / angleBase > 1.2f) {
		kb.ki.wVk = keycodes[saved[5]];
		kb.ki.dwFlags = 0;
		SendInput(1, &kb, sizeof(INPUT));

		result += "RR ";
	}
	else if (angleCurrent / angleBase < 0.8f) {
		kb.ki.wVk = keycodes[saved[4]];
		kb.ki.dwFlags = 0;
		SendInput(1, &kb, sizeof(INPUT));

		result += "RL ";
	}
	else {
		if (GetKeyState(keycodes[saved[4]]) & 0x8000) {
			kb.ki.wVk = keycodes[saved[4]];
			kb.ki.dwFlags = KEYEVENTF_KEYUP;
			SendInput(1, &kb, sizeof(INPUT));
		}
		if (GetKeyState(keycodes[saved[5]]) & 0x8000) {
			kb.ki.wVk = keycodes[saved[5]];
			kb.ki.dwFlags = KEYEVENTF_KEYUP;
			SendInput(1, &kb, sizeof(INPUT));
		}
	}

	if (current[7] / base[7] < 0.8f && current[8] / base[8] < 0.8f) {
		kb.ki.wVk = keycodes[saved[8]];
		kb.ki.dwFlags = 0;
		SendInput(1, &kb, sizeof(INPUT));

		result += "EC ";
	}
	else if (current[7] / base[7] < 0.9f) {
		kb.ki.wVk = keycodes[saved[6]];
		kb.ki.dwFlags = 0;
		SendInput(1, &kb, sizeof(INPUT));

		result += "LE ";
	}
	else if (current[8] / base[8] < 0.9f) {
		kb.ki.wVk = keycodes[saved[7]];
		kb.ki.dwFlags = 0;
		SendInput(1, &kb, sizeof(INPUT));

		result += "RE ";
	}
	else {
		if (GetKeyState(keycodes[saved[6]]) & 0x8000) {
			kb.ki.wVk = keycodes[saved[6]];
			kb.ki.dwFlags = KEYEVENTF_KEYUP;
			SendInput(1, &kb, sizeof(INPUT));
		}
		if (GetKeyState(keycodes[saved[7]]) & 0x8000) {
			kb.ki.wVk = keycodes[saved[7]];
			kb.ki.dwFlags = KEYEVENTF_KEYUP;
			SendInput(1, &kb, sizeof(INPUT));
		}
		if (GetKeyState(keycodes[saved[8]]) & 0x8000) {
			kb.ki.wVk = keycodes[saved[8]];
			kb.ki.dwFlags = KEYEVENTF_KEYUP;
			SendInput(1, &kb, sizeof(INPUT));
		}
	}

	if (current[6] / base[6] > 1.3f) {
		kb.ki.wVk = keycodes[saved[9]];
		kb.ki.dwFlags = 0;
		SendInput(1, &kb, sizeof(INPUT));

		result += "MO ";
	}
	else {
		if (GetKeyState(keycodes[saved[9]]) & 0x8000) {
			kb.ki.wVk = keycodes[saved[9]];
			kb.ki.dwFlags = KEYEVENTF_KEYUP;
			SendInput(1, &kb, sizeof(INPUT));
		}
	}


	if (result == "") {
		return "None";
	}

	return result;
}

// Same as previous function but for mouse movement
cv::String calculateMovementsMouse(vector<float> base, vector<float> current, INPUT kb, INPUT ms, int(&saved)[6], vector<int>(&keycodes), int sensitivity, int threshold) {
	POINT p;
	float angleCurrent = atan((current[1] - current[0]) / (1.0f + current[0] * current[1])) * 180.0f / 3.4159265f;
	float angleBase = atan((base[1] - base[0]) / (1.0f + base[0] * base[1])) * 180.0f / 3.4159265f;

	cv::String result = "";

	if (current[2] / base[2] < 1.0f && current[3] / base[3] > 1.0f) {
		GetCursorPos(&p);
		SetCursorPos(p.x + (((current[3] / base[3]) - 1) * sensitivity), p.y);

		result += "YR ";
	}
	else if (current[3] / base[3] < 1.0f && current[2] / base[2] > 1.0f) {
		GetCursorPos(&p);
		SetCursorPos((p.x - (1 - (current[3] / base[3]))) * sensitivity, p.y);

		result += "YL ";
	}

	if (current[4] / base[4] < 0.9f) {
		GetCursorPos(&p);
		SetCursorPos(p.x, p.y + (((1 - (current[4] / base[4]))) * sensitivity));

		result += "PD ";
	}
	else if (current[4] / base[4] > 1.1f) {
		GetCursorPos(&p);
		SetCursorPos(p.x, p.y - ((((current[4] / base[4]) - 1)) * sensitivity));

		result += "PU ";
	}

	if (angleCurrent / angleBase > 1.2f) {
		kb.ki.wVk = keycodes[saved[1]]; // virtual-key code for the D key
		kb.ki.dwFlags = 0; // 0 for key press
		SendInput(1, &kb, sizeof(INPUT));

		result += "RR ";
	}
	else if (angleCurrent / angleBase < 0.8f) {
		kb.ki.wVk = keycodes[saved[0]]; // virtual-key code for the D key
		kb.ki.dwFlags = 0; // 0 for key press
		SendInput(1, &kb, sizeof(INPUT));

		result += "RL ";
	}
	else {
		if (GetKeyState(keycodes[saved[0]]) & 0x8000) {
			kb.ki.wVk = keycodes[saved[0]];
			kb.ki.dwFlags = KEYEVENTF_KEYUP;
			SendInput(1, &kb, sizeof(INPUT));
		}
		if (GetKeyState(keycodes[saved[1]]) & 0x8000) {
			kb.ki.wVk = keycodes[saved[1]];
			kb.ki.dwFlags = KEYEVENTF_KEYUP;
			SendInput(1, &kb, sizeof(INPUT));
		}
	}


	if (current[7] / base[7] < 0.8f && current[8] / base[8] < 0.8f) {
		kb.ki.wVk = keycodes[saved[4]]; // virtual-key code for the D key
		kb.ki.dwFlags = 0; // 0 for key press
		SendInput(1, &kb, sizeof(INPUT));

		result += "EC ";
	}
	else if (current[7] / base[7] < 0.9f) {
		kb.ki.wVk = keycodes[saved[2]]; // virtual-key code for the D key
		kb.ki.dwFlags = 0; // 0 for key press
		SendInput(1, &kb, sizeof(INPUT));

		result += "LE ";
	}
	else if (current[8] / base[8] < 0.9f) {
		kb.ki.wVk = keycodes[saved[3]]; // virtual-key code for the D key
		kb.ki.dwFlags = 0; // 0 for key press
		SendInput(1, &kb, sizeof(INPUT));

		result += "RE ";
	}
	else {
		if (GetKeyState(keycodes[saved[2]]) & 0x8000) {
			kb.ki.wVk = keycodes[saved[2]];
			kb.ki.dwFlags = KEYEVENTF_KEYUP;
			SendInput(1, &kb, sizeof(INPUT));
		}
		if (GetKeyState(keycodes[saved[3]]) & 0x8000) {
			kb.ki.wVk = keycodes[saved[3]];
			kb.ki.dwFlags = KEYEVENTF_KEYUP;
			SendInput(1, &kb, sizeof(INPUT));
		}
		if (GetKeyState(keycodes[saved[4]]) & 0x8000) {
			kb.ki.wVk = keycodes[saved[4]];
			kb.ki.dwFlags = KEYEVENTF_KEYUP;
			SendInput(1, &kb, sizeof(INPUT));
		}
	}

	if (current[6] / base[6] > 1.3f) {
		kb.ki.wVk = keycodes[saved[5]]; // virtual-key code for the space button
		kb.ki.dwFlags = 0; // 0 for key press
		SendInput(1, &kb, sizeof(INPUT));

		result += "MO ";
	}
	else {
		if (GetKeyState(keycodes[saved[5]]) & 0x8000) {
			kb.ki.wVk = keycodes[saved[5]];
			kb.ki.dwFlags = KEYEVENTF_KEYUP;
			SendInput(1, &kb, sizeof(INPUT));
		}
	}

	if (result == "") {
		return "None";
	}

	return result;
}

#else
// Calculates distances and varables used for alternative method of head pose etimation proposed in 
//https://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.28.5324&rep=rep1&type=pdf
vector<float> calculateVals(vector<cv::Point2f> current, cv::Rect face, float mRatio, float nRatio) {
	// 0 is Yaw, 1 is Pitch, 2 is Roll, 3, 4, 5 are mouth, left eye and right eye
	vector<float> angles = { 0, 0, 0, 0, 0, 0 };

	// Points normalised to nose tip
	cv::Point2f noseTip = { 0.0f, 0.0f };
	cv::Point2f leftEye = { current[36].x - current[33].x, current[36].y - current[33].y };
	cv::Point2f rightEye = { current[45].x - current[33].x, current[45].y - current[33].y };
	cv::Point2f leftMouth = { current[48].x - current[33].x, current[48].y - current[33].y };
	cv::Point2f rightMouth = { current[54].x - current[33].x, current[54].y - current[33].y };

	// Midpoints between eyes and mouth
	cv::Point2f eyeMid = mid2(leftEye, rightEye);
	cv::Point2f mouthMid = mid2(leftMouth, rightMouth);

	// Point where nose base meets symmetry line
	cv::Point2f noseBase = mid2(eyeMid, mouthMid, mRatio);

	// Yaw is equal to the angle between line from nose base compared with x axis
	// Pitch is calculated from observed angle of nose line compared to symmetry line and adjusted to fit ratios
	// Roll is calculated as angle between facial points and x axis
	float gradient1 = (noseTip.y - noseBase.y) / (noseTip.x - noseBase.x);
	float gradient2 = (eyeMid.y - noseBase.y) / (eyeMid.x - noseBase.x);
	float gradient3 = (current[0].y - current[16].y) / (current[0].x - current[16].x);

	float m1 = pow( dist2(noseTip, noseBase) / dist2(eyeMid, mouthMid), 2);
	float m2 = pow(cos(atan((gradient2 - gradient1) / (1 + (gradient1 * gradient2)))), 2);
	
	float a = pow(nRatio, 2) * (1.0f - m2);
	float b = pow(nRatio, 2) * (m1 - 1.0f + (2.0f * m2));
	float c = - m2 * pow(nRatio, 2);

	float dz2P = (-b + sqrtf((b * b) - 4 * a * c)) / (2 * a);

	// Save and convert to degrees for ease of use
	// 57.2958 is 180 / pi
	angles[0] = atan(gradient1) * (57.2958f);
	angles[1] = acos(sqrtf(dz2P)) * (57.2958f);
	angles[2] =  atan(gradient3) * (57.2958f);

	// Get the size of the users eyes and mouth
	angles[3] = (dist2(current[50], current[58]) + dist2(current[51], current[57]) + dist2(current[52], current[56]));
	angles[4] = (dist2(current[37], current[41]) + dist2(current[38], current[40]));
	angles[5] = (dist2(current[43], current[47]) + dist2(current[44], current[46]));
	
	//cout << angles[0] << "		" << angles[1] << "		" << angles[2] << endl;

	return angles;
}

cv::String calculateMovements(vector<float> base, vector<float> current, INPUT kb, INPUT ms, int(&saved)[10], vector<int>(&keycodes), int threshold) {
	cv::String movements = "";

	if (0.0f < current[0] && current[0] < 70.0f + threshold) {
		kb.ki.wVk = keycodes[saved[0]];
		kb.ki.dwFlags = 0;
		SendInput(1, &kb, sizeof(INPUT));

		movements += "YL ";
	}
	else if (0.0f > current[0] && current[0] > -70.0f - threshold) {
		kb.ki.wVk = keycodes[saved[1]];
		kb.ki.dwFlags = 0;
		SendInput(1, &kb, sizeof(INPUT));

		movements += "YR ";
	}
	else {
		// Bitwise AND comparison to see if key is pressed
		if (GetKeyState(keycodes[saved[0]]) & 0x8000) {
			// If key is pressed then release the key
			kb.ki.wVk = keycodes[saved[0]];
			kb.ki.dwFlags = KEYEVENTF_KEYUP;
			SendInput(1, &kb, sizeof(INPUT));
		}
		if (GetKeyState(keycodes[saved[1]]) & 0x8000) {
			kb.ki.wVk = keycodes[saved[1]];
			kb.ki.dwFlags = KEYEVENTF_KEYUP;
			SendInput(1, &kb, sizeof(INPUT));
		}
	}

	if ((current[1] - base[1]) > 3.0f - (threshold / 10)) {
		kb.ki.wVk = keycodes[saved[2]];
		kb.ki.dwFlags = 0;
		SendInput(1, &kb, sizeof(INPUT));

		movements += "PD ";
	}
	else if ((current[1] - base[1]) < -5.0f + (threshold / 10)) {
		kb.ki.wVk = keycodes[saved[3]];
		kb.ki.dwFlags = 0;
		SendInput(1, &kb, sizeof(INPUT));

		movements += "PU ";
	}
	else {
		if (GetKeyState(keycodes[saved[2]]) & 0x8000) {
			kb.ki.wVk = keycodes[saved[2]];
			kb.ki.dwFlags = KEYEVENTF_KEYUP;
			SendInput(1, &kb, sizeof(INPUT));
		}
		if (GetKeyState(keycodes[saved[3]]) & 0x8000) {
			kb.ki.wVk = keycodes[saved[3]];
			kb.ki.dwFlags = KEYEVENTF_KEYUP;
			SendInput(1, &kb, sizeof(INPUT));
		}
	}

	if (current[2] < -5.0f) {
		kb.ki.wVk = keycodes[saved[4]];
		kb.ki.dwFlags = 0;
		SendInput(1, &kb, sizeof(INPUT));

		movements += "RR ";
	}
	else if (current[2] > 5.0f) {
		kb.ki.wVk = keycodes[saved[5]];
		kb.ki.dwFlags = 0;
		SendInput(1, &kb, sizeof(INPUT));

		movements += "RL ";
	}
	else {
		if (GetKeyState(keycodes[saved[4]]) & 0x8000) {
			kb.ki.wVk = keycodes[saved[4]];
			kb.ki.dwFlags = KEYEVENTF_KEYUP;
			SendInput(1, &kb, sizeof(INPUT));
		}
		if (GetKeyState(keycodes[saved[5]]) & 0x8000) {
			kb.ki.wVk = keycodes[saved[5]];
			kb.ki.dwFlags = KEYEVENTF_KEYUP;
			SendInput(1, &kb, sizeof(INPUT));
		}
	}

	if (current[4] / base[4] < 0.8f && current[5] / base[5] < 0.8f) {
		kb.ki.wVk = keycodes[saved[6]];
		kb.ki.dwFlags = 0;
		SendInput(1, &kb, sizeof(INPUT));

		movements += "EC ";
	}
	else if (current[4] / base[4] < 0.9f) {
		kb.ki.wVk = keycodes[saved[7]];
		kb.ki.dwFlags = 0;
		SendInput(1, &kb, sizeof(INPUT));

		movements += "LE ";
	}
	else if (current[5] / base[5] < 0.9f) {
		kb.ki.wVk = keycodes[saved[8]];
		kb.ki.dwFlags = 0;
		SendInput(1, &kb, sizeof(INPUT));

		movements += "RE ";
	}
	else {
		if (GetKeyState(keycodes[saved[6]]) & 0x8000) {
			kb.ki.wVk = keycodes[saved[6]];
			kb.ki.dwFlags = KEYEVENTF_KEYUP;
			SendInput(1, &kb, sizeof(INPUT));
		}
		if (GetKeyState(keycodes[saved[7]]) & 0x8000) {
			kb.ki.wVk = keycodes[saved[7]];
			kb.ki.dwFlags = KEYEVENTF_KEYUP;
			SendInput(1, &kb, sizeof(INPUT));
		}
		if (GetKeyState(keycodes[saved[8]]) & 0x8000) {
			kb.ki.wVk = keycodes[saved[8]];
			kb.ki.dwFlags = KEYEVENTF_KEYUP;
			SendInput(1, &kb, sizeof(INPUT));
		}
	}

	if (current[3] / base[3] > 1.3f) {
		kb.ki.wVk = keycodes[saved[9]];
		kb.ki.dwFlags = 0;
		SendInput(1, &kb, sizeof(INPUT));

		movements += "MO ";
	}
	else {
		if (GetKeyState(keycodes[saved[9]]) & 0x8000) {
			kb.ki.wVk = keycodes[saved[9]];
			kb.ki.dwFlags = KEYEVENTF_KEYUP;
			SendInput(1, &kb, sizeof(INPUT));
		}
	}

	if (movements == "") {
		movements = "None";
	}


	return movements;
}

cv::String calculateMovementsMouse(vector<float> base, vector<float> current, INPUT kb, INPUT ms, int(&saved)[6], vector<int>(&keycodes), float sensitivity, float threshold) {
	POINT p;
	cv::String movements = "";

	if (0.0f < current[0] && current[0] < 80.0f + threshold) {
		GetCursorPos(&p);
		SetCursorPos(p.x - ((90.0f - current[0]) * sensitivity), p.y);

		movements += "YL ";
	}
	else if (0.0f > current[0] && current[0] > -80.0f - threshold) {
		GetCursorPos(&p);
		SetCursorPos(p.x + ((90.0f - abs(current[0])) * sensitivity), p.y);

		movements += "YR ";
	}
	else {

	}

	if ((current[1] - base[1]) > 0.5f - (threshold / 10)) {
		GetCursorPos(&p);
		SetCursorPos(p.x, p.y + (abs(current[0]) * sensitivity/2));

		movements += "PD ";
	}
	else if ((current[1] - base[1]) < -3.0f + (threshold / 10)) {
		GetCursorPos(&p);
		SetCursorPos(p.x, p.y - (abs(current[0]) * sensitivity/2));

		movements += "PU ";
	}
	else {

	}

	if (current[2] < -3.0f) {
		kb.ki.wVk = keycodes[saved[0]];
		kb.ki.dwFlags = 0;
		SendInput(1, &kb, sizeof(INPUT));

		movements += "RR ";
	}
	else if (current[2] > 3.0f) {
		kb.ki.wVk = keycodes[saved[1]];
		kb.ki.dwFlags = 0;
		SendInput(1, &kb, sizeof(INPUT));

		movements += "RL ";
	}
	else {
		if (GetKeyState(keycodes[saved[0]]) & 0x8000) {
			kb.ki.wVk = keycodes[saved[0]];
			kb.ki.dwFlags = KEYEVENTF_KEYUP;
			SendInput(1, &kb, sizeof(INPUT));
		}
		if (GetKeyState(keycodes[saved[1]]) & 0x8000) {
			kb.ki.wVk = keycodes[saved[1]];
			kb.ki.dwFlags = KEYEVENTF_KEYUP;
			SendInput(1, &kb, sizeof(INPUT));
		}
	}

	if (current[4] / base[4] < 0.8f && current[5] / base[5] < 0.8f) {
		kb.ki.wVk = keycodes[saved[4]];
		kb.ki.dwFlags = 0;
		SendInput(1, &kb, sizeof(INPUT));

		movements += "EC ";
	}
	else if (current[4] / base[4] < 0.9f) {
		kb.ki.wVk = keycodes[saved[2]];
		kb.ki.dwFlags = 0;
		SendInput(1, &kb, sizeof(INPUT));

		movements += "LE ";
	}
	else if (current[5] / base[5] < 0.9f) {
		kb.ki.wVk = keycodes[saved[3]];
		kb.ki.dwFlags = 0;
		SendInput(1, &kb, sizeof(INPUT));

		movements += "RE ";
	}
	else {
		if (GetKeyState(keycodes[saved[2]]) & 0x8000) {
			kb.ki.wVk = keycodes[saved[2]];
			kb.ki.dwFlags = KEYEVENTF_KEYUP;
			SendInput(1, &kb, sizeof(INPUT));
		}
		if (GetKeyState(keycodes[saved[3]]) & 0x8000) {
			kb.ki.wVk = keycodes[saved[3]];
			kb.ki.dwFlags = KEYEVENTF_KEYUP;
			SendInput(1, &kb, sizeof(INPUT));
		}
		if (GetKeyState(keycodes[saved[4]]) & 0x8000) {
			kb.ki.wVk = keycodes[saved[4]];
			kb.ki.dwFlags = KEYEVENTF_KEYUP;
			SendInput(1, &kb, sizeof(INPUT));
		}
	}

	if (current[3] / base[3] > 1.3f) {
		kb.ki.wVk = keycodes[saved[5]];
		kb.ki.dwFlags = 0;
		SendInput(1, &kb, sizeof(INPUT));

		movements += "MO ";
	}
	else {
		if (GetKeyState(keycodes[saved[5]]) & 0x8000) {
			kb.ki.wVk = keycodes[saved[5]];
			kb.ki.dwFlags = KEYEVENTF_KEYUP;
			SendInput(1, &kb, sizeof(INPUT));
		}
	}

	if (movements == "") {
		movements = "None";
	}


	return movements;
}
#endif

// Saves landmarks in current frame
vector<float> saveBaseLandmarks(vector<cv::Point2f> shapes, cv::Rect face) {
	vector<cv::Point2f> base = shapes;
	vector<float> vals = calculateVals(base, face, 0.4f, 0.6f);
	return vals;
}

int main(int, char**) {
	// Pre Reqs
#if 1
	cv::Mat window = cv::Mat(480, 750, CV_8UC3);
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

	const string facemark_filename = "C:/lib/opencv/data/landmark_models/lbfmodel.yaml";
	cv::Ptr<cv::face::Facemark> facemark = createFacemarkLBF();
	facemark->loadModel(facemark_filename);
	cout << "Loaded facemark LBF model" << endl;

	vector<vector<cv::Point2f>> shapes;
	vector<float> base = { 0, 0, 0, 0, 0, 0 };

	cv::Rect lastFace;
	bool cropped = false;
	float offsetX = 0, offsetY = 0, width = 0, height = 0;

	constexpr int buffer = 30;

	int mode = -1;
	bool paused = true;
	int low_threshold = 1, high_threshold = 10;
	cv::String message = "Paused";

	vector<string> keybinds = { "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P", "Q", "R", "S", "T", "U",
	"V", "W", "X", "Y", "Z", "0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "TAB", "CAPS LCK", "SHIFT", "CTRL", "ALT", "ENTER", "ESC",
	"UP", "DOWN", "LEFT", "RIGHT", "MOUSE 1", "MOUSE 2", "MOUSE 3", "SPACE" };
	vector<int> keycodes = { 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x49, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50, 0x51, 0x52,
	0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5A, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x09, 0x14, 0x10, 0x11,
	0x12, 0x0D, 0x1B, 0x26, 0x28, 0x25, 0x27, 0x01, 0x02, 0x03, 0x20 };

	int saved[10] = { 0, 3, 22, 18, 4, 16, 38, 47, 48, 49 };
	int savedMouse[6] = { 38, 39, 37, 47, 48, 49 };

	int mouseSens = 5.0f;
	int thresholdSens = 5.0f;

	// 3D model points.
	cv::Point3f noseTip = { 0.0f, 0.0f, 0.0f };
	cv::Point3f chin = { 0.0f, -330.0f, -65.0f };
	cv::Point3f leftEye = { -225.0f, 170.0f, -135.0f };
	cv::Point3f rightEye = { 225.0f, 170.0f, -135.0f };
	cv::Point3f leftMouth = { -150.0f, -150.0f, -125.0f };
	cv::Point3f rightMouth = { 150.0f, -150.0f, -125.0f };

	cv::Point3f eyeMid = mid3(leftEye, rightEye);
	cv::Point3f mouthMid = mid3(leftMouth, rightMouth);

	// y = (-1/m)x since line passes through origin at tip of nose
	// y = mx - m*mouthMid.x + mouthmid.y
	float m = (mouthMid.y - eyeMid.y) / (mouthMid.z - eyeMid.z);

	// Intersection of lines
	float x = ((-m * eyeMid.z) + eyeMid.y) / ((-1.0f / m) - m);
	float y = (-1 / m) * x;

	cv::Point3f noseBase = { 0, y, x };

	float mRatio = dist3(mouthMid, noseBase) / dist3(eyeMid, mouthMid);
	float nRatio = dist3(noseTip, noseBase) / dist3(eyeMid, mouthMid);

#endif

	// Main Process Loop
	while (true) {
		// Fill the frame with a nice color
		window = cv::Scalar(49, 52, 49);

		// capture the next frame from the webcam
		camera >> frame;

		// If a face has been found in the last frame
		if (cropped) {
			// Crop frame to area around face
			reduced = frame(cv::Rect(offsetX, offsetY, width, height));
		}
		// Otherwise use the whole image
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
				width = faces[0].width + (2.0f * buffer);
				height = faces[0].height + (2.0f * buffer);
			}
			else {
				offsetX = faces[0].x - buffer;
				offsetY = faces[0].y - buffer;
				width = faces[0].width + (2.0f * buffer);
				height = faces[0].height + (2.0f * buffer);
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

			if (!paused) {
				if (mode == 1) {
					message = calculateMovements(base, calculateVals(shapes[0], faces[0], mRatio, nRatio), kb, ms, saved, keycodes, 5.0f-thresholdSens);
				}
				else if (mode == -1) {
					message = calculateMovementsMouse(base, calculateVals(shapes[0], faces[0], mRatio, nRatio), kb, ms, savedMouse, keycodes, (float) mouseSens/10, (float)thresholdSens);
				}
			}

			putText(reduced, message, cv::Point2f(faces[0].x, faces[0].y + faces[0].height), 0, 1.0, cv::Scalar(255, 255, 255), 2);
		}
		else {
			cropped = false;
			offsetX = 0;
			offsetY = 0;
			width = 0;
			height = 0;
		}

		// Resize the material 
		cv::resize(reduced, frame, cv::Size(210, 210), 1, 1, 1);

		// Camera feed
		cvui::image(window, 30, 30, frame);

		// Sliders
#if 1
		cvui::text(window, 350, 60, "Mouse Sensitivity", 0.4f, 0xffffff);
		cvui::trackbar(window, 475, 60, 250, &mouseSens, 1, 20, 0.1f);

		cvui::text(window, 350, 150, "Control Threshold", 0.4f, 0xffffff);
		cvui::trackbar(window, 475, 150, 250, &thresholdSens, 0, 5, 0.1f);

#endif

		// Control settings
#if 1
		if (cvui::button(window, 30, 280, 120, 30, "Switch modes")) {
			mode *= -1;
		}
		if (mode == 1) {
			// Show control mode
			// Text at position (250, 90), sized 0.4, in white
			cvui::printf(window, 160, 290, 0.4, 0xffffff, "Keyboard control");
		}
		else if (mode == -1) {
			// Show control mode
			// Text at position (250, 90), sized 0.4, in white
			cvui::printf(window, 160, 290, 0.4, 0xffffff, "Mouse control");
		}

		cvui::checkbox(window, 30, 320, "Paused", &paused);

		if (cvui::button(window, 100, 380, "Recalibrate") && faces.size() > 0) {
			base = saveBaseLandmarks(shapes[0], faces[0]);
		}
#endif

		// Keybindings
#if 1
		if (mode == 1) {
			cvui::printf(window, 300, 245, 0.4, 0xffffff, "Look Left");
			if (cvui::button(window, 380, 240, 100, 30, keybinds[saved[0]])) {
				cvui::text(window, 340, 275, "Press a Key", 0.4, 0xff0000);
				cvui::update();
				cv::imshow(WINDOW_NAME, window);

				saved[0] = charFinder(keycodes, window);
			}
			cvui::printf(window, 300, 295, 0.4, 0xffffff, "Look Right");
			if (cvui::button(window, 380, 290, 100, 30, keybinds[saved[1]])) {
				cvui::text(window, 340, 325, "Press a Key", 0.4, 0xff0000);
				cvui::update();
				cv::imshow(WINDOW_NAME, window);

				saved[1] = charFinder(keycodes, window);
			}
			cvui::printf(window, 300, 345, 0.4, 0xffffff, "Look Up");
			if (cvui::button(window, 380, 340, 100, 30, keybinds[saved[2]])) {
				cvui::text(window, 340, 375, "Press a Key", 0.4, 0xff0000);
				cvui::update();
				cv::imshow(WINDOW_NAME, window);

				saved[2] = charFinder(keycodes, window);
			}
			cvui::printf(window, 300, 395, 0.4, 0xffffff, "Look Down");
			if (cvui::button(window, 380, 390, 100, 30, keybinds[saved[3]])) {
				cvui::text(window, 340, 425, "Press a Key", 0.4, 0xff0000);
				cvui::update();
				cv::imshow(WINDOW_NAME, window);

				saved[3] = charFinder(keycodes, window);
			}
			cvui::printf(window, 300, 445, 0.4, 0xffffff, "Tilt Left");
			if (cvui::button(window, 380, 440, 100, 30, keybinds[saved[4]])) {
				cvui::text(window, 340, 475, "Press a Key", 0.4, 0xff0000);
				cvui::update();
				cv::imshow(WINDOW_NAME, window);

				saved[4] = charFinder(keycodes, window);
			}
			cvui::printf(window, 500, 245, 0.4, 0xffffff, "Tilt Right");
			if (cvui::button(window, 580, 240, 100, 30, keybinds[saved[5]])) {
				cvui::text(window, 520, 275, "Press a Key", 0.4, 0xff0000);
				cvui::update();
				cv::imshow(WINDOW_NAME, window);

				saved[5] = charFinder(keycodes, window);
			}
			cvui::printf(window, 500, 295, 0.4, 0xffffff, "Left Eye");
			if (cvui::button(window, 580, 290, 100, 30, keybinds[saved[6]])) {
				cvui::text(window, 520, 325, "Press a Key", 0.4, 0xff0000);
				cvui::update();
				cv::imshow(WINDOW_NAME, window);

				saved[6] = charFinder(keycodes, window);
			}
			cvui::printf(window, 500, 345, 0.4, 0xffffff, "Right Eye");
			if (cvui::button(window, 580, 340, 100, 30, keybinds[saved[7]])) {
				cvui::text(window, 520, 375, "Press a Key", 0.4, 0xff0000);
				cvui::update();
				cv::imshow(WINDOW_NAME, window);

				saved[7] = charFinder(keycodes, window);
			}
			cvui::printf(window, 500, 395, 0.4, 0xffffff, "Both Eyes");
			if (cvui::button(window, 580, 390, 100, 30, keybinds[saved[8]])) {
				cvui::text(window, 520, 425, "Press a Key", 0.4, 0xff0000);
				cvui::update();
				cv::imshow(WINDOW_NAME, window);

				saved[8] = charFinder(keycodes, window);
			}
			cvui::printf(window, 500, 445, 0.4, 0xffffff, "Mouth Open");
			if (cvui::button(window, 580, 440, 100, 30, keybinds[saved[9]])) {
				cvui::text(window, 520, 475, "Press a Key", 0.4, 0xff0000);
				cvui::update();
				cv::imshow(WINDOW_NAME, window);

				saved[9] = charFinder(keycodes, window);
			}
		}
		else if (mode == -1){
			cvui::printf(window, 300, 245, 0.4, 0xffffff, "Look Left");
			cvui::button(window, 380, 240, 100, 30, "Move Left");

			cvui::printf(window, 300, 295, 0.4, 0xffffff, "Look Right");
			cvui::button(window, 380, 290, 100, 30, "Move Right");

			cvui::printf(window, 300, 345, 0.4, 0xffffff, "Look Up");
			cvui::button(window, 380, 340, 100, 30, "Move Up");

			cvui::printf(window, 300, 395, 0.4, 0xffffff, "Look Down");
			cvui::button(window, 380, 390, 100, 30, "Move Down");

			cvui::printf(window, 300, 445, 0.4, 0xffffff, "Tilt Left");

			if (cvui::button(window, 380, 440, 100, 30, keybinds[savedMouse[0]])) {
				cvui::text(window, 340, 475, "Press a Key", 0.4, 0xff0000);
				cvui::update();
				cv::imshow(WINDOW_NAME, window);

				savedMouse[0] = charFinder(keycodes, window);
			}
			cvui::printf(window, 500, 245, 0.4, 0xffffff, "Tilt Right");
			if (cvui::button(window, 580, 240, 100, 30, keybinds[savedMouse[1]])) {
				cvui::text(window, 520, 275, "Press a Key", 0.4, 0xff0000);
				cvui::update();
				cv::imshow(WINDOW_NAME, window);

				savedMouse[1] = charFinder(keycodes, window);
			}
			cvui::printf(window, 500, 295, 0.4, 0xffffff, "Left Eye");
			if (cvui::button(window, 580, 290, 100, 30, keybinds[savedMouse[2]])) {
				cvui::text(window, 520, 325, "Press a Key", 0.4, 0xff0000);
				cvui::update();
				cv::imshow(WINDOW_NAME, window);

				savedMouse[2] = charFinder(keycodes, window);
			}
			cvui::printf(window, 500, 345, 0.4, 0xffffff, "Right Eye");
			if (cvui::button(window, 580, 340, 100, 30, keybinds[savedMouse[3]])) {
				cvui::text(window, 520, 375, "Press a Key", 0.4, 0xff0000);
				cvui::update();
				cv::imshow(WINDOW_NAME, window);

				savedMouse[3] = charFinder(keycodes, window);
			}
			cvui::printf(window, 500, 395, 0.4, 0xffffff, "Both Eyes");
			if (cvui::button(window, 580, 390, 100, 30, keybinds[savedMouse[4]])) {
				cvui::text(window, 520, 425, "Press a Key", 0.4, 0xff0000);
				cvui::update();
				cv::imshow(WINDOW_NAME, window);

				savedMouse[4] = charFinder(keycodes, window);
			}
			cvui::printf(window, 500, 445, 0.4, 0xffffff, "Mouth Open");
			if (cvui::button(window, 580, 440, 100, 30, keybinds[savedMouse[5]])) {
				cvui::text(window, 520, 475, "Press a Key", 0.4, 0xff0000);
				cvui::update();
				cv::imshow(WINDOW_NAME, window);

				savedMouse[5] = charFinder(keycodes, window);
			}

		}



#endif

		// Update and display elements
		cvui::update();
		cv::imshow(WINDOW_NAME, window);

		cv::waitKey(1);
	}


	return 0;
};
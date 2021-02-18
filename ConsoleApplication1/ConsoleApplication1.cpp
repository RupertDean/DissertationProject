#define WINVER 0x0500

#include "opencv2/opencv.hpp"
#include "opencv2/face.hpp"
#include "iostream"

#include <windows.h>
#include <fstream>
#include <math.h>

using namespace cv;
using namespace cv::face;
using namespace std;


// Function to detect faces in given image
void faceDetector(const Mat& image, vector<Rect>& faces, CascadeClassifier& face_cascade){
    // Histogram equalization generally aids in face detection
    equalizeHist(image, image);
    faces.clear();

    // Run the cascade classifier
    face_cascade.detectMultiScale(
        image,
        faces,
        2.0, // pyramid scale factor
        5,   // lower thershold for neighbors count
             // here we hint the classifier to only look for one face
        CASCADE_SCALE_IMAGE + CASCADE_FIND_BIGGEST_OBJECT);
}

// Returns distance between 2 2D points
float dist(Point2f a, Point2f b) {
    return sqrtf(pow(b.x - a.x, 2) + pow(b.y - a.y, 2));
}

// Returns a vector with distances between points on the face
// 0 eyeDist, 1 leftCheek, 2 rightCheek, 3 mouthDist, 4 eyeToChin, 5 browDist, 6 browEyeWidth, 7 jawChinDist, 8 eyesOpen, 9 eyeBrowDist
vector<float> calculateVals(vector<vector<Point2f>> current, Rect face){
    vector<float> vals = {
    dist(current[0][37], current[0][41]) + dist(current[0][38], current[0][40]) +
        dist(current[0][43], current[0][47]) + dist(current[0][44], current[0][46]),
    dist(current[0][1], current[0][30]) + dist(current[0][3], current[0][48]),
    dist(current[0][30], current[0][15]) + dist(current[0][54], current[0][13]),
    dist(current[0][50], current[0][58]) + dist(current[0][51], current[0][57]) +
        dist(current[0][52], current[0][56]),
    dist(current[0][17], current[0][8]) + dist(current[0][26], current[0][8]),
    dist(current[0][17], current[0][30]) + dist(current[0][21], current[0][30]) +
        dist(current[0][22], current[0][30]) + dist(current[0][26], current[0][30]),
    dist(current[0][17], current[0][26]) + dist(current[0][36], current[0][45]),
    dist(current[0][4], current[0][8]) + dist(current[0][12], current[0][8]),
    dist(current[0][37], current[0][41]) + dist(current[0][38], current[0][40]) +
        dist(current[0][43], current[0][47]) + dist(current[0][44], current[0][46]),
    current[0][8].y,
        0, 0
    };

    for (int i = 0; i < vals.size(); i++) {
        vals[i] /= face.area();
    }

    vals[10] = ((face.y + face.height) - face.y) / ((face.x + face.width) - face.x);
    vals[11] = (current[0][0].y - current[0][16].y)/(current[0][0].x - current[0][16].x);

    return vals;
}

// Estimates movements based on differences in distances between base and current images
String calculateMovements(vector<float> base, vector<float> current, INPUT kb, INPUT ms) {
    float angleCurrent = atan((current[11] - current[10])/(1.0f + current[10] * current[11])) * 180.0f / 3.4159265f;
    float angleBase = atan((base[11] - base[10]) / (1.0f + base[10] * base[11])) * 180.0f / 3.4159265f;

    if (current[3] / base[3] > 1.15f) {
        kb.ki.wVk = 0x20; // virtual-key code for the space key
        kb.ki.dwFlags = 0; // 0 for key press
        SendInput(1, &kb, sizeof(INPUT));

        // Release the key
        kb.ki.dwFlags = KEYEVENTF_KEYUP; // KEYEVENTF_KEYUP for key release
        SendInput(1, &kb, sizeof(INPUT));
    }

    if (current[8] / base[8] < 0.85f) {
        /**
        kb.ki.wVk = 0x45; // virtual-key code for left click
        kb.ki.dwFlags = 0; // 0 for key press
        SendInput(1, &kb, sizeof(INPUT));
        // Release the key
        kb.ki.dwFlags = KEYEVENTF_KEYUP; // KEYEVENTF_KEYUP for key release
        SendInput(1, &kb, sizeof(INPUT));
        **/

    }

    if (angleCurrent / angleBase > 1.1f) {
        
        //return "RR";
    }
    else if (angleCurrent / angleBase < 0.9f) {
        kb.ki.wVk = 0x1B; // virtual-key code for the ESC key
        kb.ki.dwFlags = 0; // 0 for key press
        SendInput(1, &kb, sizeof(INPUT));

        kb.ki.dwFlags = KEYEVENTF_KEYUP;
        SendInput(1, &kb, sizeof(INPUT));
        return "RL";
    }


    if (current[9] / base[9] > 1.05f) {
        kb.ki.wVk = 0x53; // virtual-key code for the A key
        kb.ki.dwFlags = 0; // 0 for key press
        SendInput(1, &kb, sizeof(INPUT));
        return "PD";
    }
    else if (current[9] / base[9] < 0.95f){
        kb.ki.wVk = 0x57; // virtual-key code for the A key
        kb.ki.dwFlags = 0; // 0 for key press
        SendInput(1, &kb, sizeof(INPUT));
        return "PU";
    }


    if (current[1] / base[1] > 1.2f && 0.95f < angleCurrent / angleBase < 1.05f) {
        if (current[2] / base[2] < 1.2f) {
            kb.ki.wVk = 0x41; // virtual-key code for the A key
            kb.ki.dwFlags = 0; // 0 for key press
            SendInput(1, &kb, sizeof(INPUT));
            return "YL";
 
        }
    }
    else if (current[2] / base[2] > 1.2f && 0.95f < angleCurrent / angleBase < 1.05f) {
        if (current[1] / base[1] < 1.2f) {
            kb.ki.wVk = 0x44; // virtual-key code for the D key
            kb.ki.dwFlags = 0; // 0 for key press
            SendInput(1, &kb, sizeof(INPUT));
            return"YR";

        }
    }

    kb.ki.wVk = 0x44;
    kb.ki.dwFlags = KEYEVENTF_KEYUP; // KEYEVENTF_KEYUP for key release
    SendInput(1, &kb, sizeof(INPUT));
    kb.ki.wVk = 0x41;
    kb.ki.dwFlags = KEYEVENTF_KEYUP;
    SendInput(1, &kb, sizeof(INPUT));
    kb.ki.wVk = 0x53; 
    kb.ki.dwFlags = KEYEVENTF_KEYUP;
    SendInput(1, &kb, sizeof(INPUT));
    kb.ki.wVk = 0x57;
    kb.ki.dwFlags = KEYEVENTF_KEYUP;
    SendInput(1, &kb, sizeof(INPUT));

    return "None";

}

// Saves landmarks in current frame
vector<float> saveBaseLandmarks(vector<vector<Point2f>> shapes, Rect face) {
    vector<vector<Point2f>> base = shapes;
    vector<float> vals = calculateVals(base, face);
    return vals;
}

void printVals(vector<vector<Point2f>> current) {
    for (int i = 0; i < current[0].size(); i++) {
        printf("\n %f", current[0][i].x);
    }

    printf("\n\n\n\n\n");

    for (int i = 0; i < current[0].size(); i++) {
        printf("\n %f", current[0][i].y);
    }
}

int main(int, char**){
#define TRAINING false

    INPUT kb;
    kb.type = INPUT_KEYBOARD;
    kb.ki.wScan = 0;
    kb.ki.time = 0;
    kb.ki.dwExtraInfo = 0;

    INPUT ms;
    ms.type = INPUT_MOUSE;


    // open the first webcam plugged in the computer
    VideoCapture camera(0);
    if (!camera.isOpened()){
        cerr << "ERROR: Could not open camera" << endl;
        return 1;
    }

    // create a window to display the images from the webcam
    namedWindow("Webcam");

    // this will contain the image from the webcam
    Mat frame;
    Mat gray;
    Mat flipped;

    vector<Rect> faces;

    // load cascade for face detection
    const String cascade_name = "C:/lib/opencv/data/haarcascades/haarcascade_frontalface_default.xml";
    CascadeClassifier face_cascade;

    if (not face_cascade.load(cascade_name)) {
        cerr << "Cannot load cascade classifier from file: " << cascade_name << endl;
        return -1;
    }

#define LOAD_PROFILE false
#if LOAD_PROFILE
    // load cascade for profile face detection
    const String profile_cascade_name = "C:/lib/opencv/data/haarcascades/haarcascade_profileface.xml";
    CascadeClassifier profile_face_cascade;

    if (not profile_face_cascade.load(profile_cascade_name)) {
        cerr << "Cannot load cascade classifier from file: " << profile_cascade_name << endl;
        return -1;
    }
#endif

    const string facemark_filename = "C:/lib/opencv/data/landmark_models/lbfmodel.yaml";
    Ptr<Facemark> facemark = createFacemarkLBF();
    facemark->loadModel(facemark_filename);
    cout << "Loaded facemark LBF model" << endl;

    vector<vector<Point2f>> shapes;
    vector<float> base = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

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
    while (1) {

        // capture the next frame from the webcam
        camera >> frame;

        // The cascade classifier works best on grayscale images
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

            // printf("%f, %f \n", shapes[0][67].x, shapes[0][67].y);

            if (cv::waitKey(1) == 32) base = saveBaseLandmarks(shapes, faces[0]);

            putText(frame, calculateMovements(base, calculateVals(shapes, faces[0]), kb, ms), Point2f(faces[0].x, faces[0].y + faces[0].height), 0, 1.0, Scalar(255, 255, 255), 2);
        }

        else{
            Sleep(1);
        }

#if 0
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
        

        // show the image on the window
        imshow("Webcam", frame);

    }
#endif
      

    return 0;
}
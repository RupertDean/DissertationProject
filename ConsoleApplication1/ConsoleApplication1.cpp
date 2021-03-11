
#define WINVER 0x0500

#include "opencv2/opencv.hpp"
#include "opencv2/face.hpp"
#include "iostream"

#include <windows.h>
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
        5,   // lower thershold for neighbors count
             // here we hint the classifier to only look for one face
        cv::CASCADE_SCALE_IMAGE + cv::CASCADE_FIND_BIGGEST_OBJECT);
}

// Returns distance between 2 2D points
float dist(cv::Point2f a, cv::Point2f b) {
    return sqrtf(pow(b.x - a.x, 2) + pow(b.y - a.y, 2));
}

// Returns a vector with distances between points on the face
// 0 eyeDist, 1 leftCheek, 2 rightCheek, 3 mouthDist, 4 eyeToChin, 5 browDist, 6 browEyeWidth, 7 jawChinDist, 8 eyesOpen, 9 eyeBrowDist, 12 left eye, 13 right eye
vector<float> calculateVals(vector<vector<cv::Point2f>> current, cv::Rect face) {
    vector<float> vals = {
    dist(current[0][37], current[0][41]) + dist(current[0][38], current[0][40]) +
        dist(current[0][43], current[0][47]) + dist(current[0][44], current[0][46]),
    dist(current[0][1], current[0][30]) + dist(current[0][3], current[0][48]),
    dist(current[0][30], current[0][15]) + dist(current[0][54], current[0][13]),
    dist(current[0][51], current[0][57]),
    dist(current[0][17], current[0][8]) + dist(current[0][26], current[0][8]),
    dist(current[0][17], current[0][30]) + dist(current[0][21], current[0][30]) +
        dist(current[0][22], current[0][30]) + dist(current[0][26], current[0][30]),
    dist(current[0][17], current[0][26]) + dist(current[0][36], current[0][45]),
    dist(current[0][4], current[0][8]) + dist(current[0][12], current[0][8]),
    dist(current[0][37], current[0][41]) + dist(current[0][38], current[0][40]) +
        dist(current[0][43], current[0][47]) + dist(current[0][44], current[0][46]),
    current[0][8].y,
        0, 0,
    dist(current[0][37], current[0][41]) + dist(current[0][38], current[0][40]),
    dist(current[0][43], current[0][47]) + dist(current[0][44], current[0][46])

    };

    for (int i = 0; i < vals.size(); i++) {
        vals[i] /= face.area();
    }

    vals[10] = ((face.y + face.height) - face.y) / ((face.x + face.width) - face.x);
    vals[11] = (current[0][0].y - current[0][16].y) / (current[0][0].x - current[0][16].x);

    return vals;
}

// Estimates movements based on differences in distances between base and current images
cv::String calculateMovements(vector<float> base, vector<float> current, INPUT kb, INPUT ms) {
    POINT p;
    float angleCurrent = atan((current[11] - current[10]) / (1.0f + current[10] * current[11])) * 180.0f / 3.4159265f;
    float angleBase = atan((base[11] - base[10]) / (1.0f + base[10] * base[11])) * 180.0f / 3.4159265f;

    if (current[3] / base[3] > 1.3f) {
        //mouse_event(MOUSEEVENTF_WHEEL, 0, 0, 1, 0);
    }

    if (angleCurrent / angleBase > 1.1f) {
        ms.mi.dwFlags = MOUSEEVENTF_RIGHTDOWN;
        //SendInput(1, &ms, sizeof(INPUT));

        Sleep(1);

        ms.mi.dwFlags = MOUSEEVENTF_RIGHTUP;
        //SendInput(1, &ms, sizeof(INPUT));
        //return "RR";
    }
    else if (angleCurrent / angleBase < 0.9f) {
        ms.mi.dwFlags = MOUSEEVENTF_LEFTDOWN;
        //SendInput(1, &ms, sizeof(INPUT));

        Sleep(1);

        ms.mi.dwFlags = MOUSEEVENTF_LEFTUP;
        //SendInput(1, &ms, sizeof(INPUT));

        //return "RL";
    }


    if (current[9] / base[9] > 1.05f) {
        kb.ki.wVk = 0x53; // virtual-key code for the S key
        kb.ki.dwFlags = 0; // 0 for key press
        //SendInput(1, &kb, sizeof(INPUT));
        //return "PD";
    }
    else if (current[9] / base[9] < 0.95f) {
        kb.ki.wVk = 0x57; // virtual-key code for the W key
        kb.ki.dwFlags = 0; // 0 for key press
        //SendInput(1, &kb, sizeof(INPUT));
        //return "PU";
    }

    if (current[10] / base[10] < 0.8f && 0.95f < current[9] / base[9] < 1.05f) {

        //return "LE";
    }

    if (current[11] / base[11] < 0.8f && 0.95f < current[9] / base[9] < 1.05f) {

        //return "RE";
    }

    if (current[8] / base[8] < 0.8f && 0.95f < current[9] / base[9] < 1.05f) {

        //return "EC";
    }

    if (current[1] / base[1] > 1.1f && 0.9f < angleCurrent / angleBase < 1.1f) {
        if (current[2] / base[2] < 0.9f) {
            kb.ki.wVk = 0x41; // virtual-key code for the A key
            kb.ki.dwFlags = 0; // 0 for key press
            //SendInput(1, &kb, sizeof(INPUT));

            GetCursorPos(&p);
            
            SetCursorPos(p.x - (25 * ((current[1] / base[1]) -1)), p.y);

            
            return "YL";

        }
    }
    else if (current[2] / base[2] > 1.1f && 0.9f < angleCurrent / angleBase < 1.1f) {
        if (current[1] / base[1] < 0.9f) {
            kb.ki.wVk = 0x44; // virtual-key code for the D key
            kb.ki.dwFlags = 0; // 0 for key press
            //SendInput(1, &kb, sizeof(INPUT));

            GetCursorPos(&p);

            SetCursorPos(p.x + (25 * ((current[2] / base[2]) -1)), p.y);

            return"YR";

        }
    }

    kb.ki.wVk = 0x44;
    kb.ki.dwFlags = KEYEVENTF_KEYUP; // KEYEVENTF_KEYUP for key release
    //SendInput(1, &kb, sizeof(INPUT));
    kb.ki.wVk = 0x41;
    kb.ki.dwFlags = KEYEVENTF_KEYUP;
    //SendInput(1, &kb, sizeof(INPUT));
    kb.ki.wVk = 0x53;
    kb.ki.dwFlags = KEYEVENTF_KEYUP;
    //SendInput(1, &kb, sizeof(INPUT));
    kb.ki.wVk = 0x57;
    kb.ki.dwFlags = KEYEVENTF_KEYUP;
    //SendInput(1, &kb, sizeof(INPUT));

    return "None";

}

// Same as previous function but uses raw input
cv::String calculateMovementsRaw(vector<float> base, vector<float> current, INPUT kb, INPUT ms) {

    return "None";
}

// Saves landmarks in current frame
vector<float> saveBaseLandmarks(vector<vector<cv::Point2f>> shapes, cv::Rect face) {
    vector<vector<cv::Point2f>> base = shapes;
    vector<float> vals = calculateVals(base, face);
    return vals;
}

void printVals(vector<vector<cv::Point2f>> current) {
    for (int i = 0; i < current[0].size(); i++) {
        printf("\n %f", current[0][i].x);
    }

    printf("\n\n\n\n\n");

    for (int i = 0; i < current[0].size(); i++) {
        printf("\n %f", current[0][i].y);
    }
}

int main(int, char**) {
#define TRAINING false

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

    // create a window to display the images from the webcam
    cv::namedWindow("Webcam", cv::WINDOW_NORMAL);

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
    cv::Ptr<cv::face::Facemark> facemark = createFacemarkLBF();
    facemark->loadModel(facemark_filename);
    cout << "Loaded facemark LBF model" << endl;

    vector<vector<cv::Point2f>> shapes;
    vector<float> base = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    cv::Rect lastFace;
    bool cropped = false;
    float offsetX, offsetY = 0, width = 0, height = 0;

    constexpr int buffer = 30;

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

            if (cv::waitKey(1) == 32) base = saveBaseLandmarks(shapes, faces[0]);

            putText(reduced, calculateMovements(base, calculateVals(shapes, faces[0]), kb, ms), cv::Point2f(faces[0].x, faces[0].y + faces[0].height), 0, 1.0, cv::Scalar(255, 255, 255), 2);
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
    // show the image on the window
    if (cropped) {
        cv::resizeWindow("Webcam", width, height);
    }
    else {
        cv::resizeWindow("Webcam", 640, 480);
    }
    imshow("Webcam", reduced);

    }
#endif


    return 0;
}

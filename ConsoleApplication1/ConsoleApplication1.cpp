// ConsoleApplication1.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "opencv2/opencv.hpp"
#include "opencv2/face.hpp"
#include "iostream"

using namespace cv;
using namespace cv::face;
using namespace std;

void faceDetector(const Mat& image, vector<Rect>& faces, CascadeClassifier& face_cascade){
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
        CASCADE_SCALE_IMAGE + CASCADE_FIND_BIGGEST_OBJECT);
}

float dist(Point2f a, Point2f b) {
    return sqrtf(pow(b.x - a.x, 2) + pow(b.y - a.y, 2));
}

// 0 eyeDist, 1 leftCheek, 2 rightCheek, 3 mouthDist, 4 eyeToChin, 5 browDist, 6 browEyeWidth, 7 jawChinDist, 8 eyesOpen
vector<float> calculateVals(vector<vector<Point2f>> current) {
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
        dist(current[0][43], current[0][47]) + dist(current[0][44], current[0][46])
    };

    return vals;
}

void calculateMovements(vector<float> base, vector<float> current) {
    if (current[4] / base[4] < 0.95f) {
        if (current[7] / base[7] > 1.05f) {
            printf("PD ");
        }
        else if (current[7] / base[7] < 0.95f){
            printf("PU ");
        }
    }

    if (current[3] / base[3] > 1.3f) {
        printf("MO ");
    }

    if (current[8] / base[8] < 0.75f) {
        printf("EC ");
    }

    if (current[1] / base[1] > 1.1f) {
        if (current[2] / base[2] < 1.1f) {
            printf("YL ");
        }
    }
    else if (current[2] / base[2] > 1.1f) {
        if (current[1] / base[1] < 1.1f) {
            printf("YR ");
        }
    }


}

vector<float> saveBaseLandmarks(vector<vector<Point2f>> shapes) {
    vector<vector<Point2f>> base = shapes;
    vector<float> vals = calculateVals(base);
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

    const string facemark_filename = "C:/lib/opencv/data/landmark_models/lbfmodel.yaml";
    Ptr<Facemark> facemark = createFacemarkLBF();
    facemark->loadModel(facemark_filename);
    cout << "Loaded facemark LBF model" << endl;

    vector<vector<Point2f>> shapes;
    vector<float> base = {0, 0, 0, 0, 0, 0, 0, 0, 0};

    while(1) {
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

            if (cv::waitKey(1) == 32) base = saveBaseLandmarks(shapes);
            calculateMovements(base, calculateVals(shapes));
        }
        

        // show the image on the window
        imshow("Webcam", frame);

        if(cv::waitKey(1) == 27) break;

    }
      

    return 0;
}
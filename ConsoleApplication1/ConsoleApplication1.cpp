// ConsoleApplication1.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "opencv2/opencv.hpp"
#include "opencv2/face.hpp"
#include "iostream"

using namespace cv;
using namespace cv::face;
using namespace std;

void faceDetector(const Mat& image,
    vector<Rect>& faces,
    CascadeClassifier& face_cascade
) {
    Mat gray;
    // The cascade classifier works best on grayscale images
    if (image.channels() > 1) {
        cvtColor(image, gray, COLOR_BGR2GRAY);
    }
    else {
        gray = image.clone();
    }
    // Histogram equalization generally aids in face detection
    equalizeHist(gray, gray);
    faces.clear();
    // Run the cascade classifier
    face_cascade.detectMultiScale(
        gray,
        faces,
        1.4, // pyramid scale factor
        3,   // lower thershold for neighbors count
             // here we hint the classifier to only look for one face
        CASCADE_SCALE_IMAGE + CASCADE_FIND_BIGGEST_OBJECT);
}

int main(int, char**) {
    // open the first webcam plugged in the computer
    VideoCapture camera(0);
    if (!camera.isOpened()) {
        cerr << "ERROR: Could not open camera" << endl;
        return 1;
    }

    // create a window to display the images from the webcam
    namedWindow("Webcam");

    // this will contain the image from the webcam
    Mat frame;

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

    while(1) {
        // capture the next frame from the webcam
        camera >> frame;

        // ... obtain an image in img
        vector<Rect> faces;
        faceDetector(frame, faces, face_cascade);
        // Check if faces detected or not
        if (faces.size() != 0) {
            // We assume a single face so we look at the first only
            cv::rectangle(frame, faces[0], Scalar(255, 0, 0), 2);
            vector<vector<Point2f> > shapes;

            if (facemark->fit(frame, faces, shapes)) {
                // Draw the detected landmarks
                drawFacemarks(frame, shapes[0], cv::Scalar(0, 0, 255));
            }
        }

        // show the image on the window
        imshow("Webcam", frame);

        if(cv::waitKey(10) >= 0) break;
    }
      

    return 0;
}
#include <iostream>
#include "opencv2/core/core.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <vector>
#include <opencv2/opencv.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <QApplication>
#include <QMouseEvent>
#include <QLabel>

#include "calibrate.h"
#include "StereoVision.h"

using namespace std;
using namespace cv;
using namespace cuda;

//kodestos
float baseline = 7.0;
float focalLength = 6.0;
float alpha = 56.6; //camera fov

void onMouse(int event, int x, int y, int flags, void* userdata) {
    if (event == EVENT_LBUTTONDOWN) {
        cout << "Mouse clicked at (" << x << ", " << y << ")" << endl;

    }
}

int main(){

    Mat leftFrame = imread("/home/c_ubuntu/Repos/sw/f1L.jpg");
    Mat rightFrame = imread("/home/c_ubuntu/Repos/sw/f1R.jpg");

    StereoVision stereovision(baseline, alpha, focalLength);

    if(leftFrame.empty()){
        cout << "Cannot open Left image" << endl;
        return -1;
    }

    if(rightFrame.empty()){
        cout << "Cannot open Right image" << endl;
        return -1;
    }

    namedWindow("Left Frame");
    namedWindow("Right Frame");
    namedWindow("Left Mask");
    namedWindow("Right Mask");

    setMouseCallback("Left Frame", onMouse, NULL);

    Mat leftMask, rightMask;
    Mat leftResFrame , rightResFrame;

    Point leftCircle, rightCircle;

    float ballDepth = 0;

    // No need for a loop if you are working with static images
    //while (true) {

        //calibration of the frames
        //stereovision.undistortFrame(leftFrame);
        //stereovision.undistortFrame(rightFrame);

        //applying hsv filter
        leftMask = stereovision.add_HSV_filter(leftFrame, 0);
        rightMask = stereovision.add_HSV_filter(rightFrame, 1);

        //frames after hsv filter
        bitwise_and(leftFrame, leftFrame, leftResFrame, leftMask);
        bitwise_and(rightFrame, rightFrame, rightResFrame, rightMask);

        //detect circles
        leftCircle = stereovision.find_ball(leftFrame,leftMask);
        leftCircle = stereovision.find_ball(rightFrame,rightMask);

        //calculate depth of the ball
        if (!leftCircle.x || !rightCircle.x){
            putText(leftFrame, "Tracking lost", {75,50}, FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2);
            putText(rightFrame, "Tracking lost", {75,50}, FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2);
        } else {
            //vector of all depths in case of several balls detected.
            ballDepth = stereovision.find_depth(leftCircle,rightCircle,leftFrame,rightFrame);
            putText(leftFrame, "Tracking", {75,50}, FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2);
            putText(rightFrame, "Tracking", {75,50}, FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2);

            cout << "Ball depth: " << ballDepth << endl;
        }

        imshow("Left Frame", leftFrame);
        imshow("Right Frame", rightFrame);
        imshow("Left Mask", leftMask);
        imshow("Right Mask", rightMask);



       waitKey(0);// Add this line if you want to wait for a key press after displaying the images
    //}

    return 0;
}

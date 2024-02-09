#ifndef STEREOVISION_H
#define STEREOVISION_H

#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include <iostream>
#include <string>

using namespace std;
using namespace cv;

class StereoVision {

public:
    StereoVision(float baseline , float alpha , float focalLength)
        : baseline(baseline), alpha(alpha),focalLength(focalLength){}
    // Calibrate the Frames
    void undistortFrame(Mat& frame);

    //Add HSV filter - Filter out / Segment the red ball
    Mat add_HSV_filter (Mat& frame, int camera);

    //Find the Circle/Ball - only find the largest one - reduse false positives
    Point find_ball(Mat& frame, Mat& mask);

    //Calculate the depth to the ball
    float find_depth(Point circleLeft, Point circleRight, Mat& leftFrame, Mat& rightFrame);


private:
    float baseline      = 0;
    float alpha           = 0;
    float focalLength = 0;
};

#endif // STEREOVISION_H

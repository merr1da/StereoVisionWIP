#pragma once
#ifndef CALIBRATE_H
#define CALIBRATE_H

#include <iostream>

#include "opencv2/core.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <opencv2/opencv.hpp>
//#include <opencv2/cudaarithm.hpp>


using namespace std;
using namespace cv;
using namespace cuda;


typedef struct CamOutputParameters{
    cv::Mat cameraM1;
    cv::Mat cameraM2;
    cv::Mat distCoeffs1;
    cv::Mat distCoeffs2;
    cv::Mat R;
    cv::Mat T;
    cv::Mat E;
    cv::Mat F;
    cv::Mat perView;
    double RMS;
}cam_par_output_t;


void calibrate_camera(std::vector<cv::String> images, std::string path, cv::Mat &cameraMatrix, cv::Mat &distCoeffs, cv::Mat &R, cv::Mat &T);
void calibrate_stereo(std::vector<cv::String> im1, std::vector<cv::String> im2, std::string path1, std::string path2, cam_par_output_t &outp_params);

#endif // CALLIBRATE_H

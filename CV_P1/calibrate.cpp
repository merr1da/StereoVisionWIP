#include "calibrate.h"

int CHECKERBOARD[2]{4,7}; // 6 x 9 - число узлов вдоль столбцов и строк шахматной доски

void calibrate_camera(std::vector<cv::String> images, std::string path, cv::Mat &cameraMatrix, cv::Mat &distCoeffs, cv::Mat &R, cv::Mat &T){
    // Creating vector to store vectors of 3D points for each checkerboard image
    std::vector<std::vector<cv::Point3f> > objpoints;

    // Creating vector to store vectors of 2D points for each checkerboard image
    std::vector<std::vector<cv::Point2f> > imgpoints;

    // Defining the world coordinates for 3D points
    std::vector<cv::Point3f> objp;
    for(int i{0}; i<CHECKERBOARD[1]; i++)
    {
        for(int j{0}; j<CHECKERBOARD[0]; j++)
            objp.push_back(cv::Point3f(j,i,0));
    }

    cv::glob(path, images);

    cv::Mat frame, gray;
    std::vector<cv::Point2f> corner_pts;

    bool success;

    // Looping over all the images in the directory
    for(int i{0}; i<images.size(); i++)
    {
        frame = cv::imread(images[i]);
        cv::cvtColor(frame,gray,cv::COLOR_BGR2GRAY);

        // Finding checker board corners
        // If desired number of corners are found in the image then success = true
        success = cv::findChessboardCorners(gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE );

      /*
       * If desired number of corner are detected,
       * we refine the pixel coordinates and display
       * them on the images of checker board
      */

      if(success)
      {
        cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001);

        // refining pixel coordinates for given 2d points.
        cv::cornerSubPix(gray,corner_pts,cv::Size(11,11), cv::Size(-1,-1),criteria);

        // Displaying the detected corner points on the checker board
        cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);

        objpoints.push_back(objp);
        imgpoints.push_back(corner_pts);
      }

      cv::imshow("Image",frame);
      cv::waitKey(0);
    }

    cv::destroyAllWindows();

    //cv::Mat cameraMatrix,distCoeffs,R,T;

    cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray.rows,gray.cols), cameraMatrix, distCoeffs, R, T);
}

void calibrate_stereo(std::vector<cv::String> im1, std::vector<cv::String> im2, std::string path1, std::string path2, cam_par_output_t &outp_params){
    std::vector<std::vector<cv::Point3f> > objpoints;
    std::vector<std::vector<cv::Point2f>> imgpoints_left, imgpoints_right;

    glob(path1, im1);
    glob(path2, im2);

    cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001);

    std::vector<cv::Point3f> objp;
    for(int i{0}; i<CHECKERBOARD[1]; i++){
        for(int j{0}; j<CHECKERBOARD[0]; j++)
            objp.push_back(cv::Point3f(j,i,0));
    }

    //int width = c1_images[0].cols;
    //int height = c1_images[0].rows;

    //cv::Mat c1_images = cv::imread(im1[i], 1);
    //cv::Mat c1_images  = cv::imread(im2[i], 1);
    std::vector<cv::Mat> c1_images, c2_images;

    for(int i{0}; i<im1.size(); i++){
        cv::Mat im_1 = cv::imread(im1[i], 1);
        c1_images.push_back(im_1);

        cv::Mat im_2  = cv::imread(im2[i], 1);
        c2_images.push_back(im_2);
    }


    for (size_t i = 0; i < im1.size(); i++) {
        cv::Mat gray1, gray2;
        cv::cvtColor(c1_images[i], gray1, cv::COLOR_BGR2GRAY);
        cv::cvtColor(c2_images[i], gray2, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> corners1, corners2;
        bool c_ret1 = cv::findChessboardCorners(gray1, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corners1);
        bool c_ret2 = cv::findChessboardCorners(gray2, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corners2);

        if (c_ret1 && c_ret2) {
            cv::cornerSubPix(gray1, corners1, cv::Size(11, 11), cv::Size(-1, -1), criteria);
            cv::cornerSubPix(gray2, corners2, cv::Size(11, 11), cv::Size(-1, -1), criteria);

            cv::drawChessboardCorners(c1_images[i], cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corners1, c_ret1);
            cv::imshow("img", c1_images[i]);

            cv::drawChessboardCorners(c2_images[i], cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corners2, c_ret2);
            cv::imshow("img2", c2_images[i]);
            cv::waitKey(0);

            objpoints.push_back(objp);
            imgpoints_left.push_back(corners1);
            imgpoints_right.push_back(corners2);
        }
    }

    outp_params.RMS = cv::stereoCalibrate(objpoints, imgpoints_left, imgpoints_right, outp_params.cameraM1, outp_params.distCoeffs1, outp_params.cameraM2, outp_params.distCoeffs2,
                                     c1_images[0].size(), outp_params.R, outp_params.T, outp_params.E, outp_params.F, outp_params.perView, CALIB_FIX_INTRINSIC, criteria);
}

/*
void calibrate_stereo(std::string path, std::vector<cv::String> images, cv::Mat CamMatL, cv::Mat distMatL, cv::Mat CamMatR, cv::Mat distMatR, cam_par_output_t &outp_params){
    std::vector<std::vector<cv::Point3f> > objpoints;
    std::vector<std::vector<cv::Point2f>> imgpoints_left, imgpoints_right;

    cv::glob(path, images);

    std::sort(images.begin(), images.end());
    std::vector<cv::Mat> c1_images, c2_images;

    for (size_t i = 0; i < images.size() / 2; i++) {
        cv::Mat im1 = cv::imread(images[i], 1);
        c1_images.push_back(im1);

        cv::Mat im2 = cv::imread(images[i + images.size() / 2], 1);
        c2_images.push_back(im2);
    }

    cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001);

    std::vector<cv::Point3f> objp;
    for(int i{0}; i<CHECKERBOARD[1]; i++){
        for(int j{0}; j<CHECKERBOARD[0]; j++)
            objp.push_back(cv::Point3f(j,i,0));
    }

    int width = c1_images[0].cols;
    int height = c1_images[0].rows;

    for (size_t i = 0; i < c1_images.size(); i++) {
        cv::Mat gray1, gray2;
        cv::cvtColor(c1_images[i], gray1, cv::COLOR_BGR2GRAY);
        cv::cvtColor(c2_images[i], gray2, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> corners1, corners2;
        bool c_ret1 = cv::findChessboardCorners(gray1, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corners1);
        bool c_ret2 = cv::findChessboardCorners(gray2, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corners2);

        if (c_ret1 && c_ret2) {
            cv::cornerSubPix(gray1, corners1, cv::Size(11, 11), cv::Size(-1, -1), criteria);
            cv::cornerSubPix(gray2, corners2, cv::Size(11, 11), cv::Size(-1, -1), criteria);

            cv::drawChessboardCorners(c1_images[i], cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corners1, c_ret1);
            cv::imshow("img", c1_images[i]);

            cv::drawChessboardCorners(c2_images[i], cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corners2, c_ret2);
            cv::imshow("img2", c2_images[i]);
            cv::waitKey(0);

            objpoints.push_back(objp);
            imgpoints_left.push_back(corners1);
            imgpoints_right.push_back(corners2);
        }
    }

    outp_params.RMS = cv::stereoCalibrate(objpoints, imgpoints_left, imgpoints_right, CamMatL, distMatL, CamMatR, distMatR,
                                     c1_images[0].size(), outp_params.R, outp_params.T, outp_params.E, outp_params.F, outp_params.perView, CALIB_FIX_INTRINSIC, criteria);
}
*/

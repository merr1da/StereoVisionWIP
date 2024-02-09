#include "StereoVision.h"





Mat StereoVision::add_HSV_filter(Mat& frame, int camera) {

    //Blurring frame to reduce noise
    GaussianBlur(frame , frame , {5,5} , 0);

    cvtColor(frame , frame , COLOR_BGR2HSV);

    Mat mask;

    vector<int> lowerLimitRedRight  = {60,110,50}; // lower limit for red ball R
    vector<int> upperLimitRedRight = {255,255,255}; //upper limit
    vector<int> lowerLimitRedLeft    = {140,110,50}; // lower limit for red ball L
    vector<int> upperLimitRedLeft   = {255,255,255}; //upper limit

    if (camera == 1) {
        inRange(frame,lowerLimitRedRight,upperLimitRedRight , mask);
    } else{
        inRange(frame,lowerLimitRedLeft,upperLimitRedLeft , mask);
    }

    erode(mask, mask, (3, 3));
    dilate(mask, mask, (3, 3));

    return mask;

}



Point StereoVision::find_ball(Mat& frame, Mat& mask){

    vector<vector<Point> > contours;

    findContours(mask, contours, RETR_EXTERNAL , CHAIN_APPROX_SIMPLE);

    //sort the contours to find biggest one
    sort(contours.begin(), contours.end(), [](const vector<Point>& c1, const vector<Point>& c2){
        return contourArea(c1, false) <contourArea(c2, false);
    });

    if (contours.size() > 0 ){
        vector<Point> largestContour = contours[contours.size() -1];
        Point2f center;
        float radius;
        minEnclosingCircle(largestContour, center, radius);
        Moments m = moments(largestContour);
        Point centerPoint(m.m10 / m.m00, m.m01 / m.m00);

        //Only preceed if the radius is grater that a minimum threshold
        if (radius > 10){
            //Draw circle and centroid
            circle(frame, center, int(radius), (0,255, 255), 2);
            circle(frame, centerPoint, 5, (0,0,255), -1);
        }
        return centerPoint;
    }

    return {0,0};
}

float StereoVision::find_depth(Point circleLeft, Point circleRight, Mat& leftFrame, Mat& rightFrame){

    int focal_pixels = 0;

    if (rightFrame.cols == leftFrame.cols){

        //Convert focal length f from [mm] to [pixel]
        focal_pixels = (rightFrame.cols * 0.5)/ tan(alpha * 0.5 * CV_PI / 180.0);
    }
    else{
        cout << "Left and Right camera frames do not have the same pixel width"<< endl;
    }
    int xLeft = circleLeft.x;
    int xRight = circleRight.x;

    //calculate disparity
    int disparity = xLeft - xRight;

    //Calculate the depth
    float zDepth = (baseline * float(focal_pixels)) / float(disparity); //depth in [cm]

    return abs(zDepth);

}

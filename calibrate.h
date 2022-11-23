#ifndef CALIBRATE_H
#define CALIBRATE_H
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <math.h>
#include <iostream>
#include <vector>
#include <string>
#include <glob.h>

using namespace cv;
using namespace std;

using namespace std;
using namespace cv;

class calibrate
{
public:
    calibrate();
    void setCalImages(string path);
    void setImage(string path);
    void setImage(Mat img);
    void calculateMatrix();
    Mat undistortImg(Mat img);
    Mat getMapx();
    Mat getMapy();

};

#endif // CALIBRATE_H

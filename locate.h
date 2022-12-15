#ifndef LOCATE_H
#define LOCATE_H
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <math.h>
#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Dense>

using namespace std;
using namespace cv;

class locate
{
public:
    locate();
    void findBall();
    void findTarget();
    void setImage(Mat);
    void setImage(string);
    void getBallCoords();
    void findTargetHough();
    Point getTargetCoords();


    double temp[3];

    double targetCord[3];
};

#endif // LOCATE_H

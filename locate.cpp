#include "locate.h"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <math.h>
#include <iostream>
#include <vector>
#include <string>

using namespace cv;
using namespace std;


Mat targetBallImg, ball, target;
Point ballCoords, targetCoords;

locate::locate()
{

}

void locate::setImage(Mat image){
    targetBallImg=image;
}

void locate::setImage(string imgpath){
    setImage(imread(imgpath));
}

void locate::findBall() {
    //bolden lokaliseres med følgende HSV stats
    Mat imageHSV;
    int hmin = 0, smin = 71, vmin = 160, hmax = 43, smax = 122, vmax = 255;
    cvtColor(targetBallImg, imageHSV, COLOR_BGR2HSV);
    Scalar ballLower(hmin, smin, vmin);
    Scalar ballUpper(hmax, smax, vmax);
    inRange(imageHSV, ballLower, ballUpper, ball);

    //noice skal fjernes
    Mat berod;

    int morph_size = 4;
    Mat ballelement = getStructuringElement(MORPH_ELLIPSE, Size(2 * morph_size +15, 2*morph_size +15),
    Point(morph_size, morph_size));
    erode(ball, berod, ballelement, Point(-1,-1),1);
    dilate(berod, ball, ballelement, Point(-1,-1),1);

    //en vector fyldes med koordinater for de hvide pixels
    vector<vector<Point> > bcoords;

    cv::findContours(ball, bcoords, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    //for bolden:
    int bsumx = 0;
    //  Vi summerer x coordinaterne
      for (int i=0 ; i < bcoords[0].size(); i++){
          bsumx += bcoords[0][i].x;
      }

    int bavx = bsumx/bcoords[0].size();
    ballCoords.x = bavx;

    int bsumy = 0;
    //  Vi summerer y coordinaterne
      for (int i=0 ; i < bcoords[0].size(); i++){
          bsumy += bcoords[0][i].y;
      }

    int bavy = bsumy/bcoords[0].size();
    ballCoords.y = bavy;
}

void locate::findTarget() {

    //Ringen lokaliseres med følgende HSV stats
    int hmin = 90, smin = 91, vmin = 109, hmax = 115, smax = 255, vmax = 172;

    //Scalars til inRange
    Mat imageHSV;
    Scalar ringLower(hmin, smin, vmin);
    Scalar ringUpper(hmax, smax, vmax);
    cvtColor(targetBallImg, imageHSV, COLOR_BGR2HSV);
    inRange(imageHSV, ringLower, ringUpper, target);


    //noise fjernes ved erode og dilate
    double morph_size = 1.3;
    Mat relement = getStructuringElement(MORPH_ELLIPSE, Size(2 * morph_size +1, 2*morph_size +1),
    Point(morph_size, morph_size));

    Mat rerod;

    erode(target, rerod, relement, Point(-1,-1),1);
    dilate(rerod, target, relement, Point(-1,-1),1);

    //Find targetCoords
    vector<vector<Point> > tcoords;

    cv::findContours(target, tcoords, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    //Find gennemsnit
    int tsumx = 0;
    //  Vi summerer x coordinaterne
      for (int i=0 ; i < tcoords[0].size(); i++){
          tsumx += tcoords[0][i].x;
      }
        //Average target x value
    int tavx = tsumx/tcoords[0].size();
    targetCoords.x = tavx;

    int tsumy = 0;
    //  Vi summerer y coordinaterne
      for (int i=0 ; i < tcoords[0].size(); i++){
          tsumy += tcoords[0][i].y;
      }
        //Average target y value
    int tavy = tsumy/tcoords[0].size();
    targetCoords.y = tavy;
}

void locate::getBallCoords(){
    cout<<"Ball Coords "<<ballCoords.x<<", "<<ballCoords.y<<endl;
    //circle(imgfix,Point(ballCoords.x,ballCoords.y));

    temp[0] = ballCoords.x;
    temp[1] = ballCoords.y;
    temp[2] = 1;
    circle(targetBallImg, Point(ballCoords.x,ballCoords.y), 1, Scalar(0,0,255), 3, LINE_AA);
    imshow("dot",targetBallImg);

}


Point locate::getTargetCoords(){
    cout<<"Target Coords "<<targetCoords.x<<", "<<targetCoords.y<<endl;
    return targetCoords;
}

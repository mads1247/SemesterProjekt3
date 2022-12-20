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
    //bolden lokaliseres med følgende HSV s
    Mat imageHSV;
    int hmin = 0, smin = 60, vmin = 170, hmax = 50, smax = 140, vmax = 255;
    //int hmin = 0, smin = 71, vmin = 160, hmax = 43, smax = 122, vmax = 255;
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
    imshow("ballbin",ball);
    waitKey(0);
}

void locate::findTargetHoughBin() {
    //Ringen lokaliseres først med følgende HSV stats
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
    imshow("target",target);
    waitKey(0);


    Mat rerod;

    erode(target, rerod, relement, Point(-1,-1),1);
    dilate(rerod, target, relement, Point(-1,-1),1);
    imshow("target",target);
    waitKey(0);

    Mat result = target;
    medianBlur(target, target, 5);

    vector<Vec3f> circles;
    HoughCircles(target, circles, HOUGH_GRADIENT, 1,
                    target.rows/16,  // change this value to detect circles with different distances to each other
                    100, 30, 30, 80 // change the last two parameters
               // (min_radius & max_radius) to detect larger circles
       );
    targetCoords.x = circles[0][0];
    targetCoords.y = circles[0][1];
    imshow("Target",target);
    for( size_t i = 0; i < circles.size(); i++ )
        {
            Vec3i c = circles[i];
            Point center = Point(c[0], c[1]);
            // circle center
            circle(targetBallImg, center, 1, Scalar(0,255,0), 3, LINE_AA);
            // circle outline
            int radius = c[2];
            circle(targetBallImg, center, radius, Scalar(255,0,0), 3, LINE_AA);
        }

    targetCord[0]=targetCoords.x;
    targetCord[1]=targetCoords.y;
    targetCord[2]=1;


    imshow("Circles", targetBallImg);
    waitKey(0);
}


void locate::findTargetHough() {
    Mat gray , result;
    result = targetBallImg;
    cvtColor(targetBallImg, gray, COLOR_RGB2GRAY);


    medianBlur(gray, gray, 5);

    vector<Vec3f> circles;
    HoughCircles(gray, circles, HOUGH_GRADIENT, 1,
                    gray.rows/16,  // change this value to detect circles with different distances to each other
                    100, 30, 30, 80 // change the last two parameters
               // (min_radius & max_radius) to detect larger circles
       );
    targetCoords.x = circles[0][0];
    targetCoords.y = circles[0][1];
    for( size_t i = 0; i < circles.size(); i++ )
        {
            Vec3i c = circles[i];
            Point center = Point(c[0], c[1]);
            // circle center
            circle(result, center, 1, Scalar(0,255,0), 3, LINE_AA);
            // circle outline
            int radius = c[2];
            circle(result, center, radius, Scalar(255,0,0), 3, LINE_AA);
        }

    targetCord[0]=targetCoords.x;
    targetCord[1]=targetCoords.y;
    targetCord[2]=1;

    imshow("Circles", result);
    waitKey(0);

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
    imshow("target", target);
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

    targetCord[0] = tavx;
    targetCord[1] = tavy;
    targetCord[2] = 1;

}

void locate::getBallCoords(){
    cout<<"Ball Coords "<<ballCoords.x<<", "<<ballCoords.y<<endl;


    temp[0] = ballCoords.x;
    temp[1] = ballCoords.y;
    temp[2] = 1;
    circle(targetBallImg, Point(ballCoords.x,ballCoords.y), 1, Scalar(0,0,255), 3, LINE_AA);
    circle(targetBallImg, Point(targetCoords.x,targetCoords.y), 1, Scalar(0,255,255), 3, LINE_AA);
    imshow("dot",targetBallImg);
    waitKey(0);

}


Point locate::getTargetCoords(){
    cout<<"Target Coords "<<targetCoords.x<<", "<<targetCoords.y<<endl;
    return targetCoords;
}

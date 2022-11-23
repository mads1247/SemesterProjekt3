#include "calibrate.h"
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

void setCalImages(string path);
void setImage(string path);
void setImage(Mat img);
void calculateMatrix();
Mat undistortImg(Mat img);
Mat getMapx();
Mat getMapy();


vector<cv::String> images;
Mat mapX, mapY;

calibrate::calibrate()
{

}

void calibrate::setCalImages(string path){
    cv::glob(path, images, false);
}

void calibrate::calculateMatrix(){
    cv::Size patternSize(10-1 , 7 -  1 );
      std::vector<std::vector<cv::Point2f>> q(images.size());

      std::vector<std::vector<cv::Point3f>> Q;
      // 1. Generate checkerboard (world) coordinates Q. The board has 25 x 18
      // fields with a size of 15x15mm

      int checkerBoard[2] = {10,7};
      // Defining the world coordinates for 3D points
        std::vector<cv::Point3f> objp;
        for(int i = 1; i<checkerBoard[1]; i++){
          for(int j = 1; j<checkerBoard[0]; j++){
            objp.push_back(cv::Point3f(j,i,0));
          }
        }

      std::vector<cv::Point2f> imgPoint;
      // Detect feature points
      std::size_t i = 0;
      for (auto const &f : images) {
        std::cout << std::string(f) << std::endl;

        // 2. Read in the image an call cv::findChessboardCorners()
        cv::Mat img = cv::imread(images[i]);
        cv::Mat gray;

        cv::cvtColor(img, gray, cv::COLOR_RGB2GRAY);

        bool patternFound = cv::findChessboardCorners(gray, patternSize, q[i], cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);

        // 2. Use cv::cornerSubPix() to refine the found corner detections
        if(patternFound){
            cv::cornerSubPix(gray, q[i],cv::Size(11,11), cv::Size(-1,-1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
            Q.push_back(objp);
        }

        // Display  --- Delete after test
        cv::drawChessboardCorners(img, patternSize, q[i], patternFound);
        //cv::imshow("chessboard detection", img);
        //cv::waitKey(0);

        i++;
      }


      cv::Matx33f K(cv::Matx33f::eye());  // intrinsic camera matrix
      cv::Vec<float, 5> k(0, 0, 0, 0, 0); // distortion coefficients

      std::vector<cv::Mat> rvecs, tvecs;
      std::vector<double> stdIntrinsics, stdExtrinsics, perViewErrors;
      int flags = cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_FIX_K3 +
                  cv::CALIB_ZERO_TANGENT_DIST + cv::CALIB_FIX_PRINCIPAL_POINT;
      cv::Size frameSize(1440, 1080);

      std::cout << "Calibrating..." << std::endl;
      // 4. Call "float error = cv::calibrateCamera()" with the input coordinates
      // and output parameters as declared above...

      float error = cv::calibrateCamera(Q, q, frameSize, K, k, rvecs, tvecs, flags);

      std::cout << "Reprojection error = " << error << "\nK =\n"
                << K << "\nk=\n"
                << k << std::endl;

      // Precompute lens correction interpolation
      cv::initUndistortRectifyMap(K, k, cv::Matx33f::eye(), K, frameSize, CV_32FC1,
                                  mapX, mapY);
}

Mat calibrate::undistortImg(Mat img) {
    Mat imgOut;
    remap(img, imgOut, mapX, mapY, cv::INTER_LINEAR);
    return imgOut;
}

Mat calibrate::getMapx(){
    return mapX;
}

Mat calibrate::getMapy(){
    return mapY;
}

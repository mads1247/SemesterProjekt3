#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <math.h>
#include "locate.h"
#include "calibrate.h"
#include "cam.h"
#include <iostream>
#include <ur_rtde/rtde_control_interface.h>
#include <Eigen/Dense>
#include <thread>
#include <chrono>
#include "kastebane.h"

using namespace cv;
using namespace std;
using namespace ur_rtde;
using namespace std::chrono;

int main()
{

   //ur_rtde::RTDEControlInterface rtde_control("192.168.100.11");

    Eigen::MatrixXf target_bord(3,1);
    target_bord << 0.200,
            0.250,
            0;

    double t = 0.25;

    Kastebane kast(target_bord,t);

    std::cout << kast.getV0_f() << std::endl;


    /*
// Parameters
  double velocity = 0.5;
  double acceleration = 0.5;
  double dt = 1.0/500; // 2ms
  double lookahead_time = 0.1;
  double gain = 300;
  std::vector<double> joint_q = {-1.54, -1.83, -2.28, -0.59, 1.60, 0.023};

  // Move to initial joint position with a regular moveJ
  rtde_control.moveJ(joint_q);

  // Execute 500Hz control loop for 2 seconds, each cycle is ~2ms
  for (unsigned int i=0; i<1000; i++)
  {
    steady_clock::time_point t_start = rtde_control.initPeriod();
    rtde_control.servoJ(joint_q, velocity, acceleration, dt, lookahead_time, gain);
    joint_q[0] += 0.001 * (1*i^3+1*i^2+1);
    joint_q[1] += 0.001 * (1*i^3+1*i^2+1);
    joint_q[2] += 0.001 * (1*i^3+1*i^2+1);
    joint_q[3] += 0.001 * (1*i^3+1*i^2+1);
    joint_q[4] += 0.001 * (1*i^3+1*i^2+1);
    joint_q[5] += 0.001 * (1*i^3+1*i^2+1);
    rtde_control.waitPeriod(t_start);
  }

  rtde_control.servoStop();
  rtde_control.stopScript();



     *
     */
/*
        locate l;
        calibrate c;
        cam cc;
        c.setCalImages("../Desktop/SKAK/Image*.png");
        c.calculateMatrix();
        cc.grabImmage("/home/mads/Documents/test.png"); //path som billedet gemmes på


        Mat img = imread("/home/mads/Documents/test.png");
        //imshow("Dis",img);
        waitKey(0);
        Mat imgfix = c.undistortImg(img);
        imshow("Undis", imgfix);
        waitKey(0);
        l.setImage("/home/mads/Documents/test.png");
        l.findBall();
        //l.findTarget(); //kræver target i billede
        l.getBallCoords();
        l.getTargetCoords(); //kræver target på billede
        waitKey(0);


        Eigen::MatrixXf TransformationTableToRobot(4,4);

        TransformationTableToRobot << -0.3773,0.9261,0,50.4975,
                0.9261,0.3773,0,-63.1134,
                0,0,1,1,
                0,0,0,1;


        Eigen::Matrix3f TransformationCameraToTable;

        TransformationCameraToTable << 0.02230201441793034, 1.205149329685084, -612.5140391701569,
        -1.248877396445485, 0.02241070532477318, 652.2172921861224,
        0.0001019882655942446, -4.568084195199251e-05, 1;

        Eigen::MatrixXf u(3,1);

        u(0,0) = l.temp[0];
        u(1,0) = l.temp[1];
        u(2,0) = l.temp[2];
        std::cout << "Pixel vector \n" << u << std::endl;

        Eigen::MatrixXf v(3,1);
        std::cout << "mm værdi i tableframe \n" << TransformationCameraToTable*u << std::endl;
        v = TransformationCameraToTable*u;

        Eigen::MatrixXf o(4,1);
        o(0,0) = v(0,0) / 10;
        o(1,0) = v(1,0) / 10;
        o(2,0) = v(2,0) / 10;
        o(3,0) = 1;

        Eigen::MatrixXf k(4,1);
        k(0,0) = 30;
        k(1,0) = -30;
        k(2,0) = 1;
        k(3,0) = 1;

        std::cout << "Laver om til homogene og cm \n "<<o << std::endl;

        Eigen::MatrixXf p(4,1);
        p = TransformationTableToRobot*o;

        p(0,0) = p(0,0) *0.01;
        p(1,0) = p(1,0) *0.01;
        p(2,0) = p(2,0) *0.01;

         Eigen::MatrixXf h(4,1);
        h(0,0) = p(0,0) *0.01;
        h(1,0) = p(1,0) *0.01;
        h(2,0) = p(2,0) *0.01;

        std::cout << "Punktet i m: \n" << p << std::endl;

        std::vector<double> point = {p(0,0),p(1,0),0.173,3.053,0.623,-0.012};
        double speed =0.2;

        double acceleration = 0.5;


        // move to tthe generated point
        rtde_control.moveL(point);

        //activate gripper (how to doe?)

        rtde_control.stopScript();




*/
        return (0);

}

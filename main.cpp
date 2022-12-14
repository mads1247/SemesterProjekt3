
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
#include "Socket.h"


using namespace cv;
using namespace std;
using namespace ur_rtde;
using namespace std::chrono;

std::vector<double> CameraToRobt(double point[3], std::string name){

    Eigen::MatrixXf TransformationTableToRobot(4,4);
    TransformationTableToRobot << -0.3773,0.9261,0,50.4975,
            0.9261,0.3773,0,-63.1134,
            0,0,1,1,
            0,0,0,1;


    Eigen::Matrix3f TransformationCameraToTable;
    TransformationCameraToTable << -1.183647186295619, 0.001916939742437651, 627.286799797398,
    0.004524711769935897, 1.173696834813918, -582.8667080093462,
    1.650033801479657e-05, -2.645270687276527e-07, 1;

    Eigen::MatrixXf ball1(3,1);

    ball1(0,0) = point[0];
    ball1(1,0) = point[1];
    ball1(2,0) = point[2];
    std::cout << "Pixel vector for " << name << "\n" << ball1 << std::endl;

    Eigen::MatrixXf ball2(3,1);

    ball2 = TransformationCameraToTable*ball1;

    Eigen::MatrixXf o(4,1);
    o(0,0) = ball2(1,0) / 10;
    o(1,0) = ball2(0,0) / 10;
    o(2,0) = ball2(2,0) / 10;
    o(3,0) = 1;

    std::cout << "Punktet i table: " << name << "\n" << o << std::endl;

    Eigen::MatrixXf p(4,1);
    p = TransformationTableToRobot*o;

    p(0,0) = p(0,0) *0.01;
    p(1,0) = p(1,0) *0.01;
    p(2,0) = p(2,0) *0.01;

    std::vector<double> res = {p(0,0),p(1,0),0.173,3.053,0.623,-0.012};

    std::cout << "Punktet i robotframe: " << name << "\n" << p << std::endl;

    return res;

}

int main()
{

    Socket gripper;
    gripper.connect();
    gripper.home();


    //Opsætning af kamera og kaliberings billeder osv
    locate l;
    calibrate c;
    cam cc;
    c.setCalImages("../Desktop/SKAK/Image*.png");
    c.calculateMatrix();
    cc.grabImmage("/home/mads/Documents/test.png"); //path som billedet gemmes på


    Mat img = imread("/home/mads/Documents/test.png");
    //imshow("Dis",img);
    Mat imgfix = c.undistortImg(img);
    //imshow("Undis", imgfix);
    l.setImage("/home/mads/Documents/test.png");
    l.findBall();
    l.findTarget(); //kræver target i billede
    l.getTargetCoords();

    l.getBallCoords();
     //kræver target på billede

    //Tag billede og find target og bold
    /*
    Eigen::MatrixXf TransformationTableToRobot(4,4);
    TransformationTableToRobot << -0.3773,0.9261,0,50.4975,
            0.9261,0.3773,0,-63.1134,
            0,0,1,1,
            0,0,0,1;


    Eigen::Matrix3f TransformationCameraToTable;
    TransformationCameraToTable << -1.183647186295619, 0.001916939742437651, 627.286799797398,
 0.004524711769935897, 1.173696834813918, -582.8667080093462,
 1.650033801479657e-05, -2.645270687276527e-07, 1;

    Eigen::MatrixXf ball1(3,1);

    ball1(0,0) = l.temp[0];
    ball1(1,0) = l.temp[1];
    ball1(2,0) = l.temp[2];
    std::cout << "Pixel vector \n" << ball1 << std::endl;

    Eigen::MatrixXf ball2(3,1);
    std::cout << "mm værdi i tableframe \n" << TransformationCameraToTable*ball1 << std::endl;
    ball2 = TransformationCameraToTable*ball1;

    Eigen::MatrixXf o(4,1);
    o(0,0) = ball2(1,0) / 10;
    o(1,0) = ball2(0,0) / 10;
    o(2,0) = ball2(2,0) / 10;
    o(3,0) = 1;

    std::cout << "Laver om til homogene og cm \n "<<o << std::endl;

    Eigen::MatrixXf p(4,1);
    p = TransformationTableToRobot*o;

    p(0,0) = p(0,0) *0.01;
    p(1,0) = p(1,0) *0.01;
    p(2,0) = p(2,0) *0.01;

    Eigen::MatrixXf target1(3,1);


    target1(0,0) = l.targetCord[1];
    target1(1,0) = l.targetCord[0];
    target1(2,0) = l.targetCord[2];
    std::cout << "target pixel vector \n" << target1 << std::endl;

    Eigen::MatrixXf target2(3,1);
    std::cout << "mm værdi i tableframe \n" << TransformationCameraToTable*target1 << std::endl;
    target2 = TransformationCameraToTable*target1;

    Eigen::MatrixXf target3(4,1);
    target3(0,0) = target2(1,0) / 10;
    target3(1,0) = target2(0,0) / 10;
    target3(2,0) = target2(2,0) / 10;
    o(3,0) = 1;

    std::cout << "Laver om til homogene og cm \n "<<o << std::endl;

    Eigen::MatrixXf target4(4,1);
    p = TransformationTableToRobot*target3;

    target4(0,0) = target4(0,0) *0.01;
    target4(1,0) = target4(1,0) *0.01;
    target4(2,0) = target4(2,0) *0.01;


    std::cout << "Punktet i m: \n" << p << std::endl;
    std::cout << "Target i m: \n" << target4 << std::endl;
    */


    std::vector<double> targetBord = /*{0.1,-0.4,0};*/CameraToRobt(l.targetCord, "Target");

    std::vector<double> point = CameraToRobt(l.temp,"Ball");

    //Find kastebane og trajectory

    QCoreApplication a(int& argc, char** argv);

    Socket client;


    double in1 = targetBord[0];
    double in2 = targetBord[1];
    double t = 0.251;

    std::this_thread::sleep_for(0.3s);

    std::cout << in1 << " " << in2 << std::endl;

    client.connectMatlab();

    client.writeMatlab(in1);
    client.writeMatlab(in2);

    client.writeMatlab(t);



    std::string data = client.readMatlab();
    std::this_thread::sleep_for(1s);
    //client.disconnectMatlab();

    std::cout << data << "\n" << std::endl;

    //std::string data = "-1.2233,-1.2603,-1.6849,-0.71437,1.3732,-0.15063,  -0.35953,0.75104,0.33189,0,0,0,  2.9922";
    //double t = 0.251;

    std::string delimiter = ",";
    std::vector<double> datav;

    size_t pos = 0;
    std::string token;
    while ((pos = data.find(delimiter)) != std::string::npos) {
        token = data.substr(0, pos);
        datav.push_back(stod(token));
        data.erase(0, pos + delimiter.length());
    }

    std::vector<double> q0{datav[0],
                datav[1],
                datav[2],
                datav[3],
                datav[4],
                datav[5]};
    std::vector<double> qdot{datav[6],
                datav[7],
                datav[8],
                datav[9],
                datav[10],
                datav[11]};
    double qdotdot = datav[12];

    std::cout << qdotdot << std::endl;

    // Connect til UR robotten




    // Rykker over til bolden og saml den op


    ur_rtde::RTDEControlInterface rtde_control("192.168.100.11");

    rtde_control.moveL(point,0.1,0.2);

    gripper.grip();

    //close gripper;


    // Begynd kastet
    rtde_control.moveJ(q0,0.5,0.2);

    rtde_control.speedJ(qdot,qdotdot,t);



    //std::chrono::duration<double, std::centi> h(t); //tjek lige om det her er sekunder
    std::this_thread::sleep_for(0.23s);
    //
    gripper.release();

    //åben gripper

    rtde_control.speedStop(qdotdot);

    std::this_thread::sleep_for(5s);

    std::vector<double> resetpoint{-0.031,-0.170,0.526,2.5,0.6,0.1};
    rtde_control.moveL(resetpoint, 0.2,0.1);

    //luk diverse forbindelser
        rtde_control.stopScript();
        client.disconnectMatlab();

        gripper.disconnect();

        return (0);


}


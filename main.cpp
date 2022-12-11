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


int main()
{
    //Opsætning af kamera og kaliberings billeder osv
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
    */

    //Tag billede og find target og bold
    /*
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

    std::cout << "Laver om til homogene og cm \n "<<o << std::endl;

    Eigen::MatrixXf p(4,1);
    p = TransformationTableToRobot*o;

    p(0,0) = p(0,0) *0.01;
    p(1,0) = p(1,0) *0.01;
    p(2,0) = p(2,0) *0.01;

    std::cout << "Punktet i m: \n" << p << std::endl;

    std::vector<double> point = {p(0,0),p(1,0),0.173,3.053,0.623,-0.012};

    */

    //Find kastebane og trajectory
    QCoreApplication a(int& argc, char** argv);

    Socket client;
    std::vector<double> targetBord{0.075,0.325,0};

    double in1 = targetBord[0];
    double in2 = targetBord[1];
    double t = 0.2;


    std::this_thread::sleep_for(0.3s);
    std::cout << in1 << " " << in2 << std::endl;

    client.connectMatlab();
    client.writeMatlab(t);

    client.writeMatlab(in1);
    client.writeMatlab(in2);
    std::string data = client.readMatlab();
    //client.disconnectMatlab();

    std::cout << data << std::endl;


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

    // Connect til UR robotten

    /*
    ur_rtde::RTDEControlInterface rtde_control("192.168.100.11");

    rtde_control.moveJ(q0,0.5,0.2, false);

    std::this_thread::sleep_for(1s);

    rtde_control.speedJ(qdot,qdotdot,t);

    std::chrono::duration<double, std::centi> h(t); //tjek lige om det her er sekunder
    std::this_thread::sleep_for(h);

    rtde_control.speedStop(qdotdot);

    rtde_control.stopScript();
    */

    // Rykker over til bolden og saml den op

    // Begynd kastet


    //luk diverse forbindelser

        return (0);

}


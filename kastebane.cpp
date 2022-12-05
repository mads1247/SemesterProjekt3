#include "kastebane.h"

Kastebane::Kastebane(Eigen::MatrixXf target_bord, double t)
{

    Jacobian << 0.4811,-0.2437,-0.0767,-0.0138,0.2433,0,
            0.0772, 0.5153,0.1621,0.0291,0.0864,0,
            0,0.4679,0.6352,0.2719,-0.0062,0;

    rotm << -0.3773,0.9261,0,
            0.9261,0.3773,0,
            0,0,1;

    translation << 0.504975,
            -0.631134,
            0.01;

    transform << -0.9350,-0.1526,0.3203,0.0209,
            -0.3459,0.1915,-0.9185,-0.3192,
            0.078,-0.9696,-0.2319,0.7,
            0,0,0,1;


    slutPos << -1.1291,
               -1.1661,
               -1.5907,
               -0.6201,
                1.4675,
               -0.0564;

    Eigen::MatrixXf taget_robot = rotm*target_bord+translation;

    Eigen::MatrixXf tcpForskydelse;
    tcpForskydelse <<   0,
            0,
            0.176,
            1;

    Eigen::MatrixXf start = transform * tcpForskydelse;

     Eigen::MatrixXf start2d;
     start2d(1,1) = start(1,1);
     start2d(2,1) = start(2,1);



}

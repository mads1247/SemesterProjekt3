#include "kastebane.h"

Kastebane::Kastebane(Eigen::MatrixXf target_bord, double t)
{


    Eigen::MatrixXf rotm(3,3);
    Eigen::MatrixXf translation(3,1);
    Eigen::MatrixXf slutPos(6,1);
    Eigen::MatrixXf startPos(6,1);
    Eigen::MatrixXf transform(4,4);
    Eigen::MatrixXf Jacobian(3,6);
    Eigen::MatrixXf qdot(6,1);



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

    Eigen::MatrixXf target_robot(3,1);
    target_robot = rotm*target_bord+translation;

    Eigen::MatrixXf tcpForskydelse(4,1);
    tcpForskydelse <<   0,
            0,
            0.176,
            1;

    Eigen::MatrixXf start(4,1);
    start = transform * tcpForskydelse;

     Eigen::MatrixXf start2d(2,1);
     start2d(0,0) = start(0,0);
     start2d(1,0) = start(1,0);

     Eigen::MatrixXf target_robot2d(2,1);

     Eigen::MatrixXf kast(2,1);
     kast = target_robot2d-start2d;

     double kasteLængde = sqrt(pow(kast(0,0),2)+pow(kast(1,0),2));

     double x = kasteLængde;

     double y0 = start(2,0);

     double v0 = ((sqrt(2)*sqrt(((-g)/(y-y0-(x*tan(angle))+(x0*tan(angle)))))*(x-x0))/(2*cos(angle)));

     std::cout << "v0 :  "<< v0 << std::endl;


     Eigen::MatrixXf retning(3,1);
     retning << -kast,
             kasteLængde;

    //std::cout << retning << std::endl;

    Eigen::MatrixXf rUnit(3,1);
    rUnit = retning/sqrt(pow(retning(0,0),2)+pow(retning(1,0),2)+pow(retning(2,0),2));

    //std::cout << "runit :" << rUnit << std::endl;

    Eigen::MatrixXf v(3,1);
    v = rUnit*v0;
    //std::cout << "v :" << v << std::endl;
    qdot = Jacobian.completeOrthogonalDecomposition().pseudoInverse() * v;


    Eigen::MatrixXf qdotdot(6,1);
    qdotdot = qdot/t;


    startPos = slutPos - 0.5*qdotdot*pow(t,2);


    for (int i = 0; i < 6; ++i) {
        mqdot.push_back(qdot(i,0));
    }

    for (int i = 0; i < 6; ++i) {
        mq.push_back(startPos(i,0));
    }

    acceleration = qdotdot.maxCoeff();
}




double Kastebane::getV0_f() const
{
    return v0_f;
}

double Kastebane::getAcceleration() const
{
    return acceleration;
}

const std::vector<double> &Kastebane::getMqdot() const
{
    return mqdot;
}

const std::vector<double> &Kastebane::getMq() const
{
    return mq;
}

#ifndef KASTEBANE_H
#define KASTEBANE_H

#include <Eigen/Dense>


class Kastebane
{
public:
    Kastebane(Eigen::MatrixXf target_bord, double t);



private:
    Eigen::MatrixXf rotm;
    Eigen::MatrixXf translation;

    Eigen::MatrixXf slutPos;
     Eigen::MatrixXf startPos;
    Eigen::MatrixXf transform;

    int angle = 45;
    int x0 = 0;
    int y = 0;
    int y0 = 0;
    double g = 9.82;

    Eigen::MatrixXf Jacobian;

    Eigen::MatrixXf qdot;

    double acceleration;








};

#endif // KASTEBANE_H

#ifndef KASTEBANE_H
#define KASTEBANE_H

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

class Kastebane
{
public:
    Kastebane(Eigen::MatrixXf target_bord, double t);

    double getV0_f() const;

private:

    double angle = 0.7854;
    int x0 = 0;
    int y = 0;
    int y0 = 0;
    double g = 9.82;

    double v0_f;

    double acceleration;








};

#endif // KASTEBANE_H

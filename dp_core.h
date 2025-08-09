#define _USE_MATH_DEFINES

#include <cmath>

using namespace std;

// default values

double g = 9.81;
double l = 1.00;

//

coord dp_base(double th1, double th2, double th1p, double th2p) {
    // differential equations for second derivatives of theta 1 and theta 2

    double delta = th1 - th2;

    double den1 = l * (2 - cos(delta) * cos(delta));
    double den2 = den1;

    double th1pp = (-g * (2 * sin(th1) + sin(th1 - 2 * th2)) 
                    - 2 * sin(delta) * (th2p * th2p * l + th1p * th1p * l * cos(delta)))
                   / den1;

    double th2pp = (2 * sin(delta) * (th1p * th1p * l 
                    + g * cos(th1) + th2p * th2p * l * cos(delta)))
                   / den2;

    coord c = {th1pp, th2pp};
    return c;
}

dp_movement dp_rk4(dp_movement m, double dt) {
    // runge kutta method

    // calculate k and l vals

    double th1, th2, th1p, th2p;
    coord thpp;

    th1 = m.th1, th2 = m.th2;
    th1p = m.th1p, th2p = m.th2p;

    double k1_1 = dt * th1p;
    double k1_2 = dt * th2p;

    thpp = dp_base(th1, th2, th1p, th2p);

    double l1_1 = dt * thpp.x;
    double l1_2 = dt * thpp.y;

    th1 = m.th1 + k1_1 / 2, th2 = m.th2 + k1_2 / 2; 
    th1p = m.th1p + l1_1 / 2, th2p = m.th2p + l1_2 / 2;

    double k2_1 = dt * th1p;
    double k2_2 = dt * th2p;

    thpp = dp_base(th1, th2, th1p, th2p);

    double l2_1 = dt * thpp.x;
    double l2_2 = dt * thpp.y;

    th1 = m.th1 + k2_1 / 2, th2 = m.th2 + k2_2 / 2; 
    th1p = m.th1p + l2_1 / 2, th2p = m.th2p + l2_2 / 2;

    double k3_1 = dt * th1p;
    double k3_2 = dt * th2p;

    thpp = dp_base(th1, th2, th1p, th2p);

    double l3_1 = dt * thpp.x;
    double l3_2 = dt * thpp.y;

    th1 = m.th1 + k3_1, th2 = m.th2 + k3_2; 
    th1p = m.th1p + l3_1, th2p = m.th2p + l3_2;

    double k4_1 = dt * th1p;
    double k4_2 = dt * th2p;

    thpp = dp_base(th1, th2, th1p, th2p);

    double l4_1 = dt * thpp.x;
    double l4_2 = dt * thpp.y;

    // calculate next theta and its derivative

    m.th1 = fmod(m.th1 + (k1_1 + 2 * k2_1 + 2 * k3_1 + k4_1) / 6, 2 * M_PI);
    m.th2 = fmod(m.th2 + (k1_2 + 2 * k2_2 + 2 * k3_2 + k4_2) / 6, 2 * M_PI);

    m.th1p = m.th1p + (l1_1 + 2 * l2_1 + 2 * l3_1 + l4_1) / 6;
    m.th2p = m.th2p + (l1_2 + 2 * l2_2 + 2 * l3_2 + l4_2) / 6;

    return m;
}
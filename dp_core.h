#define _USE_MATH_DEFINES

#include <cmath>

using namespace std;

// default values

double g = 1.00;
double l = 1.00;

//

coord dp_base(double th1, double th2, double th1p, double th2p) {
    // differential equations for second derivatives of theta 1 and theta 2

    double delta = th1 - th2;

    double den = l * (2 - cos(delta) * cos(delta));

    double th1pp = (-g * (2 * sin(th1) + sin(th1 - 2 * th2)) 
                    - 2 * sin(delta) * (th2p * th2p * l + th1p * th1p * l * cos(delta)))
                   / den;

    double th2pp = (2 * sin(delta) * (th1p * th1p * l 
                    + g * cos(th1) + th2p * th2p * l * cos(delta)))
                   / den;

    coord c = {th1pp, th2pp};
    return c;
}

dp_movement dp_rk4(dp_movement m, double dt) {
    // runge kutta method
    // not symplectic

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

    m.th1 = m.th1 + (k1_1 + 2 * k2_1 + 2 * k3_1 + k4_1) / 6;
    m.th2 = m.th2 + (k1_2 + 2 * k2_2 + 2 * k3_2 + k4_2) / 6;

    m.th1p = m.th1p + (l1_1 + 2 * l2_1 + 2 * l3_1 + l4_1) / 6;
    m.th2p = m.th2p + (l1_2 + 2 * l2_2 + 2 * l3_2 + l4_2) / 6;

    return m;
}

dp_movement dp_leapfrog(dp_movement m, double dt) {
    // velocity verlet integrator
    // not symplectic

    coord thpp = dp_base(m.th1, m.th2, m.th1p, m.th2p);

    m.th1p += 0.5 * dt * thpp.x;
    m.th2p += 0.5 * dt * thpp.y;

    m.th1 += dt * m.th1p;
    m.th2 += dt * m.th2p;

    thpp = dp_base(m.th1, m.th2, m.th1p, m.th2p);

    m.th1p += 0.5 * dt * thpp.x;
    m.th2p += 0.5 * dt * thpp.y;

    return m;
}

// vel -> canonical momenta p = M * w, with M = l^2 * [[2, cosΔ], [cosΔ, 1]]
coord vel_to_mom(double th1, double th2, double w1, double w2) {
    double delta = th1 - th2;
    double c = cos(delta);
    double p1 = l*l * (2.0 * w1 + c * w2);
    double p2 = l*l * (c * w1 + 1.0 * w2);
    return {p1, p2};
}

// p -> vel: w = M^{-1} * p ; M^{-1} = (1/(l^2 * D)) * [[1, -c], [-c, 2]] where D = 2 - c^2
coord mom_to_vel(double th1, double th2, double p1, double p2) {
    double delta = th1 - th2;
    double c = cos(delta);
    double D = 2.0 - c*c; // 1 + sin^2(delta)
    double inv = 1.0 / (l*l * D);
    double w1 = inv * ( p1 - c * p2 );
    double w2 = inv * ( -c * p1 + 2.0 * p2 );
    return {w1, w2};
}

// U and analytic gradient dU/dtheta
double potential_U(double th1, double th2) {
    // Convention: U = -g*l*(2*cos th1 + cos th2)
    return - g * l * (2.0 * cos(th1) + cos(th2));
}
coord dUdtheta(double th1, double th2) {
    // ∂U/∂th1 = +2 g l sin(th1); ∂U/∂th2 = + g l sin(th2)
    return { 2.0 * g * l * sin(th1), 1.0 * g * l * sin(th2) };
}

// dT/dθ: compute derivative wrt Delta = th1 - th2, then map {+term, -term}
coord dTdtheta(double th1, double th2, double p1, double p2) {
    double delta = th1 - th2;
    double c = cos(delta), s = sin(delta);
    double D = 2.0 - c*c;            // = 1 + s^2
    double Dp = 2.0 * c * s;         // dD/dDelta
    double inv_l2 = 1.0 / (l*l);

    // B = [[1, -c], [-c, 2]]
    double B11 = 1.0, B12 = -c, B22 = 2.0;
    double pTBp = p1 * (B11 * p1 + B12 * p2) + p2 * (B12 * p1 + B22 * p2);

    // B' = derivative of B wrt delta: B' = [[0, s], [s, 0]]
    double pTBp_prime = 2.0 * s * p1 * p2;

    // d(M^{-1})/dDelta = inv_l2 * ( - Dp / D^2 * B  +  1/D * B' )
    // scalar term = 0.5 * p^T * d(M^{-1})/dDelta * p
    double term = 0.5 * inv_l2 * ( - (Dp / (D * D)) * pTBp + (1.0 / D) * pTBp_prime );

    return { term, -term };
}

// kinetic energy T = 0.5 * w^T * M * w
double kinetic_T_from_w(double th1, double th2, double w1, double w2) {
    double delta = th1 - th2;
    double c = cos(delta);
    return 0.5 * l*l * (2.0 * w1*w1 + 2.0 * c * w1 * w2 + w2*w2);
}
// kinetic energy using p: T = 0.5 * p^T * M^{-1} * p
double kinetic_T_from_p(double th1, double th2, double p1, double p2) {
    double delta = th1 - th2;
    double c = cos(delta);
    double D = 2.0 - c*c;
    double inv_l2D = 1.0 / (l*l * D);
    // M^{-1} = inv_l2D * [[1, -c], [-c, 2]]
    double w1 = inv_l2D * ( p1 - c * p2 );
    double w2 = inv_l2D * ( -c * p1 + 2.0 * p2 );
    // compute T via 0.5 * p^T * M^{-1} * p (equivalently 0.5 * w^T * M * w)
    return 0.5 * (p1 * w1 + p2 * w2);
}

double total_energy(const dp_movement &m) {
    double T = kinetic_T_from_w(m.th1, m.th2, m.th1p, m.th2p);
    double U = potential_U(m.th1, m.th2);
    return T + U;
}

// ---- symplectic single step (canonical variables) ----
dp_movement dp_symplectic(const dp_movement &in, double dt) {
    dp_movement m = in;

    // convert velocities -> momenta
    coord p = vel_to_mom(m.th1, m.th2, m.th1p, m.th2p);

    // half kick: p <- p - (dt/2) * (dT/dtheta + dU/dtheta)
    coord dU = dUdtheta(m.th1, m.th2);
    coord dT = dTdtheta(m.th1, m.th2, p.x, p.y);
    p.x -= 0.5 * dt * (dT.x + dU.x);
    p.y -= 0.5 * dt * (dT.y + dU.y);

    // drift: theta <- theta + dt * M^{-1} p
    coord w = mom_to_vel(m.th1, m.th2, p.x, p.y);
    m.th1 += dt * w.x;
    m.th2 += dt * w.y;

    // second half-kick: recompute gradients at new theta (dT uses same p)
    dU = dUdtheta(m.th1, m.th2);
    dT = dTdtheta(m.th1, m.th2, p.x, p.y);
    p.x -= 0.5 * dt * (dT.x + dU.x);
    p.y -= 0.5 * dt * (dT.y + dU.y);

    // convert back to stored velocities
    coord w2 = mom_to_vel(m.th1, m.th2, p.x, p.y);
    m.th1p = w2.x;
    m.th2p = w2.y;

    return m;
}
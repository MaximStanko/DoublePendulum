#include "dp_visual.h"

using namespace std;

// default values

double th1_0 = - M_PI * (80.0 / 180.0);
double th2_0 = - M_PI * (90.0 / 180.0);
double th1p_0 = - M_PI * (0.0 / 180.0);
double th2p_0 = - M_PI * (0.0 / 180.0);

int debug = 0;

double dt = 0.000001;
double sim_speed = 10.0;

//

void print_pure_state(dp_movement& m, double t, bool debug) {
    string rounded_t = round_to(t, 3);
    string empty_space(rounded_t.length() + 9, ' ');
    cout << "t = " << rounded_t << " :   theta1 = " << round_to(m.th1, 3);
    cout << " (" << round_to((m.th1) / M_PI * 180.0, 1) << "*)" << "\n";
    cout << empty_space << "theta2 = " << round_to(m.th2, 3);
    cout << " (" << round_to((m.th2) / M_PI * 180.0, 1) << "*)";
    if (debug) {
        cout << "\n";
        cout << empty_space << "th1' =   " << round_to(m.th1p, 3) << "\n";
        cout << empty_space << "th2' =   " << round_to(m.th2p, 3) << endl;
    } else {
        cout << endl;
    }
}

void print_dp(double time, double dt, bool debug) {
    dp_movement m = {
        th1_0,
        th2_0,
        th1p_0,
        th2p_0
    };

    double t = 0;

    print_pure_state(m, t, debug);

    int its = (int) (time / dt);

    for (int i = 0; i < its; i++) {
        m = dp_rk4(m, dt);
        t += dt;
        print_pure_state(m, t, debug);
    }
}

int main(int argc, char* argv[]) {

    if (argc >= 3) {
        th1_0 = atof(argv[1]);
        th2_0 = atof(argv[2]);
    }

    if (argc >= 5) {
        th1p_0 = atof(argv[3]);
        th2p_0 = atof(argv[4]);
    }

    if (argc >= 6) {
        g = atof(argv[5]);
    }

    if (argc >= 7) {
        sim_speed = atof(argv[6]);
    }

    if (argc >= 8) {
        debug = atoi(argv[7]);
    }

    DoublePendulum p(dt, {th1_0, th2_0, th1p_0, th2p_0}, sim_speed, 4.5, print_pure_state, debug);

    if (p.Construct(512, 512, 1, 1)) {
        p.Start();
    }

    return 0;
}
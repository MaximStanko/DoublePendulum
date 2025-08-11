#define _USE_MATH_DEFINES

#include "struct_core.h"
#include "sim_core.h"

#include <cmath>

using namespace std;

typedef struct {
    double s;
    double v;
    double a;
} movement;

const double g = 9.81;
const double l = 1;

const double v_0 = 0;
const double theta_0 = - M_PI * (120.0 / 180.0);

double pendulum_base_func(double s) {
    double theta = s / l;
    return - sin(theta) * g / l;
}

movement leapfrog(movement m, function<double (double)> A, double dt) {
    m.s += m.v * dt + m.a / 2 * dt * dt;
    double next_a = A(m.s);
    m.v += (m.a + next_a) / 2 * dt;
    m.a = next_a;
    return m;
}

void print_pendulum_pos(movement& m, double t, bool debug) {
    coord c = calculate_coord_from_angle(m.s / l, l);
    string rounded_t = round_to(t, 3);
    string empty_space(rounded_t.length() + 9, ' ');
    cout << "t = " << rounded_t << " :   x = " << (c.x >= 0 ? " " : "") << round_to(c.x / l, 3) << "\n";
    cout << empty_space << "y = " << (c.y >= 0 ? " " : "") << round_to(c.y, 3);
    if (debug) {
        cout << "\n";
        cout << empty_space << "v = " << round_to(m.v, 3) << "\n";
        cout << empty_space << "a = " << round_to(m.a, 3) << endl;
    } else {
        cout << endl;
    }
}

void print_pure_state(movement& m, double t, bool debug) {
    string rounded_t = round_to(t, 3);
    cout << "t = " << rounded_t << " :   theta = " << round_to(m.s / l, 3);
    cout << " (" << round_to((m.s / l) / M_PI * 180.0, 1) << "*)";
    if (debug) {
        string empty_space(rounded_t.length() + 9, ' ');
        cout << "\n";
        cout << empty_space << "v =     " << round_to(m.v, 3) << "\n";
        cout << empty_space << "a =     " << round_to(m.a, 3) << endl;
    } else {
        cout << endl;
    }
}

void approximate_pendulum(int its, double dt, function<void (movement&, double, bool)> output_func, bool debug) {
    double s_0 = theta_0 * l;

    movement m = {
        s_0,
        v_0,
        pendulum_base_func(s_0)
    };

    double t = 0;

    output_func(m, t, debug);

    for (int i = 0; i < its; i++) {
        m = leapfrog(m, pendulum_base_func, dt);
        t += dt;
        output_func(m, t, debug);
    }
}

class Pendulum : public SimCore {
    private:

        function<void (movement&, double, bool)> output_func;
        pixel_pos prev_frop = {-1, -1};

    public:

        const string central_img_path = "assets/frops.png";
        olc::Sprite central_img;

        pixel_pos origin;

        int debug;

        movement m;
        double dt;
        double t;
        double real_t;
        
        Pendulum(double _dt, double _viewport_width, int _debug = 0) {
            sAppName = "Pendulum Simulation";

            dt = _dt;

            viewport_width = _viewport_width;
            debug = _debug;

            output_func = debug > 1 ? print_pure_state : print_pendulum_pos;
        }

        bool OnUserCreate() override {
            double s_0 = theta_0 * l;

            m = {
                s_0,
                v_0,
                pendulum_base_func(s_0)
            };

            t = 0;
            real_t = 0;

            origin = coord_to_pos(coord {0, 0});

            olc::Sprite img(central_img_path);
            central_img = img;

            DrawSprite(origin.x - central_img.width / 2, origin.y - central_img.height, &central_img);

            if (debug > 0)
                output_func(m, t, debug > 2);

            return true;
        }

        bool OnUserUpdate(float fElapsedTime) override {
            real_t += fElapsedTime;
            while (t < real_t) {
                m = leapfrog(m, pendulum_base_func, dt);
                t += dt;
            }

            pixel_pos frop = coord_to_pos(calculate_coord_from_angle(m.s / l, l));

            if (prev_frop.x >= 0) 
                DrawLine(prev_frop.x, prev_frop.y, origin.x, origin.y, olc::Pixel(0, 0, 0));
			DrawLine(frop.x, frop.y, origin.x, origin.y, olc::Pixel(70, 211, 63));
            Draw(origin.x, origin.y, olc::Pixel(255, 255, 255));

            prev_frop = frop;

            if (debug > 0)
                output_func(m, t, debug > 2);

            return true;
        }
};

int main() {
    Pendulum p(0.001, 2.25, 0);

    if (p.Construct(512, 512, 1, 1)) {
        p.Start();
    }

    return 0;
}
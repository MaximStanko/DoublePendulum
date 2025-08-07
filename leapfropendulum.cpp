#define OLC_PGE_APPLICATION
#include "olcPixelGameEngine.h"

#include <iostream>
#include <iomanip>
#include <sstream>
#include <functional>
#include <cmath>

using namespace std;

# define PI           3.14159265358979323846

typedef struct {
    double s;
    double v;
    double a;
} movement;

typedef struct {
    double x;
    double y;
} coord;

typedef struct {
    int x;
    int y;
} pixel_pos;

const double g = 9.81;
const double l = 1;

const double v_0 = 0;
const double theta_0 = - PI * (80.0 / 180.0);

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

string round_to(double val, int prec) {
    ostringstream string_stream;
    string_stream << fixed << setprecision(prec) << val;
    return string_stream.str();
}

coord calculate_coord_from_angle(double theta) {
    coord c = {sin(theta) * l, cos(theta) * l};
    return c;
}

void print_pendulum_pos(movement& m, double t, bool debug) {
    coord c = calculate_coord_from_angle(m.s / l);
    cout << "t = " << round_to(t, 3) << " :   x = " << (c.x >= 0 ? " " : "") << round_to(c.x / l, 3) << "\n";
    cout << "              " << "y = " << (c.y >= 0 ? " " : "") << round_to(c.y, 3);
    if (debug) {
        cout << "\n";
        cout << "              " << "v = " << round_to(m.v, 3) << "\n";
        cout << "              " << "a = " << round_to(m.a, 3) << endl;
    } else {
        cout << endl;
    }
}

void print_pure_state(movement& m, double t, bool debug) {
    cout << "t = " << round_to(t, 3) << " :   theta = " << round_to(m.s / l, 3);
    cout << " (" << round_to((m.s / l) / PI * 180.0, 1) << "*)";
    if (debug) {
        cout << "\n";
        cout << "              " << "v =     " << round_to(m.v, 3) << "\n";
        cout << "              " << "a =     " << round_to(m.a, 3) << endl;
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

class Pendulum : public olc::PixelGameEngine {
    private:

        function<void (movement&, double, bool)> output_func;
        pixel_pos prev_frop = {-1, -1};

    public:

        const string central_img_path = "frops.png";
        olc::Sprite central_img;

        int debug;
        double viewport_size;

        movement m;
        double dt;
        double t;
        
        Pendulum(double _dt, double _viewport_size, function<void (movement&, double, bool)> _output_func, int _debug = 0) {
            sAppName = "Pendulum Simulation";

            dt = _dt;

            viewport_size = _viewport_size;
            output_func = _output_func;
            debug = _debug;
        }

        pixel_pos coord_to_pixel_pos(coord c) {
            pixel_pos p = {(int) ((c.x + viewport_size) * ScreenWidth() / (viewport_size * 2)), 
                (int) ((c.y + viewport_size) * ScreenWidth() / (viewport_size * 2))};
            return p;
        }

        bool OnUserCreate() override {
            double s_0 = theta_0 * l;

            m = {
                s_0,
                v_0,
                pendulum_base_func(s_0)
            };

            t = 0;

            olc::Sprite img(central_img_path);
            central_img = img;

            if (debug > 0)
                output_func(m, t, debug > 1);

            return true;
        }

        bool OnUserUpdate(float fElapsedTime) override {
            m = leapfrog(m, pendulum_base_func, dt);
            t += dt;

            pixel_pos origin = coord_to_pixel_pos(coord {0, 0});
            pixel_pos frop = coord_to_pixel_pos(calculate_coord_from_angle(m.s / l));

            DrawSprite(origin.x - central_img.width / 2, origin.y - central_img.height, &central_img);

            if (prev_frop.x >= 0) 
                DrawLine(prev_frop.x, prev_frop.y, origin.x, origin.y, olc::Pixel(0, 0, 0));
			DrawLine(frop.x, frop.y, origin.x, origin.y, olc::Pixel(70, 211, 63));
            Draw(origin.x, origin.y, olc::Pixel(255, 255, 255));

            prev_frop = frop;

            if (debug > 0)
                output_func(m, t, debug > 1);

            return true;
        }
};

int main() {
    Pendulum p(0.001, 2.25, print_pendulum_pos, false);

    if (p.Construct(512, 512, 1, 1)) {
        p.Start();
    }

    return 0;
}
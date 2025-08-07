#define OLC_PGE_APPLICATION
#include "olcPixelGameEngine.h"

#include <iostream>
#include <iomanip>
#include <sstream>
#include <functional>
#include <cmath>
#include <map>

using namespace std;

# define PI           3.14159265358979323846

const double g = 4.00;
const double l = 1.00;

const double th1_0 = - PI * (80.0 / 180.0);
const double th2_0 = - PI * (90.0 / 180.0);
const double th1p_0 = - PI * (0.0 / 180.0);
const double th2p_0 = - PI * (0.0 / 180.0);

typedef struct {
    double th1;
    double th2;
    double th1p;
    double th2p;
} movement;

typedef struct {
    double x;
    double y;
} coord;

typedef struct {
    int x;
    int y;
} pixel_pos;

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

movement dp_rk4(movement m, double dt) {
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

    m.th1 = m.th1 + (k1_1 + 2 * k2_1 + 2 * k3_1 + k4_1) / 6, 2 * PI;
    m.th2 = m.th2 + (k1_2 + 2 * k2_2 + 2 * k3_2 + k4_2) / 6, 2 * PI;

    m.th1p = m.th1p + (l1_1 + 2 * l2_1 + 2 * l3_1 + l4_1) / 6, 2 * PI;
    m.th2p = m.th2p + (l1_2 + 2 * l2_2 + 2 * l3_2 + l4_2) / 6, 2 * PI;

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

void print_pure_state(movement& m, double t, bool debug) {
    string rounded_t = round_to(t, 3);
    string empty_space(rounded_t.length() + 9, ' ');
    cout << "t = " << rounded_t << " :   theta1 = " << round_to(m.th1, 3);
    cout << " (" << round_to((m.th1) / PI * 180.0, 1) << "*)" << "\n";
    cout << empty_space << "theta2 = " << round_to(m.th2, 3);
    cout << " (" << round_to((m.th2) / PI * 180.0, 1) << "*)";
    if (debug) {
        cout << "\n";
        cout << empty_space << "th1' =   " << round_to(m.th1p, 3) << "\n";
        cout << empty_space << "th2' =   " << round_to(m.th2p, 3) << endl;
    } else {
        cout << endl;
    }
}

void print_dp(double time, double dt, bool debug) {
    movement m = {
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

float hue_to_rgb(float p, float q, float t) {
    if (t < 0.0) t += 1.0;
    if (t > 1.0) t -= 1.0;
    if (t < 1.0 / 6.0) return p + (q - p) * 6.0 * t;
    if (t < 1.0 / 2.0) return q;
    if (t < 2.0 / 3.0) return p + (q - p) * (2.0 / 3.0 - t) * 6.0;
    return p;
}

olc::Pixel hsl_to_pixel(float h, float s, float l) {
    uint8_t r, g, b;

    if (s == 0) {
        r = g = b = (uint8_t) (l * 255.0);
    } else {
        float q = l < 0.5 ? l * (1.0 + s) : l + s - l * s;
        float p = 2.0 * l - q;
        r = (uint8_t) (255.0 * hue_to_rgb(p, q, h + 1.0 / 3.0));
        g = (uint8_t) (255.0 * hue_to_rgb(p, q, h));
        b = (uint8_t) (255.0 * hue_to_rgb(p, q, h - 1.0 / 3.0));
    }
    
    return olc::Pixel(r, g, b);
}

class DoublePendulum : public olc::PixelGameEngine {
    private:

        pixel_pos prev_pos1 = {-1, -1}, prev_pos2 = {-1, -1};
        map<vector<int>, olc::Pixel> prev_pixels;

    public:

        const string central_img_path = "frops.png";
        olc::Sprite central_img;

        pixel_pos origin;

        int debug;
        double viewport_size;

        movement m;
        double dt;
        double t;
        double real_t;
        
        DoublePendulum(double _dt, double _viewport_size, int _debug = 0) {
            sAppName = "Double Pendulum";

            dt = _dt;

            viewport_size = _viewport_size;
            debug = _debug;
        }

        pixel_pos coord_to_pixel_pos(coord c) {
            pixel_pos p = {(int) ((c.x + viewport_size) * ScreenWidth() / (viewport_size * 2)), 
                (int) ((c.y + viewport_size) * ScreenWidth() / (viewport_size * 2))};
            return p;
        }

        bool OnUserCreate() override {
            m = {
                th1_0,
                th2_0,
                th1p_0,
                th2p_0
            };

            t = 0;
            real_t = 0;

            origin = coord_to_pixel_pos(coord {0, 0});

            olc::Sprite img(central_img_path);
            central_img = img;

            DrawSprite(origin.x - central_img.width / 2, origin.y - central_img.height, &central_img);

            if (debug > 0)
                print_pure_state(m, t, debug > 1);

            return true;
        }

        bool OnUserUpdate(float fElapsedTime) override {

            real_t += fElapsedTime;
            while (t < real_t) {
                m = dp_rk4(m, dt);
                t += dt;
            }

            coord c1 = calculate_coord_from_angle(m.th1);
            pixel_pos pos1 = coord_to_pixel_pos(c1);

            coord c2 = calculate_coord_from_angle(m.th2);
            c2.x += c1.x; c2.y += c1.y;
            pixel_pos pos2 = coord_to_pixel_pos(c2);

            olc::Pixel new_pixel = hsl_to_pixel(fmod(t, 1.0), 1.0, 0.5);
            vector<int> pos2_vec = {pos2.x, pos2.y};
            prev_pixels[pos2_vec] = new_pixel;

            for (auto p : prev_pixels)
                Draw(p.first[0], p.first[1], p.second);

            DrawLine(prev_pos1.x, prev_pos1.y, origin.x, origin.y, olc::Pixel(0, 0, 0));
            DrawLine(prev_pos2.x, prev_pos2.y, prev_pos1.x, prev_pos1.y, olc::Pixel(0, 0, 0));

			DrawLine(pos1.x, pos1.y, origin.x, origin.y, olc::Pixel(70, 211, 63));
            DrawLine(pos2.x, pos2.y, pos1.x, pos1.y, olc::Pixel(70, 211, 63));

            Draw(origin.x, origin.y, olc::Pixel(255, 255, 255));
            Draw(pos1.x, pos1.y, olc::Pixel(255, 255, 255));

            prev_pos1 = pos1;
            prev_pos2 = pos2;

            if (debug > 0)
                print_pure_state(m, t, debug > 1);

            return true;
        }
};

int main() {
    DoublePendulum p(0.001, 2.25, 0);

    if (p.Construct(512, 512, 1, 1)) {
        p.Start();
    }

    return 0;
}
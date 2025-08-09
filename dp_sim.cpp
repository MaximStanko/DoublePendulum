#include "struct_core.h"
#include "dp_core.h"
#include "sim_core.h"

using namespace std;

// default values

double th1_0 = - M_PI * (80.0 / 180.0);
double th2_0 = - M_PI * (90.0 / 180.0);
double th1p_0 = - M_PI * (0.0 / 180.0);
double th2p_0 = - M_PI * (0.0 / 180.0);

int debug = 0;

double dt = 0.001;
double sim_speed = 0.8;

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

class DoublePendulum : public PendulumCore {
    private:

        pixel_pos prev_pos1 = {-1, -1}, prev_pos2 = {-1, -1};
        map<vector<int>, olc::Pixel> prev_pixels;
        double accumulator = 0.0;

    public:

        const string central_img_path = "frops.png";
        olc::Sprite central_img;

        pixel_pos origin;

        int debug;

        dp_movement m;
        double dt;
        double real_dt;
        double t;
        double sim_speed;
        
        DoublePendulum(double _dt, double _sim_speed, double _viewport_size, int _debug = 0) {
            sAppName = "Double Pendulum";

            dt = _dt;
            sim_speed = _sim_speed;

            viewport_size = _viewport_size;
            debug = _debug;
        }

        bool OnUserCreate() override {
            m = {
                th1_0,
                th2_0,
                th1p_0,
                th2p_0
            };

            t = 0;

            real_dt = dt / sim_speed;

            origin = coord_to_pixel_pos(coord {0, 0});

            olc::Sprite img(central_img_path);
            central_img = img;

            DrawSprite(origin.x - central_img.width / 2, origin.y - central_img.height, &central_img);

            if (debug > 0)
                print_pure_state(m, t, debug > 1);

            return true;
        }

        bool OnUserUpdate(float fElapsedTime) override {

            t += fElapsedTime;

            accumulator += fElapsedTime;

            if (accumulator > 20 * real_dt)
                accumulator = fmod(accumulator, 20 * real_dt);

            while (accumulator >= real_dt) {
                m = dp_rk4(m, dt);
                accumulator -= real_dt;
            }

            coord c1 = calculate_coord_from_angle(m.th1, l);
            pixel_pos pos1 = coord_to_pixel_pos(c1);

            coord c2 = calculate_coord_from_angle(m.th2, l);
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

int main(int argc, char* argv[]) {

    if (argc >= 3) {
        th1_0 = - M_PI * (atof(argv[1]) / 180);
        th2_0 = - M_PI * (atof(argv[2]) / 180);
    }

    if (argc >= 5) {
        th1p_0 = M_PI * (atof(argv[3]) / 180);
        th2p_0 = M_PI * (atof(argv[4]) / 180);
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

    DoublePendulum p(dt, sim_speed, 2.25, debug);

    if (p.Construct(512, 512, 1, 1)) {
        p.Start();
    }

    return 0;
}
#include "struct_core.h"
#include "dp_core.h"
#include "sim_core.h"

using namespace std;

class DPFractal : public SimCore {
    public:

        double T;
        double dt;
        double lightness_factor;

        DPFractal(double _T, double _dt, double _lightness_factor) {
            sAppName = "Pendulum Fractal";

            T = _T;
            dt = _dt;
            lightness_factor = _lightness_factor;
        }

        vector<vector<coord>> get_pendulum_positions(double T, double dt) {
            vector<vector<coord>> pp(ScreenWidth() + 2, vector<coord>(ScreenHeight() + 2));
            for (int x = -1; x <= ScreenWidth(); x++) {
                for (int y = -1; y <= ScreenHeight(); y++) {
                    dp_movement m = {
                        ((double) x / (double) ScreenWidth() - 0.5) * 2 * M_PI,
                        ((double) y / (double) ScreenHeight() - 0.5) * 2 * M_PI,
                        0,
                        0
                    };
                    for (int i = 0; i < T / dt; i++)
                        m = dp_symplectic(m, dt);
                    
                    coord c1 = calculate_coord_from_angle(m.th1, l);
                    coord c2 = calculate_coord_from_angle(m.th2, l);
                    c2.x += c1.x; c2.y += c1.y;
                    pp[x+1][y+1] = c2;
                }
                cout << "Calculated row " << x << "." << endl;
            }
            return pp;
        }

        vector<vector<double>> avg_pendulum_dist(vector<vector<coord>> pendulum_positions) {
            vector<vector<double>> pp(ScreenWidth() + 2, vector<double>(ScreenHeight() + 2));
            for (int x = 0; x < ScreenWidth(); x++)
                for (int y = 0; y < ScreenHeight(); y++) {
                    double sum = 0;
                    for (int xk = -1; xk <= 1; xk++)
                        for (int yk = -1; yk <= 1; yk++) {
                            double xdiff = pendulum_positions[x + 1 + xk][y + 1 + yk].x - pendulum_positions[x + 1][y + 1].x;
                            double ydiff = pendulum_positions[x + 1 + xk][y + 1 + yk].y - pendulum_positions[x + 1][y + 1].y;
                            sum += xdiff * xdiff + ydiff * ydiff;
                        }
                    pp[x][y] = sum * sum * sum;
                }
            return pp;
        }

        bool OnUserCreate() override {
            vector<vector<double>> avg_dist = avg_pendulum_dist(get_pendulum_positions(T, dt));
            cout << "Calculated pendulum distances." << endl;
            for (int x = 0; x < ScreenWidth(); x++)
                for (int y = 0; y < ScreenHeight(); y++) {
                    double lightness = avg_dist[x][y] * lightness_factor;
                    lightness = lightness > 1 ? 1 : lightness;
                    Draw(x, y, hsl_to_pixel(0, 0, lightness));
                }
            return true;
        }

        bool OnUserUpdate(float fElapsedTime) override {
            return true;
        }
};

int main(int argc, char* argv[]) {
    double T = 0.5, dt = 0.0001, lightness_factor = 10000;
    int x, y;

    if (argc >= 3) {
        T = atof(argv[1]);
        dt = atof(argv[2]);
    }

    if (argc >= 5) {
        x = atoi(argv[3]);
        y = atoi(argv[4]);
    }

    if (argc >= 6) {
        lightness_factor = atof(argv[5]);
    }

    DPFractal f(T, dt, lightness_factor);

    if (f.Construct(x, y, 512 / x, 512 / y)) {
        f.Start();
    }

    return 1;
}
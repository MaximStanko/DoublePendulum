#define OLC_PGE_APPLICATION
#include "olcPixelGameEngine.h"

using namespace std;

string round_to(double val, int prec) {
    ostringstream string_stream;
    string_stream << fixed << setprecision(prec) << val;
    return string_stream.str();
}

coord calculate_coord_from_angle(double theta, double l) {
    coord c = {sin(theta) * l, cos(theta) * l};
    return c;
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

class PendulumCore : public olc::PixelGameEngine {
    public:

        double viewport_size;

        pixel_pos coord_to_pixel_pos(coord c) {
            pixel_pos p = {(int) ((c.x + viewport_size) * ScreenWidth() / (viewport_size * 2)), 
                (int) ((c.y + viewport_size) * ScreenWidth() / (viewport_size * 2))};
            return p;
        }

        bool OnUserCreate() override {return true;};

        bool OnUserUpdate(float fElapsedTime) override {return true;};
};
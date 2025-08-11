#define OLC_PGE_APPLICATION
#include "lib/olcPixelGameEngine.h"

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

pixel_pos coord_to_viewport_pos(coord pos, coord center, double viewport_width, int width) {
    double viewport_ratio = (double) width / viewport_width;
    coord new_pos = {pos.x - center.x, pos.y - center.y};
    pixel_pos pp = {(int) ((new_pos.x + viewport_width / 2) * viewport_ratio), 
        (int) ((new_pos.y + viewport_width / 2) * viewport_ratio)};
    return pp;
}

coord viewport_pos_to_coord(pixel_pos pos, pixel_pos viewport_center, double viewport_width, int width) {
    double coord_ratio = viewport_width / (double) width;
    coord c = {(double) (pos.x - viewport_center.x) * coord_ratio,
        (double) (pos.y - viewport_center.y) * coord_ratio};
    return c;
}

vector<pixel_pos> get_line(pixel_pos pp1, pixel_pos pp2) {
    vector<pixel_pos> pixels;
    int x, y, t, dx, dy, incx, incy, pdx, pdy, ddx, ddy, deltaslowdirection, deltafastdirection, err;

    dx = pp2.x - pp1.x;
    dy = pp2.y - pp1.y;

    incx = dx < 0 ? -1 : 1;
    incy = dy < 0 ? -1 : 1;
    if (dx < 0) dx = -dx;
    if (dy < 0) dy = -dy;

    if (dx > dy) {
        pdx = incx; pdy = 0;
        ddx = incx; ddy = incy;
        deltaslowdirection = dy;   deltafastdirection = dx;
    } else {
        pdx = 0;    pdy = incy;
        ddx = incx; ddy = incy;
        deltaslowdirection = dx;   deltafastdirection = dy;
    }

    x = pp1.x;
    y = pp1.y;
    err = deltafastdirection / 2;
    pixels.emplace_back(pixel_pos {x, y});

    for (t = 0; t < deltafastdirection; ++t) {
        err -= deltaslowdirection;
        if (err < 0) {
            err += deltafastdirection;
            x += ddx;
            y += ddy;
        } else {
            x += pdx;
            y += pdy;
        }
        pixels.emplace_back(pixel_pos {x, y});
    }

    return pixels;
}

class SimCore : public olc::PixelGameEngine {
    public:

        coord center = {0, 0};
        pixel_pos viewport_center = {ScreenWidth() / 2, ScreenHeight() / 2};
        double viewport_width;

        pixel_pos coord_to_pos(coord c) {
            return coord_to_viewport_pos(c, center, viewport_width, ScreenWidth());
        }

        coord pos_to_coord(pixel_pos pp) {
            return viewport_pos_to_coord(pp, viewport_center, viewport_width, ScreenWidth());
        }
};
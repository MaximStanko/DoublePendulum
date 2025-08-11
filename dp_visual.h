#include "struct_core.h"
#include "dp_core.h"
#include "sim_core.h"

using namespace std;

class DPBase : public SimCore {
    public:

        pixel_pos origin;
        dp_movement m;
        double dt;
        double real_dt;
        double t;
        double sim_speed;
};

class DoublePendulum : public SimCore {
    private:

        pixel_pos prev_pos1 = {-1, -1}, prev_pos2 = {-1, -1};
        double accumulator = 0.0;

        olc::Sprite path;

    public:

        const string central_img_path = "assets/frops.png";
        olc::Sprite central_img;

        pixel_pos origin;

        int debug;
        function<void (dp_movement&, double, bool)> output_func;

        dp_movement m;
        double dt;
        double real_dt;
        double t;
        double sim_speed;
        
        DoublePendulum(double _dt, dp_movement m_0, double _sim_speed, double _viewport_width, function<void (dp_movement&, double, bool)> _output_func, int _debug = 0) {
            sAppName = "Double Pendulum";

            sim_speed = _sim_speed;
            viewport_width = _viewport_width;
            debug = _debug;
            output_func = _output_func;

            dt = _dt;

            m = m_0;
        }

        bool OnUserCreate() override {
            path = olc::Sprite(ScreenWidth(), ScreenHeight());

            t = 0;

            real_dt = dt / sim_speed;

            origin = coord_to_pos(coord {0, 0});

            olc::Sprite img(central_img_path);
            central_img = img;

            SetDrawTarget(&path);
            DrawSprite(origin.x - central_img.width / 2, origin.y - central_img.height, &central_img);
            SetDrawTarget(nullptr);

            if (debug > 0)
                output_func(m, t, debug > 1);

            return true;
        }

        bool OnUserUpdate(float fElapsedTime) override {

            t += fElapsedTime;

            accumulator += fElapsedTime;

            if (accumulator > 100000 * real_dt)
                accumulator = fmod(accumulator, 100000 * real_dt);

            while (accumulator >= real_dt) {
                m = dp_symplectic(m, dt);
                accumulator -= real_dt;
            }

            DrawLine(prev_pos1.x, prev_pos1.y, origin.x, origin.y, olc::Pixel(0, 0, 0));
            DrawLine(prev_pos2.x, prev_pos2.y, prev_pos1.x, prev_pos1.y, olc::Pixel(0, 0, 0));

            coord c1 = calculate_coord_from_angle(m.th1, l);
            pixel_pos pos1 = coord_to_pos(c1);

            coord c2 = calculate_coord_from_angle(m.th2, l);
            c2.x += c1.x; c2.y += c1.y;
            pixel_pos pos2 = coord_to_pos(c2);

            olc::Pixel new_pixel = hsl_to_pixel(fmod(t, 1.0), 1.0, 0.5);
            
            if (prev_pos2.x >= 0) {
                SetDrawTarget(&path);
                DrawLine(prev_pos2.x, prev_pos2.y, pos2.x, pos2.y, new_pixel);
                SetDrawTarget(nullptr);
                DrawSprite(0, 0, &path);
            }

			DrawLine(pos1.x, pos1.y, origin.x, origin.y, olc::Pixel(70, 211, 63));
            DrawLine(pos2.x, pos2.y, pos1.x, pos1.y, olc::Pixel(70, 211, 63));

            Draw(origin.x, origin.y, olc::Pixel(255, 255, 255));
            Draw(pos1.x, pos1.y, olc::Pixel(255, 255, 255));

            prev_pos1 = pos1;
            prev_pos2 = pos2;

            if (debug > 0)
                output_func(m, t, debug > 1);

            return true;
        }
};

class DPAngleSpace : public SimCore {
    private:

        pixel_pos prev_pos1 = {-1, -1}, prev_pos2 = {-1, -1};
        double accumulator = 0.0;

        olc::Sprite path;

    public:

        const string central_img_path = "assets/frops.png";
        olc::Sprite central_img;

        pixel_pos origin;

        int debug;
        function<void (dp_movement&, double, bool)> output_func;

        dp_movement m;
        double dt;
        double real_dt;
        double t;
        double sim_speed;
        
        DPAngleSpace(double _dt, dp_movement m_0, double _sim_speed, double _viewport_width, function<void (dp_movement&, double, bool)> _output_func, int _debug = 0) {
            sAppName = "Double Pendulum";

            sim_speed = _sim_speed;
            viewport_width = _viewport_width;
            debug = _debug;
            output_func = _output_func;

            dt = _dt;

            m = m_0;
        }

        bool OnUserCreate() override {
            path = olc::Sprite(ScreenWidth(), ScreenHeight());

            t = 0;

            real_dt = dt / sim_speed;

            origin = coord_to_pos(coord {0, 0});

            olc::Sprite img(central_img_path);
            central_img = img;

            SetDrawTarget(&path);
            DrawSprite(origin.x - central_img.width / 2, origin.y - central_img.height, &central_img);
            SetDrawTarget(nullptr);

            if (debug > 0)
                output_func(m, t, debug > 1);

            return true;
        }

        bool OnUserUpdate(float fElapsedTime) override {

            t += fElapsedTime;

            accumulator += fElapsedTime;

            if (accumulator > 100000 * real_dt)
                accumulator = fmod(accumulator, 100000 * real_dt);

            while (accumulator >= real_dt) {
                m = dp_symplectic(m, dt);
                accumulator -= real_dt;
            }

            DrawLine(prev_pos1.x, prev_pos1.y, origin.x, origin.y, olc::Pixel(0, 0, 0));
            DrawLine(prev_pos2.x, prev_pos2.y, prev_pos1.x, prev_pos1.y, olc::Pixel(0, 0, 0));

            coord c1 = calculate_coord_from_angle(m.th1, l);
            pixel_pos pos1 = coord_to_pos(c1);

            coord c2 = calculate_coord_from_angle(m.th2, l);
            c2.x += c1.x; c2.y += c1.y;
            pixel_pos pos2 = coord_to_pos(c2);

            olc::Pixel new_pixel = hsl_to_pixel(fmod(t, 1.0), 1.0, 0.5);
            
            if (prev_pos2.x >= 0) {
                SetDrawTarget(&path);
                DrawLine(prev_pos2.x, prev_pos2.y, pos2.x, pos2.y, new_pixel);
                SetDrawTarget(nullptr);
                DrawSprite(0, 0, &path);
            }

			DrawLine(pos1.x, pos1.y, origin.x, origin.y, olc::Pixel(70, 211, 63));
            DrawLine(pos2.x, pos2.y, pos1.x, pos1.y, olc::Pixel(70, 211, 63));

            Draw(origin.x, origin.y, olc::Pixel(255, 255, 255));
            Draw(pos1.x, pos1.y, olc::Pixel(255, 255, 255));

            prev_pos1 = pos1;
            prev_pos2 = pos2;

            if (debug > 0)
                output_func(m, t, debug > 1);

            return true;
        }
};
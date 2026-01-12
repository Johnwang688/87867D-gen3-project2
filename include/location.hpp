#pragma once

#include "bot.hpp"

// Forward declaration to break circular dependency
class Drivetrain;

class Location {
    public:
        Location(Drivetrain& drivetrain, 
            vex::distance& left_distance_forward, 
            vex::distance& right_distance_forward, 
            vex::distance& left_distance_aft, 
            vex::distance& right_distance_aft, 
            vex::distance& front_distance_left, 
            vex::distance& front_distance_right, 
            vex::distance& back_distance_left, 
            vex::distance& back_distance_right,
            bot::sensors::inertial_group& imu
        );
        void update();
        void reset(std::int16_t x, std::int16_t y, std::int16_t heading);
        void start();
        void stop();
        std::int16_t get_x();
        std::int16_t get_y();
        std::int16_t get_heading();
        Particle estimate();
        void search(Particle guess, Reading readings);
    private:
        Reading capture_readings();
        Drivetrain& _drivetrain;
        vex::distance& _left_distance_forward;
        vex::distance& _right_distance_forward;
        vex::distance& _left_distance_aft;
        vex::distance& _right_distance_aft;
        vex::distance& _front_distance_left;
        vex::distance& _front_distance_right;
        vex::distance& _back_distance_left;
        vex::distance& _back_distance_right;
        bot::sensors::inertial_group& _imu;
        std::int16_t _x;
        std::int16_t _y;
        std::int16_t _heading;
        std::vector<Particle> _particles;
        Particle _bestParticle;
        Reading _readings;
        double _last_left_encoder;
        double _last_right_encoder;
        vex::task _updateTask;
        bool _taskRunning;
        vex::mutex _positionMutex;  // Protects _x, _y, _heading from concurrent access
        void raycast(Particle& particle);
        float calculate_intersection(float s_x, float s_y, float angle_deg, const Line& wall);
        bool is_particle_valid(const Particle& particle);
        static int _updateTaskWrapper(void* instance);
};

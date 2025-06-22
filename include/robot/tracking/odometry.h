#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "pros/imu.hpp"
#include "tracking_wheel.h"
#include "utils/pose.h"
#include <cstdint>
#include <vector>

class Odometry {
    public:
            
        void configure(
            std::vector<pros::IMU*> imus, 
            std::vector<TrackingWheel*> v_wheels,
            std::vector<TrackingWheel*> h_wheels,
            double p_x,
            double p_y,
            double p_theta,
            double r_translation,
            double r_heading,
            double q
        );

        void update(Pose& pose, uint32_t delay);

    private:
        std::vector<pros::IMU*> imus;
        std::vector<TrackingWheel*> v_wheels;
        std::vector<TrackingWheel*> h_wheels;

        double p_x;
        double p_y;
        double p_theta;
        double r_translation;
        double r_heading;
        double q;
};

#endif
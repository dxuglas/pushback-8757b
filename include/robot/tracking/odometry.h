#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "pros/imu.hpp"
#include "tracking_wheel.h"
#include <vector>

class Odometry {
    public:

    private:

    void update();

    std::vector<pros::IMU> imus;
    std::vector<TrackingWheel*> v_wheels;
    std::vector<TrackingWheel*> h_wheels;
};

#endif
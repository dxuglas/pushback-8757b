/**
 * @file odometry.h
 * 
 * Contains declerations for the Odometry class.
 * 
 * @copyright (c) 2025, Noah Douglas.
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "pros/imu.hpp"
#include "tracking_wheel.h"
#include "utils/pose.h"
#include <cstdint>
#include <vector>

/**
 * @brief Odometry Class
 * 
 * This class is used to track the position of a robot, using tracking wheels 
 * and IMUs
 */

class Odometry {
    public:
        /**
         * @brief Configure the sensors and filter values used for tracking
         * 
         * @param imus vector of IMU pointers
         * @param v_wheels vector of vertical tracking wheels
         * @param h_wheels vector of horizontal tracking wheels
         * @param p_theta uncertainty of heading data
         * @param r_heading expected noise of heading data
         * @param q expected proccess nosie
         */
        void configure(std::vector<pros::IMU*> imus, std::vector<TrackingWheel*> v_wheels, std::vector<TrackingWheel*> h_wheels,
            double p_theta, double r_heading, double q);
            
        /**
         * @brief Update the estimated pose of the robot
         * 
         * This function should be run in a dedicated task to ensure tracking 
         * remains up to date
         *
         * @param pose pose to update
         * @param delay update loop delay
         */
        void update(Pose& pose, uint32_t delay);

    private:
        std::vector<pros::IMU*> imus;
        std::vector<TrackingWheel*> v_wheels;
        std::vector<TrackingWheel*> h_wheels;

        double p_theta;
        double r_heading;
        double q;
};

#endif
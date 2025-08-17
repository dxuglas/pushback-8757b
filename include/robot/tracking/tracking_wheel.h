/**
 * @file tracking_wheel.h
 * 
 * Contains declerations for the Odometry Tracking Wheel.
 * 
 * @copyright (c) 2025, Noah Douglas.
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef TRACKING_WHEEL_H
#define TRACKING_WHEEL_H

#include "pros/adi.hpp"
#include "pros/rotation.hpp"

/**
 * @brief Tracking Wheel Class
 * 
 * Represents a tracking wheel on the robot
 */
class TrackingWheel {
    public:
        enum class SensorType {
            Rotation,
            Encoder
        };

        /**
         * @brief Constructs a tracking wheel
         *
         * @param rotation tracking wheels sensor
         * @param diameter tracking wheel diameter
         * @param offset offset of tracking wheel from tracking center
         */
        TrackingWheel(pros::Rotation* rotation, float diameter, double offset);

        /**
         * @brief Constructs a tracking wheel
         *
         * @param encoder tracking wheels sensor
         * @param diameter tracking wheel diameter
         * @param offset offset of tracking wheel from tracking center
         */
        TrackingWheel(pros::adi::Encoder* encoder, float diameter, double offset);
        
        /**
         * @brief get tracking wheel distance since last call
         * 
         * calculates the distance travelled by the tracking wheel since the method was
         * last called.
         * 
         * @return double distance delta (in inches)
         */
        double get_distance_delta();

        /**
         * @brief get total tracking wheel distance
         * 
         * retrieves the total distance travelled by the tracking wheel since the program
         * started. 
         * 
         * @return double total distance travelled (in inches)
         */   
        double get_distance_total();

        /** 
         * @brief get the tracking wheel offset
         * 
         * @return double offset (in inches)
         */
        double get_offset();

        /**
         * @brief reset tracking wheel data
         */
        void tare();
        
    private:
        pros::Rotation* rotation;
        pros::adi::Encoder* encoder;
        float zero_position;
        float diameter;
        float offset;
        float last_total;
        SensorType sensor_type;
};

#endif // TRACKING_WHEEL_H
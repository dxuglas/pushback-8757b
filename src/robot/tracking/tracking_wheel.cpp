/**
 * @file tracking_wheel.cpp
 * 
 * Contains implementations for the Odometry Tracking Wheel.
 * 
 * @copyright (c) 2025, Noah Douglas.
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "tracking_wheel.h"
#include "pros/adi.hpp"
#include "pros/rotation.hpp"
#include <cmath>

/**
 * @brief creates a new tracking wheel object with a V5 Rotation Sensor
 *
 * @param rotation the VEX V5 Rotation Sensor used on the tracking wheel
 * @param diameter the diameter of the tracking wheel (in inches)
 * @param offset the offset of the tracking wheel from the tracking center
 */
TrackingWheel::TrackingWheel(pros::Rotation* rotation, float diameter, double offset)
    : rotation(rotation),
    encoder(nullptr), 
    diameter(diameter), 
    offset(offset), 
    last_total(get_distance_total()),
    sensor_type(SensorType::Rotation) {}

/**
 * @brief creates a new tracking wheel object with a V5 Quad Encoder
 *
 * @param encoder the VEX V5 Quad Encoder used on the tracking wheel
 * @param diameter the diameter of the tracking wheel (in inches)
 * @param offset the offset of the tracking wheel from the tracking center
 */
TrackingWheel::TrackingWheel(pros::adi::Encoder* encoder, float diameter, double offset)
    : rotation(nullptr),
    encoder(encoder), 
    diameter(diameter), 
    offset(offset), 
    last_total(get_distance_total()),
    sensor_type(SensorType::Encoder) {}

/**
 * @brief get total tracking wheel distance
 * 
 * retrieves the total distance travelled by the tracking wheel since the program
 * started. 
 * 
 * @return double total distance travelled (in inches)
 */    
double TrackingWheel::get_distance_total() {
    if (sensor_type == SensorType::Rotation && rotation) {
        return (rotation->get_position() - zero_position) / 36000.0f * M_PI * diameter;
    } else if (sensor_type == SensorType::Encoder && encoder) {
        return (encoder->get_value() - zero_position) / 360.0f * M_PI * diameter;
    } 
    return 0.0;
}

/**
 * @brief get tracking wheel distance since last call
 * 
 * calculates the distance travelled by the tracking wheel since the method was
 * last called.
 * 
 * @return double distance delta (in inches)
 */
double TrackingWheel::get_distance_delta() {
    const double total = get_distance_total();
    const double delta = total - last_total;
    last_total = total;
    return delta;
}

/** 
 * @brief get the tracking wheel offset
 * 
 * @return double offset (in inches)
 */
double TrackingWheel::get_offset() {
    return offset;
}

/**
 * @brief reset tracking wheel data
 */
void TrackingWheel::tare() {
    if (sensor_type == SensorType::Rotation && rotation) {

        zero_position = rotation->get_position();
    } else if (sensor_type == SensorType::Encoder && encoder) {
        zero_position = encoder->get_value();
    } 
    last_total = get_distance_total();
}
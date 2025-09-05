/**
 * @file angle.h
 * 
 * Contains helper functions for working with angles.
 * 
 * @copyright (c) 2025, Noah Douglas.
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef ANGLE_H
#define ANGLE_H

#include <cmath>

/** 
 * @brief wrap angle from -pi to pi
 * 
 * @param angle angle to be wrapped (in radians)
 * 
 * @return double wrapped angle
 */
inline double wrap_angle(double angle) {
    while (angle > M_PI) angle -= 2*M_PI;
    while (angle < -M_PI) angle += 2*M_PI;
    return angle;
}


/**
 * @brief convert angle from degrees to radians
 * 
 * @param angle angle in degrees
 * 
 * @return double anlge in radians
 */
inline double to_radians(double angle) {
    return angle * M_PI / 180.0;
}

#endif
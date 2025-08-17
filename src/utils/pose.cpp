/**
 * @file pose.cpp
 * 
 * Contains implementations for the robots pose
 * 
 * @copyright (c) 2025, Noah Douglas.
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "pose.h"

/**
 * @brief constructs a pose from a given x, y, and heading
 */
Pose::Pose(float x, float y, float heading)
    : x(x), y(y), heading(heading) {}
/**
 * @file pose.h
 * 
 * Contains declerations related to the robots pose
 * 
 * @copyright (c) 2025, Noah Douglas.
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef POSE_H
#define POSE_H

/**
 @brief a struct representing the pose of the robot
 */
struct Pose {
    float x;
    float y;
    float heading;

    Pose (float x, float y, float heading);
};

#endif // POSE_H
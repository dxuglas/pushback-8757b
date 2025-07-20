#ifndef ANGLE_H
#define ANGLE_H
#include <cmath>
#include "constants.h"

inline double wrap_angle(double angle) {
    // Use modulo operation for better performance than while loops
    // Normalize to [-π, π] range
    angle = fmod(angle, TWO_PI);
    if (angle > M_PI) {
        angle -= TWO_PI;
    } else if (angle < -M_PI) {
        angle += TWO_PI;
    }
    return angle;
}

inline double to_radians(double angle) {
    return angle * DEG_TO_RAD;
}

#endif
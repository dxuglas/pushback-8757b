#ifndef ANGLE_H
#define ANGLE_H
#include <cmath>

inline double wrap_angle(double angle) {
    // Use modulo operation for better performance than while loops
    angle = fmod(angle + M_PI, 2*M_PI);
    if (angle < 0) angle += 2*M_PI;
    return angle - M_PI;
}

inline double to_radians(double angle) {
    return angle * M_PI / 180.0;
}

#endif
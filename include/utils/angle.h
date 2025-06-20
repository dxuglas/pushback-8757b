#ifndef ANGLE_H
#define ANGLE_H
#include <cmath>

inline double wrap_angle(double angle) {
    while (angle > M_PI) angle -= 2*M_PI;
    while (angle < -M_PI) angle += 2*M_PI;
    return angle;
}

inline double to_radians(double angle) {
    return angle * M_PI / 180.0;
}

#endif
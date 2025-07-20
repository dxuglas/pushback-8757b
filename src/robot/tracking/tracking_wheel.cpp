#include "tracking_wheel.h"
#include "pros/rotation.hpp"
#include <cmath>

TrackingWheel::TrackingWheel(pros::Rotation* encoder, double diameter, double offset)
    : encoder(encoder), 
    diameter(diameter), 
    offset(offset), 
    last_total(get_distance_total()) {}

double TrackingWheel::get_distance_total() {
    // Use consistent double precision and cache M_PI calculation
    static const double pi_over_360 = M_PI / 360.0;
    return encoder->get_angle() * pi_over_360 * diameter;
}

double TrackingWheel::get_distance_delta() {
    const double total = this->get_distance_total();
    const double delta = total - last_total;
    last_total = total;
    return delta;
}

double TrackingWheel::get_offset() {
    return offset;
}
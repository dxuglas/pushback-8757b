#include "tracking_wheel.h"
#include "pros/rotation.hpp"
#include <cmath>

TrackingWheel::TrackingWheel(pros::Rotation* encoder, float diameter, double offset)
    : encoder(encoder), 
    diameter(diameter), 
    offset(offset), 
    last_total(get_distance_total()) {}

double TrackingWheel::get_distance_total() {
    return encoder->get_angle() / 360.0f * M_PI * diameter;
}

double TrackingWheel::get_distance_delta() {
    const double total = get_distance_total();
    const double delta = total - last_total;
    last_total = total;
    return delta;
}

double TrackingWheel::get_offset() {
    return offset;
}

void TrackingWheel::tare() {
    encoder->reset();
}
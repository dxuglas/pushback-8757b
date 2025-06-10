#include "tracking_wheel.h"
#include "pros/rotation.hpp"
#include <cmath>

TrackingWheel::TrackingWheel(pros::Rotation* encoder, float diameter, float offset)
    : encoder(encoder), 
    diameter(diameter), 
    offset(offset), 
    last_total(get_distance_total()) {}

float TrackingWheel::get_distance_total() {
    return encoder->get_angle() / 360.0f * M_PI * diameter;
}

float TrackingWheel::get_distance_delta() {
    const float total = this->get_distance_total();
    const float delta = total - last_total;
    last_total = total;
    return delta;
}
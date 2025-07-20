#ifndef TRACKING_WHEEL_H
#define TRACKING_WHEEL_H

#include "pros/rotation.hpp"

class TrackingWheel {
    public:
        TrackingWheel(pros::Rotation* encoder, double diameter, double offset);
        double get_distance_delta();
        double get_distance_total();
        double get_offset();
        
    private:
        pros::Rotation* encoder;
        double diameter;
        double offset;
        double last_total;
};

#endif // TRACKING_WHEEL_H
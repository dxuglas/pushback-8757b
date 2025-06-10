#ifndef TRACKING_WHEEL_H
#define TRACKING_WHEEL_H

#include "pros/rotation.hpp"

class TrackingWheel {
    public:
        TrackingWheel(pros::Rotation* encoder, float diameter, float offset);
        float get_distance_delta();
        float get_distance_total();
        
    private:
        pros::Rotation* encoder;
        float diameter;
        float offset;
        float last_total;
};

#endif // TRACKING_WHEEL_H
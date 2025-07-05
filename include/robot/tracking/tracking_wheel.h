#ifndef TRACKING_WHEEL_H
#define TRACKING_WHEEL_H

#include "pros/rotation.hpp"

/**
 * @brief Tracking Wheel Class
 * 
 * Represents a tracking wheel on the robot
 */
class TrackingWheel {
    public:
        /**
         * @brief Constructs a tracking wheel
         *
         * @param encoder tracking wheels sensor
         * @param diameter tracking wheel diameter
         * @param offset offset of tracking wheel from tracking center
         */
        TrackingWheel(pros::Rotation* encoder, float diameter, double offset);
        /**
         * @brief Get the change in distance measured since last call
         */
        double get_distance_delta();
        /**
         */
        double get_distance_total();
        double get_offset();
        void tare();
        
    private:
        pros::Rotation* encoder;
        float diameter;
        float offset;
        float last_total;
};

#endif // TRACKING_WHEEL_H
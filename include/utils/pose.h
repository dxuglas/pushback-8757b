#ifndef POSE_H
#define POSE_H

struct Pose {
    float x;
    float y;
    float heading;

    Pose (float x, float y, float heading);
};

#endif // POSE_H
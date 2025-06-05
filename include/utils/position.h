#ifndef POSITION_H
#define POSITION_H

struct Position {
    float x;
    float y;
    float heading;

    Position (float x, float y, float heading);
};

#endif // POSITION_H
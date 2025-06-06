#include "check_threshold.h"

float check_threshold(float value, float min_value) {
    return value > min_value ? value : 0.0f;
}
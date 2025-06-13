#include "check_threshold.h"
#include <cstdlib>

float check_threshold(float value, float min_value) {
    return std::abs(value) > min_value ? value : 0.0f;
}
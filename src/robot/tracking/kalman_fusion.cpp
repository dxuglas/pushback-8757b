#include "robot/tracking/kalman_fusion.h"
#include <algorithm>
#include <numeric>

KalmanFusion::KalmanFusion(double Q)
    : Q(Q), x(0), P(1) {}

void KalmanFusion::predict() {
    P += Q;
}

void KalmanFusion::update(std::vector<float> data, std::vector<float> noise) {
    std::vector<float> weight;
    for (int i = 0; i < noise.size(); i++) {
        weight.at(i) = 1.0f / noise.at(i);
    }
    std::vector<float> weighted_data(std::min(data.size(), weight.size()));
    std::transform(
        data.begin(), 
        data.begin() + weighted_data.size(), 
        weight.begin(), weighted_data.begin(), 
        [](float x, float y) { return x * y;});
    float weighted_sum = std::reduce(weighted_data.begin(), weighted_data.end());
    float weight_sum = std::reduce(weight.begin(), weight.end());
    double z = weighted_sum / weight_sum;
    double r_fused = 1.0f / weight_sum;

    double K = P / (P + r_fused);
    x += K* (z-x);
    P = (1-K) * P;
}

double KalmanFusion::get_state() { return x; }

void KalmanFusion::set_state(double value) { x = value; }
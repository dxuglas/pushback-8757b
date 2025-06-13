#ifndef KALMAN_FUSION_H
#define KALMAN_FUSION_H

#include <vector>

class KalmanFusion {
    public:

        KalmanFusion(double Q = 1e-5);

        void predict();

        void update(std::vector<float> data, std::vector<float> noise);

        double get_state();

        void set_state(double value);

    private:
        double Q;
        double x;
        double P;
};

#endif
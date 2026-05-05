#pragma once

class KalmanFilter {
public:
    // Altitude only filter
    double state_variable = 0.0; // X -  starts on the ground
    double state_covariance = 1.0; // P
    double process_model = 1.0; // F
    double process_noise = 1.0; // Q
    double measurement_model = 1.0; // H
    double measurement_noise = 1.0; // R
    double new_state;
    double new_cov;

    void predict(double acceleration, double dt){
        new_state = state_variable + (acceleration * dt);
        new_cov = state_covariance + process_noise;
    };

    void update(double measurement){
        double y = measurement - new_state;
        double K = (new_cov)/(new_cov + measurement_noise);
        state_variable = new_state + K * y;
        state_covariance = (1 - K) * new_cov;
    }
};

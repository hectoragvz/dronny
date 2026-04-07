#pragma once

struct PIDController {
    double Kp;
    double Ki;
    double Kd;
    double integral = 0.0;
    double prev_error = 0.0;

    double totErr = 0;

    PIDController(double Kp, double Ki, double Kd) : Kp(Kp), Ki(Ki), Kd(Kd){}

    double compute(double error, double dt){
        integral += error * dt;
        double derivative = (error - prev_error) / dt;
        prev_error = error;
        return Kp * error + Ki * integral + Kd * derivative;
    }
};

#pragma once
#include <math/vec2.hpp>

struct QuadrotorModel {
    Vec2 position;
    Vec2 velocity;
    double pitch;
    double pitch_rate;
    double mass;
    double arm_length;
    double inertia;
    static constexpr double gravity = 9.81;

    QuadrotorModel(double mass, double arm_length, double inertia)
        : position(0.0, 0.0)
        , velocity(0.0, 0.0)
        , pitch(0.0)
        , pitch_rate(0.0)
        , mass(mass)
        , arm_length(arm_length)
        , inertia(inertia) {}

    // Advance simulation by dt seconds
    void step(double dt, double T1, double T2) {
        double total_thrust = T1 + T2;
        double torque = (T2 - T1) * arm_length;
        double ax = - total_thrust * sin(pitch) / mass;
        double az =  total_thrust * cos(pitch) / mass - gravity;
        double pitch_accel = torque / inertia;
        velocity = velocity + Vec2(ax, az) * dt;
        position = position + velocity * dt;
        pitch_rate = pitch_rate + pitch_accel * dt;
        pitch = pitch + pitch_rate * dt;
    }
};

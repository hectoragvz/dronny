
#include <iostream>
#include <math/vec2.hpp>
#include <cassert>
#include <sim/quadrotor_model.hpp>
#include <sim/sim_clock.hpp>
#include <control/pid.hpp>

int main (){

    Vec2 a(1.0, 2.0);
    Vec2 b(3.0, 4.0);
    Vec2 c = a + b;
    std::cout << "c: " << c.x << ", " << c.z << std::endl;
    std::cout << "b norm: " << b.norm() << std::endl;
    assert(b.norm() == 5.0);
    Vec2 n = a.normalized();
    assert(std::abs(n.norm() - 1.0) < 1e-9);

    std::cout << "QUADROTOR TESTS" << '\n';


    // Quadrotor
    QuadrotorModel model(1.5, 0.25, 0.1);
    double hover_thrust = model.mass * model.gravity / 2.0;
    SimClock clock(0.001);
    while (clock.current_time < 1.0){
        model.step(clock.dt, hover_thrust, hover_thrust);
        clock.tick();
    }

    std::cout << "Position: " << model.position.x << ", " << model.position.z << std::endl;
    std::cout << "Velocity: " << model.velocity.x << ", " << model.velocity.z << std::endl;
    std::cout << "Pitch: " << model.pitch << std::endl;
    assert(std::abs(model.position.z) < 0.01);
    assert(std::abs(model.position.x) < 0.01);
    assert(std::abs(model.pitch) < 0.01);

    // Unequal Thrust
    QuadrotorModel model2(1.5, 0.25, 0.1);
    SimClock clock2(0.001);
    while (clock2.current_time < 1.0){
        model2.step(clock2.dt, 6.0, 8.0);
        clock2.tick();
    }
    std::cout << "After unequal thrust:" << std::endl;
    std::cout << "Position: " << model2.position.x << ", " << model2.position.z << std::endl;
    std::cout << "Pitch: " << model2.pitch << std::endl;

    PIDController pid(0.5, 0.05, 1.5);

    std::cout << "PID CONTROLLER TESTS" << '\n';

    double position = 10.0;
    double target = 0.0;
    double velocity = 0.0;
    double dt = 0.01;

    for (int i = 0; i < 500; i++) {
        double error = target - position;
        double acceleration = pid.compute(error, dt);
        velocity += acceleration * dt;
        position += velocity * dt;
        if (i % 50 == 0) {
            std::cout << "t=" << i * dt << " pos=" << position << std::endl;
        }
    }

    return 0;
}

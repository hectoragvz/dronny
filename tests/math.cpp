
#include <iostream>
#include <math/vec2.hpp>
#include <cassert>
#include <sim/quadrotor_model.hpp>
#include <sim/sim_clock.hpp>

int main (){

    Vec2 a(1.0, 2.0);
    Vec2 b(3.0, 4.0);
    Vec2 c = a + b;
    std::cout << "c: " << c.x << ", " << c.z << std::endl;
    std::cout << "b norm: " << b.norm() << std::endl;
    assert(b.norm() == 5.0);
    Vec2 n = a.normalized();
    assert(std::abs(n.norm() - 1.0) < 1e-9);

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

    return 0;
}

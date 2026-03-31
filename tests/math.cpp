
#include <iostream>
#include <math/vec2.hpp>
#include <cassert>

int main (){
    Vec2 a(1.0, 2.0);
    Vec2 b(3.0, 4.0);
    Vec2 c = a + b;
    std::cout << "c: " << c.x << ", " << c.z << std::endl;
    std::cout << "b norm: " << b.norm() << std::endl;
    assert(b.norm() == 5.0);
    Vec2 n = a.normalized();
    assert(std::abs(n.norm() - 1.0) < 1e-9);
    return 0;
}

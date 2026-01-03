#include <iostream>
#include "math/vector2.hpp"

int main() {
    //std::cout << "Robot Arm Planner - Phase 0 setup OK\n";
    //return 0;
    math::Vector2 a{3.0, 4.0};
    math::Vector2 b{1.0, 2.0};

    std::cout << "a = " << a << "\n";
    std::cout << "b = " << b << "\n";

    std::cout << "a + b = " << (a + b) << "\n";
    std::cout << "a - b = " << (a - b) << "\n";
    std::cout << "a * 2 = " << (a * 2.0) << "\n";

    std::cout << "dot(a, b) = " << a.dot(b) << "\n";
    std::cout << "norm(a) = " << a.norm() << "\n";
    std::cout << "a.normalized() = " << a.normalized() << "\n";

    return 0;
}
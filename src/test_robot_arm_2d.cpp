#include <iostream>
#include <cassert>
#include <cmath>

#include "robot/robot_arm_2d.hpp"
#include "math/se2.hpp"

using robot::RobotArm2d;
using math::Vector2;

static constexpr double EPS = 1e-9;

void test_fk_two_links_straight() {
    RobotArm2d arm{1.0, 1.0};

    std::vector<double> q = {0.0, 0.0};
    auto T = arm.forward_kinematics(q);

    Vector2 p = T * Vector2{0.0, 0.0};

    assert(std::abs(p.x - 2.0) < EPS);
    assert(std::abs(p.y - 0.0) < EPS);
}

void test_fk_three_links_straight() {
    RobotArm2d arm{1.0, 1.0, 1.0};

    std::vector<double> q = {0.0, 0.0, 0.0};
    auto T = arm.forward_kinematics(q);

    Vector2 p = T * Vector2{0.0, 0.0};

    assert(std::abs(p.x - 3.0) < EPS);
    assert(std::abs(p.y - 0.0) < EPS);
}

void test_fk_three_links_right_angle(){
    RobotArm2d arm{1.0, 1.0, 1.0};

    std::vector<double> q = {M_PI/2.0, 0.0, 0.0};
    auto T = arm.forward_kinematics(q);

    Vector2 p = T * Vector2{0.0, 0.0};

    assert(std::abs(p.x + 1.0) < EPS);
    assert(std::abs(p.y - 0.0) < EPS);
}

void test_fk_three_links_mixed_angles() {
    RobotArm2d arm{1.0, 1.0};

    std::vector<double> q = {M_PI/2.0, -M_PI/2.0};
    auto T = arm.forward_kinematics(q);

    Vector2 p = T * Vector2{0.0, 0.0};

    //first link goes up to (0,1)
    //second link rotates -90° relative -> points right -> end at (1,1)
    assert(std::abs(p.x - 0.0) < EPS);
    assert(std::abs(p.y - 2.0) < EPS);
}

void test_fk_angles() {
    RobotArm2d arm{1.0, 1.0};
    std::vector<double> q = {M_PI/2, 0.0};

    auto T = arm.forward_kinematics(q);
    Vector2 p = T * Vector2{0.0, 0.0};

    // arm should point straight up: (0, 2)
    assert(std::abs(p.x + 1.0) < 1e-6);
    assert(std::abs(p.y - 1.0) < 1e-6);
}

void test_fk_cumulative() {
    RobotArm2d arm{1.0, 1.0, 1.0};
    std::vector<double> q = {M_PI/2, -M_PI/2, 0.0};

    auto T = arm.forward_kinematics(q);
    Vector2 p = T * Vector2{0.0, 0.0};

    // expected: first link up, second link right, third link right
    // final position = (0, 3)
    assert(std::abs(p.x - 0.0) < 1e-6);
    assert(std::abs(p.y - 3.0) < 1e-6);
}

int main() {
    test_fk_two_links_straight();
    test_fk_three_links_straight();
    test_fk_three_links_right_angle();
    test_fk_three_links_mixed_angles();
    test_fk_angles();
    test_fk_cumulative();

    std::cout << "All RobotArm2d tests passed\n";
    return 0;
}
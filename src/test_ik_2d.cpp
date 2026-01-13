#include <iostream>
#include <cassert>
#include <cmath>
#include <vector>

#include "robot/ik_2d.hpp"
#include "robot/robot_arm_2d.hpp"
#include "math/se2.hpp"

using robot::IK2d;
using robot::RobotArm2d;
using math::Vector2;

static constexpr double EPS = 1e-4;

// ----------------------------------------------
// Helper: compute end-effector position
// ----------------------------------------------
Vector2 end_effector(const RobotArm2d& arm, const std::vector<double>& q) {
    auto T = arm.forward_kinematics(q);
    return T * Vector2{0.0, 0.0};
}

// ----------------------------------------------
// Test 1: Reachable target on x-axis
// ----------------------------------------------
void test_ik_reachable_xaxis() {
    RobotArm2d arm{1.0, 1.0};

    Vector2 target{1.5, 0.0};
    std::vector<double> q0 = {0.001, 0.0};

    auto q = IK2d::solve(arm, target, q0, 1e-6, 100, 0.1);

    Vector2 p = end_effector(arm, q);
    // debugging: std::cout << "p = (" << p.x << ", " << p.y << ")\n";

    assert(std::abs(p.x - target.x) < EPS);
    assert(std::abs(p.y - target.y) < EPS); 
}

// -------------------------------------------------
// Test 2: Diagonal target
// -------------------------------------------------
void test_ik_diagonal() {
    RobotArm2d arm{1.0, 1.0};

    Vector2 target{1.0, 1.0};
    std::vector<double> q0 = {0.5, -0.5};

    auto q = IK2d::solve(arm, target, q0, 1e-6, 100, 0.1);

    Vector2 p = end_effector(arm, q);

    assert(std::abs(p.x - target.x) < EPS);
    assert(std::abs(p.y - target.y) < EPS);
}

// -----------------------------------------------
// Test 3: Convergence from a far initial guess
// -----------------------------------------------
void test_ik_far_initial_guess() {
    RobotArm2d arm{1.0, 1.0};

    Vector2 target{0.5, 1.5};
    std::vector<double> q0 = {3.0, -2.0}; //wild initial guess

    auto q = IK2d::solve(arm, target, q0, 1e-6, 100, 0.1);

    Vector2 p = end_effector(arm, q);

    assert(std::abs(p.x - target.x) < EPS);
    assert(std::abs(p.y - target.y) < EPS);
}

// ----------------------------------------------------
// Test 4: Unreachable target (arm should stretch out)
// ----------------------------------------------------
void test_ik_unreachable() {
    RobotArm2d arm{1.0, 1.0};

    Vector2 target{3.0, 0.0}; //beyond max reach of 2.0
    std::vector<double> q0 = {0.0, 0.0};

    auto q = IK2d::solve(arm, target, q0, 1e-6, 100, 0.1);

    Vector2 p = end_effector(arm, q);

    double dist =  std::sqrt(p.x*p.x + p.y*p.y);

    //should stretch max reach to max reach (2.0)
    assert(std::abs(dist - 2.0) < EPS);
}

// --------------------------------
// Main
// --------------------------------
int main() {
    test_ik_reachable_xaxis();
    test_ik_diagonal();
    test_ik_far_initial_guess();
    test_ik_unreachable();

    std::cout << "All IK2d tests passed\n";
    return 0;
}
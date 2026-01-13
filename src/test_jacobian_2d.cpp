#include <iostream>
#include <cassert>
#include <cmath>
#include <vector>

#include "robot/jacobian_2d.hpp"
#include "robot/robot_arm_2d.hpp"
#include "math/se2.hpp"

using robot::Jacobian2d;
using robot::RobotArm2d;
using math::Vector2;

static constexpr double EPS = 1e-6;

// ------------------------------------------------------------
// Helper: numerical finite-difference Jacobian for validation
// ------------------------------------------------------------
std::vector<std::vector<double>>
numerical_jacobian(const RobotArm2d& arm,
                   const std::vector<double>& q,
                   double h = 1e-6)
{
    size_t N = q.size();
    std::vector<std::vector<double>> J(2, std::vector<double>(N, 0.0));

    //Current end-effector position
    auto T0 = arm.forward_kinematics(q);
    Vector2 p0 = T0 * Vector2{0.0, 0.0};

    for (size_t i = 0; i < N; ++i) {
        std::vector<double> q_perturbed = q;
        q_perturbed[i] += h;

        auto T1 = arm.forward_kinematics(q_perturbed);
        Vector2 p1 = T1 * Vector2{0.0, 0.0};

        J[0][i] = (p1.x - p0.x) / h;
        J[1][i] = (p1.y - p0.y) / h;
    }
    return J;
}

// ----------------------------------------
// Test 1: Straight arm (all angles = 0)
// ----------------------------------------
void test_jacobian_straight() {
    RobotArm2d arm{1.0, 1.0};

    std::vector<double> q = {0.0, 0.0};

    auto J = Jacobian2d::compute({1.0, 1.0}, q);

    // expected:
    // joint 1 affects both links:
    // dx/dq1 = -(1*sin(0) + 1*sin(0)) = 0
    // dy/dq1 = (1*cos(0) + 1*cos(0)) = 2

    // joint 2 affects only link 2:
    // dx/dq2 = -1*sin(0) = 0
    // dy/dq2 = 1*cos(0) = 1

    assert(std::abs(J[0][0] - 0.0) < EPS);
    assert(std::abs(J[1][0] - 2.0) < EPS);

    assert(std::abs(J[0][1] - 0.0) < EPS);
    assert(std::abs(J[1][1] - 1.0) < EPS);
}

// ------------------------------------------
// Test 2: Right-angle configuration
// ------------------------------------------
void test_jacobian_right_angle() {
    RobotArm2d arm{1.0, 1.0};

    std::vector<double> q = {M_PI/2.0, 0.0};

    auto J = Jacobian2d::compute({1.0, 1.0}, q);

    // expected:
    // joint 1 affects both links:
    // dx/dq1 = -(1*sin(90°) + 1*sin(90°)) = -2
    // dy/dq1 = (1*cos(90°) + 1*cos(90°)) = 0

    // joint 2 affects only link 2:
    // dx/dq2 = -1*sin(90°) = -1
    // dy/dq2 = 1*cos(90°) = 0

    assert(std::abs(J[0][0] + 2.0) < EPS);
    assert(std::abs(J[1][0] - 0.0) < EPS);

    assert(std::abs(J[0][1] + 1.0) < EPS);
    assert(std::abs(J[1][1] - 0.0) < EPS);
}

// -----------------------------------------------
// Test 3: Numerical finite-difference validation
// -----------------------------------------------
void test_jacobian_numerical() {
    RobotArm2d arm{1.0, 1.0, 0.5};

    std::vector<double> q = {0.3, -0.7, 1.2};

    auto J_analytic = Jacobian2d::compute({1.0, 1.0, 0.5}, q);
    auto J_numeric = numerical_jacobian(arm, q);

    for (size_t r = 0; r < 2; ++r) {
        for (size_t c = 0; c < q.size(); ++c) {
            assert(std::abs(J_analytic[r][c] - J_numeric[r][c]) < 1e-4);
        }
    }
}

// -----------------------------------------------
// Main
// -----------------------------------------------
int main() {
    test_jacobian_straight();
    test_jacobian_right_angle();
    test_jacobian_numerical();

    std::cout << "All Jacobian2d tests passed\n";
    return 0;
}
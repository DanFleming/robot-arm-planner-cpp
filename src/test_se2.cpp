#include <iostream>
#include <cassert>
#include <cmath>

#include "math/vector2.hpp"
#include "math/matrix2.hpp"
#include "math/se2.hpp"

using math::Vector2;
using math::Matrix2;
using math::SE2;

// --------------------------
// SE2 Tests
// --------------------------

void test_se2_identity() {
    SE2 I; //default constructor = identity
    Vector2 p{1.0, 2.0};
    Vector2 r = I * p;

    assert(std::abs(r.x - 1.0) < 1e-9);
    assert(std::abs(r.y - 2.0) < 1e-9);
}

void test_se2_transform_point() {
    double theta = M_PI / 2.0; //90 degrees
    Vector2 t{1.0, 0.0};

    SE2 T = SE2::from_angle_translation(theta, t);

    Vector2 p{1.0, 0.0};
    Vector2 r = T * p;

    //Rotation 90°: (1,0) -> (0,1), then + translation (1,0)
    assert(std::abs(r.x - 1.0) < 1e-9);
    assert(std::abs(r.y - 1.0) < 1e-9);
}

void test_se2_composition() {
    SE2 A = SE2::from_angle_translation(0.0, Vector2{1.0, 0.0});
    SE2 B = SE2::from_angle_translation(0.0, Vector2{0.0, 2.0});

    SE2 C = A * B;

    Vector2 p{0.0, 0.0};
    Vector2 r = C * p;

    //translate by (0,2), then (1,0)
    assert(std::abs(r.x - 1.0) < 1e-9);
    assert(std::abs(r.y - 2.0) < 1e-9);
}

void test_se2_inverse() {
    SE2 T = SE2::from_angle_translation(M_PI / 4.0, Vector2{2.0, 3.0});
    SE2 Ti = T.inverse();

    //T * Ti should be identity
    SE2 I = T * Ti;

    Vector2 p{4.0, -1.0};
    Vector2 r = I * p;

    assert(std::abs(r.x - p.x) < 1e-9);
    assert(std::abs(r.y - p.y) < 1e-9);
}

void test_se2_round_trip() {
    SE2 T = SE2::from_angle_translation(1.0, Vector2{3.0, -2.0});
    SE2 Ti = T.inverse();

    Vector2 p{5.0, 7.0};
    Vector2 r = Ti * (T * p);

    assert(std::abs(r.x - p.x) < 1e-9);
    assert(std::abs(r.y - p.y) < 1e-9);
}

// -------------------------------------------
// Main Test Runner
// -------------------------------------------

int main() {
    test_se2_identity();
    test_se2_transform_point();
    test_se2_composition();
    test_se2_inverse();
    test_se2_round_trip();

    std::cout << "All se2 tests passed\n";
    return 0;
}
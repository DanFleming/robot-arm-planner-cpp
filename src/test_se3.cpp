#include <iostream>
#include <cassert>
#include <cmath>

#include "math/vector3.hpp"
#include "math/matrix3.hpp"
#include "math/se3.hpp"

using math::Vector3;
using math::Matrix3;
using math::SE3;

// ------------------------
// SE3 Tests
// ------------------------

void test_se3_identity() {
    SE3 I; //default constructor = identity
    Vector3 p{1.0, 2.0, 3.0};
    Vector3 r = I * p;

    assert(std::abs(r.x - 1.0) < 1e-9);
    assert(std::abs(r.y - 2.0) < 1e-9);
    assert(std::abs(r.z - 3.0) < 1e-9);
}

void test_se3_transform_point() {
    //90° rotation around z axis
    Matrix3 R = Matrix3::rotation_z(M_PI / 2.0);
    Vector3 t{1.0, 0.0, 0.0};

    SE3 T = SE3::from_rotation_translation(R, t);

    Vector3 p{1.0, 0.0, 0.0};
    Vector3 r = T * p;

    //Rotation around z: (1,0,0) -> (0,1,0), then + translation (1,0,0)
    assert(std::abs(r.x - 1.0) < 1e-9);
    assert(std::abs(r.y - 1.0) < 1e-9);
    assert(std::abs(r.z - 0.0) < 1e-9);
}

void test_se3_composition() {
    SE3 A = SE3::from_rotation_translation(Matrix3::identity(), Vector3{1.0, 0.0, 0.0});
    SE3 B = SE3::from_rotation_translation(Matrix3::identity(), Vector3{0.0, 2.0, 0.0});

    SE3 C = A * B;

    Vector3 p{0.0, 0.0, 0.0};
    Vector3 r = C * p;

    //First translate by (0,2,0), then by (1,0,0)
    assert(std::abs(r.x - 1.0) < 1e-9);
    assert(std::abs(r.y - 2.0) < 1e-9);
    assert(std::abs(r.z - 0.0) < 1e-9);
}

void test_se3_inverse() {
    Matrix3 R = Matrix3::rotation_x(M_PI / 4.0);
    Vector3 t{2.0, 3.0, -1.0};

    SE3 T = SE3::from_rotation_translation(R, t);
    SE3 Ti = T.inverse();

    //T * Ti should be identity
    SE3 I = T * Ti;

    Vector3 p{4.0, -1.0, 2.0};
    Vector3 r = I * p;

    assert(std::abs(r.x - p.x) < 1e-9);
    assert(std::abs(r.y - p.y) < 1e-9);
    assert(std::abs(r.z - p.z) < 1e-9);
}

void test_se3_round_trip() {
    Matrix3 R = Matrix3::rotation_y(1.0);
    Vector3 t{3.0, -2.0, 5.0};

    SE3 T = SE3::from_rotation_translation(R, t);
    SE3 Ti = T.inverse();

    Vector3 p{5.0, 7.0, -3.0};
    Vector3 r = Ti * (T * p);

    assert(std::abs(r.x - p.x) < 1e-9);
    assert(std::abs(r.y - p.y) < 1e-9);
    assert(std::abs(r.z - p.z) < 1e-9);
}

// ----------------------------------------
// Main Test Runner
// ----------------------------------------

int main() {
    test_se3_identity();
    test_se3_transform_point();
    test_se3_composition();
    test_se3_inverse();
    test_se3_round_trip();

    std::cout << "All SE3 tests passed\n";
    return 0;
}
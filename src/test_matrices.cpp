#include <iostream>
#include <cassert>
#include <cmath>

#include "math/vector2.hpp"
#include "math/vector3.hpp"
#include "math/matrix2.hpp"
#include "math/matrix3.hpp"

using math::Vector2;
using math::Vector3;
using math::Matrix2;
using math::Matrix3;

// ----------------------------
// Matrix2 Tests
// ----------------------------

void test_matrix2_identity() {
    Matrix2 I;
    Vector2 v{3.0, 4.0}:
    Vector2 r = I * v;
    assert(r.x == 3.0 && r.y == 4.0);
}

void test_matrix2_multiply_matrix() {
    Matrix2 A{1.0, 2.0,
              3.0, 4.0};
    
    Matrix2 B{2.0, 0.0,
              1.0, 2.0};

    Matrix2 C = A * B;

    assert(C.m00 == 4.0);
    assert(C.m01 == 4.0);
    assert(C.m10 == 10.0);
    assert(C.m11 == 8.0);
}

void test_matrix2_determinant() {
    Matrix2 M{3.0, 8.0,
              4,0, 6.0};
    assert(std::abs(M.det() - (14.0)) < 1e-9);
}

void test_matrix2_transpose() {
    Matrix2 M{1.0, 2.0,
              3.0, 4.0};
    Matrix2 T = M.transpose();
    assert(T.m00 == 1.0 && T.m01 == 3.0);
    assert(T.m10 == 2.0 && T.m11 == 4.0);
}

void test_matrix2_rotation() {
    double theta = M_PI / 2.0; //90 degrees
    Matrix2 R = Matrix2::rotation(theta);
    Vector2 v{1.0, 0.0};
    Vector2 r = R * v;

    assert(std::abs(r.x - 0.0) < 1e-9);
    assert(std::abs(r.y - 1.0) < 1e-9);
}

// -----------------------------------------
// Matrix3 Tests
// -----------------------------------------


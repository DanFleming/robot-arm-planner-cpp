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
    Vector2 v{3.0, 4.0};
    Vector2 r = I * v;
    assert(r.x == 3.0 && r.y == 4.0);
}

void test_matrix2_multiply_vector() {
    Matrix2 I;
    Vector2 v{3.0, 4.0};
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
              4.0, 6.0};
    assert(std::abs(M.det() + 14.0) < 1e-9);
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

void test_matrix3_identity() {
    Matrix3 I;
    Vector3 v{1.0, 2.0, 3.0};
    Vector3 r = I * v;
    assert(r.x == 1.0 && r.y == 2.0 && r.z == 3.0);
}

void test_matrix3_multiply_vector(){
    Matrix3 M{2.0, 0.0, 0.0,
              0.0, 3.0, 0.0,
              0.0, 0.0, 4.0};
    
    Vector3 v{1.0, 2.0, 3.0};
    Vector3 r = M * v;

    assert(r.x == 2.0);
    assert(r.y == 6.0);
    assert(r.z == 12.0);
}

void test_matrix3_multiply_matrix() {
    Matrix3 A{1.0, 2.0, 3.0,
              4.0, 5.0, 6.0,
              7.0, 8.0, 9.0};
    
    Matrix3 B{9.0, 8.0, 7.0,
              6.0, 5.0, 4.0,
              3.0, 2.0, 1.0};

    Matrix3 C = A * B;

    assert(C.m00 == 30.0);
    assert(C.m01 == 24.0);
    assert(C.m02 == 18.0);

    assert(C.m10 == 84.0);
    assert(C.m11 == 69.0);
    assert(C.m12 == 54.0);

    assert(C.m20 == 138.0);
    assert(C.m21 == 114.0);
    assert(C.m22 == 90.0);
}

void test_matrix3_determinant(){
    Matrix3 M{1.0, 2.0, 3.0,
              0.0, 1.0, 4.0,
              5.0, 6.0, 0.0};

    assert(std::abs(M.det() - 1.0) < 1e-9);
}

void test_matrix3_transpose() {
    Matrix3 M{1.0, 2.0, 3.0,
              4.0, 5.0, 6.0,
              7.0, 8.0, 9.0};

    Matrix3 T = M.transpose();

    assert(T.m00 == 1.0 && T.m01 == 4.0 && T.m02 == 7.0);
    assert(T.m10 == 2.0 && T.m11 == 5.0 && T.m12 == 8.0);
    assert(T.m20 == 3.0 && T.m21 == 6.0 && T.m22 == 9.0);
}

void test_matrix3_rotation_x() {
    double t = M_PI / 2.0;
    Matrix3 R = Matrix3::rotation_x(t);
    Vector3 v{0.0, 1.0, 0.0};
    Vector3 r = R * v;

    assert(std::abs(r.x - 0.0) < 1e-9);
    assert(std::abs(r.y - 0.0) < 1e-9);
    assert(std::abs(r.z - 1.0) < 1e-9);
}

void test_matrix3_rotation_y(){
    double t = M_PI / 2.0;
    Matrix3 R = Matrix3::rotation_y(t);
    Vector3 v{1.0, 0.0, 0.0};
    Vector3 r = R * v;

    assert(std::abs(r.x - 0.0) < 1e-9);
    assert(std::abs(r.y - 0.0) < 1e-9);
    assert(std::abs(r.z + 1.0) < 1e-9);
}

void test_matrix3_rotation_z(){
    double t = M_PI / 2.0;
    Matrix3 R = Matrix3::rotation_z(t);
    Vector3 v{1.0, 0.0, 0.0};
    Vector3 r = R * v;

    assert(std::abs(r.x - 0.0) < 1e-9);
    assert(std::abs(r.y - 1.0) < 1e-9);
    assert(std::abs(r.z - 0.0) < 1e-9);
}

// ------------------------------------------
// Main Test Runner
// ------------------------------------------

int main() {
    //Matrix2
    test_matrix2_identity();
    test_matrix2_multiply_vector();
    test_matrix2_multiply_matrix();
    test_matrix2_determinant();
    test_matrix2_transpose();
    test_matrix2_rotation();

    //Matrix3
    test_matrix3_identity();
    test_matrix3_multiply_vector();
    test_matrix3_multiply_matrix();
    test_matrix3_determinant();
    test_matrix3_transpose();
    test_matrix3_rotation_x();
    test_matrix3_rotation_y();
    test_matrix3_rotation_z();

    std::cout << "All matrix tests passed\n";
    return 0;
}

#include <iostream>
#include <cassert>
#include <cmath>
#include "math/vector2.hpp"
#include "math/vector3.hpp"

using math::Vector2;
using math::Vector3;

// ---------------------------
// Vector2 Tests
// ---------------------------

void test_vector2_addition() {
    Vector2 a{1.0, 2.0};
    Vector2 b{3.0, 4.0};
    Vector2 c = a + b;
    assert(c.x == 4.0 && c.y == 6.0);
}

void test_vector2_subtraction() {
    Vector2 a{5.0, 7.0};
    Vector2 b{2.0, 3.0};
    Vector2 c = a - b;
    assert(c.x == 3.0 && c.y == 4.0);
}

void test_vector2_scalar_multiply() {
    Vector2 a{2.0, 3.0};
    Vector2 b = a * 2.0;
    assert(b.x == 4.0 && b.y == 6.0);
}

void test_vector2_dot() {
    Vector2 a{1.0, 2.0};
    Vector2 b{3.0, 4.0};
    assert(a.dot(b) == 11.0);
}

void test_vector2_norm() {
    Vector2 a{3.0, 4.0};
    assert(a.norm() == 5.0);
}

void test_vector2_normalized() {
    Vector2 a{3.0, 4.0};
    Vector2 n = a.normalized();
    assert(std::abs(n.x -0.6) < 1e-9);
    assert(std::abs(n.y -0.8) < 1e-9);
}

// ------------------------------
// Vector3 Tests
// ------------------------------

void test_vector3_addition() {
    Vector3 a{1.0, 2.0, 3.0};
    Vector3 b{4.0, 5.0, 6.0};
    Vector3 c = a+b;
    assert(c.x == 5.0 && c.y == 7.0 && c.z == 9.0);
}

void test_vector3_subtraction() {
    Vector3 a{5.0, 7.0, 9.0};
    Vector3 b{1.0, 2.0, 3.0};
    Vector3 c = a - b;
    assert(c.x == 4.0 && c.y == 5.0 && c.z == 6.0);
}

void test_vector3_scalar_multiply() {
    Vector3 a{1.0, 2.0, 3.0};
    Vector3 b = a * 3.0;
    assert(b.x == 3.0 && b.y == 6.0 && b.z == 9.0);
}

void test_vector3_dot() {
    Vector3 a{1.0, 0.0, 0.0};
    Vector3 b{0.0, 1.0, 0.0};
    assert(a.dot(b) == 0.0);
}

void test_vector3_cross() {
    Vector3 a{1.0, 0.0, 0.0};
    Vector3 b{0.0, 1.0, 0.0};
    Vector3 c = a.cross(b);
    assert(c.x == 0.0 && c.y == 0.0 && c.z == 1.0);
}

void test_vector3_norm() {
    Vector3 a{0.0, 3.0, 4.0};
    assert(a.norm() == 5.0);
}

void test_vector3_normalized() {
    Vector3 a{0.0, 3.0, 4.0};
    Vector3 n = a.normalized();
    assert(std::abs(n.y - 0.6) < 1e-9);
    assert(std::abs(n.z - 0.8) < 1e-9);
}

// -------------------------------------
// Main Test Runner
// -------------------------------------

int main() {
    // Vector 2
    test_vector2_addition();
    test_vector2_subtraction();
    test_vector2_scalar_multiply();
    test_vector2_dot();
    test_vector2_norm();
    test_vector2_normalized();

    // Vector 3
    test_vector3_addition();
    test_vector3_subtraction();
    test_vector3_scalar_multiply();
    test_vector3_dot();
    test_vector3_cross();
    test_vector3_norm();
    test_vector3_normalized();

    std::cout << "All vector tests passed \n";
    return 0;
}
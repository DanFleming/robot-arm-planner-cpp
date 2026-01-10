#pragma once

#include <cmath>
#include <ostream>
#include "vector3.hpp"

namespace math {

struct Matrix3 {
    //Row-major storage
    double m00{1.0}, m01{0.0}, m02{0.0};
    double m10{0.0}, m11{1.0}, m12{0.0};
    double m20{0.0}, m21{0.0}, m22{1.0};

    Matrix3() = default;

    Matrix3(double a00, double a01, double a02,
            double a10, double a11, double a12,
            double a20, double a21, double a22)
        : m00(a00), m01(a01), m02(a02),
          m10(a10), m11(a11), m12(a12),
          m20(a20), m21(a21), m22(a22) {}

    //Identity matrix
    static Matrix3 identity() {
        return Matrix3(
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        );
    }
    
    //Matrix * Vector3
    Vector3 operator*(const Vector3& v) const {
        return {
            m00 * v.x + m01 * v.y + m02 * v.z,
            m10 * v.x + m11 * v.y + m12 * v.z,
            m20 * v.x + m21 * v.y + m22 * v.z
        };
    }

    //Matrix * Matrix
    Matrix3 operator*(const Matrix3& o) const {
        return {
            m00 * o.m00 + m01 * o.m10 + m02 * o.m20,
            m00 * o.m01 + m01 * o.m11 + m02 * o.m21,
            m00 * o.m02 + m01 * o.m12 + m02 * o.m22,

            m10 * o.m00 + m11 * o.m10 + m12 * o.m20,
            m10 * o.m01 + m11 * o.m11 + m12 * o.m21,
            m10 * o.m02 + m11 * o.m12 + m12 * o.m22,

            m20 * o.m00 + m21 * o.m10 + m22 * o.m20,
            m20 * o.m01 + m21 * o.m11 + m22 * o.m21,
            m20 * o.m02 + m21 * o.m12 + m22 * o.m22
        };
    }

    //Determinant
    double det() const {
        return
            m00 * (m11 * m22 - m12 * m21) -
            m01 * (m10 * m22 - m12 * m20) +
            m02 * (m10 * m21 - m11 * m20);
    }

    //Transpose
    Matrix3 transpose() const {
        return {
            m00, m10, m20,
            m01, m11, m21,
            m02, m12, m22
        };
    }

    //Rotation about x axis
    static Matrix3 rotation_x(double t) {
        double c = std::cos(t), s = std::sin(t);
        return {
            1,  0,  0,
            0,  c, -s,
            0,  s,  c
        };
    }

    //Rotation about y axis
    static Matrix3 rotation_y(double t) {
        double c = std::cos(t), s = std::sin(t);
        return {
            c, 0, s,
            0, 1, 0,
           -s, 0, c
        };
    }

    //Rotation about z axis
    static Matrix3 rotation_z(double t) {
        double c = std::cos(t), s = std::sin(t);
        return {
            c, -s,  0,
            s,  c,  0,
            0,  0,  1
        };
    }
};

//Pretty printing
inline std::ostream& operator<<(std::ostream& os, const Matrix3& M) {
    return os << "[[" << M.m00 << ", " << M.m01 << ", " << M.m02 << "], "
              << "[" << M.m10 << ", " << M.m11 << ", " << M.m12 << "], "
              << "[" << M.m20 << ", " << M.m21 << ", " << M.m22 << "]]";
}

} //namespace math
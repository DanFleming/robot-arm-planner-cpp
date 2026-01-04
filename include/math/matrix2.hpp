#pragma once

#include <cmath>
#include <ostream>
#include "vector2.hpp"

namespace math {

struct Matrix2 {
    //Row-major storage
    double m00{1.0}, m01{0.0};
    double m10{0.0}, m11{1.0};

    Matrix2() = default;

    Matrix2(double a00, double a01,
            double a10, double a11)
        : m00(a00), m01(a01),
          m10(a10), m11(a11) {}

    //Matrix * Vector2
    Vector2 operator*(const Vector2& v) const {
        return {
            m00 * v.x + m01 * v.y,
            m10 * v.x + m11 * v.y
        };
    }    

    //Matrix * Matrix
    Matrix2 operator*(const Matrix2& other) const {
        return {
            m00 * other.m00 + m01 * other.m10,
            m00 * other.m01 + m01 * other.m11,

            m10 * other.m00 + m11 * other.m10,
            m10 * other.m01 + m11 * other.m11
        };
    }

    //Determinant
    double det() const {
        return m00 * m11 - m01 * m10;
    }

    //Transpose
    Matrix2 transpose() const {
        return {
            m00, m10,
            m01, m11
        };
    }

    //Rotation matrix factory
    static Matrix2 rotation(double theta) {
        double c = std::cos(theta);
        double s = std::sin(theta);
        return { c, -s,
                 s, c };
    }
};

//Pretty printing
inline std::ostream& operator<<(std::ostream& os, const Matrix2& M) {
    return os << "[[" << M.m00 << ", " << M.m01 << "], "
              << "[" << M.m10 << ", " << M.m11 << "]]";
}

} //namespace math
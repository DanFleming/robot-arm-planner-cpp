#pragma once
#include <cmath>
#include <ostream>

namespace math {

struct Vector2 {
    double x{0.0};
    double y{0.0};

    Vector2() = default;
    Vector2(double x_, double y_) : x(x_), y(y_) {}

    // Addition
    Vector2 operator+(const Vector2& other) const {
        return {x + other.x, y + other.y};
    }

    // Subtraction
    Vector2 operator-(const Vector2& other) const {
        return {x - other.x, y - other.y};
    }

    // Scalar multiply
    Vector2 operator*(double s) const {
        return {x * s, y * s};
    }

    // Dot product
    double dot(const Vector2& other) const {
        return x * other.x + y * other.y;
    }

    // Norm (length)
    double norm() const {
        return std::sqrt(x*x + y * y);
    }

    //Normalised vector
    Vector2 normalized() const {
        double n = norm();
        return (n > 0.0) ? Vector2{x / n, y/n} : Vector2{0.0, 0.0};
   
    }
};

// Pretty printing
inline std::ostream& operator<<(std::ostream& os, const Vector2& v) {
    return os << "(" << v.x << ", " << v.y << ")";
}

} // end namespace math
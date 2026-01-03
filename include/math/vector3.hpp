#pragma once

#include <cmath>
#include <ostream>

namespace math {

struct Vector3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    Vector3() = default;
    Vector3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    //Addition
    Vector3 operator+(const Vector3& other) const {
        return {x + other.x, y + other.y, z + other.z};
    }

    //Subtraction
    Vector3 operator-( const Vector3& other) const {
        return {x - other.x, y - other.y, z - other.z};
    }

    //Scalar multiply
    Vector3 operator*(double s) const {
        return {x * s, y * s, z * s};
    }

    //Dot product
    double dot(const Vector3& other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    //Cross product
    Vector3 cross(const Vector3& other) const {
        return {
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        };
    }

    //Norm (length)
    double norm() const {
        return std::sqrt(x * x + y * y + z * z);
    }

    //Normalized vector
    Vector3 normalized() const {
        double n = norm();
        return (n > 0.0) ? Vector3{x / n, y / n, z / n} 
        : Vector3{0.0, 0.0, 0.0};    
    }
};

//Pretty printing
inline std::ostream& operator<<(std::ostream& os, const Vector3& v) {
    return os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
}

} //namespace math
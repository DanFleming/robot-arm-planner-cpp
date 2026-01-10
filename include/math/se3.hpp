#pragma once

#include "vector3.hpp"
#include "matrix3.hpp"
#include <ostream>

namespace math {

struct SE3 {
    Matrix3 R; //rotation
    Vector3 t; //translation

    //constructors
    SE3() = default;

    SE3(const Matrix3& R_in, const Vector3& t_in)
        : R(R_in), t(t_in) {}

    static SE3 from_rotation_translation(const Matrix3& R_in, const Vector3& t_in) {
        return SE3(R_in, t_in);
    }

    SE3 operator*(const SE3& other) const {
        SE3 result;
        result.R = R * other.R;
        result.t = R * other.t + t;
        return result;
    }

    //inverse
    SE3 inverse() const {
        Matrix3 Rt = R.transpose();
        return SE3(Rt, Rt * (t * -1.0));
    }

    //transform a point
    Vector3 operator*(const Vector3& p) const {
        return R * p + t;
    }
};

//Pretty printing
inline std::ostream& operator<<(std::ostream& os, const SE3& T) {
    return os << "SE3(R=" << T.R << ", t=" << T.t << ")";
}

} //namespace math
#pragma once

#include "vector2.hpp"
#include "matrix2.hpp"
#include <ostream>

namespace math {

struct SE2 {
    Matrix2 R; //rotation
    Vector2 t; //translation

    SE2() : R(Matrix2::identity()), t{0.0, 0.0} {}
    
    //constructors
    SE2(const Matrix2& R_in, const Vector2& t_in)
        : R(R_in), t(t_in) {}

    static SE2 from_angle_translation(double theta, const Vector2& t_in) {
        return SE2(Matrix2::rotation(theta), t_in);
    }

    //composition
    SE2 operator*(const SE2& other) const {
        SE2 result;
        result.R = R * other.R;
        result.t = R * other.t + t;
        return result;
    }

    //inverse
    SE2 inverse() const {
        Matrix2 Rt = R.transpose();
        return SE2(Rt, Rt * (t * -1.0));
    }

    //transform point
    Vector2 operator*(const Vector2& p) const {
        return R * p + t;
    }
};

//Pretty printing
inline std::ostream& operator<<(std::ostream& os, const SE2& T) {
    return os << "SE2(R=" << T.R << ", t=" << T.t << ")";
}

} //namespace math
#pragma once

#include "math/se2.hpp"
#include <vector>

namespace robot {

struct RobotArm2d {
    std::vector<double> link_lengths;

    RobotArm2d(std::initializer_list<double> lengths)
        : link_lengths(lengths) {}

    math::SE2 forward_kinematics(const std::vector<double>& q) const {
        using math::SE2;
        using math::Vector2;

        SE2 T; //identity transform

        for(size_t i = 0; i < link_lengths.size(); ++i) {
            double theta = q[i];
            double L = link_lengths[i];

            //Rotate by angle
            SE2 joint = SE2::from_angle_translation(theta, Vector2{0.0, 0.0});

            //Translate along the link
            SE2 link = SE2::from_angle_translation(0.0, Vector2{L, 0.0});

            //Compose SE2
            T = T * joint * link;
        }

        return T;
    } 
}; //struct closing
} //namespace robot
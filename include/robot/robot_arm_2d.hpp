#pragma once

#include "math/se2.hpp"
#include <vector>
#include <cassert>

namespace robot {

struct RobotArm2d {
    std::vector<double> link_lengths;

    explicit RobotArm2d(std::initializer_list<double> lengths)
        : link_lengths(lengths) {}

    math::SE2 forward_kinematics(const std::vector<double>& q) const {
        using math::SE2;
        using math::Vector2;

        assert(q.size() == link_lengths.size());

        SE2 T; //identity transform
        double theta_total = 0.0;

        for(size_t i = 0; i < link_lengths.size(); ++i) {
            theta_total += q[i];
            double L = link_lengths[i];

            //Rotate by cumulative joint angle
            SE2 joint = SE2::from_angle_translation(theta_total, Vector2{0.0, 0.0});
            //Translate along the link in the rotated frame
            SE2 link = SE2::from_angle_translation(0.0, Vector2{L, 0.0});

            //Compose SE2
            T = T * joint * link;
        }

        return T;
    }
    
    //world-frame positions of each joint origin
    std::vector<math::Vector2> joint_positions(const std::vector<double>& q) const {
        using math::SE2;
        using math::Vector2;

        assert(q.size() == link_lengths.size());

        std::vector<Vector2> positions;
        positions.reserve(q.size());

        SE2 T; //identity
        double theta_total = 0.0;

        for (size_t i = 0; i < link_lengths.size(); ++i) {
            theta_total += q[i];
            double L = link_lengths[i];

            SE2 joint = SE2::from_angle_translation(theta_total, Vector2{0.0, 0.0});
            SE2 link = SE2::from_angle_translation(0.0, Vector2{L, 0.0});

            // move to joint i frame
            T = T * joint;

            // store joint origin before translating along the link
            positions.push_back(T * Vector2{0.0, 0.0});

            // advance to end of link i
            T = T * link;
        }

        return positions;
    }

    // 2xN Jacobian represented as N column vectors (each Vector2 is a column)
    std::vector<math::Vector2> jacobian(const std::vector<double>& q) const {
        using math::Vector2;

        assert(q.size() == link_lengths.size());

        // joint positions and end-effector position
        auto joints = joint_positions(q);
        auto T_end = forward_kinematics(q);
        Vector2 p_end = T_end * Vector2{0.0, 0.0};

        std::vector<Vector2> J;
        J.reserve(q.size());

        for (size_t j = 0; j < q.size(); ++j) {
            const Vector2& pj = joints[j];

            // r = p_end - pj
            Vector2 r{
                p_end.x - pj.x,
                p_end.y - pj.y
            };

            //z-hat cross r -> (-r_y, r_x)
            Vector2 col{
                -r.y,
                r.x
            };

            J.push_back(col);
         }
         return J;
    }
};

}//namespace robot
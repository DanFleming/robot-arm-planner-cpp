#pragma once

#include <vector>
#include <cmath>

#include "robot/robot_arm_2d.hpp"
#include "robot/jacobian_2d.hpp"
#include "math/se2.hpp"

namespace robot {

struct IK2d {
    static std::vector<double>
    solve(const RobotArm2d& arm,
          const math::Vector2& target,
          const std::vector<double>& q0,
          double tol = 1e-6,
          int max_iters = 100,
          double alpha = 1.0,
          double lambda = 0.1)
    {
        std::vector<double> q = q0;
        size_t N = q.size();

        for (int iter = 0; iter < max_iters; ++iter) {

            auto T = arm.forward_kinematics(q);
            math::Vector2 p = T * math::Vector2{0.0, 0.0};

            double ex = target.x - p.x;
            double ey = target.y - p.y;

            if (std::sqrt(ex*ex + ey*ey) < tol)
                return q;

            //previous incompatible code    
            //auto J = Jacobian2d::compute(arm.link_lengths, q);

            //new compatible code
            auto Jcols = arm.jacobian(q);

            //convert to 2xN matrix
            std::vector<std::vector<double>> J(2, std::vector<double>(N));
            for (size_t i = 0; i < N; ++i) {
                J[0][i] = Jcols[i].x;
                J[1][i] = Jcols[i].y;
            }

            /* debugging
            if (iter == 0) { 
                std::cout << "J row 0: "; 
                for (size_t i = 0; i < N; ++i) std::cout << J[0][i] << " "; 
                std::cout << "\nJ row 1: "; 
                for (size_t i = 0; i < N; ++i) std::cout << J[1][i] << " "; 
                std::cout << "\n"; 
            }*/
            double a = 0.0, b = 0.0, c = 0.0;
            for (size_t i = 0; i < N; ++i) {
                a += J[0][i] * J[0][i];
                b += J[0][i] * J[1][i];
                c += J[1][i] * J[1][i];
            }

            a += lambda * lambda;
            c += lambda * lambda;

            double det = a * c - b * b;
            if (std::abs(det) < 1e-12)
                break;

            // Inverse of 2×2 matrix
            double inv00 =  c / det;
            double inv01 = -b / det;
            double inv10 = -b / det;
            double inv11 =  a / det;

            std::vector<double> dq(N, 0.0);

            double v0 = inv00 * ex + inv01 * ey;
            double v1 = inv10 * ex + inv11 * ey;

            for (size_t i = 0; i < N; ++i) {
                dq[i] = alpha * (J[0][i] * v0 + J[1][i] * v1);
            }

            for (size_t i = 0; i < N; ++i)
                q[i] += dq[i];
        }

        return q;
    }
};

} // namespace robot

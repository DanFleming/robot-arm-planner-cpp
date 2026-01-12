#pragma once

#include <vector>
#include <cmath>

namespace robot {

struct Jacobian2d {
    // computes the 2xN Jacobian for an N-link planar arm
    // link_lengths: [L1, L2, ..., LN]
    // q: joint angles [q1, q2, ..., qN]
    static std::vector<std::vector<double>>
    compute(const std::vector<double>& link_lengths,
            const std::vector<double>& q)
    {
        size_t N = link_lengths.size();

        //Jacobian: 2 rows, N columns
        std::vector<std::vector<double>> J(2, std::vector<double>(N, 0.0));

        //Precompute cumulative angles: theta1, theta1 + theta2, ...
        std::vector<double> cumulative(N);
        double sum = 0.0;
        for (size_t i = 0; i < N; ++i) {
            sum += q[i];
            cumulative[i] = sum;
        }

        //Compute each column of the Jacobian
        for (size_t i = 0; i < N; ++i) {
            double dx = 0.0;
            double dy = 0.0;

            //sum contributions from link i to link N
            for (size_t k = i; k < N; ++k) {
                double L = link_lengths[k];
                double angle = cumulative[k];

                dx += -L * std::sin(angle);
                dy += L * std::cos(angle);
            }

            J[0][i] = dx;
            J[1][i] = dy;
        }
        return J;
    }
};
} //namespace robot
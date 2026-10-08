#include "robot_id/random.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>

int main() {
    robot_id::DeterministicNormalGenerator a(20260910U);
    robot_id::DeterministicNormalGenerator b(20260910U);
    robot_id::DeterministicNormalGenerator c(20260911U);

    constexpr std::array<double, 6> kExpectedUniform = {
        0x1.7f345aa8p-4,
        0x1.37d16b0cp-3,
        0x1.d3d69dbbp-1,
        0x1.5f3eb742p-2,
        0x1.098fbbc5p-1,
        0x1.7af52de3p-1,
    };

    bool same = true;
    bool different = false;
    std::array<double, 6> ua{};
    for (std::size_t i = 0; i < ua.size(); ++i) {
        ua[i] = a.uniformOpen01();
        const double ub = b.uniformOpen01();
        const double uc = c.uniformOpen01();
        same = same && (ua[i] == ub);
        different = different || (ua[i] != uc);
        if (!(ua[i] > 0.0 && ua[i] < 1.0) || ua[i] != kExpectedUniform[i]) {
            std::cerr << "uniform golden mismatch at " << i << '\n';
            return 1;
        }
    }

    // The target environment is frozen to Linux x86_64 + glibc. These values
    // guard accidental changes to the project's Box-Muller implementation.
    // A tiny tolerance is used because log/sqrt/sin/cos are supplied by libm.
    constexpr std::array<double, 12> kExpectedGaussian = {
         0x1.00e6e8096d884p-2,
         0x1.6c572fbddc5f8p-2,
        -0x1.7feecd275462ep-5,
         0x1.22330e5e5ef1bp-4,
        -0x1.d05ea167c2fb2p-7,
        -0x1.d4706bdf5714p-3,
        -0x1.0afb05f7544a7p-2,
         0x1.db8b3a43e34a8p-4,
         0x1.dfe1784299452p-3,
        -0x1.329c1dcc44e8bp-3,
         0x1.5fd4dcd287de9p-8,
        -0x1.1b803fbf1356cp-3,
    };

    robot_id::DeterministicNormalGenerator golden(20260910U);
    double max_golden_diff = 0.0;
    for (std::size_t i = 0; i < kExpectedGaussian.size(); ++i) {
        const double actual = golden.sample(0.0, 0.20);
        max_golden_diff = std::max(max_golden_diff, std::abs(actual - kExpectedGaussian[i]));
    }

    robot_id::DeterministicNormalGenerator g1(20260910U);
    robot_id::DeterministicNormalGenerator g2(20260910U);
    double max_pair_diff = 0.0;
    for (int i = 0; i < 1000; ++i) {
        max_pair_diff = std::max(max_pair_diff,
            std::abs(g1.sample(0.0, 0.20) - g2.sample(0.0, 0.20)));
    }

    std::cout << "uniform golden=PASS"
              << ", Gaussian golden max diff=" << max_golden_diff
              << ", same-seed Gaussian pair max diff=" << max_pair_diff << '\n';

    constexpr double kGoldenTolerance = 2e-15;
    return (same && different && max_pair_diff == 0.0 &&
            max_golden_diff <= kGoldenTolerance) ? 0 : 1;
}

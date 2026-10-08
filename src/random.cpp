#include "robot_id/random.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace robot_id {
namespace {
constexpr double kTwoPi = 6.283185307179586476925286766559;
constexpr double kTwoTo32 = 4294967296.0;  // exactly representable in binary64
}  // namespace

DeterministicNormalGenerator::DeterministicNormalGenerator(std::uint32_t seed)
    : engine_(seed) {}

double DeterministicNormalGenerator::uniformOpen01() {
    // std::mt19937 has a standardized 32-bit output sequence. The +0.5 mapping
    // avoids exact 0 and 1, which keeps Box-Muller away from log(0).
    return (static_cast<double>(engine_()) + 0.5) / kTwoTo32;
}

double DeterministicNormalGenerator::sample(double mean, double stddev) {
    if (!std::isfinite(mean) || !std::isfinite(stddev) || stddev < 0.0) {
        throw std::runtime_error("Gaussian mean/stddev must be finite and stddev non-negative.");
    }
    if (stddev == 0.0) {
        return mean;
    }

    if (has_spare_) {
        has_spare_ = false;
        return mean + stddev * spare_;
    }

    const double u1 = uniformOpen01();
    const double u2 = uniformOpen01();
    const double radius = std::sqrt(-2.0 * std::log(u1));
    const double angle = kTwoPi * u2;
    const double z0 = radius * std::cos(angle);
    spare_ = radius * std::sin(angle);
    has_spare_ = true;
    return mean + stddev * z0;
}

}  // namespace robot_id

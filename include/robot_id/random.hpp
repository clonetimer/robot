#pragma once

#include <cstdint>
#include <random>

namespace robot_id {

// Deterministic Gaussian sampler built on the standardized std::mt19937 engine.
// We intentionally do not use std::normal_distribution because the C++ standard
// does not require identical sample sequences across standard-library implementations.
class DeterministicNormalGenerator {
public:
    explicit DeterministicNormalGenerator(std::uint32_t seed);

    // Return N(mean, stddev^2). stddev must be finite and non-negative.
    double sample(double mean, double stddev);

    // Exposed for reproducibility tests. Returns a value strictly inside (0,1).
    double uniformOpen01();

private:
    std::mt19937 engine_;
    bool has_spare_{false};
    double spare_{0.0};
};

}  // namespace robot_id

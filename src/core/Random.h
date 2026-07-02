#pragma once

#include <random>

// A uniformly random index in [0, size). The generator is static so it is
// seeded once. Caller must ensure size > 0.
inline size_t randomIndex(size_t size)
{
    static std::mt19937 gen(std::random_device{}());
    return std::uniform_int_distribution<size_t>(0, size - 1)(gen);
}

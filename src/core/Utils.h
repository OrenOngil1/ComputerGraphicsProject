#pragma once
#include <random>

size_t randomIndex(size_t size) {
    static std::mt19937 gen(std::random_device{}());
    return std::uniform_int_distribution<size_t>(0, size - 1)(gen);
}
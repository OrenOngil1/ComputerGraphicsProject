// Shared PASS/FAIL helper for the headless checks, split by topic across
// tests/*.cpp and driven by one main() in main.cpp. Exit code = number of
// failed checks, so CI-style use works.
#pragma once

#include <iostream>
#include <string>

inline int failures = 0;

inline void check(bool ok, const std::string &name)
{
    std::cout << (ok ? "  PASS  " : "  FAIL  ") << name << std::endl;
    if (!ok)
        failures++;
}

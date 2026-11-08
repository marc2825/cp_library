#pragma once
#include "computational_geometry/all.hpp"
#include <iostream>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#define CHECK(x)                                                                                             \
    do {                                                                                                     \
        if (!(x))                                                                                            \
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__) + ": " + #x);    \
    } while (false)
namespace htest {
namespace r = cp::geometry::rational;
namespace e = cp::geometry::exact;
namespace f = cp::geometry::floating;
namespace robust = cp::geometry::robust;
namespace s = cp::geometry::spatial;
using cp::BigInt;
using cp::BigRational;
using cp::geometry::Location;
inline void near(long double a, long double b, long double error = 1e-8L) {
    CHECK(std::abs(a - b) <= error * (1 + std::abs(b)));
}
template <class E, class F> void throws(F fn) {
    bool caught = false;
    try {
        fn();
    } catch (const E &) {
        caught = true;
    }
    CHECK(caught);
}
inline int random_int(std::mt19937_64 &rng, int low, int high) {
    return low + int(rng() % unsigned(high - low + 1));
}
inline r::Point random_point(std::mt19937_64 &rng, int bound = 5) {
    return {random_int(rng, -bound, bound), random_int(rng, -bound, bound)};
}
inline BigRational triangle_area(const r::Point &a, const r::Point &b, const r::Point &c) {
    return cp::abs(r::cross(a, b, c)) / 2;
}
} // namespace htest

#pragma once
#include <optional>
#include <vector>
#include "rotating_calipers.hpp"
namespace cp { namespace geometry {
namespace exact {inline std::optional<PairResult> closest_pair(const std::vector<Point>&points){return detail::closest(points,Kernel{});}}
namespace floating {inline std::optional<PairResult> closest_pair(const std::vector<Point>&points){return detail::closest(points,Kernel(Tolerance(0)));}}
} }

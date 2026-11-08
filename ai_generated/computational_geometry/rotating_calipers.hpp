#pragma once
#include <optional>
#include <vector>
#include "convex_hull.hpp"
namespace cp { namespace geometry {
namespace exact {using PairResult=detail::PairResult<Wide>;inline std::optional<PairResult> convex_diameter(const std::vector<Point>&strict_hull){return detail::diameter(strict_hull,Kernel{});}}
namespace floating {using PairResult=detail::PairResult<long double>;inline std::optional<PairResult> convex_diameter(const std::vector<Point>&strict_hull){return detail::diameter(strict_hull,Kernel(Tolerance(0)));}}
} }

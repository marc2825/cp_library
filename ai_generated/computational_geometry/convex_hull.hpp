#pragma once
#include <utility>
#include <vector>
#include "exact/kernel.hpp"
#include "floating/kernel.hpp"
#include "detail/algorithms.hpp"
namespace cp { namespace geometry {
namespace exact {inline std::vector<Point> convex_hull(std::vector<Point> p,bool keep_collinear=false){return detail::hull(std::move(p),keep_collinear,Kernel{});}}
namespace floating {inline std::vector<Point> convex_hull(std::vector<Point> p,bool keep_collinear=false,Tolerance t=Tolerance()){return detail::hull(std::move(p),keep_collinear,Kernel(t));}}
} }

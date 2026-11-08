#pragma once
#include "exact/point.hpp"
#include "floating/point.hpp"
namespace cp { namespace geometry {
inline floating::Point to_floating(exact::Point p){return {static_cast<long double>(p.x),static_cast<long double>(p.y)};}
inline floating::Point to_floating(const exact::RationalPoint&p){return {p.x.approximate(),p.y.approximate()};}
} }

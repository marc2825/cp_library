#pragma once
#include <algorithm>
#include <optional>
#include <vector>
#include "rotating_calipers.hpp"
namespace cp { namespace geometry {
namespace exact {
// 入力点群の元添字で最遠点対。凸包+既存キャリパー。空はnullopt。
inline std::optional<PairResult> farthest_pair(const std::vector<Point>&points){
    const auto hull=convex_hull(points);const auto diameter=convex_diameter(hull);if(!diameter)return std::nullopt;
    PairResult result=*diameter;result.first=std::size_t(std::find(points.begin(),points.end(),hull[diameter->first])-points.begin());result.second=std::size_t(std::find(points.begin(),points.end(),hull[diameter->second])-points.begin());return result;
}
}
namespace floating {
inline std::optional<PairResult> farthest_pair(const std::vector<Point>&points){
    const auto hull=convex_hull(points,false,Tolerance(0));const auto diameter=convex_diameter(hull);if(!diameter)return std::nullopt;
    PairResult result=*diameter;result.first=std::size_t(std::find(points.begin(),points.end(),hull[diameter->first])-points.begin());result.second=std::size_t(std::find(points.begin(),points.end(),hull[diameter->second])-points.begin());return result;
}
}
} }

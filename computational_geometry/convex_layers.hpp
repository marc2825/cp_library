#pragma once
#include <algorithm>
#include <cstddef>
#include <numeric>
#include <vector>
#include "convex_hull.hpp"

namespace cp { namespace geometry { namespace exact {
// 全境界点を1層として取り除く。重複座標は同じ層。0始まりの層番号。
inline std::vector<int> convex_layers(const std::vector<Point>&points){
    std::vector<int> layer(points.size());std::vector<std::size_t> active(points.size());std::iota(active.begin(),active.end(),0);int depth=0;
    while(!active.empty()){
        std::vector<Point> remaining;for(auto i:active)remaining.push_back(points[i]);auto boundary=convex_hull(std::move(remaining),true);std::sort(boundary.begin(),boundary.end());
        std::vector<std::size_t> next;for(auto i:active){if(std::binary_search(boundary.begin(),boundary.end(),points[i]))layer[i]=depth;else next.push_back(i);}active=std::move(next);++depth;
    }
    return layer;
}
} } } // namespace cp::geometry::exact

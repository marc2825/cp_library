#pragma once
// 説明: docs/graph_basic/manhattan_mst.md
#include <algorithm>
#include <map>
#include <numeric>
#include <utility>
#include <vector>
#include "kruskal.hpp"
#include "../combinatorial_optimization/detail/arithmetic.hpp"

namespace cp {
// 4方向のsweepで候補をO(N)本に絞る。KACTL (CC0), chilli/Takanori Maeharaの方法。
// https://github.com/kth-competitive-programming/kactl/blob/main/content/geometry/ManhattanMST.h
inline MinimumSpanningForestResult<long long> manhattan_mst(const std::vector<std::pair<long long,long long>>& points) {
    using W=OptimizationWide;
    int n=int(points.size()); std::vector<std::pair<W,W>> transformed;
    for(auto p:points) transformed.emplace_back(W(p.first),W(p.second));
    std::vector<int> order(n); std::iota(order.begin(),order.end(),0); Graph<long long> candidates(n);
    for(int direction=0;direction<4;++direction) {
        std::sort(order.begin(),order.end(),[&](int a,int b) {
            W x=transformed[a].first+transformed[a].second,y=transformed[b].first+transformed[b].second;
            return x!=y?x<y:a<b;
        });
        std::map<W,int> frontier;
        for(int i:order) {
            const auto [x,y]=transformed[i]; auto it=frontier.lower_bound(-y);
            while(it!=frontier.end()) {
                int j=it->second; W dx=x-transformed[j].first,dy=y-transformed[j].second;
                if(dx<dy) break;
                candidates.add_edge(i,j,optimization_detail::narrow<long long>(dx+dy)); it=frontier.erase(it);
            }
            frontier[-y]=i;
        }
        for(auto& p:transformed) { if(direction&1) p.first=-p.first; else std::swap(p.first,p.second); }
    }
    return kruskal(candidates);
}
} // namespace cp

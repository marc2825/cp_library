#pragma once
// 説明: docs/tree/tree_distance_distribution.md
#include <algorithm>
#include <vector>
#include "contour_query.hpp"
#include "../convolution/convolution_ll.hpp"
namespace cp {
// 無向森の異なる頂点の非順序対を辺数距離ごとに数える。別成分は数えない。
template<class W> std::vector<long long> tree_distance_distribution(const Graph<W>& g){
    ContourIndex index(g);std::vector<std::vector<long long>> all,sub;
    for(int size:index.all_size)all.emplace_back(size,0);
    for(int size:index.branch_size)sub.emplace_back(size,0);
    for(const auto& row:index.records)for(auto a:row){++all[a.centroid][a.distance];if(a.branch!=-1)++sub[a.branch][a.distance];}
    std::vector<long long> answer(g.size(),0);
    auto accumulate=[&](const std::vector<long long>& a,int sign){auto c=convolution_ll(a,a);for(std::size_t i=1;i<answer.size() && i<c.size();++i)answer[i]+=sign*c[i];};
    for(const auto& a:all)accumulate(a,1);
    for(const auto& a:sub)accumulate(a,-1);
    for(auto& x:answer)x/=2;
    return answer;
}
} // namespace cp

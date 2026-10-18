#pragma once
#include <algorithm>
#include <climits>
#include <cstdint>
#include <functional>
#include <numeric>
#include <stdexcept>
#include <vector>
#include "detail/arithmetic.hpp"
#include "../graph/graph.hpp"
namespace cp {
struct WeightedIndependentSetResult {OptimizationWide weight=0;std::vector<int> selected;};
// 1つのマトロイドの独立集合。重み降順の貪欲法、負/零重みは選ばない。
template <class Oracle> WeightedIndependentSetResult maximum_weight_matroid_independent_set(const std::vector<long long>&weights,Oracle independent){
    if(weights.size()>static_cast<std::size_t>(INT_MAX))throw std::length_error("too many weighted matroid elements");
    if(!independent(std::vector<int>{}))throw std::invalid_argument("matroid must contain empty set");
    std::vector<int> order(weights.size());std::iota(order.begin(),order.end(),0);
    std::sort(order.begin(),order.end(),[&](int a,int b){return weights[a]!=weights[b]?weights[a]>weights[b]:a<b;});
    WeightedIndependentSetResult result;
    for(int e:order)if(weights[e]>0){auto candidate=result.selected;candidate.push_back(e);if(independent(candidate)){result.selected.push_back(e);result.weight=optimization_detail::add(result.weight,weights[e]);}}
    std::sort(result.selected.begin(),result.selected.end());return result;
}
// 一般の無向グラフは別問題。64頂点までの厳密分枝限定、最悪指数時間。
template <class Cost> WeightedIndependentSetResult maximum_weight_independent_set(const Graph<Cost>&graph,const std::vector<long long>&weights){
    const int n=graph.size();if(graph.directed()||n>64||weights.size()!=static_cast<std::size_t>(n))throw std::invalid_argument("weighted graph MIS needs an undirected graph of <=64 vertices");
    using Mask=std::uint64_t;std::vector<Mask> adjacent(n);Mask candidates=0;
    for(int v=0;v<n;++v)if(weights[v]>0)candidates|=Mask(1)<<v;
    for(const auto&e:graph.edges()){if(e.from==e.to)candidates&=~(Mask(1)<<e.from);else{adjacent[e.from]|=Mask(1)<<e.to;adjacent[e.to]|=Mask(1)<<e.from;}}
    WeightedIndependentSetResult best;std::vector<int> current;
    const auto popcount=[](Mask mask){int k=0;for(;mask;mask&=mask-1)++k;return k;};
    std::function<void(Mask,OptimizationWide)> search=[&](Mask active,OptimizationWide score){
        OptimizationWide bound=score;int vertex=-1,degree=-1;
        for(int v=0;v<n;++v)if(active&(Mask(1)<<v)){bound+=weights[v];const int d=popcount(active&adjacent[v]);if(d>degree){degree=d;vertex=v;}}
        if(bound<=best.weight)return;
        if(vertex<0){if(score>best.weight){best.weight=score;best.selected=current;}return;}
        if(degree==0){auto selected=current;for(int v=0;v<n;++v)if(active&(Mask(1)<<v))selected.push_back(v);best={bound,std::move(selected)};return;}
        const Mask rest=active&~(Mask(1)<<vertex);current.push_back(vertex);search(rest&~adjacent[vertex],score+weights[vertex]);current.pop_back();search(rest,score);
    };
    search(candidates,0);std::sort(best.selected.begin(),best.selected.end());return best;
}
} // namespace cp

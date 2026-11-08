#pragma once
// 説明: docs/dp_optimization/branch_bound_knapsack.md
#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <vector>
#include "knapsack_common.hpp"
#include "../combinatorial_optimization/detail/arithmetic.hpp"
namespace cp {
struct BranchBoundKnapsackResult {long long value=0,weight=0;std::vector<int> items;std::size_t visited_nodes=0;};
// 0-1ナップサック。密度順のfractional LP上界で枝刈り。最悪指数時間。
inline BranchBoundKnapsackResult branch_bound_knapsack(long long capacity,const std::vector<KnapsackItem>& items){
    if(capacity<0)throw std::invalid_argument("negative capacity");
    using W=OptimizationWide;
    struct Item{int weight,id;long long value;};std::vector<Item> sorted;
    struct Choice{int id,parent;};std::vector<Choice> choices;int initial_path=-1;W initial=0;
    for(int i=0;i<static_cast<int>(items.size());++i){auto x=items[i];if(x.weight<0)throw std::invalid_argument("negative item weight");
    if(x.value<=0){continue;}if(x.weight==0){initial=optimization_detail::add(initial,W(x.value));choices.push_back({i,initial_path});initial_path=static_cast<int>(choices.size())-1;}else if(x.weight<=capacity)sorted.push_back({x.weight,i,x.value});}
    std::sort(sorted.begin(),sorted.end(),[](auto a,auto b){W left=W(a.value)*b.weight,right=W(b.value)*a.weight;return left!=right?left>right:a.id<b.id;});
    std::vector<W> weights(sorted.size()+1),values(sorted.size()+1);for(std::size_t i=0;i<sorted.size();++i){weights[i+1]=weights[i]+sorted[i].weight;values[i+1]=optimization_detail::add(values[i],W(sorted[i].value));}
    auto bound=[&](int i,long long weight,W value){W remaining=W(capacity)-weight;auto it=std::upper_bound(weights.begin()+i,weights.end(),weights[i]+remaining);int end=static_cast<int>(it-weights.begin())-1;W result=value+values[end]-values[i];if(end<static_cast<int>(sorted.size()))result+=(remaining-(weights[end]-weights[i]))*sorted[end].value/sorted[end].weight;return result;};
    struct State{int index,path;long long weight;W value;};std::vector<State> stack{{0,initial_path,0,initial}};W best=initial;int best_path=initial_path;long long best_weight=0;std::size_t visited=0;
    while(!stack.empty()){auto s=stack.back();stack.pop_back();++visited;if(s.value>best){best=s.value;best_path=s.path;best_weight=s.weight;}if(s.index==static_cast<int>(sorted.size()) || bound(s.index,s.weight,s.value)<=best)continue;auto item=sorted[s.index];stack.push_back({s.index+1,s.path,s.weight,s.value});if(item.weight<=capacity-s.weight){choices.push_back({item.id,s.path});stack.push_back({s.index+1,static_cast<int>(choices.size())-1,s.weight+item.weight,optimization_detail::add(s.value,W(item.value))});}}
    BranchBoundKnapsackResult result{optimization_detail::narrow<long long>(best),best_weight,{},visited};for(int p=best_path;p!=-1;p=choices[p].parent)result.items.push_back(choices[p].id);std::sort(result.items.begin(),result.items.end());return result;
}
} // namespace cp

#pragma once
// 説明: docs/dp_optimization/monge_shortest_path_k_edges.md
#include <algorithm>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>
#include "smawk.hpp"
#include "detail/arithmetic.hpp"
namespace cp {
struct MongeKEdgePathResult {long long cost;std::vector<int> vertices;};
// 0..nの完全前向きMonge DAG。ちょうどk辺。SMAWKをk段に適用し復元する。
template<class Cost> std::optional<MongeKEdgePathResult> monge_shortest_path_k_edges(int n,int k,Cost cost){
    if(n<0 || k<0)throw std::invalid_argument("negative Monge dimension");
    if(k>n || (k==0 && n))return std::nullopt;
    if(k==0)return MongeKEdgePathResult{0,{0}};
    std::vector<long long> previous(static_cast<std::size_t>(n)+1);std::vector<std::vector<int>> parent(static_cast<std::size_t>(k)+1,std::vector<int>(static_cast<std::size_t>(n)+1,-1));
    for(int v=1;v<=n;++v){previous[v]=cost(0,v);parent[1][v]=0;}
    for(int stage=2;stage<=k;++stage){int size=n-stage+1;auto minima=smawk(size,size,[&](int row,int column){int v=stage+row,p=stage-1+column;if(p>=v)return std::make_pair(true,0LL);return std::make_pair(false,detail::dp_add(previous[p],cost(p,v)));});std::vector<long long> next(static_cast<std::size_t>(n)+1);for(int row=0;row<size;++row){int v=stage+row,p=stage-1+minima[row];parent[stage][v]=p;next[v]=detail::dp_add(previous[p],cost(p,v));}previous=std::move(next);}
    MongeKEdgePathResult result{previous[n],{n}};int v=n;for(int stage=k;stage>0;--stage){v=parent[stage][v];result.vertices.push_back(v);}std::reverse(result.vertices.begin(),result.vertices.end());return result;
}
} // namespace cp

#pragma once
// 説明: docs/dp_optimization/monge_shortest_path.md
#include <algorithm>
#include <stdexcept>
#include <utility>
#include <vector>
#include "larsch.hpp"
#include "detail/arithmetic.hpp"
namespace cp {
struct MongeShortestPathResult {
    std::vector<long long> distance;std::vector<int> parent;
    std::vector<int> path_to(int v)const{if(v<0 || v>=static_cast<int>(parent.size()))throw std::out_of_range("Monge target");
    std::vector<int> p;for(;;v=parent[v]){p.push_back(v);if(v==0)break;}std::reverse(p.begin(),p.end());return p;}
};
// 頂点0..n、全てのi<jに有限辺cost(i,j)。四辺形Monge条件は利用者の前提。
template<class Cost> MongeShortestPathResult monge_shortest_path(int n,Cost cost){
    if(n<0)throw std::invalid_argument("negative Monge size");
    MongeShortestPathResult result{std::vector<long long>(static_cast<std::size_t>(n)+1),std::vector<int>(static_cast<std::size_t>(n)+1)};
    using Entry=std::pair<bool,long long>;
    Larsch<Entry> rows(n,[&](int row,int column)->Entry{if(column>row)return {true,0};return {false,detail::dp_add(result.distance[column],cost(column,row+1))};});
    for(int v=1;v<=n;++v){int p=rows.next_argmin();result.parent[v]=p;result.distance[v]=detail::dp_add(result.distance[p],cost(p,v));}
    return result;
}
} // namespace cp

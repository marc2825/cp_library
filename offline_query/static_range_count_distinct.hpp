#pragma once
// 説明: docs/offline_query/static_range_count_distinct.md
#include <algorithm>
#include <cassert>
#include <functional>
#include <map>
#include <numeric>
#include <utility>
#include <vector>
#include "../range_query/fenwick_tree.hpp"

namespace cp {
template<class T,class Compare=std::less<T>>
std::vector<int> static_range_count_distinct(const std::vector<T>& a,const std::vector<std::pair<int,int>>& queries,Compare comp=Compare{}) {
    std::vector<int> order(queries.size()),answer(queries.size()); std::iota(order.begin(),order.end(),0);
    for(auto [l,r]:queries) { assert(0<=l && l<=r && r<=int(a.size())); (void)l; (void)r; }
    std::stable_sort(order.begin(),order.end(),[&](int x,int y){return queries[x].second<queries[y].second;});
    FenwickTree<int> bit(int(a.size())); std::map<T,int,Compare> last(comp); int r=0;
    for(int id:order) {
        while(r<queries[id].second) {
            auto it=last.find(a[r]); if(it!=last.end()) bit.add(it->second,-1);
            last.insert_or_assign(a[r],r); bit.add(r,1); ++r;
        }
        answer[id]=bit.sum(queries[id].first,queries[id].second);
    }
    return answer;
}
} // namespace cp

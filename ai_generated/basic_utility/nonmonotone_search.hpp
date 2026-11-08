#pragma once
// 説明: docs/basic_utility/nonmonotone_search.md
#include <cstdint>
#include <optional>
#include <queue>
#include <utility>
#include <vector>

namespace cp {
// 任意の非単調関数を有限整数区間[l,r)で最小化する分枝限定法。
// lower_bound(a,b) <= min f([a,b)) が必要。同値は最左。最悪全点評価。
template<class Value,class Function,class LowerBound>
std::optional<std::pair<std::uint64_t,Value>> nonmonotone_argmin(
    std::uint64_t l,std::uint64_t r,Function f,LowerBound lower_bound) {
    struct Node { std::uint64_t l,r; Value bound; };
    auto compare=[](const Node& a,const Node& b) { return b.bound<a.bound; };
    std::priority_queue<Node,std::vector<Node>,decltype(compare)> queue(compare);
    std::optional<std::pair<std::uint64_t,Value>> best;
    if (l<r) queue.push({l,r,lower_bound(l,r)});
    while (!queue.empty()) {
        Node n=queue.top(); queue.pop();
        if (best && (best->second<n.bound || (!(n.bound<best->second) && n.l>=best->first))) continue;
        if (n.r-n.l==1) {
            Value v=f(n.l);
            if (!best || v<best->second || (!(best->second<v) && n.l<best->first)) best=std::make_pair(n.l,v);
        } else {
            auto m=n.l+(n.r-n.l)/2;
            queue.push({n.l,m,lower_bound(n.l,m)}); queue.push({m,n.r,lower_bound(m,n.r)});
        }
    }
    return best;
}
} // namespace cp

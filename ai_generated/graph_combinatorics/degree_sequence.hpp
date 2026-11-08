#pragma once
#include <algorithm>
#include <functional>
#include <optional>
#include <queue>
#include <utility>
#include <vector>
#include "../graph/graph.hpp"

namespace cp {
// Erdos-Gallai; simple undirected graph, including empty graph. O(n log n).
inline bool is_graphical_degree_sequence(std::vector<int> degree) {
    int n=int(degree.size());
    for (int d:degree) if (d<0 || d>=n) return false;
    std::sort(degree.begin(),degree.end(),std::greater<int>());
    std::vector<long long> sum(n+1);
    for (int i=0; i<n; ++i) sum[i+1]=sum[i]+degree[i];
    if (sum[n]%2) return false;
    int j=n;
    for (int k=1; k<=n; ++k) {
        while (j>k && degree[j-1]<k) --j;
        j=std::max(j,k);
        if (sum[k]>1LL*k*(k-1)+1LL*k*(j-k)+sum[n]-sum[j]) return false;
    }
    return true;
}
// Havel-Hakimi; original labels retained. O((n+m) log n), O(n+m) space.
inline std::optional<Graph<int>> realize_degree_sequence(const std::vector<int>& degree) {
    if (!is_graphical_degree_sequence(degree)) return std::nullopt;
    Graph<int> g(int(degree.size()));
    std::priority_queue<std::pair<int,int>> heap;
    for (int v=0; v<g.size(); ++v) if (degree[v]) heap.emplace(degree[v],v);
    while (!heap.empty()) {
        auto [d,v]=heap.top(); heap.pop();
        std::vector<std::pair<int,int>> changed;
        for (int i=0; i<d; ++i) {
            auto [a,u]=heap.top(); heap.pop(); g.add_edge(v,u);
            if (a>1) changed.emplace_back(a-1,u);
        }
        for (auto e:changed) heap.push(e);
    }
    return g;
}
} // namespace cp

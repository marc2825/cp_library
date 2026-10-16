#define PROBLEM "https://atcoder.jp/contests/agc002/tasks/agc002_d"
#include <iostream>
#include <utility>
#include <vector>
#include "offline_query/parallel_binary_search.hpp"
#include "union_find/disjoint_set_union.hpp"

int main() {
    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);
    int n, m, q;
    std::cin >> n >> m;
    std::vector<std::pair<int, int>> edges(m);
    for (auto& [u, v] : edges) { std::cin >> u >> v; --u; --v; }
    std::cin >> q;
    struct Query { int x, y, z; };
    std::vector<Query> queries(q);
    for (auto& [x, y, z] : queries) { std::cin >> x >> y >> z; --x; --y; }
    cp::DisjointSetUnion dsu(n);
    auto answer = cp::parallel_binary_search(m, q, [&] { dsu.reset(n); },
        [&](int i) { dsu.merge(edges[i].first, edges[i].second); },
        [&](int id) {
            auto [x, y, z] = queries[id];
            return dsu.size(x) + (dsu.same(x, y) ? 0 : dsu.size(y)) >= z;
        });
    for (int t : answer) std::cout << t << '\n';
}

#include <iostream>
#include <utility>
#include <vector>
#include "offline_query/parallel_binary_search.hpp"
#include "union_find/disjoint_set_union.hpp"

int main() {
    std::vector<std::pair<int, int>> edges{{0, 1}, {2, 3}, {1, 2}};
    std::vector<std::pair<int, int>> queries{{0, 0}, {0, 3}, {0, 4}};
    cp::DisjointSetUnion dsu(5);
    auto answer = cp::parallel_binary_search(int(edges.size()), int(queries.size()),
        [&] { dsu.reset(5); },
        [&](int i) { dsu.merge(edges[i].first, edges[i].second); },
        [&](int q) { return dsu.same(queries[q].first, queries[q].second); });
    for (int t : answer) std::cout << t << '\n'; // 0, 3, 4 (4 means absent)
}

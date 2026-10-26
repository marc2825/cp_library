#include <iostream>
#include "dp_optimization/bidirectional_search.hpp"
int main() {
    std::vector<std::vector<cp::StateSearchEdge<int>>> forward(5), reverse(5);
    std::vector<std::pair<int,int>> edges{{0,1},{0,2},{1,3},{2,3},{3,4}};
    for (int i = 0; i < int(edges.size()); ++i) {
        auto [u,v] = edges[i]; forward[u].push_back({v,1,i}); reverse[v].push_back({u,1,i});
    }
    auto result = cp::bidirectional_bfs(0,4,[&](int v) { return forward[v]; },[&](int v) { return reverse[v]; });
    std::cout << result->cost;
    for (int v : result->states) std::cout << ' ' << v;
    std::cout << '\n';
}

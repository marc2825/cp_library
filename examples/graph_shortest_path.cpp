// 入力: n m s t、その後 m 行に 0-indexed の有向辺 u v cost (cost >= 0)。
// 出力: 到達不能なら -1。それ以外は距離と、経路上の頂点列。
#include <iostream>
#include "graph/dijkstra.hpp"
#include "graph/input.hpp"

int main() {
    int n, m, s, t;
    if (!(std::cin >> n >> m >> s >> t)) return 0;
    cp::GraphInputOptions options;
    options.kind = cp::GraphKind::Directed;
    options.weighted = true;
    options.index_base = 0;
    auto graph = cp::read_graph<long long>(std::cin, n, m, options);
    auto result = cp::dijkstra(graph, s);
    if (!result.reachable(t)) {
        std::cout << -1 << '\n';
        return 0;
    }
    std::cout << result.dist[t] << '\n';
    auto path = result.vertex_path(t);
    for (int i = 0; i < static_cast<int>(path.size()); ++i)
        std::cout << path[i] << (i + 1 == static_cast<int>(path.size()) ? '\n' : ' ');
}

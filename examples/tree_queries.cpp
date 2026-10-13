// 入力: n q、n-1 行に 0-indexed の無向辺 u v cost、q 行に問い合わせ u v。
// 出力: 根を0とした LCA と重み付き距離。
#include <iostream>
#include "graph/input.hpp"
#include "tree/doubling_lca.hpp"

int main() {
    int n, q;
    if (!(std::cin >> n >> q)) return 0;
    cp::GraphInputOptions options;
    options.weighted = true;
    options.index_base = 0;
    auto graph = cp::read_graph<long long>(std::cin, n, n-1, options);
    cp::DoublingLCA<long long> tree(graph, 0);
    while (q--) {
        int u, v;
        std::cin >> u >> v;
        std::cout << tree.lca(u, v) << ' ' << tree.distance(u, v) << '\n';
    }
}

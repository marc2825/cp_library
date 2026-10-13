// 頂点の文字をパス順に連結する例。順序を持つ演算では reversed を使う。
#include <iostream>
#include <string>
#include "tree/heavy_light_decomposition.hpp"

int main() {
    cp::Graph<int> graph(5);
    graph.add_edge(0, 1);
    graph.add_edge(1, 2);
    graph.add_edge(1, 3);
    graph.add_edge(3, 4);
    cp::HeavyLightDecomposition<int> hld(graph);
    std::string label = "ABCDE", result;
    for (auto segment : hld.path_segments(2, 4)) {
        if (segment.reversed) {
            for (int i = segment.r - 1; i >= segment.l; --i) result += label[hld.vertex_at[i]];
        } else {
            for (int i = segment.l; i < segment.r; ++i) result += label[hld.vertex_at[i]];
        }
    }
    std::cout << result << '\n'; // CBDE
}

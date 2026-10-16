#include <iostream>
#include <set>
#include "tree/dsu_on_tree.hpp"
int main() {
    cp::Graph<int> g(4); g.add_edge(0, 1); g.add_edge(0, 2); g.add_edge(1, 3);
    cp::RootedTree<int> tree(g); std::set<int> bag; int result[4] = {};
    cp::dsu_on_tree(tree, [&](int v) { bag.insert(v); }, [&](int v) { bag.erase(v); },
        [&](int v) { result[v] = int(bag.size()); });
    for (int x : result) std::cout << x << ' ';
    std::cout << '\n';
}

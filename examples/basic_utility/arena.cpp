#include <iostream>
#include <limits>
#include "basic_utility/arena.hpp"
struct Node { int value; std::size_t previous; };
int main() {
    cp::Arena<Node, 2> nodes;
    const auto nil = std::numeric_limits<std::size_t>::max();
    auto common = nodes.emplace(Node{10, nil});
    const auto* saved = &nodes[common];
    auto first = nodes.emplace(Node{20, common});
    auto second = nodes.emplace(Node{30, common});
    std::cout << nodes[common].value << " " << nodes[first].value << " " << nodes[second].value << "\n";
    std::cout << (saved == &nodes[common]) << "\n";
}

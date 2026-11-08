#include <iostream>
#include "tree/rerooting.hpp"
struct State { long long count = 0, sum = 0; };
int main() {
    cp::Graph<> g(4);
    g.add_edge(0, 1); g.add_edge(1, 2); g.add_edge(1, 3);
    auto answer = cp::rerooting(g, State{},
        [](State a, State b) { return State{a.count + b.count, a.sum + b.sum}; },
        [](State a, int) { ++a.count; return a; },
        [](State a, const cp::Edge<long long>& e) { a.sum += a.count * e.cost; return a; });
    for (auto s : answer) std::cout << s.sum << ' ';
    std::cout << '\n';
}

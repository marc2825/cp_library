#include "combinatorial_optimization/base_polyhedron.hpp"
#include <iostream>
using namespace cp;
int main() {
    std::vector<std::vector<int>> covers{{0, 1}, {1, 2}, {2, 3}};
    auto f = [&](const std::vector<int> &s) {
        std::vector<bool> seen(4);
        for (int e : s)
            for (int v : covers[e])
                seen[v] = true;
        int count = 0;
        for (bool x : seen)
            count += x;
        return BigRational(count);
    };
    auto r = base_polyhedron_greedy({3, 2, 1}, f);
    std::cout << r.value << "\n";
    for (std::size_t i = 0; i < r.base.size(); i++)
        std::cout << (i ? " " : "") << r.base[i];
    std::cout << "\n";
}

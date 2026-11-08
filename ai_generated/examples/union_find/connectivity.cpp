#include <iostream>
#include "union_find/disjoint_set_union.hpp"

int main() {
    cp::DisjointSetUnion dsu(5);
    dsu.merge(0, 1);
    dsu.merge(1, 2);
    std::cout << dsu.same(0, 2) << ' ' << dsu.size(1) << ' ' << dsu.component_count() << '\n';
    for (const auto& group : dsu.groups()) {
        for (int i = 0; i < static_cast<int>(group.size()); ++i)
            std::cout << group[i] << (i + 1 == static_cast<int>(group.size()) ? '\n' : ' ');
    }
}

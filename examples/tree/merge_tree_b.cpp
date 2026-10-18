#include <iostream>
#include "tree/merge_tree.hpp"
int main() {
    cp::MergeTree t(3); int a=t.merge(0,1); int b=t.merge(1,2);
    std::cout << a << ' ' << b << ' ' << t.parent[a] << '\n';
}

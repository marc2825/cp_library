#include <iostream>
#include "dp_optimization/li_chao_tree.hpp"
int main() {
    cp::LiChaoTree<> t({0, 1, 2, 3}); t.add_line(2, 1); t.add_line(-1, 5);
    t.add_segment(1, 3, 0, -7);
    std::cout << *t.query(0) << ' ' << *t.query(2) << ' ' << *t.query(3) << '\n';
}

#include <iostream>
#include "range_query/range_tree.hpp"
int main() {
    cp::RangeTree<> s({{1,2},{3,4}}); s.add(0,7); s.add(1,5);
    std::cout << s.sum(0,0,3,4) << "\n";
}

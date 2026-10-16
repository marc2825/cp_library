#include <iostream>
#include "ordered_structure/persistent_order_statistics.hpp"
int main() {
    cp::PersistentOrderStatistics<int> s({5, 1, 4, 1, 3});
    std::cout << *s.kth_smallest(1, 5, 2) << ' ' << s.count_less(0, 5, 4) << '\n';
}

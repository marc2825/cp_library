#include "ordered_structure/compressed_order_statistics.hpp"
#include <iostream>
int main() {
    cp::CompressedOrderStatistics<long long> values({-5, 20, 100});
    values.insert(20, 2); values.insert(100); values.insert(-5);
    std::cout << *values.kth(2) << ' ' << values.order_of_key(20) << '\n'; // 20 1
    std::cout << values.count_less_equal(20) << ' ' << values.count_range(0, 100) << '\n'; // 3 2
    std::cout << *values.predecessor(20) << ' ' << *values.successor(20, true) << '\n'; // -5 20
    std::cout << values.insert(7) << ' ' << values.erase(20, 3) << '\n'; // 0 0 (both leave the multiset unchanged)
    values.erase(20);
    std::cout << values.size() << ' ' << *values.kth(2) << '\n'; // 3 100
}

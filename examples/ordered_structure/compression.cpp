#include "ordered_structure/coordinate_compression.hpp"
#include <iostream>
#include <vector>
int main() {
    std::vector<long long> a{100, -5, 100, 20};
    cp::CoordinateCompression<long long> coordinates(a);
    for (auto x : a) std::cout << *coordinates.index(x) << ' '; // 2 0 2 1
    std::cout << '\n';
    std::cout << coordinates.lower_bound(10) << ' ' << coordinates.upper_bound(20) << '\n'; // 1 2
    std::cout << coordinates.contains(10) << ' ' << coordinates.value(1) << '\n'; // 0 20
}

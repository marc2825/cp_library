#include <iostream>
#include "linear_algebra/space_intersection.hpp"
int main() {
    const cp::Matrix<int> u{{1, 0, 0}, {0, 1, 0}}, v{{0, 1, 0}, {0, 0, 1}};
    const auto common = cp::linear_space_intersection_mod(u, v);
    std::cout << common.rows() << '\n';
    for (auto x : common.data()) std::cout << x << ' ';
    std::cout << '\n';
    cp::XorBasis<> a, b; a.insert(3); a.insert(5); b.insert(6);
    std::cout << cp::xor_space_intersection(a, b).max_xor() << '\n';
}

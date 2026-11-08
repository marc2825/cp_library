#include <iostream>
#include <vector>
#include "linear_algebra/gauss_jordan.hpp"
int main() {
    cp::Matrix<double> a{{1, 1}, {2, -1}};
    auto real = cp::gauss_jordan(a, std::vector<double>{3, 0});
    for (auto x : real.particular) std::cout << x << ' ';
    std::cout << '\n';
    auto mod = cp::gauss_jordan_mod<7>(cp::Matrix<int>{{1, 2, 3}, {2, 4, 6}}, {4, 1});
    std::cout << mod.rank << ' ' << mod.nullspace_basis.size() << '\n';
    for (auto x : mod.particular) std::cout << x << ' ';
    std::cout << '\n';
    for (const auto& v : mod.nullspace_basis) {
        for (auto x : v) std::cout << x << ' ';
        std::cout << '\n';
    }
}

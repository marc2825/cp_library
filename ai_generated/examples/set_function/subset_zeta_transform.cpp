#include <iostream>
#include <vector>
#include "set_function/subset_zeta_transform.hpp"
int main() {
    std::vector<long long> f{1, 2, 3, 4};
    cp::subset_zeta_transform(f);
    for (auto x : f) std::cout << x << ' ';
    std::cout << '\n';
}

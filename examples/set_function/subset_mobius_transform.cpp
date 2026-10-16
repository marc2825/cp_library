#include <iostream>
#include <vector>
#include "set_function/subset_mobius_transform.hpp"
int main() {
    std::vector<long long> f{1, 3, 4, 10};
    cp::subset_mobius_transform(f);
    for (auto x : f) std::cout << x << ' ';
    std::cout << '\n';
}

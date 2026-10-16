#include <iostream>
#include <vector>
#include "set_function/superset_mobius_transform.hpp"
int main() {
    std::vector<long long> f{10, 6, 7, 4};
    cp::superset_mobius_transform(f);
    for (auto x : f) std::cout << x << ' ';
    std::cout << '\n';
}

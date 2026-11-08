#include <iostream>
#include "set_function/fwht.hpp"
int main() {
    std::vector<long long> a{1, 2, 3, 4};
    cp::fwht(a);
    for (auto x : a) std::cout << x << ' ';
    std::cout << '\n';
    cp::fwht(a, true);
    std::cout << a[0] << ' ' << a[3] << '\n';
}

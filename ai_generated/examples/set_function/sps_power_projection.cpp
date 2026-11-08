#include "set_function/sps_power_projection.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}
int main() { print(cp::sps_power_projection({1, 1, 1, 1}, {0, 2, 3, 4}, 5)); }

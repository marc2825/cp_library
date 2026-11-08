#include "fps/fractional_linear_transform.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}
int main() { print(cp::fps_fractional_linear_transform({1, 2, 3}, 1, 5)); }

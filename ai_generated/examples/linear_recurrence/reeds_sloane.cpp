#include "linear_recurrence/reeds_sloane.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}
int main() { print(cp::reeds_sloane({1, 1, 2, 3, 5, 8, 4, 3}, 9)); }

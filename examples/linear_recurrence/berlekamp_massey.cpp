#include <iostream>
#include <vector>
#include "linear_recurrence/berlekamp_massey.hpp"
int main() {
    const std::vector<int> s{0, 1, 1, 2, 3, 5, 8, 13};
    for (int x : cp::berlekamp_massey(s)) std::cout << x << ' ';
    std::cout << '\n';
    std::cout << cp::linear_recurrence_from_prefix(s, 10) << '\n';
}

#include <iostream>
#include <vector>
#include "fps/division.hpp"

void print(const std::vector<int>& a) {
    for (int x : a) std::cout << x << ' ';
    std::cout << '\n';
}
int main() {
    const auto [q, r] = cp::polynomial_divmod<17, 3>({1, 2, 3, 4}, {1, 1});
    print(q); print(r);
}

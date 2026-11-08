#include <iostream>
#include <vector>
#include "fps/sqrt.hpp"

void print(const std::vector<int>& a) {
    for (int x : a) std::cout << x << ' ';
    std::cout << '\n';
}
int main() {
    const auto root = cp::fps_sqrt<17, 3>({0, 0, 4, 4, 1}, 6);
    if (root) print(*root);
    else std::cout << "no square root\n";
}

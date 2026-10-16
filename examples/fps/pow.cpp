#include <iostream>
#include <vector>
#include "fps/pow.hpp"

void print(const std::vector<int>& a) {
    for (int x : a) std::cout << x << ' ';
    std::cout << '\n';
}
int main() {
    print(cp::fps_pow<17, 3>({0, 2, 1}, 3, 7));
}

#include <iostream>
#include <vector>
#include "fps/inv.hpp"

void print(const std::vector<int>& a) {
    for (int x : a) std::cout << x << ' ';
    std::cout << '\n';
}
int main() {
    print(cp::fps_inv<17, 3>({1, -1}, 6));
}

#include <iostream>
#include <vector>
#include "fps/exp.hpp"

void print(const std::vector<int>& a) {
    for (int x : a) std::cout << x << ' ';
    std::cout << '\n';
}
int main() {
    print(cp::fps_exp<17, 3>({0, 1}, 5));
}

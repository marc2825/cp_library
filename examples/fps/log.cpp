#include <iostream>
#include <vector>
#include "fps/log.hpp"

void print(const std::vector<int>& a) {
    for (int x : a) std::cout << x << ' ';
    std::cout << '\n';
}
int main() {
    print(cp::fps_log<17, 3>({1, 1}, 5));
}

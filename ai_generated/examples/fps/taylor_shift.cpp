#include <iostream>
#include <vector>
#include "fps/taylor_shift.hpp"

void print(const std::vector<int>& a) {
    for (int x : a) std::cout << x << ' ';
    std::cout << '\n';
}
int main() {
    print(cp::taylor_shift<17, 3>({1, 2, 3}, 2));
}

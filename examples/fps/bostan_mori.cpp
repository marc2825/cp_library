#include <iostream>
#include <vector>
#include "fps/bostan_mori.hpp"

void print(const std::vector<int>& a) {
    for (int x : a) std::cout << x << ' ';
    std::cout << '\n';
}
int main() {
    print({cp::bostan_mori<17, 3>({0, 1}, {1, -1, -1}, 10)});
    print({cp::linear_recurrence_nth<17, 3>({0, 1}, {1, 1}, 10)});
}

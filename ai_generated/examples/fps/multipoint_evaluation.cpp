#include <iostream>
#include <vector>
#include "fps/multipoint_evaluation.hpp"

void print(const std::vector<int>& a) {
    for (int x : a) std::cout << x << ' ';
    std::cout << '\n';
}
int main() {
    print(cp::multipoint_evaluation<17, 3>({1, 2, 3}, {0, 1, 2}));
}

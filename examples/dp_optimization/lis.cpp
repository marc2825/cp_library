#include <iostream>
#include <vector>
#include "dp_optimization/lis.hpp"
int main() {
    std::vector<int> a{3, 1, 2, 2, 4};
    auto result = cp::lis(a); // false を渡すと広義単調増加。
    std::cout << result.length << '\n';
    for (int i : result.indices) std::cout << a[i] << ' ';
    std::cout << '\n';
}

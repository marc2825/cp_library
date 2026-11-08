#include <iostream>
#include <string>
#include "dp_optimization/lcs.hpp"
int main() {
    std::string a = "abcde", b = "ace";
    auto result = cp::lcs(a, b);
    std::cout << result.length << '\n';
    for (auto [i, j] : result.matches) { (void)j; std::cout << a[i]; }
    std::cout << '\n';
}

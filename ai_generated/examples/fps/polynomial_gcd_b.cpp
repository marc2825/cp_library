#include <iostream>
#include <vector>
#include "../../fps/polynomial_gcd.hpp"

int main() {
    auto g = cp::polynomial_gcd(std::vector<int>{-1, 0, 1}, std::vector<int>{1, -2, 1});
    std::cout << g[0] << " " << g[1] << "\n";
}

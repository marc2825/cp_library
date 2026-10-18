#include <iostream>
#include <vector>
#include "../../fps/polynomial_mod_pow.hpp"

int main() {
    auto c = cp::polynomial_mod_pow(std::vector<int>{0, 1}, 5, std::vector<int>{-1, -1, 1});
    std::cout << c[0] << " " << c[1] << "\n";
}

#include <iostream>
#include <vector>
#include "../../linear_algebra/wiedemann.hpp"

int main() {
    auto apply = [](const std::vector<int>& x) { return x; };
    auto x = cp::wiedemann_solve(2, apply, std::vector<int>{5, 8});
    if (x) std::cout << (*x)[0] << " " << (*x)[1] << "\n";
}

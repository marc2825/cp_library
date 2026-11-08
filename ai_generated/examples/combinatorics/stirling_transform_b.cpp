#include <iostream>
#include "combinatorics/stirling_transform.hpp"
int main() {
    auto b=cp::stirling_transform(std::vector<long long>{1,1,1,1,1}); std::cout << b[4] << '\n';
}

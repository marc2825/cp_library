#include <iostream>
#include "combinatorics/binomial_transform.hpp"
int main() {
    auto b=cp::binomial_transform(std::vector<long long>{1,2,3,4}); std::cout << b[3] << '\n';
}

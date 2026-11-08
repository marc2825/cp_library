#include <iostream>
#include "dp_optimization/min_plus_convolution.hpp"
int main() {
    auto c=cp::min_plus_convolution_arbitrary_concave(std::vector<long long>{3,1,4},std::vector<long long>{0,3,4});
    std::cout << c[2] << '\n';
}

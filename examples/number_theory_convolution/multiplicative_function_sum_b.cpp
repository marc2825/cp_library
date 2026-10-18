#include <iostream>
#include "number_theory_convolution/multiplicative_function_sum.hpp"
int main() {
    auto r=cp::multiplicative_function_sums<long long>(10,[](int,int e,std::uint64_t){return e+1;});
    std::cout << r.prefix[10] << '\n';
}

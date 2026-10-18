#include <iostream>
#include "basic_utility/sqrt_decomposition.hpp"
long long op(long long a,long long b) { return a+b; }
long long e() { return 0; }
int main() {
    cp::SqrtDecomposition<long long,op,e> s({1,2,3});
    s.set(1,7);
    std::cout << s.prod(0,3) << "\n";
}

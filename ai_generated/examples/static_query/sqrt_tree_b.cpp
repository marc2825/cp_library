#include <iostream>
#include "static_query/sqrt_tree.hpp"
long long op(long long a,long long b) { return a+b; }
long long e() { return 0; }
int main() {
    cp::SqrtTree<long long,op,e> s({1,2,3,4});
    std::cout << s.prod(1,4) << "\n";
}

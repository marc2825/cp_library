#include <iostream>
#include "range_query/polynomial_add_sum.hpp"
int main() {
    using AP = cp::ArithmeticProgressionAddSumAction<long long>;
    cp::ActedMonoidLazySegtree<AP> arithmetic(AP::values({1, 2, 3, 4}));
    arithmetic.apply(1, 4, AP::progression(1, 3, 2));
    std::cout << arithmetic.all_prod().sum << "\n";
    using Poly = cp::PolynomialAddSumAction<long long, 2>;
    cp::ActedMonoidLazySegtree<Poly> polynomial(Poly::values({1, 2, 3, 4}));
    polynomial.apply(0, 4, {0, 0, 1}); // 絶対添字iに対してi*iを加える。
    std::cout << polynomial.all_prod().sum << "\n";
}

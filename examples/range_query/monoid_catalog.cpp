#include <iostream>
#include <vector>
#include "basic_utility/static_modint.hpp"
#include "range_query/monoid_catalog.hpp"
int main() {
    using Min = cp::MinCountMonoid<int>;
    cp::MonoidSegtree<Min> minimum(std::vector<Min::S>{Min::leaf(3), Min::leaf(1), Min::leaf(1)});
    std::cout << minimum.all_prod().value << " " << minimum.all_prod().count << "\n";
    using Best = cp::MaxSubarrayMonoid<long long>;
    cp::MonoidSegtree<Best> best(std::vector<Best::S>{Best::leaf(-3), Best::leaf(5), Best::leaf(-1)});
    std::cout << best.all_prod().best << "\n";
    using Mint = cp::StaticModInt<101>;
    using Affine = cp::AffineFunctionMonoid<Mint>;
    std::cout << Affine::op({2, 1}, {3, 4})(5) << "\n";
    using Matrix = cp::MatrixProductMonoid<Mint, 2>;
    std::cout << Matrix::op({{1, 1}, {0, 1}}, {{1, 0}, {2, 1}})(0, 0) << "\n";
    using Fraction = cp::LinearFractionalMonoid<Mint>;
    auto reciprocal = Fraction::make(0, 1, 1, 0);
    std::cout << *Fraction::apply(reciprocal, Mint(2)) << " " << Fraction::apply(reciprocal, Mint(0)).has_value() << "\n";
    using Hash = cp::RollingHashMonoid<Mint, 7>;
    std::cout << Hash::op(Hash::op(Hash::leaf(1), Hash::leaf(2)), Hash::leaf(3)).hash << "\n";
}

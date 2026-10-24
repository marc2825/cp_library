#include <iostream>
#include "../../tree/fully_dynamic_tree_dp.hpp"
#include "../../basic_utility/static_modint.hpp"
int main() {
    using Mint = cp::StaticModInt<998244353>;
    cp::FullyDynamicIndependentSet<Mint> dp({1, 1, 1, 1});
    dp.link(0, 1); dp.link(1, 2); dp.link(1, 3);
    std::cout << dp.query(0).val() << '\n';
    dp.cut(1, 3); dp.link(2, 3);
    std::cout << dp.query(3).val() << '\n';
    dp.set_vertex(1, 0);
    std::cout << dp.query(0).val() << '\n';
}

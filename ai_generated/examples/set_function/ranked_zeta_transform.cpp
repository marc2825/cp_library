#include <iostream>
#include "set_function/ranked_zeta_transform.hpp"
int main() {
    auto f = cp::ranked_zeta_transform(std::vector<long long>{1, 2, 3, 4});
    std::cout << f[1][3] << '\n';
    cp::ranked_mobius_transform(f);
    std::cout << f[2][3] << '\n';
}

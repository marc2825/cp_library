#include <iostream>
#include "linear_recurrence/matrix_recurrence.hpp"
int main() {
    std::cout << cp::linear_recurrence_matrix({0, 1}, {1, 1}, 10) << '\n';
    // x <- x+3 を定数1を含む状態で表す。合成数法も使える。
    const auto s = cp::matrix_power_apply<12>({{1, 3}, {0, 1}}, {2, 1}, 5);
    std::cout << s[0] << ' ' << s[1] << '\n';
}

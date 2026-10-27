#include "linear_algebra/smith_normal_form.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}

int main() {
    cp::Matrix<int> a{{2, 4}, {4, 14}};
    auto s = cp::smith_normal_form(a);
    std::cout << s.diagonal(0, 0) << " " << s.diagonal(1, 1) << "\n";
    auto z = cp::solve_integer_system(a, std::vector<int>{6, 18});
    print(z.particular);
    auto c = cp::solve_congruence_system(a, std::vector<int>{6, 18}, cp::BigInt(6));
    cp::BigInt count = c.solvable ? 1 : 0;
    for (const auto &order : c.orders)
        count *= order;
    std::cout << count << "\n";
}

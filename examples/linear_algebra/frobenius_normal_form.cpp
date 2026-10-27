#include "linear_algebra/frobenius_normal_form.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}

int main() {
    auto f = cp::frobenius_normal_form<5>(cp::Matrix<int>{{1, 1}, {0, 1}});
    for (const auto &p : f.invariant_factors)
        print(p);
    print(f.form.data());
}

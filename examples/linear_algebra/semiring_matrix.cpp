#include "linear_algebra/semiring_matrix.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}

int main() {
    using S = cp::MinPlusSemiring<long long>;
    S s;
    cp::Matrix<S::value_type> a(2, 2, s.zero());
    a(0, 1) = 3;
    a(1, 0) = -1;
    auto b = cp::semiring_matrix_power(a, 2, s);
    for (std::size_t i = 0; i < b.data().size(); ++i) {
        if (i)
            std::cout << " ";
        if (b.data()[i])
            std::cout << *b.data()[i];
        else
            std::cout << "inf";
    }
    std::cout << "\n";
}

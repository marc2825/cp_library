#include "linear_algebra/finite_extension_field.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}

int main() {
    cp::FiniteExtensionField<3> f({1, 0, 1});
    print(f.multiply({1, 1}, {2, 1}));
    print(f.inverse({1, 1}));
}

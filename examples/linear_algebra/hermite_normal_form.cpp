#include "linear_algebra/hermite_normal_form.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}

int main() {
    auto h = cp::hermite_normal_form(cp::Matrix<int>{{4, 6}, {2, 8}});
    print(h.form.data());
}

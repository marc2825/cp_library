#include "fps/multivariate_fps.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}
int main() {
    cp::MultivariateFPS<> ring({2, 2});
    print(ring.inverse({1, -1, -1, 0}));
}

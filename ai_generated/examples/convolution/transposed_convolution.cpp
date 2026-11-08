#include "convolution/transposed_convolution.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}
int main() {
    cp::TransposedConvolutionPlan<> plan({1, 2}, 3, 3);
    print(plan.apply({3, 4, 5}));
    print(plan.transpose({7, 8, 9}));
}

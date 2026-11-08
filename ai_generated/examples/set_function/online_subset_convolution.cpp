#include "set_function/online_subset_convolution.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}
int main() {
    cp::OnlineSubsetConvolution<> online(2);
    std::vector<int> out;
    for (int i = 0; i < 4; ++i)
        out.push_back(online.push(i + 1, 1));
    print(out);
}

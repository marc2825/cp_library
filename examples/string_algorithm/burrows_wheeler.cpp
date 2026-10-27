#include "string_algorithm/burrows_wheeler.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}

int main() {
    auto b = cp::burrows_wheeler_transform("banana");
    std::cout << b.primary << '\n';
    print(b.last);
    std::cout << (cp::inverse_burrows_wheeler(b) == "banana") << '\n';
}

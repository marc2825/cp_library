#include "string_algorithm/fm_index.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}

int main() {
    cp::FMIndex index("banana", 3);
    std::cout << index.count("ana") << '\n';
    print(index.locate("ana"));
}

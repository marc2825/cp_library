#include "string_algorithm/border_tree.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}

int main() {
    cp::BorderTree tree("ababab");
    print(tree.borders(6));
    print(tree.periods(6));
    std::cout << tree.prefix_occurrences()[2] << '\n';
}

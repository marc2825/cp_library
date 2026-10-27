#include "linear_recurrence/combinatorial_composition.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}
int main() { print(cp::labelled_generating_function_compose({0, 1}, 5, cp::LabelledConstruction::sequence)); }

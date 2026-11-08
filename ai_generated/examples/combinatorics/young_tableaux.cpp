#include <iostream>
#include "combinatorics/young_tableaux.hpp"
int main() {
    const auto count = cp::enumerate_standard_young_tableaux({3, 2}, [](const auto& tableau) {
        return cp::is_standard_young_tableau(tableau);
    });
    std::cout << count << '\n';
    const auto shape = cp::conjugate_young_shape({3, 2});
    for (std::size_t i = 0; i < shape.size(); ++i) std::cout << shape[i] << (i + 1 == shape.size() ? '\n' : ' ');
}

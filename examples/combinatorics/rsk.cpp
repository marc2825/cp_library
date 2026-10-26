#include <iostream>
#include "combinatorics/rsk.hpp"
int main() {
    const auto pair = cp::rsk_word(std::vector<int>{3, 1, 4, 1, 5});
    const auto shape = cp::young_tableau_shape(pair.insertion);
    for (std::size_t i = 0; i < shape.size(); ++i) std::cout << shape[i] << (i + 1 == shape.size() ? '\n' : ' ');
    const auto word = cp::inverse_rsk_word(pair);
    for (std::size_t i = 0; i < word.size(); ++i) std::cout << word[i] << (i + 1 == word.size() ? '\n' : ' ');
}

#include "string_algorithm/suffix_automaton_utilities.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}

int main() {
    cp::SuffixAutomatonUtilities index("banana");
    std::cout << *index.kth_substring(2) << '\n';
    print(index.occurrences("ana"));
}

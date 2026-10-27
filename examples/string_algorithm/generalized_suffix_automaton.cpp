#include "string_algorithm/generalized_suffix_automaton.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}

int main() {
    cp::GeneralizedSuffixAutomaton index(std::vector<std::string>{"banana", "band", "ana"});
    std::cout << index.count_occurrences("ana") << ' ' << index.distinct_substrings() << '\n';
    print(index.containing_documents("ana"));
}

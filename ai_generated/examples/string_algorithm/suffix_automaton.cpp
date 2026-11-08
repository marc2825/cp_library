#include <iostream>
#include "string_algorithm/suffix_automaton.hpp"
int main() {
    cp::SuffixAutomaton a("abcbc");
    std::cout << a.distinct_substrings() << ' ' << a.count_occurrences("bc") << ' ' << *a.first_occurrence("bc") << '\n';
    a.append("bc");
    std::cout << a.size() << ' ' << a.count_occurrences("bc") << '\n';
}

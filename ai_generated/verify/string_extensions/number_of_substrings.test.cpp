#define PROBLEM "https://judge.yosupo.jp/problem/number_of_substrings"
#include "string_algorithm/suffix_automaton_utilities.hpp"
#include <iostream>
int main() {
    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);
    std::string s;
    std::cin >> s;
    std::cout << cp::SuffixAutomatonUtilities(s).distinct_substrings() << '\n';
}

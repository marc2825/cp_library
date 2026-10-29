#include "parsing/regex_nfa.hpp"
#include <iostream>
using namespace cp;
int main() {
    auto nfa = regex_to_nfa("(ab|c)*d?");
    const std::vector<std::string> words{"", "abccabd", "ac", "d", "abc"};
    for (std::size_t i = 0; i < words.size(); ++i)
        std::cout << (i ? " " : "") << regex_full_match(nfa, words[i]);
    std::cout << "\n";
}

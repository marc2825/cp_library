#include <iostream>
#include "string_algorithm/aho_corasick.hpp"
int main() {
    cp::AhoCorasick ac; ac.insert("he"); ac.insert("she"); ac.insert(""); ac.build();
    for (auto x : ac.count_occurrences("shehe")) std::cout << x << ' ';
    std::cout << '\n';
}

#include "ordered_structure/ordered_associative.hpp"
#include <iostream>
#include <map>
#include <set>
#include <string>
int main() {
    std::set<int> unique{1, 4, 7};
    std::cout << *cp::predecessor(unique, 4) << ' ' << *cp::successor(unique, 4, true) << '\n'; // 1 4
    std::multiset<int> multiple{2, 2, 5};
    cp::erase_one(multiple, 2);
    std::cout << multiple.count(2) << ' ' << multiple.erase(2) << '\n'; // 1 1
    std::map<int, std::string> words;
    words.try_emplace(3, "first"); words.try_emplace(3, "ignored");
    words.insert_or_assign(5, "last");
    auto it = cp::predecessor(words, 4);
    if (it != words.end()) std::cout << it->first << ' ' << it->second << '\n'; // 3 first
    std::cout << *cp::pop_last(unique) << '\n'; // 7
}

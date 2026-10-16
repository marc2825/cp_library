#include <iostream>
#include "string_algorithm/trie.hpp"
int main() {
    cp::Trie trie;
    trie.insert("app"); trie.insert("apple"); trie.insert("app"); trie.insert("");
    std::cout << trie.size() << ' ' << trie.count("app") << ' ' << trie.count_prefix("app") << '\n';
    trie.erase("app");
    std::cout << trie.count("app") << ' ' << trie.contains("ap") << ' ' << trie.count("") << '\n';
}

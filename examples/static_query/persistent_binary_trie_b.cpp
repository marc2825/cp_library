#include <iostream>
#include "static_query/persistent_binary_trie.hpp"
int main() {
    cp::PersistentBinaryTrie s; auto a=s.insert(0,7); auto b=s.insert(a,2);
    std::cout << *s.kth(a,0) << " " << *s.kth(b,0) << "\n";
}

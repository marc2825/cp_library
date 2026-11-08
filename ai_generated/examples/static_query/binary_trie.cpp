#include <iostream>
#include "static_query/binary_trie.hpp"
int main() {
    cp::BinaryTrie<> s; s.insert(1); s.insert(6); s.insert(6); s.xor_all(3);
    std::cout<<*s.kth(0)<<" "<<*s.kth_xor(0,7)<<"\n";
}

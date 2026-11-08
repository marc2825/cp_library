#define PROBLEM "https://judge.yosupo.jp/problem/set_xor_min"
#include <iostream>
#include "static_query/binary_trie.hpp"
int main(){
    int q;std::cin>>q;cp::BinaryTrie<unsigned,30> trie;
    while(q--){int type;unsigned x;std::cin>>type>>x;if(type==0){if(!trie.count(x))trie.insert(x);}else if(type==1)trie.erase(x);else std::cout<<(*trie.kth_xor(0,x)^x)<<'\n';}
}

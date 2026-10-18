#pragma once
// 説明: docs/static_query/persistent_binary_trie.md
#include <cassert>
#include <cstdint>
#include <optional>
#include <vector>

namespace cp {
// uint64多重集合の版を分岐。root=0は空。各insert/eraseで65節点。
class PersistentBinaryTrie {
    struct Node { int child[2]={0,0}; std::uint64_t count=0; };
    std::vector<Node> nodes_{Node{}};
    int change(int k,int bit,std::uint64_t x,bool erase) {
        int next=int(nodes_.size()); nodes_.push_back(nodes_[k]);
        if(erase) --nodes_[next].count; else ++nodes_[next].count;
        if(bit>=0) { int side=int((x>>bit)&1),child=change(nodes_[k].child[side],bit-1,x,erase); nodes_[next].child[side]=child; }
        return next;
    }
    void check(int root) const { assert(0<=root && root<int(nodes_.size())); (void)root; }
public:
    std::uint64_t size(int root) const { check(root); return nodes_[root].count; }
    std::uint64_t count(int root,std::uint64_t x) const {
        check(root); for(int b=63;b>=0;--b) root=nodes_[root].child[(x>>b)&1]; return nodes_[root].count;
    }
    int insert(int root,std::uint64_t x) { check(root); return change(root,63,x,false); }
    std::optional<int> erase(int root,std::uint64_t x) { if(!count(root,x)) return std::nullopt; return change(root,63,x,true); }
    // XOR後の値のk番目。返値もXOR後の値。
    std::optional<std::uint64_t> kth(int root,std::uint64_t k,std::uint64_t xor_mask=0) const {
        if(k>=size(root)) return std::nullopt;
        std::uint64_t result=0;
        for(int b=63;b>=0;--b) {
            int side=int((xor_mask>>b)&1); auto count0=nodes_[nodes_[root].child[side]].count;
            if(k>=count0) { k-=count0; side^=1; result|=std::uint64_t(1)<<b; }
            root=nodes_[root].child[side];
        }
        return result;
    }
};
} // namespace cp

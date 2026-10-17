#pragma once
#include <cassert>
#include <cstdint>
#include <limits>
#include <optional>
#include <type_traits>
#include <vector>

namespace cp {
// 符号なし整数の多重集合。xor_all後も順位は現在の値の昇順。
template<class U=std::uint64_t,int Bits=std::numeric_limits<U>::digits> class BinaryTrie {
    static_assert(std::is_unsigned<U>::value && !std::is_same<U,bool>::value,"unsigned integer required");
    static_assert(0<Bits && Bits<=std::numeric_limits<U>::digits,"invalid bit width");
    struct Node { int child[2]={-1,-1}; long long count=0; };
    std::vector<Node> nodes_{Node{}}; U mask_=0;
    static void check(U x) { if constexpr(Bits<std::numeric_limits<U>::digits) assert((x>>Bits)==0); (void)x; }
public:
    long long size() const { return nodes_[0].count; }
    bool empty() const { return size()==0; }
    void clear() { nodes_.assign(1,Node{}); mask_=0; }
    long long count(U x) const {
        check(x); x^=mask_; int k=0;
        for (int b=Bits-1;b>=0;--b) { k=nodes_[k].child[(x>>b)&1]; if(k==-1) return 0; }
        return nodes_[k].count;
    }
    void insert(U x) {
        check(x); assert(size()<std::numeric_limits<long long>::max()); x^=mask_; int k=0; ++nodes_[k].count;
        for (int b=Bits-1;b>=0;--b) {
            int c=int((x>>b)&1);
            if (nodes_[k].child[c]==-1) { int j=int(nodes_.size()); nodes_.push_back(Node{}); nodes_[k].child[c]=j; }
            k=nodes_[k].child[c]; ++nodes_[k].count;
        }
    }
    bool erase(U x) {
        if (!count(x)) return false;
        x^=mask_; int k=0; --nodes_[k].count;
        for (int b=Bits-1;b>=0;--b) { k=nodes_[k].child[(x>>b)&1]; --nodes_[k].count; }
        return true;
    }
    void xor_all(U x) { check(x); mask_^=x; }
    // xとのXOR値の昇順でk番目に当たる「格納値」。範囲外nullopt。
    std::optional<U> kth_xor(long long rank,U x=0) const {
        check(x); if(rank<0 || rank>=size()) return std::nullopt;
        U key=x^mask_,raw=0; int k=0;
        for (int b=Bits-1;b>=0;--b) {
            int c=int((key>>b)&1),j=nodes_[k].child[c]; long long n=j==-1?0:nodes_[j].count;
            if (rank>=n) { rank-=n; c^=1; }
            raw|=U(c)<<b; k=nodes_[k].child[c];
        }
        return raw^mask_;
    }
    std::optional<U> kth(long long k) const { return kth_xor(k); }
    long long order_of_key(U x) const {
        check(x); long long answer=0; int k=0;
        for (int b=Bits-1;b>=0 && k!=-1;--b) {
            int c=int((mask_>>b)&1);
            if ((x>>b)&1) { int j=nodes_[k].child[c]; if(j!=-1) answer+=nodes_[j].count; c^=1; }
            k=nodes_[k].child[c];
        }
        return answer;
    }
};
} // namespace cp

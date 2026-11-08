#pragma once
#include <cassert>
#include <cstdint>
#include <utility>
#include <vector>

namespace cp {
// 挿入・削除可能なrank/select。非圧縮のimplicit treap、期待O(log n)/操作。
class DynamicBitVector {
    struct Node { int left=0,right=0,size=1,ones=0; std::uint64_t priority=0; bool bit=false; };
    std::vector<Node> nodes_{Node{0,0,0,0,0,false}};
    std::vector<int> free_; int root_=0; std::uint64_t state_=0x6a09e667f3bcc909ULL;
    int length(int p) const { return nodes_[p].size; }
    int ones(int p) const { return nodes_[p].ones; }
    void pull(int p) { auto& n=nodes_[p]; n.size=1+length(n.left)+length(n.right); n.ones=int(n.bit)+ones(n.left)+ones(n.right); }
    std::uint64_t random() {
        state_+=0x9e3779b97f4a7c15ULL; auto x=state_;
        x=(x^(x>>30))*0xbf58476d1ce4e5b9ULL; x=(x^(x>>27))*0x94d049bb133111ebULL; return x^(x>>31);
    }
    int make(bool bit) {
        int p;
        if(free_.empty()) { p=int(nodes_.size()); nodes_.push_back({}); }
        else { p=free_.back(); free_.pop_back(); }
        nodes_[p]={0,0,1,int(bit),random(),bit}; return p;
    }
    int join(int a,int b) {
        if(!a || !b) return a?a:b;
        if(nodes_[a].priority>nodes_[b].priority) { nodes_[a].right=join(nodes_[a].right,b); pull(a); return a; }
        nodes_[b].left=join(a,nodes_[b].left); pull(b); return b;
    }
    std::pair<int,int> split(int p,int count) {
        if(!p) return {};
        int n=length(nodes_[p].left);
        if(count<=n) { auto [a,b]=split(nodes_[p].left,count); nodes_[p].left=b; pull(p); return {a,p}; }
        auto [a,b]=split(nodes_[p].right,count-n-1); nodes_[p].right=a; pull(p); return {p,b};
    }
public:
    DynamicBitVector()=default;
    explicit DynamicBitVector(const std::vector<bool>& bits) { for(bool b:bits) insert(size(),b); }
    int size() const { return length(root_); }
    bool empty() const { return !root_; }
    void clear() { nodes_.resize(1); free_.clear(); root_=0; }
    bool access(int p) const {
        assert(0<=p && p<size()); int k=root_;
        while(true) { int n=length(nodes_[k].left); if(p<n) k=nodes_[k].left;
            else if(p==n) return nodes_[k].bit; else { p-=n+1; k=nodes_[k].right; } }
    }
    int rank(bool bit,int end) const {
        assert(0<=end && end<=size()); int p=root_,remaining=end,result=0;
        while(p && remaining) {
            const auto& n=nodes_[p]; int left=length(n.left);
            if(remaining<=left) p=n.left;
            else { result+=ones(n.left)+int(n.bit); remaining-=left+1; p=n.right; }
        }
        return bit?result:end-result;
    }
    // 0-indexed occurrence。存在しなければ-1。
    int select(bool bit,int occurrence) const {
        if(occurrence<0 || occurrence>= (bit?ones(root_):size()-ones(root_))) return -1;
        int p=root_,offset=0;
        while(p) {
            const auto& n=nodes_[p]; int left=bit?ones(n.left):length(n.left)-ones(n.left);
            if(occurrence<left) { p=n.left; continue; }
            occurrence-=left; offset+=length(n.left);
            if(n.bit==bit) { if(!occurrence) return offset; --occurrence; }
            ++offset; p=n.right;
        }
        return -1;
    }
    void insert(int p,bool bit) {
        assert(0<=p && p<=size()); int k=make(bit); auto [a,b]=split(root_,p); root_=join(join(a,k),b);
    }
    bool erase(int p) {
        assert(0<=p && p<size()); auto [a,b]=split(root_,p); auto [mid,c]=split(b,1);
        bool bit=nodes_[mid].bit; free_.push_back(mid); root_=join(a,c); return bit;
    }
    void set(int p,bool bit) { assert(0<=p && p<size()); erase(p); insert(p,bit); }
};
} // namespace cp

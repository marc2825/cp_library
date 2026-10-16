#pragma once
#include <cstddef>
#include <map>
#include <queue>
#include <stdexcept>
#include <string_view>
#include <vector>
#include "detail/common.hpp"

namespace cp {
// 任意byteの多パターン照合。空・重複pattern可。insert後にbuild。
// 疎なTrieとfailure link。docs/string_algorithm/aho_corasick.md
class AhoCorasick {
    struct Node { std::map<unsigned char,int> next; int fail=0,output=-1; std::vector<int> ids; };
    std::vector<Node> nodes_{1}; std::vector<int> terminal_,order_; bool built_=false;
    int step(int v,unsigned char c) const { for(;;) { auto it=nodes_[v].next.find(c); if(it!=nodes_[v].next.end()) return it->second; if(!v) return 0; v=nodes_[v].fail; } }
    void require_built() const { if(!built_) throw std::logic_error("AhoCorasick::build is required"); }
public:
    int insert(std::string_view pattern) {
        string_algorithm_detail::checked_size(pattern.size()); int v=0;
        for(unsigned char c:pattern) { auto it=nodes_[v].next.find(c); if(it==nodes_[v].next.end()) { int next=string_algorithm_detail::checked_size(nodes_.size()); nodes_[v].next.emplace(c,next); nodes_.emplace_back(); v=next; } else v=it->second; }
        int id=string_algorithm_detail::checked_size(terminal_.size()); terminal_.push_back(v); nodes_[v].ids.push_back(id); built_=false; return id;
    }
    int pattern_count() const { return int(terminal_.size()); }
    std::size_t node_count() const { return nodes_.size(); }
    void build() {
        order_.clear(); order_.push_back(0); std::queue<int> q;
        for(auto [c,v]:nodes_[0].next) { (void)c; nodes_[v].fail=0; nodes_[v].output=nodes_[0].ids.empty()?-1:0; q.push(v); }
        while(!q.empty()) {
            int v=q.front(); q.pop(); order_.push_back(v);
            for(auto [c,to]:nodes_[v].next) { int f=step(nodes_[v].fail,c); nodes_[to].fail=f; nodes_[to].output=nodes_[f].ids.empty()?nodes_[f].output:f; q.push(to); }
        }
        built_=true;
    }
    std::vector<std::size_t> count_occurrences(std::string_view text) const {
        require_built(); string_algorithm_detail::checked_size(text.size()); std::vector<std::size_t> count(nodes_.size()); count[0]=1; int v=0;
        for(unsigned char c:text) { v=step(v,c); ++count[v]; }
        for(auto it=order_.rbegin();it!=order_.rend();++it) if(*it) count[nodes_[*it].fail]+=count[*it];
        std::vector<std::size_t> result; result.reserve(terminal_.size()); for(int t:terminal_) result.push_back(count[t]); return result;
    }
    // callback(pattern_id, end_exclusive)。空patternは全ての文字境界で報告。
    template<class Callback> void scan(std::string_view text,Callback callback) const {
        require_built(); string_algorithm_detail::checked_size(text.size()); int v=0;
        auto report=[&](int end) { for(int w=v;w!=-1;w=nodes_[w].output) for(int id:nodes_[w].ids) callback(id,end); };
        report(0); int end=0; for(unsigned char c:text) { v=step(v,c); report(++end); }
    }
};
} // namespace cp

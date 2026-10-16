#pragma once
#include <cstddef>
#include <map>
#include <string_view>
#include <utility>
#include <vector>
#include "detail/common.hpp"

namespace cp {
// オンライン回文木。node0/1は長さ-1/0の根。実回文はnode2以降。
class Eertree {
public:
    struct Node { int length=0,link=0,first_end=-1; std::map<unsigned char,int> next; std::size_t suffix_count=0; };
private:
    std::vector<Node> nodes_; std::vector<unsigned char> text_; int last_=1;
    int extendable(int v,int p,unsigned char c) const { while(p-1-nodes_[v].length<0 || text_[p-1-nodes_[v].length]!=c) v=nodes_[v].link; return v; }
public:
    Eertree() { nodes_.resize(2); nodes_[0].length=-1; }
    explicit Eertree(std::string_view text):Eertree() { string_algorithm_detail::checked_size(text.size()); for(unsigned char c:text) append(c); }
    int append(unsigned char c) {
        int p=string_algorithm_detail::checked_size(text_.size()); text_.push_back(c); int v=extendable(last_,p,c);
        auto it=nodes_[v].next.find(c);
        if(it!=nodes_[v].next.end()) last_=it->second;
        else {
            int k=string_algorithm_detail::checked_size(nodes_.size()); Node node; node.length=nodes_[v].length+2; node.first_end=p;
            if(node.length==1) node.link=1;
            else { int f=extendable(nodes_[v].link,p,c); node.link=nodes_[f].next.at(c); }
            nodes_[v].next.emplace(c,k); nodes_.push_back(std::move(node)); last_=k;
        }
        ++nodes_[last_].suffix_count; return last_;
    }
    int size() const { return int(text_.size()); }
    int distinct_count() const { return int(nodes_.size())-2; }
    int longest_suffix_node() const { return last_; }
    const std::vector<Node>& nodes() const { return nodes_; }
    // node ID順の出現回数。根は0。呼出しは非破壊、追記後にも再計算可。
    std::vector<std::size_t> occurrence_counts() const {
        std::vector<std::size_t> result(nodes_.size()); for(std::size_t i=2;i<nodes_.size();++i) result[i]=nodes_[i].suffix_count;
        for(int i=int(nodes_.size())-1;i>=2;--i) result[nodes_[i].link]+=result[i];
        result[0]=result[1]=0; return result;
    }
};
} // namespace cp

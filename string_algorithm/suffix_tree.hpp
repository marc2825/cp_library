#pragma once
#include <algorithm>
#include <climits>
#include <map>
#include <string>
#include <string_view>
#include <vector>
#include "suffix_array.hpp"
#include "lcp_array.hpp"

namespace cp {
// SA+LCPから作る静的な圧縮suffix trie。全byteを使え、番兵を入力に予約しない。
class SuffixTree {
public:
    struct Node {int parent=-1,depth=0,begin=0,end=0,suffix_start=-1;std::map<unsigned char,int> children;};
private:
    std::string text_;std::vector<Node> nodes_;
    int match(std::string_view pattern) const {
        string_algorithm_detail::checked_size(pattern.size());int node=0;std::size_t at=0;
        while(at<pattern.size()){
            auto it=nodes_[node].children.find(static_cast<unsigned char>(pattern[at]));
            if(it==nodes_[node].children.end())return -1;
            node=it->second;
            for(int i=nodes_[node].begin;i<nodes_[node].end&&at<pattern.size();++i,++at)
                if(static_cast<unsigned char>(text_[i])!=static_cast<unsigned char>(pattern[at]))return -1;
        }
        return node;
    }
public:
    explicit SuffixTree(std::string_view text={}) {
        if(text.size()>static_cast<std::size_t>((INT_MAX-1)/2))throw std::length_error("suffix tree node IDs exceed int");
        text_=std::string(text);const int n=int(text.size());nodes_.push_back({-1,0,0,0,n,{}});
        const auto sa=suffix_array(text),lcp=lcp_array(text,sa);std::vector<int> stack{0};
        for(int i=0;i<n;++i){
            const int common=i?lcp[i-1]:0;
            while(nodes_[stack.back()].depth>common)stack.pop_back();
            int parent=stack.back();
            if(nodes_[parent].depth<common){
                const int previous=sa[i-1];const auto key=static_cast<unsigned char>(text_[previous+nodes_[parent].depth]);
                const int old=nodes_[parent].children.at(key),middle=int(nodes_.size());
                nodes_.push_back({parent,common,previous+nodes_[parent].depth,previous+common,-1,{}});
                nodes_[parent].children[key]=middle;nodes_[middle].children[static_cast<unsigned char>(text_[nodes_[old].begin+common-nodes_[parent].depth])]=old;
                nodes_[old].begin+=common-nodes_[parent].depth;nodes_[old].parent=middle;parent=middle;stack.push_back(middle);
            }
            if(n-sa[i]==common)nodes_[parent].suffix_start=sa[i];
            else{
                const int leaf=int(nodes_.size()),start=sa[i]+nodes_[parent].depth;
                nodes_.push_back({parent,n-sa[i],start,n,sa[i],{}});nodes_[parent].children[static_cast<unsigned char>(text_[start])]=leaf;stack.push_back(leaf);
            }
        }
    }
    int size()const{return int(text_.size());}
    const std::string& text()const{return text_;}
    const std::vector<Node>& nodes()const{return nodes_;}
    bool contains(std::string_view pattern)const{return match(pattern)>=0;}
    std::vector<int> occurrences(std::string_view pattern)const{
        const int node=match(pattern);if(node<0)return {};
        std::vector<int> result,stack{node};while(!stack.empty()){const int v=stack.back();stack.pop_back();if(nodes_[v].suffix_start>=0)result.push_back(nodes_[v].suffix_start);for(auto [c,to]:nodes_[v].children){(void)c;stack.push_back(to);}}
        std::sort(result.begin(),result.end());return result;
    }
};
} // namespace cp

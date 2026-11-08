#include <algorithm>
#include <iostream>
#include <map>
#include <random>
#include <string>
#include <utility>
#include <vector>
#include "common.hpp"
#include "string_algorithm/aho_corasick.hpp"
#include "string_algorithm/eertree.hpp"
std::mt19937 rng(3212026);
int pick(int n) { return int(rng()%n); }
bool palindrome(const std::string& s) { return std::equal(s.begin(),s.end(),s.rbegin()); }
void check_eertree(const std::string& text) {
    cp::Eertree tree; std::map<std::string,std::size_t> want;
    for(int p=0;p<int(text.size());++p) {
        int suffix=0; for(int l=0;l<=p;++l) { auto sub=text.substr(l,p-l+1); if(palindrome(sub)) { ++want[sub]; suffix=std::max(suffix,p-l+1); } }
        int node=tree.append(static_cast<unsigned char>(text[p]));
        require(node==tree.longest_suffix_node() && tree.nodes()[node].length==suffix,"eertree longest suffix each prefix");
        require(tree.distinct_count()==int(want.size()),"eertree distinct palindrome set");
        auto counts=tree.occurrence_counts(); std::map<std::string,std::size_t> got;
        for(int i=2;i<int(tree.nodes().size());++i) { auto &v=tree.nodes()[i]; auto s=text.substr(v.first_end-v.length+1,v.length); require(palindrome(s),"eertree node is palindrome"); got[s]=counts[i]; require(tree.nodes()[v.link].length<v.length,"eertree suffix links shorter"); }
        require(got==want,"eertree occurrence counts independent substring enumeration");
    }
    cp::Eertree other(text); require(other.occurrence_counts()==tree.occurrence_counts(),"eertree batch vs streaming");
}
void ac() {
    for(int trial=0;trial<250;++trial) {
        cp::AhoCorasick ac; std::vector<std::string> patterns;
        rejects<std::logic_error>([&]{ac.count_occurrences("");});
        for(int round=0;round<3;++round) {
            for(int j=0;j<15;++j) { std::string p(pick(9),'\0'); for(char &c:p) c=static_cast<char>(pick(5)==0?pick(256):pick(3)); require(ac.insert(p)==int(patterns.size()),"AC insertion ID"); patterns.push_back(p); }
            ac.build(); ac.build();
            for(int q=0;q<8;++q) {
                std::string text(pick(80),'\0'); for(char &c:text) c=static_cast<char>(pick(5)==0?pick(256):pick(3));
                std::vector<std::size_t> want(patterns.size()); std::vector<std::pair<int,int>> matches;
                for(int id=0;id<int(patterns.size());++id) for(int l=0;l<=int(text.size());++l) if(text.compare(l,patterns[id].size(),patterns[id])==0) { ++want[id]; matches.emplace_back(id,l+int(patterns[id].size())); }
                require(ac.count_occurrences(text)==want,"AC counts incl empty/duplicate/binary patterns");
                std::vector<std::pair<int,int>> got; ac.scan(text,[&](int id,int end){got.emplace_back(id,end);});
                std::sort(got.begin(),got.end()); std::sort(matches.begin(),matches.end()); require(got==matches,"AC match positions by direct comparison");
            }
        }
    }
    std::string s(200000,'a'); cp::AhoCorasick ac; ac.insert(s); ac.insert("a"); ac.insert(""); ac.build(); require(ac.count_occurrences(s)==std::vector<std::size_t>({1,200000,200001}),"deep sparse AC no recursion");
}
int main() {
    for(int n=0;n<=9;++n) for(int code=0;code<(1<<n);++code) { std::string s(n,'a'); for(int i=0;i<n;++i) if(code>>i&1) s[i]='b'; check_eertree(s); }
    for(int q=0;q<250;++q) { std::string s(pick(50),'\0'); for(char &c:s) c=static_cast<char>(pick(256)); check_eertree(s); }
    cp::Eertree large(std::string(200000,'a')); auto counts=large.occurrence_counts(); require(large.distinct_count()==200000 && counts.back()==1 && counts[2]==200000,"large repeated eertree");
    ac(); std::cout<<"strings: exhaustive binary palindromes, all-byte patterns, duplicates, empties and 200000-byte inputs passed\n";
}

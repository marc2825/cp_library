#pragma once
#include <algorithm>
#include <climits>
#include <string>
#include <string_view>
#include <vector>
#include "detail/lce.hpp"

namespace cp {
// 複数文字列の部分文字列を衝突なしで比較。構築O(N log N)、問い合わせO(1)。
class MultiStringCompare {
    std::vector<int> offsets_,lengths_;
    string_algorithm_detail::LCEIndex index_;
    static std::vector<int> encode(const std::vector<std::string>& strings){
        if(strings.size()>static_cast<std::size_t>(INT_MAX-256))throw std::length_error("too many string separators");
        std::vector<int> values;std::size_t total=strings.size();for(const auto&s:strings){if(s.size()>static_cast<std::size_t>(INT_MAX)-total)throw std::length_error("combined text too long");total+=s.size();}if(total>=static_cast<std::size_t>(INT_MAX))throw std::length_error("combined text too long");values.reserve(total);
        for(std::size_t i=0;i<strings.size();++i){for(unsigned char c:strings[i])values.push_back(c);values.push_back(256+int(i));}return values;
    }
    void range(int id,int l,int r)const{if(id<0||id>=int(lengths_.size()))throw std::out_of_range("string ID");string_algorithm_detail::range(lengths_[id],l,r);}
public:
    explicit MultiStringCompare(const std::vector<std::string>& strings):index_(encode(strings)){
        int offset=0;for(const auto&s:strings){offsets_.push_back(offset);lengths_.push_back(int(s.size()));offset+=int(s.size())+1;}
    }
    int size()const{return int(lengths_.size());}
    int lcp(int a,int l,int r,int b,int x,int y)const{range(a,l,r);range(b,x,y);return std::min({r-l,y-x,index_.lce(offsets_[a]+l,offsets_[b]+x)});}
    int compare(int a,int l,int r,int b,int x,int y)const{
        const int common=lcp(a,l,r,b,x,y);if(common==std::min(r-l,y-x))return (r-l>y-x)-(r-l<y-x);
        return index_.rank[offsets_[a]+l]<index_.rank[offsets_[b]+x]?-1:1;
    }
};
} // namespace cp

#pragma once
#include <algorithm>
#include <string_view>
#include <tuple>
#include <vector>
#include "detail/lce.hpp"

namespace cp {
struct StringRun {
    int period,left,right; // 最小周期、極大な半開区間。長さ>=2*period。
    friend bool operator==(const StringRun&a,const StringRun&b){return std::tie(a.period,a.left,a.right)==std::tie(b.period,b.left,b.right);}
};
// 2通りのLyndon順とLCE/LCS。O(n log n)時間・領域。
inline std::vector<StringRun> runs_enumeration(std::string_view text) {
    const int n=string_algorithm_detail::checked_size(text.size());
    std::vector<int> values;for(unsigned char c:text)values.push_back(c);
    const string_algorithm_detail::LCEIndex forward(values);
    auto reversed=values;std::reverse(reversed.begin(),reversed.end());
    const string_algorithm_detail::LCEIndex backward(reversed);
    std::vector<StringRun> result;
    for(int order=0;order<2;++order){
        auto ordered=values;if(order)for(auto&x:ordered)x=255-x;
        const auto sa=suffix_array(ordered);std::vector<int> rank(n),stack;
        for(int i=0;i<n;++i)rank[sa[i]]=i;
        for(int i=n;i-->0;){
            while(!stack.empty()&&rank[stack.back()]>rank[i])stack.pop_back();
            const int j=stack.empty()?n:stack.back();stack.push_back(i);
            if(j==n)continue;
            const int period=j-i,left=backward.lce(n-i,n-j),right=forward.lce(i,j);
            if(left+right>=period)result.push_back({period,i-left,j+right});
        }
    }
    const auto less=[](const auto&a,const auto&b){return std::tie(a.period,a.left,a.right)<std::tie(b.period,b.left,b.right);};
    std::sort(result.begin(),result.end(),less);result.erase(std::unique(result.begin(),result.end()),result.end());return result;
}
} // namespace cp

#pragma once
#include <algorithm>
#include <climits>
#include <string_view>
#include <utility>
#include <vector>
#include "eertree.hpp"
namespace cp {
struct PalindromePartitionResult {std::vector<int> prefix_minimum;std::vector<std::pair<int,int>> intervals;};
// Series linksによる最小回文分割。prefix_minimum[0]=0。O(n log n)。
inline PalindromePartitionResult palindrome_partition(std::string_view text){
    const int n=string_algorithm_detail::checked_size(text.size());if(n>INT_MAX-2)throw std::length_error("palindrome partition text too long");Eertree tree;
    std::vector<int> difference(2),series(2),best(2),origin(2),parent(text.size()+1);
    PalindromePartitionResult result;result.prefix_minimum.assign(text.size()+1,n+1);result.prefix_minimum[0]=0;
    for(int i=1;i<=n;++i){
        const int last=tree.append(static_cast<unsigned char>(text[i-1]));const auto& nodes=tree.nodes();
        if(difference.size()<nodes.size()){
            const int v=int(nodes.size())-1,link=nodes[v].link,d=nodes[v].length-nodes[link].length;
            difference.push_back(d);series.push_back(d==difference[link]?series[link]:link);best.push_back(n);origin.push_back(0);
        }
        for(int v=last;nodes[v].length>0;v=series[v]){
            const int start=i-(nodes[series[v]].length+difference[v]);best[v]=result.prefix_minimum[start];origin[v]=start;
            const int link=nodes[v].link;if(difference[v]==difference[link]&&best[link]<best[v]){best[v]=best[link];origin[v]=origin[link];}
            if(best[v]+1<result.prefix_minimum[i]){result.prefix_minimum[i]=best[v]+1;parent[i]=origin[v];}
        }
    }
    for(int end=n;end;end=parent[end])result.intervals.emplace_back(parent[end],end);
    std::reverse(result.intervals.begin(),result.intervals.end());return result;
}
} // namespace cp

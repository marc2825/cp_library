#pragma once
#include <algorithm>
#include <climits>
#include <cstddef>
#include <vector>
#include "../suffix_array.hpp"
#include "../lcp_array.hpp"

namespace cp { namespace string_algorithm_detail {
struct LCEIndex {
    static int checked_length(std::size_t length){if(length>=static_cast<std::size_t>(INT_MAX))throw std::length_error("LCE text must have length < INT_MAX");return int(length);}
    int n;
    std::vector<int> sa, rank, logs;
    std::vector<std::vector<int>> table;
    explicit LCEIndex(const std::vector<int>& text) : n(checked_length(text.size())), sa(suffix_array(text)), rank(n), logs(text.size()+1) {
        for (int i=0;i<n;++i) rank[sa[i]]=i;
        for (int i=2;i<=n;++i) logs[i]=logs[i/2]+1;
        table.push_back(lcp_array(text,sa));
        for (std::size_t width=2;width<=table[0].size();width*=2) {
            std::vector<int> row(table[0].size()-width+1);
            for(std::size_t i=0;i<row.size();++i)row[i]=std::min(table.back()[i],table.back()[i+width/2]);
            table.push_back(std::move(row));
        }
    }
    int lce(int i,int j) const {
        if(i==j)return n-i;
        if(i==n||j==n)return 0;
        int l=rank[i],r=rank[j];if(l>r)std::swap(l,r);
        const int k=logs[r-l];return std::min(table[k][l],table[k][r-(1<<k)]);
    }
};
} } // namespace cp::string_algorithm_detail

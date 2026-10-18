#pragma once
#include <algorithm>
#include <string_view>
#include "detail/common.hpp"
namespace cp {
// unsigned byte順。最小巡回シフトの最初の添字、空は0。O(n)、O(1)。
inline int minimum_rotation(std::string_view s){
    string_algorithm_detail::checked_size(s.size());const auto n=s.size();if(!n)return 0;
    std::size_t i=0,j=1,k=0;
    while(i<n&&j<n&&k<n){const auto a=static_cast<unsigned char>(s[(i+k)%n]),b=static_cast<unsigned char>(s[(j+k)%n]);if(a==b){++k;continue;}if(a>b){i+=k+1;if(i<=j)i=j+1;}else{j+=k+1;if(j<=i)j=i+1;}k=0;}
    return int(std::min(i,j));
}
} // namespace cp

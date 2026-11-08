#pragma once
#include <algorithm>
#include <climits>
#include <string_view>
#include <vector>
#include "detail/common.hpp"
#include "../convolution/convolution_ll.hpp"

namespace cp {
// wildcardは両入力で任意の1byteに一致（既定'?'）。'*'による可変長は扱わない。
// 3本の正確な整数畳み込み。通常の結果長<=2^24。
inline std::vector<int> wildcard_pattern_matching(std::string_view text,std::string_view pattern,unsigned char wildcard='?'){
    const int n=string_algorithm_detail::checked_size(text.size()),m=string_algorithm_detail::checked_size(pattern.size());
    if(!m){std::vector<int> result;for(std::size_t i=0;i<=text.size();++i)result.push_back(int(i));return result;}
    if(n<m)return {};
    convolution_detail::shape(text.size(),pattern.size(),std::size_t(1)<<24);
    std::vector<long long>a(n),a2(n),a3(n),b(m),b2(m),b3(m);
    for(int i=0;i<n;++i){a[i]=static_cast<unsigned char>(text[i])==wildcard?0:static_cast<unsigned char>(text[i])+1;a2[i]=a[i]*a[i];a3[i]=a2[i]*a[i];}
    for(int i=0;i<m;++i){b[m-1-i]=static_cast<unsigned char>(pattern[i])==wildcard?0:static_cast<unsigned char>(pattern[i])+1;}
    for(int i=0;i<m;++i){b2[i]=b[i]*b[i];b3[i]=b2[i]*b[i];}
    const auto x=convolution_ll(a3,b),y=convolution_ll(a2,b2),z=convolution_ll(a,b3);
    std::vector<int> result;for(int i=0;i<=n-m;++i){const auto at=i+m-1;if(x[at]-2*y[at]+z[at]==0)result.push_back(i);}return result;
}
} // namespace cp

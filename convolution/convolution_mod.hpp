#pragma once
#include <cstdint>
#include <stdexcept>
#include <type_traits>
#include <vector>
#include "convolution.hpp"

namespace cp {
// 任意の法1..2e9。3 NTT + 混合基数CRT。非空の結果長<=2^24。
// 入力は64bit以下の標準整数、負数も正規化。docs/convolution/convolution_mod.md
template<class T,class U> std::vector<int> convolution_mod(const std::vector<T>& a,const std::vector<U>& b,int modulus) {
    static_assert(std::is_integral_v<T> && std::is_integral_v<U> && !std::is_same_v<T,bool> && !std::is_same_v<U,bool> && sizeof(T)<=8 && sizeof(U)<=8,"integer coefficients required");
    if(modulus<1 || modulus>2000000000) throw std::invalid_argument("convolution modulus must be in [1,2e9]");
    auto shape=convolution_detail::shape(a.size(),b.size(),std::size_t(1)<<24); if(!shape.first) return {};
    std::vector<int> x(a.size()),y(b.size());
    auto normalize=[&](auto v) { using V=decltype(v); if constexpr(std::is_signed_v<V>) { auto r=static_cast<long long>(v)%modulus; return int(r<0?r+modulus:r); } else return int(static_cast<std::uint64_t>(v)%static_cast<unsigned>(modulus)); };
    for(std::size_t i=0;i<a.size();++i) x[i]=normalize(a[i]);
    for(std::size_t i=0;i<b.size();++i) y[i]=normalize(b[i]);
    if(modulus==1) return std::vector<int>(shape.first);
    constexpr int p0=167772161,p1=469762049,p2=1224736769;
    constexpr long long p01=static_cast<long long>(p0)*p1;
    constexpr auto inv0=convolution_detail::power(p0,p1-2,p1),inv01=convolution_detail::power(p01%p2,p2-2,p2);
    auto c0=convolution<p0>(x,y),c1=convolution<p1>(x,y),c2=convolution<p2>(x,y);
    std::vector<int> result(shape.first);
    for(std::size_t i=0;i<result.size();++i) {
        auto t1=(c1[i]-static_cast<long long>(c0[i])+p1)%p1*inv0%p1;
        auto low=c0[i]+p0*t1;
        auto t2=(c2[i]-low%p2+p2)%p2*inv01%p2;
        result[i]=int((low%modulus+(p01%modulus)*t2)%modulus);
    }
    return result;
}
} // namespace cp

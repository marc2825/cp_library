#pragma once
#include <cstdint>
#include <limits>
#include <stdexcept>
#include "nim_product.hpp"
namespace cp {
inline std::uint64_t nimber_power(std::uint64_t a,std::uint64_t exponent){std::uint64_t result=1;for(;exponent;exponent>>=1){if(exponent&1)result=nim_product(result,a);if(exponent>1)a=nim_product(a,a);}return result;}
// 64bit nimberはGF(2^64)。a^(2^64-2)。0はdomain_error。
inline std::uint64_t nimber_inverse(std::uint64_t a){if(!a)throw std::domain_error("zero nimber has no inverse");return nimber_power(a,std::numeric_limits<std::uint64_t>::max()-1);}
inline std::uint64_t nimber_sqrt(std::uint64_t a){for(int i=0;i<63;++i)a=nim_product(a,a);return a;}
class Nimber {
    std::uint64_t value_;
public:
    explicit Nimber(std::uint64_t value=0):value_(value){}
    std::uint64_t value()const{return value_;}
    Nimber& operator+=(Nimber b){value_^=b.value_;return *this;}
    Nimber& operator-=(Nimber b){return *this+=b;}
    Nimber& operator*=(Nimber b){value_=nim_product(value_,b.value_);return *this;}
    Nimber& operator/=(Nimber b){value_=nim_product(value_,nimber_inverse(b.value_));return *this;}
    friend Nimber operator+(Nimber a,Nimber b){return a+=b;}friend Nimber operator-(Nimber a,Nimber b){return a-=b;}
    friend Nimber operator*(Nimber a,Nimber b){return a*=b;}friend Nimber operator/(Nimber a,Nimber b){return a/=b;}
    friend bool operator==(Nimber a,Nimber b){return a.value_==b.value_;}friend bool operator!=(Nimber a,Nimber b){return !(a==b);}
    Nimber inverse()const{return Nimber(nimber_inverse(value_));}
    Nimber power(std::uint64_t exponent)const{return Nimber(nimber_power(value_,exponent));}
};
} // namespace cp

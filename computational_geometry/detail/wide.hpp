#pragma once
#include <algorithm>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

namespace cp { namespace geometry { namespace exact {
// GCC/Clangの128ビット整数。すべての演算で範囲を検査する。
__extension__ typedef __int128 Wide;
__extension__ typedef unsigned __int128 UWide;
namespace arithmetic {
inline Wide add(Wide a,Wide b){Wide r;if(__builtin_add_overflow(a,b,&r))throw std::overflow_error("exact geometry addition overflow");return r;}
inline Wide sub(Wide a,Wide b){Wide r;if(__builtin_sub_overflow(a,b,&r))throw std::overflow_error("exact geometry subtraction overflow");return r;}
inline Wide mul(Wide a,Wide b){Wide r;if(__builtin_mul_overflow(a,b,&r))throw std::overflow_error("exact geometry multiplication overflow");return r;}
inline Wide neg(Wide a){return sub(0,a);}
inline Wide abs(Wide a){return a<0?neg(a):a;}
inline int sign(Wide a){return (a>0)-(a<0);}
inline UWide magnitude(Wide a){return a<0?UWide(-(a+1))+1:UWide(a);}
inline UWide gcd(UWide a,UWide b){while(b){const auto r=a%b;a=b;b=r;}return a;}
inline std::int64_t narrow(Wide a){if(a<std::numeric_limits<std::int64_t>::min()||a>std::numeric_limits<std::int64_t>::max())throw std::overflow_error("integer point coordinate overflow");return static_cast<std::int64_t>(a);}
}
inline std::string to_string(Wide a){const bool negative=a<0;auto x=arithmetic::magnitude(a);std::string s;do{s.push_back(char('0'+x%10));x/=10;}while(x);if(negative)s.push_back('-');std::reverse(s.begin(),s.end());return s;}
class Rational {
    Wide numerator_=0,denominator_=1;
    static int compare_positive(UWide a,UWide b,UWide c,UWide d){int direction=1;for(;;){const auto q=a/b,r=c/d;if(q!=r)return direction*(q<r?-1:1);a%=b;c%=d;if(!a||!c)return direction*((a>0)-(c>0));std::swap(a,b);std::swap(c,d);direction=-direction;}}
public:
    Rational()=default;
    Rational(Wide numerator,Wide denominator=1):numerator_(numerator),denominator_(denominator){
        if(!denominator)throw std::domain_error("rational denominator is zero");
        if(denominator_<0){numerator_=arithmetic::neg(numerator_);denominator_=arithmetic::neg(denominator_);}
        const auto g=arithmetic::gcd(arithmetic::magnitude(numerator_),UWide(denominator_));numerator_/=Wide(g);denominator_/=Wide(g);
    }
    Wide numerator()const{return numerator_;} Wide denominator()const{return denominator_;}
    int sign()const{return arithmetic::sign(numerator_);}
    long double approximate()const{return static_cast<long double>(numerator_)/static_cast<long double>(denominator_);}
    std::string str()const{return to_string(numerator_)+(denominator_==1?"":"/"+to_string(denominator_));}
    friend Rational operator-(const Rational&a){return {arithmetic::neg(a.numerator_),a.denominator_};}
    friend Rational operator+(const Rational&a,const Rational&b){const Wide g=Wide(arithmetic::gcd(UWide(a.denominator_),UWide(b.denominator_)));const Wide x=b.denominator_/g,y=a.denominator_/g;return {arithmetic::add(arithmetic::mul(a.numerator_,x),arithmetic::mul(b.numerator_,y)),arithmetic::mul(a.denominator_,x)};}
    friend Rational operator-(const Rational&a,const Rational&b){return a+(-b);}
    friend Rational operator*(const Rational&a,const Rational&b){const Wide g=Wide(arithmetic::gcd(arithmetic::magnitude(a.numerator_),UWide(b.denominator_))),h=Wide(arithmetic::gcd(arithmetic::magnitude(b.numerator_),UWide(a.denominator_)));return {arithmetic::mul(a.numerator_/g,b.numerator_/h),arithmetic::mul(a.denominator_/h,b.denominator_/g)};}
    friend Rational operator/(const Rational&a,const Rational&b){if(!b.numerator_)throw std::domain_error("rational division by zero");return a*Rational(b.denominator_,b.numerator_);}
    friend bool operator==(const Rational&a,const Rational&b){return a.numerator_==b.numerator_&&a.denominator_==b.denominator_;}
    friend bool operator!=(const Rational&a,const Rational&b){return !(a==b);}
    friend bool operator<(const Rational&a,const Rational&b){if(a.sign()!=b.sign())return a.sign()<b.sign();if(!a.sign())return false;return a.sign()*compare_positive(arithmetic::magnitude(a.numerator_),UWide(a.denominator_),arithmetic::magnitude(b.numerator_),UWide(b.denominator_))<0;}
};
} } } // namespace cp::geometry::exact

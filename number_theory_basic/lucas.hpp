#pragma once
// 説明: docs/number_theory_basic/lucas.md
#include <cstdint>
#include <stdexcept>
#include "../combinatorics/binomial.hpp"
#include "../number_theory_convolution/miller_rabin.hpp"
namespace cp {
// 素数p、O(p)表。n,kはuint64全域。各p進桁にLucasを適用。
class LucasBinomial {
    int prime_;FactorialTable table_;
    static int checked(int p){if(p<2 || !is_prime_64(static_cast<std::uint64_t>(p)))throw std::invalid_argument("Lucas modulus must be prime");
    return p;}
public:
    explicit LucasBinomial(int prime):prime_(checked(prime)),table_(prime_-1,prime_){}
    int modulus()const{return prime_;}
    long long operator()(std::uint64_t n,std::uint64_t k)const{if(k>n)return 0;long long result=1;while(n || k){auto a=n%prime_,b=k%prime_;if(b>a)return 0;result=mod_multiply(result,binomial(table_,static_cast<long long>(a),static_cast<long long>(b)),prime_);n/=prime_;k/=prime_;}return result;}
};
} // namespace cp

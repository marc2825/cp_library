#pragma once
// 説明: docs/number_theory_convolution/prime_counting.md
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>
#include "../basic_utility/integer_kth_root.hpp"
#include "../basic_utility/quotient_enumeration.hpp"
namespace cp {
// Lucy DP。Nの商floor(N/i)についてpiを保持。領域O(sqrt N)。
class PrimeCounting {
    std::uint64_t n_,root_;std::vector<std::uint64_t> values_,counts_;std::vector<std::size_t> small_,large_;
    std::size_t index(std::uint64_t x)const{return x<=root_?small_[x]:large_[n_/x];}
public:
    explicit PrimeCounting(std::uint64_t n,std::uint64_t max_root=5000000):n_(n),root_(integer_kth_root(n,2)){
        if(root_>max_root)throw std::length_error("prime counting root budget exceeded");
    small_.resize(static_cast<std::size_t>(root_)+1);large_.resize(static_cast<std::size_t>(root_)+1);
        for_each_quotient(n,[&](auto,auto,std::uint64_t q){auto id=values_.size();values_.push_back(q);counts_.push_back(q-1);if(q<=root_)small_[q]=id;else large_[n_/q]=id;});
        for(std::uint64_t p=2;p<=root_;++p){auto before=counts_[index(p-1)];if(counts_[index(p)]==before)continue;for(std::size_t i=0;i<values_.size() && values_[i]>=p*p;++i)counts_[i]-=counts_[index(values_[i]/p)]-before;}
    }
    std::uint64_t count()const{return n_?counts_[0]:0;}
    std::uint64_t count_quotient(std::uint64_t x)const{if(!x)return 0;if(x>n_ || n_/(n_/x)!=x)throw std::invalid_argument("not a floor quotient of N");
    return counts_[index(x)];}
    const std::vector<std::uint64_t>& quotients()const{return values_;}
};
inline std::uint64_t prime_count(std::uint64_t n,std::uint64_t max_root=5000000){return PrimeCounting(n,max_root).count();}
} // namespace cp

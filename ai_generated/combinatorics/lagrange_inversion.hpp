#pragma once
// 説明: docs/combinatorics/lagrange_inversion.md
#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <vector>
#include "../fps/pow.hpp"
namespace cp {
// g=x*phi(g)のとき[x^n]F(g)=[t^(n-1)]F'(t)*phi(t)^n/n。
template<int Mod=998244353,int Root=3> int lagrange_inversion_coefficient(const std::vector<int>& phi,const std::vector<int>& function,std::size_t n){
    using Field=fps_detail::Field<Mod>;if(n==0)return function.empty()?0:Field::norm(function[0]);if(n>=static_cast<std::size_t>(Mod))throw std::domain_error("Lagrange inversion requires n<modulus");
    auto power=fps_pow<Mod,Root>(phi,n,n);int result=0;for(std::size_t i=1;i<function.size() && i<=n;++i)result=Field::add(result,Field::mul(Field::mul(static_cast<int>(i),Field::norm(function[i])),power[n-i]));return Field::mul(result,Field::inv(static_cast<int>(n)));
}
} // namespace cp

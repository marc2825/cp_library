#pragma once
// 説明: docs/dp_optimization/four_russians.md
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>
namespace cp {
using BooleanMatrix=std::vector<std::vector<bool>>;
// Boolean積OR_k(A[i,k] AND B[k,j])。内側次元をb bitに分けたM4RM。
inline BooleanMatrix boolean_matrix_product_four_russians(const BooleanMatrix& a,const BooleanMatrix& b,unsigned block_bits=0){
    std::size_t m=a.size(),k=b.size(),n=b.empty()?0:b[0].size();for(const auto& row:a)if(row.size()!=k)throw std::invalid_argument("Boolean matrix inner dimension");
    for(const auto& row:b)if(row.size()!=n)throw std::invalid_argument("ragged Boolean matrix");
    if(block_bits>12)throw std::invalid_argument("Four Russians block bits must be <=12");
    if(!block_bits){block_bits=1;while(block_bits<12 && (std::size_t(1)<<(block_bits+1))<=std::max<std::size_t>(m,2))++block_bits;}
    std::size_t words=(n+63)/64;std::vector<std::vector<std::uint64_t>> packed(k,std::vector<std::uint64_t>(words)),output(m,std::vector<std::uint64_t>(words));for(std::size_t i=0;i<k;++i)for(std::size_t j=0;j<n;++j)if(b[i][j])packed[i][j/64]|=std::uint64_t(1)<<(j%64);
    for(std::size_t start=0;start<k;start+=block_bits){unsigned width=static_cast<unsigned>(std::min<std::size_t>(block_bits,k-start));std::size_t states=std::size_t(1)<<width;std::vector<std::vector<std::uint64_t>> table(states,std::vector<std::uint64_t>(words));for(std::size_t mask=1;mask<states;++mask){unsigned bit=static_cast<unsigned>(__builtin_ctz(static_cast<unsigned>(mask)));auto rest=mask&(mask-1);for(std::size_t w=0;w<words;++w)table[mask][w]=table[rest][w]|packed[start+bit][w];}for(std::size_t i=0;i<m;++i){unsigned mask=0;for(unsigned bit=0;bit<width;++bit)if(a[i][start+bit])mask|=1u<<bit;for(std::size_t w=0;w<words;++w)output[i][w]|=table[mask][w];}}
    BooleanMatrix result(m,std::vector<bool>(n));for(std::size_t i=0;i<m;++i)for(std::size_t j=0;j<n;++j)result[i][j]=bool(output[i][j/64]>>(j%64)&1);return result;
}
} // namespace cp

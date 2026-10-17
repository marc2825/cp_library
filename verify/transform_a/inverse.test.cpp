#define PROBLEM "https://judge.yosupo.jp/problem/inverse_matrix"
#include <iostream>
#include "linear_algebra/inverse.hpp"
int main(){std::size_t n;std::cin>>n;cp::Matrix<int>a(n,n);for(std::size_t i=0;i<n;++i)for(std::size_t j=0;j<n;++j)std::cin>>a(i,j);auto b=cp::matrix_inverse_mod(a);if(!b){std::cout<<-1<<'\n';return 0;}for(std::size_t i=0;i<b->rows();++i){for(std::size_t j=0;j<b->cols();++j)std::cout<<(*b)(i,j)<<' ';std::cout<<'\n';}}

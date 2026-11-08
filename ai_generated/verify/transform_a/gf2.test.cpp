#define PROBLEM "https://judge.yosupo.jp/problem/system_of_linear_equations_mod_2"
#include <bitset>
#include <iostream>
#include <string>
#include <vector>
#include "linear_algebra/gf2_gauss.hpp"
int main(){std::size_t n,m;std::cin>>n>>m;if(m>4096)return 1;std::vector<std::bitset<4096>>a(n);std::vector<int>b(n);std::string s;for(auto&row:a){std::cin>>s;for(std::size_t j=0;j<m;++j)row[j]=s[j]=='1';}std::cin>>s;for(std::size_t i=0;i<n;++i)b[i]=s[i]-'0';auto r=cp::gf2_gauss_jordan(a,b,m);if(!r.has_solution()){std::cout<<-1<<'\n';return 0;}std::cout<<r.nullspace_basis.size()<<'\n';for(std::size_t j=0;j<m;++j)std::cout<<r.particular[j];std::cout<<'\n';for(const auto&v:r.nullspace_basis){for(std::size_t j=0;j<m;++j)std::cout<<v[j];std::cout<<'\n';}}

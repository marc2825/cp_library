#include <bitset>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>
#include "convolution/all.hpp"
#include "set_function/all.hpp"
#include "linear_algebra/all.hpp"

template <class T> std::vector<T> read_vector(std::size_t n) { std::vector<T> a(n); for (auto& x : a) std::cin >> x; return a; }
cp::Matrix<int> read_matrix(std::size_t n, std::size_t m) { cp::Matrix<int> a(n, m); for (std::size_t i=0;i<n;++i) for (std::size_t j=0;j<m;++j) std::cin>>a(i,j); return a; }
template <class T> void print(const std::vector<T>& a) { std::cout<<a.size(); for(const auto& x:a)std::cout<<' '<<x; std::cout<<'\n'; }
template <int P> void algebra(char op) {
    std::size_t n; std::cin>>n;
    if(op=='S') {
        std::size_t r,s;std::cin>>r>>s;const auto a=read_matrix(r,n),b=read_matrix(s,n);const auto c=cp::linear_space_intersection_mod<P>(a,b);
        std::cout<<c.rows()<<' '<<c.cols();for(auto x:c.data())std::cout<<' '<<x;std::cout<<'\n';return;
    }
    const auto a=read_matrix(n,n);
    if(op=='C'){print(cp::characteristic_polynomial_mod<P>(a));return;}
    const auto b=cp::matrix_inverse_mod<P>(a);
    if(!b)std::cout<<"-1\n";else print(b->data());
}
int main(){
    std::ios::sync_with_stdio(false);std::cin.tie(nullptr);char op;
    while(std::cin>>op){
        if(op=='K'){std::size_t n,m;std::cin>>n>>m;auto a=read_vector<long long>(n),b=read_vector<long long>(m);print(cp::convolution_karatsuba(a,b));}
        else if(op=='W'){std::size_t n;bool inverse;std::cin>>n>>inverse;auto a=read_vector<long long>(n);cp::fwht(a,inverse);print(a);}
        else if(op=='B'){std::size_t n;int operation;std::cin>>n>>operation;auto a=read_vector<long long>(n),b=read_vector<long long>(n);print(operation==0?cp::bitwise_and_convolution(a,b):operation==1?cp::bitwise_or_convolution(a,b):cp::bitwise_xor_convolution(a,b));}
        else if(op=='R'){std::size_t n;std::cin>>n;auto a=read_vector<long long>(n);auto r=cp::ranked_zeta_transform(a);std::vector<long long> flat;for(const auto& row:r)flat.insert(flat.end(),row.begin(),row.end());print(flat);}
        else if(op=='I'||op=='C'||op=='S'){int p;std::cin>>p;if(p==2)algebra<2>(op);else if(p==3)algebra<3>(op);else if(p==5)algebra<5>(op);else algebra<998244353>(op);}
        else if(op=='G'){
            std::size_t n,m;std::cin>>n>>m;std::vector<std::bitset<129>> a(n);
            for(auto& row:a){std::string s;if(m)std::cin>>s;for(std::size_t j=0;j<m;++j)row[j]=s[j]=='1';}
            auto b=read_vector<int>(n);auto r=cp::gf2_gauss_jordan(a,b,m);std::cout<<r.rank<<' '<<r.has_solution();
            if(r.has_solution()){std::cout<<' '<<r.particular.to_string()<<' '<<r.nullspace_basis.size();for(auto v:r.nullspace_basis)std::cout<<' '<<v.to_string();}std::cout<<'\n';
        }else if(op=='X'){
            std::size_t n,m;std::cin>>n>>m;auto a=read_vector<std::uint64_t>(n),b=read_vector<std::uint64_t>(m);cp::XorBasis<> u,v;for(auto x:a)u.insert(x);for(auto x:b)v.insert(x);print(cp::xor_space_intersection(u,v).basis());
        }
    }
}

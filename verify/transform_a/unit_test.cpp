#include <algorithm>
#include <bitset>
#include <cmath>
#include <complex>
#include <cstdint>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>
#include "convolution/all.hpp"
#include "set_function/all.hpp"
#include "linear_algebra/all.hpp"

void check(bool ok){if(!ok)throw std::runtime_error("transform A check failed");}
template <class E, class F> void throws(F f){bool caught=false;try{f();}catch(const E&){caught=true;}check(caught);}
struct Ring {
    int x; Ring(long long a=0):x(int((a%12+12)%12)){}
    Ring& operator+=(Ring y){x=(x+y.x)%12;return *this;}
    Ring& operator-=(Ring y){x=(x-y.x+12)%12;return *this;}
    friend Ring operator*(Ring a,Ring b){return Ring(a.x*b.x);}
    friend bool operator==(Ring a,Ring b){return a.x==b.x;}
}; // 除算のない合成数法の環。
struct Count {
    long long x;inline static std::size_t products=0;Count(long long a=0):x(a){}
    Count& operator+=(Count y){x+=y.x;return *this;}Count& operator-=(Count y){x-=y.x;return *this;}
    friend Count operator*(Count a,Count b){++products;return Count(a.x*b.x);}
};
int main(){
    std::mt19937 rng(316320);
    for(int t=0;t<300;++t){
        std::size_t n=rng()%170,m=rng()%170;std::vector<long long>a(n),b(m),want(n&&m?n+m-1:0);
        for(auto& x:a)x=int(rng()%11)-5;
        for(auto& x:b)x=int(rng()%11)-5;
        for(std::size_t i=0;i<n;++i)for(std::size_t j=0;j<m;++j)want[i+j]+=a[i]*b[j];
        check(cp::convolution_karatsuba(a,b)==want);
    }
    std::vector<Count> ones(1024,Count(1));Count::products=0;auto counts=cp::convolution_karatsuba(ones,ones);
    check(Count::products<400000);for(std::size_t i=0;i<counts.size();++i)check(counts[i].x==static_cast<long long>(std::min(i+1,2047-i)));
    std::vector<Ring> rings(65,Ring(11));auto rc=cp::convolution_karatsuba(rings,rings);check(rc[64]==Ring(65));
    for(std::size_t n=1;n<=256;n*=2){
        std::vector<long long>a(n),b(n);for(auto&x:a)x=int(rng()%7)-3;for(auto&x:b)x=int(rng()%7)-3;
        auto f=a;cp::fwht(f);for(std::size_t i=0;i<n;++i){long long w=0;for(std::size_t j=0;j<n;++j){auto bits=i&j;int parity=0;while(bits){parity^=1;bits&=bits-1;}w+=(parity?-1:1)*a[j];}check(f[i]==w);}cp::fwht(f,true);check(f==a);
        for(int op=0;op<3;++op){std::vector<long long>w(n);for(std::size_t i=0;i<n;++i)for(std::size_t j=0;j<n;++j)w[op==0?(i&j):op==1?(i|j):(i^j)]+=a[i]*b[j];check(w==(op==0?cp::bitwise_and_convolution(a,b):op==1?cp::bitwise_or_convolution(a,b):cp::bitwise_xor_convolution(a,b)));}
        auto ranks=cp::ranked_zeta_transform(a);cp::ranked_mobius_transform(ranks);for(std::size_t s=0;s<n;++s){auto x=s;std::size_t rank=0;while(x){++rank;x&=x-1;}for(std::size_t k=0;k<ranks.size();++k)check(ranks[k][s]==(k==rank?a[s]:0));}
        std::vector<long long> subset(n);for(std::size_t s=0;s<n;++s){auto t=s;while(true){subset[s]+=a[t]*b[s^t];if(!t)break;t=(t-1)&s;}}check(cp::subset_convolution(a,b)==subset);
    }
    check(cp::bitwise_and_convolution(std::vector<Ring>{1,2},std::vector<Ring>{3,4})==std::vector<Ring>({1,8}));
    check(cp::bitwise_or_convolution(std::vector<Ring>{1,2},std::vector<Ring>{3,4})==std::vector<Ring>({3,6}));
    using F=cp::linear_algebra_detail::PrimeField<998244353>;
    std::vector<F> ma{1,2,3,4},mb{5,6,7,8};auto mc=cp::bitwise_xor_convolution(ma,mb);check(mc[0]==F(70));
    std::vector<long long> large(1<<16,1),identity(1<<16);identity[0]=1;check(cp::bitwise_xor_convolution(large,identity)==large);
    std::vector<std::bitset<129>> a(129);std::vector<int>b(129);for(std::size_t i=0;i<129;++i){a[i][i]=true;b[i]=i%2;}
    auto g=cp::gf2_gauss_jordan(a,b);check(g.has_solution()&&g.rank==129&&g.nullspace_basis.empty());for(std::size_t i=0;i<129;++i)check(g.particular[i]==bool(i%2));check(cp::gf2_rank(a)==129);
    auto free=cp::gf2_gauss_jordan(std::vector<std::bitset<129>>{},{});check(free.nullspace_basis.size()==129);
    check(cp::gf2_gauss_jordan(std::vector<std::bitset<0>>{},{}).has_solution());
    check(!cp::gf2_gauss_jordan(std::vector<std::bitset<0>>(1),{1}).has_solution());
    auto inverse=cp::matrix_inverse(cp::Matrix<double>{{0,2},{1,3}});check(inverse.has_value());auto product=cp::matrix_multiply(cp::Matrix<double>{{0,2},{1,3}},*inverse);for(std::size_t i=0;i<2;++i)for(std::size_t j=0;j<2;++j)check(std::abs(product(i,j)-(i==j))<1e-12);
    check(!cp::matrix_inverse(cp::Matrix<double>{{1,2},{2,4}}));check(cp::matrix_inverse_mod(cp::Matrix<int>(0,0)).has_value());
    auto real=cp::linear_space_intersection(cp::Matrix<double>{{1,0,0},{0,1,0}},cp::Matrix<double>{{0,1,0},{0,0,1}});check(real.rows()==1&&std::abs(real(0,0))+std::abs(real(0,2))<1e-12&&std::abs(real(0,1))>0.5);
    using Z=std::complex<double>;auto complex=cp::matrix_inverse(cp::Matrix<Z>{{Z(0,1)}});check(complex&&std::abs((*complex)(0,0)-Z(0,-1))<1e-12);
    check(cp::characteristic_polynomial(cp::Matrix<double>{{1,2},{3,4}})==std::vector<double>({-2,-5,1}));
    auto ch=cp::characteristic_polynomial_mod<2>(cp::Matrix<int>{{0,0,1},{1,0,0},{0,1,0}});check(ch==std::vector<int>({1,0,0,1}));
    throws<std::invalid_argument>([]{std::vector<int>x;cp::fwht(x);});throws<std::invalid_argument>([]{std::vector<int>x(3);cp::fwht(x);});
    throws<std::invalid_argument>([]{cp::bitwise_or_convolution(std::vector<int>(2),std::vector<int>(4));});
    throws<std::domain_error>([]{std::vector<int>x{1,0};cp::fwht(x,true);});
    throws<std::overflow_error>([]{std::vector<std::int8_t>x{127,1};cp::fwht(x);});
    throws<std::overflow_error>([]{cp::convolution_karatsuba(std::vector<long long>{std::numeric_limits<long long>::max()},std::vector<long long>{2});});
    throws<std::overflow_error>([]{std::vector<unsigned>x{0,1};cp::fwht(x);});
    throws<std::invalid_argument>([]{std::vector<std::vector<int>>x{{1,2},{1}};cp::ranked_mobius_transform(x);});
    throws<std::invalid_argument>([]{cp::matrix_inverse_mod(cp::Matrix<int>(1,2));});
    throws<std::invalid_argument>([]{cp::characteristic_polynomial_mod(cp::Matrix<int>(1,2));});
    throws<std::invalid_argument>([]{cp::linear_space_intersection_mod(cp::Matrix<int>(0,1),cp::Matrix<int>(0,2));});
    throws<std::invalid_argument>([]{cp::matrix_inverse(cp::Matrix<double>{{1}},0.0);});
    throws<std::invalid_argument>([]{cp::matrix_inverse(cp::Matrix<double>{{std::numeric_limits<double>::infinity()}});});
    throws<std::invalid_argument>([]{cp::gf2_gauss_jordan(std::vector<std::bitset<4>>{std::bitset<4>(8)},{0},3);});
    throws<std::invalid_argument>([]{cp::gf2_gauss_jordan(std::vector<std::bitset<4>>(1),{2});});
    throws<std::invalid_argument>([]{cp::gf2_gauss_jordan(std::vector<std::bitset<4>>{}, {},5);});
    std::cout<<"C++: recursive products, ring12, FWHT definition, subset reuse, 129-bit GF(2), fields, boundaries and exceptions passed\n";
}

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <numeric>
#include <optional>
#include <random>
#include <vector>
#include "number_theory_basic/all.hpp"
#include "number_theory_convolution/all.hpp"
#define CHECK(x) do{if(!(x)){std::cerr<<"line "<<__LINE__<<": "<<#x<<'\n';std::abort();}}while(false)
int main(){
 for(int m=1;m<=180;++m)for(int a=-2;a<=m+1;++a){std::vector<std::optional<std::uint64_t>> first(m);long long value=1%m;std::uint64_t exponent=0;while(!first[value]){first[value]=exponent++;value=cp::mod_multiply(value,a,m);}for(int b=-1;b<=m;++b)CHECK(cp::discrete_log(a,b,m)==first[(b%m+m)%m]);}
 for(int p=2;p<=600;++p)if(cp::is_prime_64(p))for(int a=-p;a<p;++a){std::optional<long long> expected;for(long long x=0;x<p;++x)if(x*x%p==(a%p+p)%p){expected=x;break;}CHECK(cp::mod_sqrt(a,p)==expected);}
 for(int m=1;m<=80;++m)for(int a=-10;a<=10;++a)for(int b=-10;b<=10;++b){std::vector<long long> expected;for(int x=0;x<m;++x)if(((a*x-b)%m+m)%m==0)expected.push_back(x);auto r=cp::linear_congruence(a,b,m);CHECK(bool(r)==!expected.empty());if(r){std::vector<long long> actual;for(long long t=0;t<r->count;++t)actual.push_back(r->first+t*r->step);CHECK(actual==expected);}}
 for(int p:{2,3,5,7,11,17,101}){cp::LucasBinomial lucas(p);std::vector<long long> row{1};for(int n=0;n<=350;++n){for(int k=0;k<=n+1;++k)CHECK(lucas(n,k)==(k<=n?row[k]:0));std::vector<long long> next(row.size()+1,1);for(std::size_t k=1;k<row.size();++k)next[k]=(row[k-1]+row[k])%p;row=std::move(next);}CHECK(lucas(std::numeric_limits<std::uint64_t>::max(),0)==1);}
 for(int a=0;a<=8;++a)for(int h=0;h<=3;++h){std::uint64_t tower=1;bool fits=true;for(int i=0;i<h;++i){std::uint64_t next=1;for(std::uint64_t k=0;k<tower;++k){if(a && next>std::numeric_limits<std::uint64_t>::max()/a){fits=false;break;}next*=a;}if(!fits)break;tower=next;}if(fits)for(int m=1;m<=240;++m)CHECK(cp::mod_tetration(a,h,m)==static_cast<long long>(tower%m));}
 for(int m=1;m<=100;++m){CHECK(cp::mod_tetration(0,1000000000000000000ULL,m)==1%m);CHECK(cp::mod_tetration(0,1000000000000000001ULL,m)==0);CHECK(cp::mod_tetration(1,std::numeric_limits<std::uint64_t>::max(),m)==1%m);CHECK(cp::mod_tetration(2,1000000000000000000ULL,m)==cp::mod_tetration(2,100,m));}
 cp::PrimeSieve sieve(20000);std::vector<std::uint64_t> prime_prefix(20001);for(int i=1;i<=20000;++i)prime_prefix[i]=prime_prefix[i-1]+sieve.is_prime(i);
 for(int n=0;n<=2500;++n){cp::PrimeCounting lucy(n);CHECK(lucy.count()==prime_prefix[n]);for(auto q:lucy.quotients())CHECK(lucy.count_quotient(q)==prime_prefix[q]);}
 CHECK(cp::prime_count(1000000000)==50847534);
 cp::SummatoryTotient phi(32);cp::MertensFunction mertens(32);cp::OptimizationWide phi_sum=0,mu_sum=0,div_sum=0;std::uint64_t square_free=0;
 for(int n=1;n<=20000;++n){int coprime=0;for(int k=1;k<=n && n<=700;++k)coprime+=std::gcd(k,n)==1;int x=n,mu=1,tau=1;std::uint64_t totient=n;for(int p=2;p<=x/p;++p)if(x%p==0){int e=0;while(x%p==0){x/=p;++e;}tau*=e+1;mu=e>1?0:-mu;totient=totient/p*(p-1);}if(x>1){tau*=2;mu=-mu;totient=totient/x*(x-1);}if(n<=700)CHECK(totient==static_cast<std::uint64_t>(coprime));phi_sum+=totient;mu_sum+=mu;div_sum+=tau;square_free+=mu!=0;CHECK(phi(n)==phi_sum);CHECK(mertens(n)==mu_sum);CHECK(cp::divisor_count_sum(n)==div_sum);if(n<=1200 || n%123==0)CHECK(cp::square_free_count(n)==square_free);}
 auto sums=cp::multiplicative_function_sums<long long>(15000,[](int,int e,std::uint64_t){return e+1;});for(int n=1;n<=15000;++n){int tau=0;for(int d=1;d<=n/d;++d)if(n%d==0)tau+=d==n/d?1:2;CHECK(sums.values[n]==tau);CHECK(sums.prefix[n]==sums.prefix[n-1]+tau);}
 {auto max=std::numeric_limits<std::uint64_t>::max();CHECK(cp::integer_kth_root(max,2)==4294967295ULL);std::uint64_t covered=0;cp::for_each_quotient(1000000,[&](auto l,auto r,auto q){CHECK(l==covered+1);CHECK(1000000/l==q && 1000000/r==q);covered=r;});CHECK(covered==1000000);CHECK(cp::mod_sqrt(std::numeric_limits<long long>::min(),2)==0);bool rejected=false;try{cp::mod_sqrt(1,15);}catch(const std::invalid_argument&){rejected=true;}CHECK(rejected);rejected=false;try{cp::PrimeCounting too_big(max);}catch(const std::length_error&){rejected=true;}CHECK(rejected);}
 CHECK(phi(1000000)==cp::OptimizationWide(303963552392LL));CHECK(mertens(1000000)==212);CHECK(cp::square_free_count(1000000)==607926);
 std::cout<<"number theory: all modular residues, Lucas/Pascal, exact towers, prime sieve, divisor/coprimality/square-free definitions passed\n";
}

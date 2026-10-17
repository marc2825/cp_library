#include <algorithm>
#include <climits>
#include <cstdint>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <vector>
#include "number_theory_basic/all.hpp"
void check(bool ok){if(!ok)throw std::runtime_error("number-theory mismatch");}
template<class E,class F>void throws(F f){bool caught=false;try{f();}catch(const E&){caught=true;}check(caught);}
int main(){
    auto phi=cp::euler_phi_sieve(2500);auto mu=cp::mobius_sieve(2500);
    for(int n=1;n<=2500;++n){int count=0;for(int k=1;k<=n;++k)count+=std::gcd(n,k)==1;check(phi[n]==std::uint64_t(count)&&cp::euler_phi(n)==phi[n]);int divisor_sum=0;for(int d=1;d<=n;++d)if(n%d==0)divisor_sum+=mu[d];check(divisor_sum==(n==1)&&cp::mobius(n)==mu[n]);}
    check(cp::euler_phi_sieve(0)==std::vector<std::uint64_t>{0}&&cp::mobius_sieve(0)==std::vector<int>{0});
    for(int m=1;m<=18;++m)for(int n=1;n<=18;++n)for(int a=-3;a<=3;++a)for(int b=-3;b<=3;++b){
        auto result=cp::crt({a,b},{m,n});long long expected=-1,mod=std::lcm(m,n);
        for(int x=0;x<mod;++x)if((x-a)%m==0&&(x-b)%n==0){expected=x;break;}
        check(bool(result)==(expected>=0));if(result)check(result->remainder==expected&&result->modulus==mod);
        if(std::gcd(m,n)==1)for(int target:{1,2,7,25})check(cp::garner({a,b},{m,n},target)==expected%target);
    }
    for(int n=0;n<=12;++n)for(int m=1;m<=12;++m)for(int a=-18;a<=18;++a)for(int b=-18;b<=18;++b){long long expected=0;for(int i=0;i<n;++i){int x=a*i+b;expected+=x/m-(x%m<0);}check(cp::floor_sum(n,m,a,b)==expected);}
    cp::PrimeSieve primes(4000);
    for(int p:primes.primes()){
        auto root=cp::primitive_root(p);std::vector<bool> seen(p);long long x=1;
        for(int i=0;i<p-1;++i){check(!seen[x]);seen[x]=true;x=x*static_cast<long long>(root)%p;}
        check(x==1);for(std::uint64_t candidate=1;candidate<root;++candidate){x=1;int order=0;do{x=x*static_cast<long long>(candidate)%p;++order;}while(x!=1&&order<=p);check(order<p-1);}
    }
    check(cp::euler_phi(18446744073709551557ULL)==18446744073709551556ULL);
    check(cp::mobius(18446744073709551557ULL)==-1);
    check(cp::floor_sum(4294967295LL,4294967295LL,4294967294LL,4294967294LL)==9223372030412324865LL);
    check(cp::floor_sum(0,1,LLONG_MIN,LLONG_MAX)==0&&cp::floor_sum(1,1,LLONG_MIN,LLONG_MIN)==LLONG_MIN);
    auto identity=cp::crt({},{});check(identity&&identity->remainder==0&&identity->modulus==1&&cp::garner({},{},7)==0);
    check(cp::crt({LLONG_MIN},{LLONG_MAX})->remainder==LLONG_MAX-1);
    throws<std::invalid_argument>([]{cp::euler_phi(0);});throws<std::invalid_argument>([]{cp::mobius(0);});
    throws<std::invalid_argument>([]{cp::euler_phi_sieve(-1);});throws<std::invalid_argument>([]{cp::mobius_sieve(-1);});
    throws<std::invalid_argument>([]{cp::crt({0},{});});throws<std::invalid_argument>([]{cp::crt({0},{0});});
    throws<std::overflow_error>([]{cp::crt({0,0},{LLONG_MAX,2});});
    throws<std::invalid_argument>([]{cp::garner({1,1},{4,6},7);});throws<std::invalid_argument>([]{cp::garner({},{},0);});
    throws<std::invalid_argument>([]{cp::floor_sum(-1,1,0,0);});throws<std::invalid_argument>([]{cp::floor_sum(1,4294967296LL,0,0);});
    throws<std::overflow_error>([]{cp::floor_sum(3,1,LLONG_MAX,0);});throws<std::invalid_argument>([]{cp::primitive_root(15);});
    std::cout<<"phi/mobius divisor identities, exhaustive CRT/Garner/floor sums and prime generators passed\n";
}

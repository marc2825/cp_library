#include <algorithm>
#include <cstdint>
#include <iostream>
#include <limits>
#include <numeric>
#include <random>
#include <tuple>
#include <vector>
#include "common.hpp"
#include "dp_optimization/all.hpp"
#include "number_theory_convolution/pollard_rho.hpp"
#include "number_theory_basic/prime_sieve.hpp"
#include "set_function/subset_convolution.hpp"
#include "convolution/convolution_mod.hpp"
#include "verify/number_theory_convolution/modint.hpp"
std::mt19937_64 rng(3122026);
int pick(int n) { return int(rng()%n); }
template<class T> std::vector<T> naive_minplus(const std::vector<T>& a,const std::vector<T>& b) {
    if(a.empty() || b.empty()) return {};
    std::vector<T> result(a.size()+b.size()-1,std::numeric_limits<T>::max());
    for(std::size_t i=0;i<a.size();++i) for(std::size_t j=0;j<b.size();++j) result[i+j]=std::min(result[i+j],a[i]+b[j]);
    return result;
}
std::vector<long long> convex(int n) { std::vector<long long> a(n); long long d=-30; for(int i=1;i<n;++i) { d+=pick(6); a[i]=a[i-1]+d; } return a; }
void optimization() {
    for(int trial=0;trial<700;++trial) {
        int n=pick(70),m=pick(60); auto a=convex(n),b=convex(m); require(cp::min_plus_convolution_convex_convex(a,b)==naive_minplus(a,b),"both convex minplus");
        for(auto &x:a) x=pick(200)-100;
        auto want=naive_minplus(a,b); require(cp::min_plus_convolution_arbitrary_convex(a,b)==want && cp::min_plus_convolution_convex_arbitrary(b,a)==want,"one convex minplus");
        auto offsets=convex(m); std::vector<long long> row(n); for(auto &x:row) x=pick(21)-10;
        std::size_t calls=0;
        auto get=[&](int r,int c) { ++calls; long long d=r-c; return d*d+offsets[c]+row[r]; };
        auto slow=std::vector<int>(n,-1); if(m) for(int r=0;r<n;++r) { slow[r]=0; for(int c=1;c<m;++c) if(get(r,c)<get(r,slow[r])) slow[r]=c; }
        require(cp::monotone_minima(n,m,get)==slow,"monotone minima direct rows"); calls=0;
        require(cp::smawk(n,m,get)==slow,"SMAWK Monge rows"); require(calls<=20*std::size_t(n+m),"SMAWK linear evaluation count");
    }
    for(int n=0;n<50;++n) for(int m=0;m<50;++m) {
        auto same=[](int,int){return 7;}; auto a=cp::smawk(n,m,same); require(a==std::vector<int>(n,m?0:-1),"SMAWK leftmost ties");
    }
    rejects<std::invalid_argument>([]{cp::smawk(-1,2,[](int,int){return 0;});});
    for(int trial=0;trial<100;++trial) {
        std::vector<long long> xs; for(int i=-50;i<=50;++i) xs.push_back(i);
        cp::LiChaoTree<> tree(xs); cp::LiChaoTree<false> max_tree(xs);
        std::vector<std::tuple<long long,long long,long long,long long>> lines;
        for(int q=0;q<300;++q) {
            long long a=pick(31)-15,b=pick(101)-50,l=pick(121)-60,r=pick(121)-60; if(l>r) std::swap(l,r);
            if(pick(2)) { tree.add_line(a,b); max_tree.add_line(a,b); l=-100; r=100; }
            else { tree.add_segment(l,r,a,b); max_tree.add_segment(l,r,a,b); }
            lines.emplace_back(a,b,l,r);
            long long x=pick(101)-50; std::optional<long long> lo,hi;
            for(auto [aa,bb,ll,rr]:lines) if(ll<=x && x<rr) { auto y=aa*x+bb; if(!lo || y<*lo) lo=y; if(!hi || y>*hi) hi=y; }
            require(tree.query(x)==lo && max_tree.query(x)==hi,"Li Chao line/segment scan");
        }
    }
    cp::LiChaoTree<> empty({0}); require(!empty.query(0),"Li Chao empty envelope"); rejects<std::out_of_range>([&]{empty.query(1);});
    cp::LiChaoTree<> large({2}); large.add_line(std::numeric_limits<long long>::max(),0); rejects<std::overflow_error>([&]{large.query(2);});
    rejects<std::overflow_error>([]{ cp::min_plus_convolution_convex_convex(std::vector<long long>{std::numeric_limits<long long>::max()},std::vector<long long>{1}); });
}
void primes() {
    const int n=100000; std::vector<bool> prime(n,true); prime[0]=prime[1]=false;
    for(int p=2;p*p<n;++p) if(prime[p]) for(int j=p*p;j<n;j+=p) prime[j]=false;
    for(int i=0;i<n;++i) require(cp::is_prime_64(i)==prime[i],"Miller Rabin vs sieve");
    for(std::uint64_t x:{341550071728321ULL,3825123056546413051ULL,18446744073709551615ULL,9223372036854775807ULL}) require(!cp::is_prime_64(x),"strong pseudoprime and 64bit composites");
    for(std::uint64_t x:{2ULL,3ULL,2305843009213693951ULL,18446744073709551557ULL}) require(cp::is_prime_64(x),"64bit primes");
    for(int trial=0;trial<500;++trial) {
        std::uint64_t x=1+std::uint64_t(pick(10000000)),left=x; auto factors=cp::factorize_64(x,rng());
        require(std::is_sorted(factors.begin(),factors.end()),"sorted factors");
        for(auto p:factors) { require(p>=2 && cp::is_prime_64(p) && left%p==0,"factor prime and divides remaining"); left/=p; }
        require(left==1,"factorization exact product");
    }
    for(std::uint64_t x:{1ULL,1ULL<<63,1000000007ULL*1000000009ULL,4294967291ULL*4294967279ULL,18446744073709551615ULL,18446744073709551557ULL}) {
        auto factors=cp::factorize_64(x); auto left=x;
        for(auto p:factors) { require(cp::is_prime_64(p) && left%p==0,"hard semiprime prime factors"); left/=p; } require(left==1,"hard factorization product");
        auto grouped=cp::prime_factorization_64(x); std::vector<std::uint64_t> flattened; for(auto [p,k]:grouped) while(k--) flattened.push_back(p); require(flattened==factors,"grouped factorization");
    }
    rejects<std::invalid_argument>([]{cp::factorize_64(0);});
}
void convolution() {
    for(int trial=0;trial<300;++trial) {
        int n=pick(70),m=pick(70),mod=std::vector<int>{1,2,6,998244353,1000000007,2000000000}[pick(6)];
        std::vector<long long> a(n),b(m); for(auto &x:a) x=static_cast<long long>(rng()); for(auto &x:b) x=static_cast<long long>(rng());
        std::vector<int> want(n && m?n+m-1:0);
        auto norm=[&](long long x) { x%=mod; return x<0?x+mod:x; };
        for(int i=0;i<n;++i) for(int j=0;j<m;++j) want[i+j]=int((want[i+j]+norm(a[i])*norm(b[j]))%mod);
        require(cp::convolution_mod(a,b,mod)==want,"arbitrary-mod convolution vs quadratic modular sums");
    }
    require(cp::convolution_mod(std::vector<std::uint64_t>{std::numeric_limits<std::uint64_t>::max()},std::vector<int>{-1},1000000007)==std::vector<int>{int((1000000006ULL*(std::numeric_limits<std::uint64_t>::max()%1000000007))%1000000007)},"mixed signed unsigned CRT input");
    int n=65537; auto big=cp::convolution_mod(std::vector<int>(n,1999999999),std::vector<int>(n,1999999999),2000000000);
    for(int k=0;k<2*n-1;++k) require(big[k]==std::min(k+1,2*n-1-k),"CRT coefficient exceeds 64bit");
    rejects<std::invalid_argument>([]{cp::convolution_mod(std::vector<int>{},std::vector<int>{},0);});
    for(int bits=0;bits<=8;++bits) for(int trial=0;trial<25;++trial) {
        int count=1<<bits; std::vector<long long> a(count),b(count),want(count); for(auto &x:a) x=pick(7)-3; for(auto &x:b) x=pick(7)-3;
        for(int s=0;s<count;++s) for(int t=s;;t=(t-1)&s) { want[s]+=a[t]*b[s^t]; if(!t) break; }
        require(cp::subset_convolution(a,b)==want,"ranked subset convolution vs all submasks");
    }
    for(int bits=0;bits<=7;++bits) {
        int count=1<<bits; std::vector<ModInt> a(count),b(count),want(count);
        for(auto &x:a) x=ModInt(static_cast<long long>(rng()%ModInt::modulus));
        for(auto &x:b) x=ModInt(static_cast<long long>(rng()%ModInt::modulus));
        for(int s=0;s<count;++s) for(int t=s;;t=(t-1)&s) { want[s]+=a[t]*b[s^t]; if(!t) break; }
        require(cp::subset_convolution(a,b)==want,"subset convolution with nontrivial modular coefficients");
    }
    rejects<std::invalid_argument>([]{cp::subset_convolution(std::vector<int>{},std::vector<int>{});});
    rejects<std::invalid_argument>([]{cp::subset_convolution(std::vector<int>{1,2},std::vector<int>{1});});
}
int main() { optimization(); primes(); convolution(); std::cout<<"algebra: Monge rows, convex convolution, Li Chao scans, sieve, uint64 factors and exact convolutions passed\n"; }

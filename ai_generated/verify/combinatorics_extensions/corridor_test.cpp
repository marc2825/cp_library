#include "common.hpp"
#include "combinatorics/bounded_monotone.hpp"
#include "combinatorics/binomial.hpp"
int brute(const std::vector<long long>& lo,const std::vector<long long>& hi,bool strict) {
    if(lo.empty())return 1;
    auto low=*std::min_element(lo.begin(),lo.end()), high=*std::max_element(hi.begin(),hi.end());
    if(high<low)return 0;
    std::vector<int> dp(static_cast<std::size_t>(high-low+1)),next(dp.size());
    for(std::size_t i=0;i<lo.size();++i){int sum=0;for(std::size_t j=0;j<dp.size();++j){
        if(!strict)sum=(sum+dp[j])%998244353;
        next[j]=(low+static_cast<long long>(j)>=lo[i]&&low+static_cast<long long>(j)<=hi[i])?(i?sum:1):0;
        if(strict)sum=(sum+dp[j])%998244353;
    }dp=next;}
    int result=0;for(int x:dp)result=(result+x)%998244353;return result;
}
int main(){
    std::mt19937 random(981);std::size_t count=0;
    for(int n=0;n<=4;++n){int bound=1;for(int i=0;i<n*2;++i)bound*=3;
        for(int mask=0;mask<bound;++mask){int code=mask;std::vector<long long>l(n),h(n);for(auto&x:l){x=code%3-1;code/=3;}for(auto&x:h){x=code%3-1;code/=3;}
            for(bool strict:{false,true}){CHECK(cp::count_bounded_monotone_sequences(l,h,strict)==brute(l,h,strict));++count;}}
    }
    for(int it=0;it<1200;++it){int n=1+random()%100, span=1+random()%150;std::vector<long long>l(n),h(n);
        for(int i=0;i<n;++i){l[i]=random()%span;h[i]=random()%span;if(l[i]>h[i])std::swap(l[i],h[i]);}
        if(it%3){std::sort(l.begin(),l.end());std::sort(h.begin(),h.end());}
        for(bool strict:{false,true}){CHECK(cp::count_bounded_monotone_sequences(l,h,strict)==brute(l,h,strict));++count;}
    }
    for(int n:{1,2,17,100,777})for(int span:{0,1,17,99,501}){
        std::vector<long long>l(n,-50),h(n,span-50);cp::FactorialTable f(n+span);
        CHECK(cp::count_bounded_monotone_sequences(l,h)==cp::binomial(f,n+span,n));
    }
    const int n=20000;std::vector<long long>l(n),h(n,n);cp::FactorialTable f(2*n);
    CHECK(cp::count_bounded_monotone_sequences(l,h)==cp::binomial(f,2*n,n));
    for(int i=0;i<n;++i)h[i]=i;
    CHECK(cp::count_bounded_monotone_sequences(l,h)==cp::binomial(f,2*n,n)*cp::mod_pow(n+1,998244351,998244353)%998244353);
    for(int i=0;i<n;++i)l[i]=i;
    CHECK(cp::count_bounded_monotone_sequences(l,h)==1);
    CHECK(cp::count_bounded_monotone_sequences({LLONG_MIN,LLONG_MIN},{LLONG_MIN+3,LLONG_MIN+3},true)==6);
    CHECK(cp::count_bounded_monotone_sequences({LLONG_MAX-3,LLONG_MAX-3},{LLONG_MAX,LLONG_MAX},true)==6);
    CHECK(cp::count_bounded_monotone_sequences({3},{2})==0);
    throws<std::invalid_argument>([]{cp::count_bounded_monotone_sequences({1},{});});
    throws<std::length_error>([]{cp::count_bounded_monotone_sequences({LLONG_MIN},{LLONG_MAX});});
    throws<std::length_error>([]{cp::count_bounded_monotone_sequences({0},{10},false,12);});
    CHECK((cp::count_bounded_monotone_sequences<17,3>({0,0},{1,1})==3));
    std::cout<<"corridor: "<<count<<" independent DP comparisons, wide rectangles, Catalan and 20000-step boundaries passed\n";
}

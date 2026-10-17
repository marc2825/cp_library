#include <algorithm>
#include <climits>
#include <iostream>
#include <numeric>
#include <optional>
#include <random>
#include <stdexcept>
#include <vector>
#include "dp_optimization/all.hpp"
#include "data_structure/ConvexHullTrick.hpp"
void check(bool ok) { if(!ok) throw std::runtime_error("DP differential mismatch"); }
template<class E,class F> void throws(F f) { bool caught=false; try {f();} catch(const E&) {caught=true;} check(caught); }
int main() {
    std::mt19937 rng(88731);
    for(int trial=0;trial<550;++trial) {
        int n=rng()%22,groups=rng()%25; std::vector<long long> prefix(n+1);
        for(int i=0;i<n;++i) prefix[i+1]=prefix[i]+rng()%6;
        auto cost=[&](int l,int r) {return (prefix[r]-prefix[l])*(prefix[r]-prefix[l]);};
        auto a=cp::divide_conquer_dp(n,groups,cost);
        std::vector<std::vector<std::optional<long long>>> brute(groups+1,std::vector<std::optional<long long>>(n+1)); brute[0][0]=0;
        for(int k=1;k<=groups;++k) for(int r=1;r<=n;++r) for(int l=0;l<r;++l) if(brute[k-1][l]) {
            long long v=*brute[k-1][l]+cost(l,r);
            if(!brute[k][r] || v<*brute[k][r]) brute[k][r]=v;
        }
        check(a.dp==brute);
        for(int k=0;k<=groups;++k) for(int r=0;r<=n;++r) {
            auto parts=a.partitions(k,r);check(bool(parts)==bool(brute[k][r]));if(!parts)continue;
            int end=0;long long value=0;check(int(parts->size())==k);
            for(auto [l,u]:*parts) {check(l==end&&l<u);value+=cost(l,u);end=u;}
            check(end==r&&value==*brute[k][r]);
        }
        auto b=cp::knuth_optimization(n,[&](int l,int r){return prefix[r]-prefix[l];});
        auto c=cp::interval_dp(n,[&](int l,int,int r){return prefix[r]-prefix[l];});
        check(b.dp==c.dp&&b.split==c.split&&b.merge_order()==c.merge_order());
        std::vector<long long> heights(n),slow(n),cht_dp(n),li_dp(n);for(auto& h:heights) h=int(rng()%51)-25;
        long long constant=int(rng()%41)-10;
        CHT_min<long long> cht;cp::LiChaoTree<> li(heights);
        for(int i=0;i<n;++i) {
            if(i) {slow[i]=LLONG_MAX;for(int j=0;j<i;++j) slow[i]=std::min(slow[i],slow[j]+(heights[i]-heights[j])*(heights[i]-heights[j])+constant);}
            cht_dp[i]=i?cht.query(heights[i])+heights[i]*heights[i]+constant:0;
            li_dp[i]=i?*li.query(heights[i])+heights[i]*heights[i]+constant:0;
            cht.add(-2*heights[i],cht_dp[i]+heights[i]*heights[i]);li.add_line(-2*heights[i],li_dp[i]+heights[i]*heights[i]);
        }
        check(slow==cht_dp&&slow==li_dp);
    }
    for(int trial=0;trial<250;++trial) {
        int n=1+rng()%20;std::vector<long long> f(n+1);long long slope=-60;
        for(int i=1;i<=n;++i) {slope+=rng()%10;f[i]=f[i-1]+slope;}
        auto oracle=[&](long long penalty) {cp::PenalizedDPResult result{LLONG_MAX,0};for(int i=0;i<=n;++i){long long value=f[i]+penalty*i;if(value<=result.value)result={value,i};}return result;};
        for(int k=0;k<=n;++k) {auto r=cp::aliens_trick(k,-300,300,oracle);check(r&&r->value==f[k]);}
        check(!cp::aliens_trick(n+1,-300,300,oracle));
    }
    // 値の配列を渡すだけでなく、実際の罰則付き分割DPをoracleとして使う。
    for(int trial=0;trial<150;++trial) {
        int n=1+rng()%12;std::vector<long long> prefix(n+1);
        for(int i=0;i<n;++i)prefix[i+1]=prefix[i]+rng()%5;
        auto cost=[&](int l,int r){auto x=prefix[r]-prefix[l];return x*x;};
        auto exact=cp::divide_conquer_dp(n,n,cost);
        auto oracle=[&](long long penalty){
            std::vector<cp::PenalizedDPResult> dp(n+1,{LLONG_MAX,0});dp[0]={0,0};
            for(int r=1;r<=n;++r)for(int l=0;l<r;++l){auto v=dp[l].value+cost(l,r)+penalty;int count=dp[l].count+1;if(v<dp[r].value||(v==dp[r].value&&count>dp[r].count))dp[r]={v,count};}
            return dp[n];
        };
        for(int k=1;k<=n;++k){auto answer=cp::aliens_trick(k,-10000,10000,oracle);check(answer&&answer->value==*exact.dp[k][n]);}
    }
    // 全ての個数が同値になるpenalty、負の探索範囲、幅がLLONG_MAXを超える範囲。
    auto wide=cp::aliens_trick(1,LLONG_MIN,LLONG_MAX,[](long long p){return cp::PenalizedDPResult{p<=0?p:0,p<=0?1:0};});
    check(wide&&wide->penalty==0&&wide->value==0);
    for(int trial=0;trial<220;++trial) {
        constexpr int bound=250; std::vector<long long> grid(2*bound+1); cp::SlopeTrick f;
        for(int op=0;op<45;++op) {
            int type=rng()%9,a=int(rng()%21)-10;
            if(type<3) {if(type==0)f.add_abs(a);if(type==1)f.add_a_minus_x(a);if(type==2)f.add_x_minus_a(a);for(int x=-bound;x<=bound;++x)grid[x+bound]+=type==0?std::abs(x-a):type==1?std::max(a-x,0):std::max(x-a,0);}
            else if(type==3) {f.add_constant(a);for(auto& x:grid)x+=a;}
            else if(type==4) {f.prefix_min();for(int x=1;x<=2*bound;++x)grid[x]=std::min(grid[x],grid[x-1]);}
            else if(type==5) {f.suffix_min();for(int x=2*bound-1;x>=0;--x)grid[x]=std::min(grid[x],grid[x+1]);}
            else if(type<8) {int lo=int(rng()%5)-2,hi=lo+(type==6?0:int(rng()%4));f.shift(lo,hi);auto next=grid;for(int x=-bound;x<=bound;++x){long long best=LLONG_MAX/4;for(int y=x-hi;y<=x-lo;++y)if(y>=-bound&&y<=bound)best=std::min(best,grid[y+bound]);next[x+bound]=best;}grid.swap(next);}
            else {cp::SlopeTrick other;other.add_abs(a);other.shift(1);f.merge(other);for(int x=-bound;x<=bound;++x)grid[x+bound]+=std::abs(x-1-a);}
            for(int x=-30;x<=30;++x)check(f.evaluate(x)==grid[x+bound]);
            check(f.minimum()==*std::min_element(grid.begin()+100,grid.end()-100));
            auto interval=f.argmin();if(interval.first)check(f.evaluate(*interval.first)==f.minimum());if(interval.second)check(f.evaluate(*interval.second)==f.minimum());
        }
    }
    cp::SlopeTrick self;self.add_abs(2);self.merge(self);check(self.evaluate(-3)==10);
    throws<std::invalid_argument>([]{cp::divide_conquer_dp(-1,2,[](int,int){return 0;});});
    throws<std::invalid_argument>([]{cp::divide_conquer_dp_stage(std::vector<std::optional<long long>>{},[](int,int){return 0;});});
    throws<std::out_of_range>([]{cp::divide_conquer_dp(0,0,[](int,int){return 0;}).partitions(1,0);});
    throws<std::overflow_error>([]{cp::knuth_optimization(3,[](int,int){return LLONG_MAX;});});
    throws<std::overflow_error>([]{cp::detail::dp_subtract(0,LLONG_MIN);});
    throws<std::overflow_error>([]{cp::SlopeTrick f;f.add_constant(LLONG_MAX);f.add_constant(1);});
    throws<std::invalid_argument>([]{cp::SlopeTrick f;f.shift(1,0);});
    std::size_t calls=0;auto fast=cp::knuth_optimization(700,[&](int l,int r){++calls;return r-l;});check(fast.value()>0&&calls<500000);
    std::cout<<"partition/Knuth restoration, CHT/LiChao DP, WQS and slope-function oracles passed\n";
}

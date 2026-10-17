#include <algorithm>
#include <climits>
#include <cstdint>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <vector>
#include "basic_utility/all.hpp"

void check(bool ok) { if(!ok) throw std::runtime_error("basic differential mismatch"); }
int main() {
    for(int n=1;n<=20;++n) for(int s=-100;s<=100;++s) {
        std::vector<int> a(n); std::iota(a.begin(),a.end(),0); auto b=a;
        cp::rotate_left(b,s); for(int i=0;i<n;++i) check(b[i]==a[((i+s)%n+n)%n]);
        auto c=cp::cyclic_slice(a,s,3*n); for(int i=0;i<3*n;++i) check(c[i]==a[((i+s)%n+n)%n]);
    }
    std::vector<int> empty; cp::rotate_left(empty,LLONG_MIN); check(cp::cyclic_slice(empty,0,0).empty());
    check(cp::cyclic_index(LLONG_MIN,7)==6);
    for(int n=0;n<=300;++n) {
        int last=0;
        cp::for_each_quotient(n,[&](auto l,auto r,auto q) {
            check(l==std::uint64_t(last+1));
            for(auto i=l;i<=r;++i) check(n/i==q);
            check(r==std::uint64_t(n) || n/(r+1)!=q); last=int(r);
        }); check(last==n);
    }
    for(std::uint64_t l=0;l<30;++l) for(auto r=l;r<40;++r) for(std::uint64_t d=1;d<=50;++d) {
        std::vector<std::uint64_t> a,b;
        cp::for_each_multiple(l,r,d,[&](auto x){a.push_back(x);});
        for(auto x=l;x<r;++x) if(x%d==0) b.push_back(x);
        check(a==b);
    }
    std::vector<std::uint64_t> edge;
    cp::for_each_multiple(UINT64_MAX-5,UINT64_MAX,UINT64_MAX-2,[&](auto x){edge.push_back(x);});
    check(edge==std::vector<std::uint64_t>{UINT64_MAX-2});
    for(int bits=0;bits<=12;++bits) {
        std::vector<bool> seen(1<<bits); std::uint64_t previous=0; int count=0;
        cp::for_each_gray_code(bits,[&](auto g){
            check(!seen[g]); seen[g]=true; check(cp::gray_decode(g)==std::uint64_t(count));
            if(count) { auto x=previous^g; check(x && !(x&(x-1))); }
            previous=g; ++count;
        }); check(count==1<<bits);
    }
    for(auto x:{UINT64_MAX,UINT64_C(1)<<63,UINT64_C(0)}) check(cp::gray_decode(cp::gray_encode(x))==x);
    for(int n=0;n<=8;++n) {
        std::uint64_t count=0; cp::for_each_subset_pair(n,[&](auto s,auto t){check((s&t)==t);++count;});
        std::uint64_t expected=1; for(int i=0;i<n;++i) expected*=3; check(count==expected);
        std::vector<int> p(n); std::iota(p.begin(),p.end(),0); std::uint64_t rank=0;
        do { check(cp::permutation_rank(p)==rank); check(cp::permutation_unrank(n,rank)==p); ++rank; } while(std::next_permutation(p.begin(),p.end()));
    }
    auto f=cp::permutation_factorials(20); check(cp::permutation_rank(cp::permutation_unrank(20,f[20]-1))==f[20]-1);
    bool thrown=false; try {cp::permutation_rank({0,0});} catch(const std::invalid_argument&) {thrown=true;} check(thrown);
    thrown=false; try {cp::permutation_unrank(2,2);} catch(const std::out_of_range&) {thrown=true;} check(thrown);
    for(int mask=0;mask<1024;++mask) {
        std::vector<std::pair<int,int>> a; std::vector<bool> used(10);
        for(int i=0;i<10;++i) if(mask>>i&1) {a.emplace_back(i,i+1);used[i]=true;}
        a.emplace_back(0,0); auto merged=cp::merge_intervals(a);
        std::vector<bool> actual(10); int previous=-1;
        for(auto [l,r]:merged) { check(previous<l); previous=r; for(int i=l;i<r;++i) actual[i]=true; }
        check(actual==used);
        for(auto [l,r]:merged) { auto parts=cp::split_interval(l,r,{r,l-1,l,l+1,l+1,r+1}); check(cp::merge_intervals(parts)==std::vector<std::pair<int,int>>{{l,r}}); }
    }
    std::cout<<"basic: exhaustive permutations/subset pairs, intervals, cyclic/Gray and integer boundaries passed\n";
}

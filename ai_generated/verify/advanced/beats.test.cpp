#define PROBLEM "https://judge.yosupo.jp/problem/range_chmin_chmax_add_range_sum"
#include <iostream>
#include "range_query/segment_tree_beats.hpp"
int main() {
    int n,q; std::cin>>n>>q; std::vector<long long>a(n); for(auto &x:a)std::cin>>x;
    cp::SegmentTreeBeats s(a); while(q--) { int t,l,r; std::cin>>t>>l>>r; if(t==3)std::cout<<s.range_sum(l,r)<<'\n'; else { long long x; std::cin>>x; if(t==0)s.range_chmin(l,r,x); if(t==1)s.range_chmax(l,r,x); if(t==2)s.range_add(l,r,x); } }
}

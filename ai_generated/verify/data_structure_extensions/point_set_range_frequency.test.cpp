#define PROBLEM "https://judge.yosupo.jp/problem/point_set_range_frequency"
#include <iostream>
#include <vector>
#include "range_query/point_set_range_frequency.hpp"
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int n,q;std::cin>>n>>q;std::vector<int> a(n);for (int& x:a)std::cin>>x;
    cp::PointSetRangeFrequency<int> tree(a);
    while (q--) {int type;std::cin>>type;if (!type) {int p,x;std::cin>>p>>x;tree.set(p,x);} else {int l,r,x;std::cin>>l>>r>>x;std::cout<<tree.frequency(l,r,x)<<'\n';}}
}

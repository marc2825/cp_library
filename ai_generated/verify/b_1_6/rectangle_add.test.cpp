#define PROBLEM "https://judge.yosupo.jp/problem/static_rectangle_add_rectangle_sum"
#include <array>
#include <iostream>
#include <vector>
#include "offline_query/rectangle_add_rectangle_sum.hpp"
#include "modint.hpp"
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int n,q; std::cin>>n>>q; std::vector<std::array<long long,4>> rectangles(n); std::vector<long long> weights(n);
    for(int i=0;i<n;++i) std::cin>>rectangles[i][0]>>rectangles[i][1]>>rectangles[i][2]>>rectangles[i][3]>>weights[i];
    cp::RectangleAddRectangleSum<BModInt> tree(rectangles);
    for(int i=0;i<n;++i) { auto r=rectangles[i]; tree.add(r[0],r[1],r[2],r[3],weights[i]); }
    while(q--) { long long l,d,r,u; std::cin>>l>>d>>r>>u; std::cout<<tree.sum(l,d,r,u).value<<'\n'; }
}

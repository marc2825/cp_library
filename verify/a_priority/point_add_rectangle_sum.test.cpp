#define PROBLEM "https://judge.yosupo.jp/problem/point_add_rectangle_sum"
#include <array>
#include <iostream>
#include <utility>
#include <vector>
#include "offline_query/compressed_fenwick_tree_2d.hpp"
int main(){
    int n,q;std::cin>>n>>q;std::vector<std::array<long long,3>> initial(n);std::vector<std::pair<long long,long long>> points;
    for(auto& p:initial){std::cin>>p[0]>>p[1]>>p[2];points.emplace_back(p[0],p[1]);}
    std::vector<std::array<long long,5>> ops(q);
    for(auto& op:ops){std::cin>>op[0]>>op[1]>>op[2]>>op[3];if(op[0]==0)points.emplace_back(op[1],op[2]);else std::cin>>op[4];}
    cp::CompressedFenwickTree2D<long long> bit(points);for(auto p:initial)bit.add(p[0],p[1],p[2]);
    for(auto op:ops)if(op[0]==0)bit.add(op[1],op[2],op[3]);else std::cout<<bit.sum(op[1],op[2],op[3],op[4])<<'\n';
}

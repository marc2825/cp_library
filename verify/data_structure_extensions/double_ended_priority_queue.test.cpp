#define PROBLEM "https://judge.yosupo.jp/problem/double_ended_priority_queue"
#include <iostream>
#include "ordered_structure/double_ended_priority_queue.hpp"
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int n,q; std::cin>>n>>q; cp::DoubleEndedPriorityQueue<long long> heap;
    for (int i=0;i<n;++i) {long long x;std::cin>>x;heap.push(x);}
    while (q--) {int type;std::cin>>type;if (!type) {long long x;std::cin>>x;heap.push(x);} else if (type==1) {std::cout<<heap.min()<<'\n';heap.pop_min();} else {std::cout<<heap.max()<<'\n';heap.pop_max();}}
}

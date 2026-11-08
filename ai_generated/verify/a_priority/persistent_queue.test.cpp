#define PROBLEM "https://judge.yosupo.jp/problem/persistent_queue"
#include <iostream>
#include <vector>
#include "ordered_structure/persistent_queue.hpp"
int main(){
    int q;std::cin>>q;cp::PersistentQueue<int> queue;std::vector<int> roots(q+1,0);
    for(int i=0;i<q;++i){int type,t;std::cin>>type>>t;if(type==0){int x;std::cin>>x;roots[i+1]=queue.push(roots[t+1],x);}else{std::cout<<queue.front(roots[t+1])<<'\n';roots[i+1]=queue.pop(roots[t+1]);}}
}

#define PROBLEM "https://judge.yosupo.jp/problem/predecessor_problem"
#include <iostream>
#include <string>
#include "static_query/fast_set.hpp"
int main(){
    int n,q;std::string s;std::cin>>n>>q>>s;cp::FastSet set(n);for(int i=0;i<n;++i)if(s[i]=='1')set.insert(i);
    while(q--){int type,k;std::cin>>type>>k;if(type==0)set.insert(k);else if(type==1)set.erase(k);else if(type==2)std::cout<<set.contains(k)<<'\n';else if(type==3){int x=set.next(k);std::cout<<(x==n?-1:x)<<'\n';}else std::cout<<set.prev(k)<<'\n';}
}
